#!/usr/bin/env python3
"""mower_scheduler — time-based mowing schedule for OpenMower.

Subscribes to xbot_monitoring/robot_state to know whether the mower is idle
and safe to dispatch. Loads its schedules from disk, ticks every minute,
fires mower_logic/start_mowing via the /xbot/action publisher when a job is
due. Three RPC methods (schedule.list / schedule.upsert / schedule.delete)
let the frontend manage the schedule store.

The persisted file at $ROS_HOME/openmower/schedules.json has the shape:

  {
    "version": 1,
    "schedules": [
      {
        "id": "abc123",
        "name": "Front lawn weekly",
        "enabled": true,
        "areas": [0],
        "rrule": "FREQ=WEEKLY;BYDAY=MO,WE,FR;BYHOUR=10;BYMINUTE=0",
        "duration_minutes": 60,
        "timezone": "Europe/Vienna",
        "weather": {"skip_if_rain": true},
        "pattern": {"angle_offset": 0.0, "rotate_by_days": 7}
      }
    ]
  }
"""

from __future__ import annotations

import datetime
import json
import os
import secrets
import tempfile
import threading
from typing import Any, Optional

import rospy
from dateutil.rrule import rrulestr
from dateutil.tz import gettz
from std_msgs.msg import String
from xbot_msgs.msg import RobotState
from xbot_rpc.msg import RpcError, RpcRequest, RpcResponse
from xbot_rpc.srv import RegisterMethodsSrv, RegisterMethodsSrvRequest

from mower_scheduler import event_publisher


def _resolve_zone(name: str):
    """Resolve an IANA zone name. Returns None if the name is unknown.

    `dateutil.tz.gettz` works on Python 3.8 (Noetic) without needing a separate
    zoneinfo backport. We avoid `zoneinfo` directly because it pulls in either
    a 3.9+ stdlib or a `python3-backports.zoneinfo` apt package that has no
    rosdep mapping on Focal.
    """
    if not isinstance(name, str) or not name:
        return None
    return gettz(name)

# JSON-RPC error codes mirror xbot_rpc/RpcError.msg constants. We avoid pulling
# them off the message class because that requires generated bindings.
ERROR_INVALID_PARAMS = -32602
ERROR_INTERNAL = -32603

DEFAULT_PATH = os.path.expanduser(
    os.environ.get("ROS_HOME", "~/.ros") + "/openmower/schedules.json"
)
# Mirrors MOWER_ACTIONS.startMowing in openmower-app/src/lib/mowerActions.ts
# and the IdleBehavior::handle_action match in
# open_mower_ros/src/mower_logic/.../IdleBehavior.cpp. The ":idle/" namespace
# prefix is mandatory — without it IdleBehavior silently drops the message.
START_ACTION_ID = "mower_logic:idle/start_mowing"
# ROS param read by MowingBehavior to pick the starting area; the same channel
# that map.start_in_area writes to. MowingBehavior resets it to -1 after use.
NEXT_AREA_PARAM = "/mower_logic/next_area_index"
TICK_INTERVAL_SECONDS = 60.0
# How far back a missed occurrence can still be picked up. Covers transient
# blockers like a 5-min docking cycle, but bounds the window so a schedule
# stuck in "charging" all night does not suddenly fire at 04:00.
CATCHUP_WINDOW = datetime.timedelta(hours=1)
ROBOT_STATE_TOPIC = "xbot_monitoring/robot_state"
ACTION_TOPIC = "/xbot/action"

NODE_ID = "mower_scheduler"
RPC_METHODS = ("schedule.list", "schedule.upsert", "schedule.delete")

# Skip reason constants shared with the openrpc.json contract.
SKIP_NO_STATE = "no_state"
SKIP_EMERGENCY = "emergency"
SKIP_NOT_IDLE = "not_idle"
SKIP_CHARGING = "charging"
SKIP_RAIN = "rain"


class RpcException(Exception):
    """Raised inside an RPC handler to control the JSON-RPC error response."""

    def __init__(self, code: int, message: str):
        super().__init__(message)
        self.code = code
        self.message = message


def _utcnow() -> datetime.datetime:
    return datetime.datetime.now(datetime.timezone.utc)


def _parse_iso_utc(value: Optional[str]) -> Optional[datetime.datetime]:
    if not value:
        return None
    try:
        dt = datetime.datetime.fromisoformat(value)
    except ValueError:
        return None
    if dt.tzinfo is None:
        # Legacy values written before TZ-aware times existed are in local time;
        # normalise via the system zone so dedup comparisons stay consistent.
        dt = dt.replace(tzinfo=datetime.timezone.utc)
    return dt.astimezone(datetime.timezone.utc)


class ScheduleStore:
    """Atomic JSON file backing for the schedule list.

    The store keeps every schedule's `_last_fired_iso` and skip-reason fields
    separate from the user-visible payload so RPC clients see a clean shape.
    """

    def __init__(self, path: str):
        self.path = path
        self._lock = threading.Lock()
        self._schedules: list[dict] = []
        self._load()

    def _load(self) -> None:
        try:
            with open(self.path, "r", encoding="utf-8") as f:
                data = json.load(f)
        except FileNotFoundError:
            self._schedules = []
            return
        except (OSError, ValueError) as e:
            rospy.logwarn("Failed to load %s, starting empty: %s", self.path, e)
            self._schedules = []
            return
        self._schedules = list(data.get("schedules", []))

    def _save_unlocked(self) -> None:
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        # Tempfile + os.replace gives us an atomic write; a crash during write
        # leaves the previous file untouched.
        fd, tmp = tempfile.mkstemp(prefix=".schedules.", dir=os.path.dirname(self.path))
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as f:
                json.dump({"version": 1, "schedules": self._schedules}, f, indent=2)
            os.replace(tmp, self.path)
        except Exception:
            try:
                os.unlink(tmp)
            except OSError:
                pass
            raise

    def list_public(self) -> list[dict]:
        now = _utcnow()
        with self._lock:
            return [self._public_view(s, now) for s in self._schedules]

    def upsert(self, schedule: dict) -> dict:
        cleaned = self._validate(schedule)
        with self._lock:
            for i, existing in enumerate(self._schedules):
                if existing["id"] == cleaned["id"]:
                    # Preserve runtime state across edits.
                    for key in ("_last_fired_iso", "_last_skip_reason", "_last_skip_at"):
                        if key in existing:
                            cleaned[key] = existing[key]
                    self._schedules[i] = cleaned
                    self._save_unlocked()
                    return self._public_view(cleaned, _utcnow())
            self._schedules.append(cleaned)
            self._save_unlocked()
            return self._public_view(cleaned, _utcnow())

    def delete(self, schedule_id: str) -> None:
        with self._lock:
            before = len(self._schedules)
            self._schedules = [s for s in self._schedules if s["id"] != schedule_id]
            if len(self._schedules) != before:
                self._save_unlocked()

    def evaluate_due(self, now_utc: datetime.datetime) -> list[tuple[dict, datetime.datetime]]:
        """Return enabled schedules with an unfired occurrence inside the
        catch-up window, paired with the (UTC) occurrence timestamp.

        Pure read against the store — does not mutate or persist; the caller
        is expected to call `record_fire` / `record_skip` afterwards.
        """
        candidates: list[tuple[dict, datetime.datetime]] = []
        window_start_utc = now_utc - CATCHUP_WINDOW
        with self._lock:
            for s in self._schedules:
                if not s.get("enabled", False):
                    continue
                rule_str = s.get("rrule")
                if not rule_str:
                    continue
                tz = self._zone_for(s)
                window_start_local = window_start_utc.astimezone(tz)
                now_local = now_utc.astimezone(tz)
                try:
                    rule = rrulestr(rule_str, dtstart=window_start_local)
                except Exception as e:  # noqa: BLE001 — defensive against malformed RRULEs
                    rospy.logwarn_throttle(
                        300, "Schedule %s has invalid RRULE %r: %s", s.get("id"), rule_str, e
                    )
                    continue
                between = rule.between(window_start_local, now_local, inc=True)
                if not between:
                    continue
                most_recent_utc = between[-1].astimezone(datetime.timezone.utc)
                last_fired = _parse_iso_utc(s.get("_last_fired_iso"))
                if last_fired is not None and most_recent_utc <= last_fired:
                    continue
                # Snapshot the dict so callers see a stable view even if the
                # store mutates underneath.
                candidates.append((dict(s), most_recent_utc))
        return candidates

    def record_fire(self, schedule_id: str, occurrence_utc: datetime.datetime) -> None:
        iso = occurrence_utc.astimezone(datetime.timezone.utc).isoformat()
        with self._lock:
            for s in self._schedules:
                if s["id"] != schedule_id:
                    continue
                s["_last_fired_iso"] = iso
                # A successful firing clears any stale skip annotation.
                s.pop("_last_skip_reason", None)
                s.pop("_last_skip_at", None)
                self._save_unlocked()
                return

    def record_skip(self, schedule_id: str, reason: str, now_utc: datetime.datetime) -> None:
        iso = now_utc.astimezone(datetime.timezone.utc).isoformat()
        with self._lock:
            for s in self._schedules:
                if s["id"] != schedule_id:
                    continue
                # Avoid rewriting the file when the same reason persists across
                # consecutive ticks — that would thrash the SD card.
                if s.get("_last_skip_reason") == reason and s.get("_last_skip_at"):
                    return
                s["_last_skip_reason"] = reason
                s["_last_skip_at"] = iso
                self._save_unlocked()
                return

    @staticmethod
    def _zone_for(schedule: dict) -> Any:
        tz_name = schedule.get("timezone") or "UTC"
        tz = _resolve_zone(tz_name)
        if tz is None:
            rospy.logwarn_throttle(
                300, "Schedule %s has unknown timezone %r, falling back to UTC",
                schedule.get("id"), tz_name,
            )
            return _resolve_zone("UTC") or datetime.timezone.utc
        return tz

    def _public_view(self, schedule: dict, now_utc: datetime.datetime) -> dict:
        out: dict = {k: v for k, v in schedule.items() if not k.startswith("_")}
        # Only emit read-only status fields when they have a value, mirroring
        # the openrpc.json contract (these properties are optional, not nullable).
        last_fired = schedule.get("_last_fired_iso")
        if last_fired:
            out["last_fired_at"] = last_fired
        skip_reason = schedule.get("_last_skip_reason")
        if skip_reason:
            out["last_skip_reason"] = skip_reason
        skip_at = schedule.get("_last_skip_at")
        if skip_at:
            out["last_skip_at"] = skip_at
        nxt = self._next_run(schedule, now_utc)
        if nxt:
            out["next_run"] = nxt
        return out

    @staticmethod
    def _next_run(schedule: dict, now_utc: datetime.datetime) -> Optional[str]:
        if not schedule.get("enabled", False):
            return None
        rule_str = schedule.get("rrule")
        if not rule_str:
            return None
        tz = ScheduleStore._zone_for(schedule)
        now_local = now_utc.astimezone(tz)
        try:
            rule = rrulestr(rule_str, dtstart=now_local)
        except Exception:  # noqa: BLE001
            return None
        nxt = rule.after(now_local, inc=False)
        if nxt is None:
            return None
        return nxt.astimezone(datetime.timezone.utc).isoformat()

    @staticmethod
    def _validate(schedule: Any) -> dict:
        if not isinstance(schedule, dict):
            raise RpcException(ERROR_INVALID_PARAMS, "schedule must be an object")

        sid = schedule.get("id") or secrets.token_hex(8)
        name = schedule.get("name", "Unnamed")
        if not isinstance(name, str):
            raise RpcException(ERROR_INVALID_PARAMS, "name must be a string")
        enabled = bool(schedule.get("enabled", True))
        areas = schedule.get("areas", [])
        if not isinstance(areas, list) or not all(isinstance(a, int) for a in areas):
            raise RpcException(ERROR_INVALID_PARAMS, "areas must be a list of integers")
        rrule = schedule.get("rrule", "")
        if not isinstance(rrule, str) or not rrule.strip():
            raise RpcException(ERROR_INVALID_PARAMS, "rrule must be a non-empty string")
        try:
            rrulestr(rrule)
        except Exception as e:  # noqa: BLE001
            raise RpcException(ERROR_INVALID_PARAMS, f"invalid rrule: {e}")
        duration_minutes = schedule.get("duration_minutes", 60)
        if not isinstance(duration_minutes, (int, float)) or duration_minutes <= 0:
            raise RpcException(ERROR_INVALID_PARAMS, "duration_minutes must be a positive number")
        # Timezone is required for the contract; legacy files without it are
        # patched to UTC on load so they keep working until the user re-saves.
        tz_name = schedule.get("timezone")
        if tz_name is None:
            tz_name = "UTC"
            rospy.logwarn(
                "Schedule %s has no timezone, defaulting to UTC. Re-save in the app to pin it.",
                sid,
            )
        if not isinstance(tz_name, str):
            raise RpcException(ERROR_INVALID_PARAMS, "timezone must be a string")
        if _resolve_zone(tz_name) is None:
            raise RpcException(ERROR_INVALID_PARAMS, f"invalid timezone {tz_name!r}")

        cleaned = {
            "id": sid,
            "name": name,
            "enabled": enabled,
            "areas": list(areas),
            "rrule": rrule,
            "duration_minutes": int(duration_minutes),
            "timezone": tz_name,
        }
        if "weather" in schedule and isinstance(schedule["weather"], dict):
            cleaned["weather"] = {"skip_if_rain": bool(schedule["weather"].get("skip_if_rain", False))}
        if "pattern" in schedule and isinstance(schedule["pattern"], dict):
            p = schedule["pattern"]
            cleaned["pattern"] = {
                "angle_offset": float(p.get("angle_offset", 0.0)),
                "rotate_by_days": int(p.get("rotate_by_days", 0)),
            }
        return cleaned


class SchedulerNode:
    def __init__(self) -> None:
        rospy.init_node(NODE_ID)
        event_publisher.init("mower_scheduler")
        path = rospy.get_param("~schedules_path", DEFAULT_PATH)
        self.store = ScheduleStore(path)
        rospy.loginfo("mower_scheduler loaded %d schedule(s) from %s", len(self.store.list_public()), path)

        self._state_lock = threading.Lock()
        self._state: Optional[RobotState] = None

        self._action_pub = rospy.Publisher(ACTION_TOPIC, String, queue_size=10)
        self._rpc_response_pub = rospy.Publisher("/xbot/rpc/response", RpcResponse, queue_size=10)
        self._rpc_error_pub = rospy.Publisher("/xbot/rpc/error", RpcError, queue_size=10)

        rospy.Subscriber(ROBOT_STATE_TOPIC, RobotState, self._on_state, queue_size=1)
        rospy.Subscriber("/xbot/rpc/request", RpcRequest, self._on_rpc, queue_size=20)

        # The xbot_monitoring service tracks which methods are owned by which
        # node so the request dispatcher knows who to route to.
        try:
            rospy.wait_for_service("/xbot/rpc/register", timeout=10.0)
            register = rospy.ServiceProxy("/xbot/rpc/register", RegisterMethodsSrv)
            req = RegisterMethodsSrvRequest()
            req.node_id = NODE_ID
            req.methods = list(RPC_METHODS)
            register(req)
            rospy.loginfo("mower_scheduler registered %d RPC method(s)", len(RPC_METHODS))
        except rospy.ROSException:
            rospy.logwarn("RPC registration service not available; methods will not be routable")

        # Tick loop. We use a Timer so it runs even if rospy.spin() is busy
        # processing a flood of RPC messages.
        rospy.Timer(rospy.Duration(TICK_INTERVAL_SECONDS), self._tick)
        # The Timer's first fire is `TICK_INTERVAL_SECONDS` away; do an
        # immediate evaluation so a schedule that just became due at boot
        # is picked up without the 60 s delay.
        rospy.Timer(rospy.Duration(1.0), self._tick, oneshot=True)

    def _on_state(self, msg: RobotState) -> None:
        with self._state_lock:
            self._state = msg

    def _on_rpc(self, request: RpcRequest) -> None:
        method = request.method
        if method not in RPC_METHODS:
            return  # routed to a different provider
        try:
            params = json.loads(request.params) if request.params else None
        except ValueError as e:
            self._publish_error(request.id, ERROR_INVALID_PARAMS, f"Invalid params JSON: {e}")
            return
        try:
            result = self._dispatch(method, params)
        except RpcException as e:
            self._publish_error(request.id, e.code, e.message)
            return
        except Exception as e:  # noqa: BLE001 — return a clean error to the client
            rospy.logerr("RPC %s crashed: %s", method, e)
            self._publish_error(request.id, ERROR_INTERNAL, str(e))
            return
        self._publish_response(request.id, result)

    def _dispatch(self, method: str, params: Any) -> Any:
        if method == "schedule.list":
            return self.store.list_public()
        if method == "schedule.upsert":
            schedule = self._extract_param(params, "schedule")
            return self.store.upsert(schedule)
        if method == "schedule.delete":
            sid = self._extract_param(params, "id")
            if not isinstance(sid, str):
                raise RpcException(ERROR_INVALID_PARAMS, "id must be a string")
            self.store.delete(sid)
            return None
        raise RpcException(ERROR_INTERNAL, f"Unhandled method: {method}")

    @staticmethod
    def _extract_param(params: Any, name: str) -> Any:
        # Accept by-name {key: value}, single-element positional [{key: ...}],
        # or single-element positional that is the value itself ([value]).
        if isinstance(params, dict) and name in params:
            return params[name]
        if isinstance(params, list) and len(params) == 1:
            inner = params[0]
            if isinstance(inner, dict) and name in inner:
                return inner[name]
            return inner
        raise RpcException(ERROR_INVALID_PARAMS, f"missing parameter '{name}'")

    def _publish_response(self, request_id: str, result: Any) -> None:
        if not request_id:
            return
        msg = RpcResponse()
        msg.id = request_id
        msg.result = json.dumps(result if result is not None else None)
        self._rpc_response_pub.publish(msg)

    def _publish_error(self, request_id: str, code: int, message: str) -> None:
        if not request_id:
            return
        msg = RpcError()
        msg.id = request_id
        msg.code = code
        msg.message = message
        self._rpc_error_pub.publish(msg)

    def _tick(self, _event) -> None:
        now_utc = _utcnow()
        candidates = self.store.evaluate_due(now_utc)
        if not candidates:
            return

        with self._state_lock:
            state = self._state

        # Determine the global block reason once — it applies uniformly to
        # every due candidate, so we annotate them all with the same skip.
        global_reason = self._global_block_reason(state)
        if global_reason:
            rospy.loginfo(
                "Skipping %d due schedule(s) — %s",
                len(candidates),
                global_reason,
            )
            for schedule, _ in candidates:
                self.store.record_skip(schedule["id"], global_reason, now_utc)
                self._emit_skip(schedule, global_reason)
            return

        for schedule, occurrence_utc in candidates:
            if schedule.get("weather", {}).get("skip_if_rain") and getattr(state, "rain_detected", False):
                rospy.loginfo("Skipping schedule %s — rain detected", schedule["id"])
                self.store.record_skip(schedule["id"], SKIP_RAIN, now_utc)
                self._emit_skip(schedule, SKIP_RAIN)
                continue
            self._fire(schedule, occurrence_utc, now_utc)

    @staticmethod
    def _global_block_reason(state: Optional[RobotState]) -> Optional[str]:
        if state is None:
            return SKIP_NO_STATE
        if state.emergency:
            return SKIP_EMERGENCY
        if state.current_state != "IDLE":
            return SKIP_NOT_IDLE
        if state.is_charging and state.battery_percentage < 0.95:
            return SKIP_CHARGING
        return None

    def _fire(self, schedule: dict, occurrence_utc: datetime.datetime, now_utc: datetime.datetime) -> None:
        areas = schedule.get("areas") or []
        area_index = areas[0] if areas else None
        if len(areas) > 1:
            rospy.logwarn_throttle(
                300,
                "Schedule %s lists %d areas; only the first (%d) will be used. "
                "Multi-area scheduling is not implemented yet.",
                schedule["id"], len(areas), area_index,
            )
        if area_index is not None:
            # Same channel map.start_in_area uses; MowingBehavior consumes and
            # clears it on entry, so we don't need to reset it here.
            rospy.set_param(NEXT_AREA_PARAM, int(area_index))
            rospy.loginfo(
                "Firing schedule %s (%s) in area %d", schedule["id"], schedule["name"], area_index,
            )
        else:
            rospy.loginfo("Firing schedule %s (%s)", schedule["id"], schedule["name"])
        # Persist before publishing so a crash between the two does not lead
        # to the action firing again on the next tick: the dedup check in
        # evaluate_due() relies on _last_fired_iso being on disk.
        self.store.record_fire(schedule["id"], occurrence_utc)
        msg = String()
        msg.data = START_ACTION_ID
        self._action_pub.publish(msg)
        event_publisher.info(
            "schedule.run_started",
            f"Schedule '{schedule['name']}' fired",
            {"schedule_id": schedule["id"], "name": schedule["name"], "area_index": area_index},
        )

    @staticmethod
    def _emit_skip(schedule: dict, reason: str) -> None:
        # Most skip reasons are mundane (e.g. mower not idle yet) — surface
        # them at INFO. Rain and emergency are explicit user-visible signals
        # so bump them to WARNING.
        severity = (
            event_publisher.SEVERITY_WARNING
            if reason in (SKIP_RAIN, SKIP_EMERGENCY)
            else event_publisher.SEVERITY_INFO
        )
        event_publisher.emit(
            severity,
            "schedule.run_skipped",
            f"Schedule '{schedule['name']}' skipped — {reason}",
            {"schedule_id": schedule["id"], "name": schedule["name"], "reason": reason},
        )


def main() -> None:
    SchedulerNode()
    rospy.spin()


if __name__ == "__main__":
    main()
