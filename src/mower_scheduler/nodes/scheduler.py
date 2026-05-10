#!/usr/bin/env python3
"""mower_scheduler — time-based mowing schedule for OpenMower.

Subscribes to xbot_monitoring/robot_state to know whether the mower is idle
and safe to dispatch. Loads its schedules from disk, ticks every minute,
fires mower_logic/start_mowing via the /xbot/action publisher when a job is
due. Three RPC methods (schedule.list / schedule.upsert / schedule.delete)
let the frontend manage the schedule store.

The persisted file at ~/.openmower/schedules.json has the shape:

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
        "weather": {"skip_if_rain": true},
        "pattern": {"angle_offset": 0.0, "rotate_by_days": 7}
      }
    ]
  }
"""

import datetime
import json
import os
import secrets
import tempfile
import threading
import time
from typing import Any, Optional

import rospy
from dateutil.rrule import rrulestr
from std_msgs.msg import String
from xbot_msgs.msg import RobotState
from xbot_rpc.msg import RpcError, RpcRequest, RpcResponse
from xbot_rpc.srv import RegisterMethodsSrv, RegisterMethodsSrvRequest

# JSON-RPC error codes mirror xbot_rpc/RpcError.msg constants. We avoid pulling
# them off the message class because that requires generated bindings.
ERROR_INVALID_PARAMS = -32602
ERROR_INTERNAL = -32603

DEFAULT_PATH = os.path.expanduser("~/.openmower/schedules.json")
START_ACTION_ID = "mower_logic/start_mowing"
TICK_INTERVAL_SECONDS = 60.0
ROBOT_STATE_TOPIC = "xbot_monitoring/robot_state"
ACTION_TOPIC = "/xbot/action"

NODE_ID = "mower_scheduler"
RPC_METHODS = ("schedule.list", "schedule.upsert", "schedule.delete")


class RpcException(Exception):
    """Raised inside an RPC handler to control the JSON-RPC error response."""

    def __init__(self, code: int, message: str):
        super().__init__(message)
        self.code = code
        self.message = message


class ScheduleStore:
    """Atomic JSON file backing for the schedule list.

    The store keeps every schedule's `_last_fired_iso` field separate from the
    user-visible payload — that lets us deduplicate fires across restarts
    without leaking implementation details into the RPC surface.
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
        with self._lock:
            return [self._strip_internal(s) for s in self._schedules]

    def upsert(self, schedule: dict) -> dict:
        cleaned = self._validate(schedule)
        with self._lock:
            for i, existing in enumerate(self._schedules):
                if existing["id"] == cleaned["id"]:
                    cleaned["_last_fired_iso"] = existing.get("_last_fired_iso")
                    self._schedules[i] = cleaned
                    self._save_unlocked()
                    return self._strip_internal(cleaned)
            self._schedules.append(cleaned)
            self._save_unlocked()
            return self._strip_internal(cleaned)

    def delete(self, schedule_id: str) -> None:
        with self._lock:
            before = len(self._schedules)
            self._schedules = [s for s in self._schedules if s["id"] != schedule_id]
            if len(self._schedules) != before:
                self._save_unlocked()

    def take_due(self, now: datetime.datetime, window_seconds: float) -> list[dict]:
        """Return enabled schedules with an RRULE occurrence inside the
        window [now - window, now]; mark them as fired.

        Mutates the store and persists; should be called from the tick loop.
        """
        fired: list[dict] = []
        window_start = now - datetime.timedelta(seconds=window_seconds)
        with self._lock:
            mutated = False
            for s in self._schedules:
                if not s.get("enabled", False):
                    continue
                rule_str = s.get("rrule")
                if not rule_str:
                    continue
                try:
                    rule = rrulestr(rule_str, dtstart=window_start.replace(tzinfo=None))
                except Exception as e:  # noqa: BLE001 — defensive against malformed RRULEs
                    rospy.logwarn_throttle(
                        300, "Schedule %s has invalid RRULE %r: %s", s.get("id"), rule_str, e
                    )
                    continue
                between = rule.between(window_start.replace(tzinfo=None), now.replace(tzinfo=None), inc=True)
                if not between:
                    continue
                # Deduplicate against last fire time.
                last_iso = s.get("_last_fired_iso")
                last_dt = (
                    datetime.datetime.fromisoformat(last_iso) if last_iso else None
                )
                most_recent = between[-1]
                if last_dt is not None and most_recent <= last_dt:
                    continue
                s["_last_fired_iso"] = most_recent.isoformat()
                fired.append(self._strip_internal(s))
                mutated = True
            if mutated:
                self._save_unlocked()
        return fired

    @staticmethod
    def _strip_internal(schedule: dict) -> dict:
        return {k: v for k, v in schedule.items() if not k.startswith("_")}

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

        cleaned = {
            "id": sid,
            "name": name,
            "enabled": enabled,
            "areas": list(areas),
            "rrule": rrule,
            "duration_minutes": int(duration_minutes),
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
        now = datetime.datetime.now()
        # On first tick after start we pretend the previous tick was 60 s ago
        # so we don't replay the entire morning's schedule; subsequent ticks
        # cover the actual interval between firings.
        due = self.store.take_due(now, TICK_INTERVAL_SECONDS)
        if not due:
            return

        with self._state_lock:
            state = self._state

        if state is None:
            rospy.loginfo("Skipping %d due schedule(s) — robot_state not received yet", len(due))
            return
        if state.emergency:
            rospy.loginfo("Skipping %d due schedule(s) — emergency active", len(due))
            return
        if state.current_state != "IDLE":
            rospy.loginfo(
                "Skipping %d due schedule(s) — current_state is %s",
                len(due),
                state.current_state,
            )
            return
        if state.is_charging and state.battery_percentage < 0.95:
            rospy.loginfo("Skipping %d due schedule(s) — still charging", len(due))
            return

        for schedule in due:
            if schedule.get("weather", {}).get("skip_if_rain") and state.rain_detected:
                rospy.loginfo("Skipping schedule %s — rain detected", schedule["id"])
                continue
            self._fire(schedule)

    def _fire(self, schedule: dict) -> None:
        rospy.loginfo("Firing schedule %s (%s)", schedule["id"], schedule["name"])
        # Patch 3 will introduce per-area triggers; for now we just kick off
        # the generic start_mowing action and rely on whatever area selection
        # mower_logic has cached. The areas[] list is forwarded as a ROS
        # parameter so a future Patch-3 follow-up can pick it up.
        if schedule.get("areas"):
            rospy.set_param("/mower_scheduler/last_fired_areas", list(schedule["areas"]))
        msg = String()
        msg.data = START_ACTION_ID
        self._action_pub.publish(msg)


def main() -> None:
    SchedulerNode()
    rospy.spin()


if __name__ == "__main__":
    main()
