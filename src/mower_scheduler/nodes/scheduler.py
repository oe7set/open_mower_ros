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

import collections
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

try:
    import holidays as _holidays
except ImportError:  # pragma: no cover - optional dependency
    # Country-holiday blocking is a best-effort feature. Without the package,
    # manual blocking days still work; holiday checks are simply disabled.
    _holidays = None
from dynamic_reconfigure.msg import IntParameter, Config as ReconfigureConfig
from dynamic_reconfigure.srv import Reconfigure, ReconfigureRequest
from std_msgs.msg import String
from xbot_msgs.msg import Event, RobotState
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
DEFAULT_HISTORY_PATH = os.path.expanduser(
    os.environ.get("ROS_HOME", "~/.ros") + "/openmower/run_history.json"
)
DEFAULT_EXCEPTIONS_PATH = os.path.expanduser(
    os.environ.get("ROS_HOME", "~/.ros") + "/openmower/exceptions.json"
)
# Global lifecycle-event topic the EventPublisher writes to. We subscribe to
# correlate mowing outcomes back to the run we dispatched (by run_id).
EVENTS_TOPIC = "/events"
# Mirrors MOWER_ACTIONS.startMowing in openmower-app/src/lib/mowerActions.ts
# and the IdleBehavior::handle_action match in
# open_mower_ros/src/mower_logic/.../IdleBehavior.cpp. The ":idle/" namespace
# prefix is mandatory — without it IdleBehavior silently drops the message.
START_ACTION_ID = "mower_logic:idle/start_mowing"
# Action that stops the current mow and sends the robot home to dock. Handled by
# MowingBehavior::handle_action; the scheduler uses it to enforce stop-at-time
# (duration_minutes for time_area, the window end for time_window).
ABORT_ACTION_ID = "mower_logic:mowing/abort_mowing"
# ROS param read by MowingBehavior to pick the starting area; the same channel
# that map.start_in_area writes to. MowingBehavior resets it to -1 after use.
NEXT_AREA_PARAM = "/mower_logic/next_area_index"
# Namespace for the richer per-run dispatch block the scheduler writes before
# firing: <ns>/area_indices (int[]), /pattern, /angle_deg, /speed_mps,
# /outline_count, /stop_at_epoch, /run_id. MowingBehavior consumes and clears
# area_indices on entry.
NEXT_RUN_PARAM = "/mower_logic/next_run"
TICK_INTERVAL_SECONDS = 60.0
# How far back a missed occurrence can still be picked up. Covers transient
# blockers like a 5-min docking cycle, but bounds the window so a schedule
# stuck in "charging" all night does not suddenly fire at 04:00.
CATCHUP_WINDOW = datetime.timedelta(hours=1)
ROBOT_STATE_TOPIC = "xbot_monitoring/robot_state"
ACTION_TOPIC = "/xbot/action"

# mower_logic automatic_mode values (mirrors eAutoMode in mower_logic.cpp).
AUTO_MODE_PARAM = "/mower_logic/automatic_mode"
AUTO_MODE_MANUAL = 0
AUTO_MODE_SEMIAUTO = 1
AUTO_MODE_AUTO = 2
# dynamic_reconfigure service for mower_logic, used to flip automatic_mode for
# the continuous (24/7) mode. Setting the bare param is not enough — the value
# only takes effect through the reconfigure server.
MOWER_LOGIC_RECONFIGURE = "/mower_logic/set_parameters"

NODE_ID = "mower_scheduler"
RPC_METHODS = (
    "schedule.list",
    "schedule.upsert",
    "schedule.delete",
    "schedule.history",
    "exceptions.get",
    "exceptions.set",
    "exceptions.holidays",
    "mower.start_mowing",
)

# On-disk schema version. Bumped from 1 to 2 when per-appointment overrides,
# mowing modes, time windows and EXDATEs were added. Older files are migrated
# lazily on load (see ScheduleStore._migrate) and rewritten on the next save.
SCHEDULE_VERSION = 2

# Mowing-logic modes. Mirrors the `mode` enum in openrpc.json's Schedule.
MODE_TIME_AREA = "time_area"
MODE_TIME_WINDOW = "time_window"
MODE_CONTINUOUS = "continuous"
VALID_MODES = (MODE_TIME_AREA, MODE_TIME_WINDOW, MODE_CONTINUOUS)

# Coverage fill patterns selectable per appointment. Mirrors the
# `overrides.pattern` enum in openrpc.json. The integer values are the
# slic3r_coverage_planner PlanPath fill enum consumed by MowingBehavior; the
# scheduler forwards them via the next_run param block.
PATTERN_TO_FILL = {
    "linear": 0,             # FILL_LINEAR
    "concentric_lines": 1,   # FILL_CONCENTRIC (follows the area outline)
    "concentric_circle": 2,  # FILL_CONCENTRIC_CIRCLE (Archimedean spiral)
    "hilbert": 3,            # FILL_HILBERT
    "grid": 4,               # FILL_GRID (rectilinear, two perpendicular passes)
    "honeycomb": 5,          # FILL_HONEYCOMB
    "octagram": 6,           # FILL_OCTAGRAM (octagram spiral)
}
VALID_PATTERNS = tuple(PATTERN_TO_FILL.keys())

# Skip reason constants shared with the openrpc.json contract.
SKIP_NO_STATE = "no_state"
SKIP_EMERGENCY = "emergency"
SKIP_NOT_IDLE = "not_idle"
SKIP_CHARGING = "charging"
SKIP_RAIN = "rain"
SKIP_BLOCKED = "blocked"
SKIP_HOLIDAY = "holiday"
# A recurring block-window (time-of-day range, e.g. nightly 20:00-08:00) is
# currently active. Distinct from SKIP_BLOCKED (a full-day manual block).
SKIP_BLOCK_WINDOW = "block_window"

# Mowing lifecycle event types that close out a dispatched run, mapped to the
# (run_history status, reason) they record. Events are matched by the run_id in
# their details payload; MowingBehavior echoes the run_id we set in the next_run
# block into mowing.* events (including an aborted exit — the failure / stuck /
# stop-at-time path). docking.failed is emitted by DockingBehavior, which does
# not carry the run_id, so it is intentionally not in this map.
_RUN_OUTCOME_EVENTS = {
    "mowing.session_completed": ("completed", ""),
    "mowing.aborted": ("aborted", "aborted"),
}


# A schedule that is due to fire this tick. occurrence_utc is the rrule (or
# window-start) instant; stop_at_epoch is the UTC epoch the run must stop at
# (0 = no time limit).
Candidate = collections.namedtuple("Candidate", ["schedule", "occurrence_utc", "stop_at_epoch"])


class RpcException(Exception):
    """Raised inside an RPC handler to control the JSON-RPC error response."""

    def __init__(self, code: int, message: str):
        super().__init__(message)
        self.code = code
        self.message = message


def _utcnow() -> datetime.datetime:
    return datetime.datetime.now(datetime.timezone.utc)


# iCal two-letter weekday codes, used by time-window day masks.
WEEKDAY_CODES = ("MO", "TU", "WE", "TH", "FR", "SA", "SU")


def _pattern_to_fill(pattern: Optional[str]) -> Optional[int]:
    """Map an overrides.pattern string to the slic3r fill enum, or None."""
    if pattern is None:
        return None
    return PATTERN_TO_FILL.get(pattern)


def _parse_hhmm(value: str) -> Optional[tuple[int, int]]:
    """Parse an 'HH:MM' string into (hour, minute), or None if malformed."""
    parts = value.split(":")
    if len(parts) != 2:
        return None
    try:
        hour, minute = int(parts[0]), int(parts[1])
    except ValueError:
        return None
    if 0 <= hour <= 23 and 0 <= minute <= 59:
        return hour, minute
    return None


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
        version = data.get("version", 1)
        self._schedules = [self._migrate(s, version) for s in data.get("schedules", [])]

    @staticmethod
    def _migrate(schedule: dict, version: int) -> dict:
        """Bring a persisted schedule up to the v2 shape in place.

        v1 had neither `mode` nor an `overrides` block; the dead
        `pattern.angle_offset` is lifted into `overrides.angle_deg` so the
        value the user once entered is preserved once the consumer lands. The
        migrated dict is written back as version 2 on the next save; until then
        v1 files keep loading because every new field is optional.
        """
        if not isinstance(schedule, dict):
            return schedule
        if version >= SCHEDULE_VERSION:
            return schedule
        schedule.setdefault("mode", MODE_TIME_AREA)
        legacy = schedule.get("pattern")
        if isinstance(legacy, dict) and "angle_offset" in legacy:
            overrides = schedule.setdefault("overrides", {})
            overrides.setdefault("angle_deg", float(legacy["angle_offset"]))
        return schedule

    def _save_unlocked(self) -> None:
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        # Tempfile + os.replace gives us an atomic write; a crash during write
        # leaves the previous file untouched.
        fd, tmp = tempfile.mkstemp(prefix=".schedules.", dir=os.path.dirname(self.path))
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as f:
                json.dump({"version": SCHEDULE_VERSION, "schedules": self._schedules}, f, indent=2)
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
                    for key in ("_last_fired_iso", "_last_skip_reason", "_last_skip_at", "_last_run_id"):
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

    def snapshot(self) -> list[dict]:
        """Return a shallow copy of every schedule dict (thread-safe read)."""
        with self._lock:
            return [dict(s) for s in self._schedules]

    def evaluate_due(self, now_utc: datetime.datetime) -> list[Candidate]:
        """Return time_area schedules with an unfired rrule occurrence inside
        the catch-up window, as Candidates with a duration-based stop time.

        Pure read against the store — does not mutate or persist; the caller
        is expected to call `record_fire` / `record_skip` afterwards.
        """
        candidates: list[Candidate] = []
        window_start_utc = now_utc - CATCHUP_WINDOW
        with self._lock:
            for s in self._schedules:
                if not s.get("enabled", False):
                    continue
                # Only time_area uses discrete rrule firing. time_window has its
                # own evaluator; continuous drives automatic_mode instead.
                if s.get("mode", MODE_TIME_AREA) != MODE_TIME_AREA:
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
                if self._is_exdate(s, most_recent_utc):
                    continue
                duration = int(s.get("duration_minutes", 0) or 0)
                stop_at = (most_recent_utc + datetime.timedelta(minutes=duration)).timestamp() if duration else 0.0
                # Snapshot the dict so callers see a stable view even if the
                # store mutates underneath.
                candidates.append(Candidate(dict(s), most_recent_utc, stop_at))
        return candidates

    def evaluate_windows(self, now_utc: datetime.datetime) -> list[Candidate]:
        """Return time_window schedules whose daily window has just started and
        has not yet fired today, as Candidates that stop at the window end.

        A window fires once per active day: the moment `now` is inside
        [start, start+CATCHUP_WINDOW] on an enabled weekday and we have not
        already fired for that day's window-start instant.
        """
        candidates: list[Candidate] = []
        with self._lock:
            for s in self._schedules:
                if not s.get("enabled", False) or s.get("mode") != MODE_TIME_WINDOW:
                    continue
                window = s.get("window") or {}
                start_hhmm = _parse_hhmm(window.get("start", ""))
                end_hhmm = _parse_hhmm(window.get("end", ""))
                if not start_hhmm or not end_hhmm:
                    continue
                tz = self._zone_for(s)
                now_local = now_utc.astimezone(tz)
                days = window.get("days")
                if days and WEEKDAY_CODES[now_local.weekday()] not in days:
                    continue
                start_local = now_local.replace(
                    hour=start_hhmm[0], minute=start_hhmm[1], second=0, microsecond=0
                )
                # Only fire shortly after the window opens (within the catch-up
                # window), never before it and never long after.
                if now_local < start_local or now_local - start_local > CATCHUP_WINDOW:
                    continue
                start_utc = start_local.astimezone(datetime.timezone.utc)
                last_fired = _parse_iso_utc(s.get("_last_fired_iso"))
                if last_fired is not None and start_utc <= last_fired:
                    continue
                if self._is_exdate(s, start_utc):
                    continue
                end_local = now_local.replace(
                    hour=end_hhmm[0], minute=end_hhmm[1], second=0, microsecond=0
                )
                # An end <= start means the window crosses midnight; push end to
                # the next day so the stop time stays in the future.
                if end_local <= start_local:
                    end_local += datetime.timedelta(days=1)
                stop_at = end_local.astimezone(datetime.timezone.utc).timestamp()
                candidates.append(Candidate(dict(s), start_utc, stop_at))
        return candidates

    @staticmethod
    def _is_exdate(schedule: dict, occurrence_utc: datetime.datetime) -> bool:
        exdates = schedule.get("exdates")
        if not exdates:
            return False
        tz = ScheduleStore._zone_for(schedule)
        local_date = occurrence_utc.astimezone(tz).date().isoformat()
        return local_date in exdates

    def record_fire(
        self, schedule_id: str, occurrence_utc: datetime.datetime, run_id: Optional[str] = None
    ) -> None:
        iso = occurrence_utc.astimezone(datetime.timezone.utc).isoformat()
        with self._lock:
            for s in self._schedules:
                if s["id"] != schedule_id:
                    continue
                s["_last_fired_iso"] = iso
                if run_id is not None:
                    s["_last_run_id"] = run_id
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

        mode = schedule.get("mode", MODE_TIME_AREA)
        if mode not in VALID_MODES:
            raise RpcException(ERROR_INVALID_PARAMS, f"mode must be one of {VALID_MODES}")

        cleaned = {
            "id": sid,
            "name": name,
            "enabled": enabled,
            "mode": mode,
            "areas": list(areas),
            "rrule": rrule,
            "duration_minutes": int(duration_minutes),
            "timezone": tz_name,
        }
        exdates = ScheduleStore._validate_exdates(schedule.get("exdates"))
        if exdates:
            cleaned["exdates"] = exdates
        window = ScheduleStore._validate_window(schedule.get("window"))
        if window is not None:
            cleaned["window"] = window
        if "weather" in schedule and isinstance(schedule["weather"], dict):
            cleaned["weather"] = {"skip_if_rain": bool(schedule["weather"].get("skip_if_rain", False))}
        overrides = ScheduleStore._validate_overrides(schedule.get("overrides"))
        if overrides:
            cleaned["overrides"] = overrides
        # The legacy `pattern` block is deprecated in favour of `overrides`.
        # We migrate its one live field on load; drop it on re-save so the
        # stored shape converges on v2 rather than carrying both forever.
        return cleaned

    @staticmethod
    def _validate_exdates(value: Any) -> list[str]:
        if value is None:
            return []
        if not isinstance(value, list):
            raise RpcException(ERROR_INVALID_PARAMS, "exdates must be a list of ISO dates")
        out: list[str] = []
        for d in value:
            if not isinstance(d, str):
                raise RpcException(ERROR_INVALID_PARAMS, "exdates entries must be strings")
            try:
                # Accept full ISO timestamps too, but normalise to the date.
                parsed = datetime.date.fromisoformat(d[:10])
            except ValueError:
                raise RpcException(ERROR_INVALID_PARAMS, f"invalid exdate {d!r} (expected YYYY-MM-DD)")
            out.append(parsed.isoformat())
        return out

    @staticmethod
    def _validate_window(value: Any) -> Optional[dict]:
        if value is None:
            return None
        if not isinstance(value, dict):
            raise RpcException(ERROR_INVALID_PARAMS, "window must be an object")
        out: dict = {}
        for key in ("start", "end"):
            t = value.get(key)
            if t is None:
                continue
            if not isinstance(t, str) or not _parse_hhmm(t):
                raise RpcException(ERROR_INVALID_PARAMS, f"window.{key} must be 'HH:MM'")
            out[key] = t
        days = value.get("days")
        if days is not None:
            if not isinstance(days, list) or not all(d in WEEKDAY_CODES for d in days):
                raise RpcException(ERROR_INVALID_PARAMS, f"window.days entries must be in {WEEKDAY_CODES}")
            out["days"] = list(days)
        wait = value.get("area_wait_minutes")
        if wait is not None:
            if not isinstance(wait, (int, float)) or wait < 0:
                raise RpcException(ERROR_INVALID_PARAMS, "window.area_wait_minutes must be >= 0")
            out["area_wait_minutes"] = int(wait)
        return out

    @staticmethod
    def _validate_overrides(value: Any) -> dict:
        if value is None:
            return {}
        if not isinstance(value, dict):
            raise RpcException(ERROR_INVALID_PARAMS, "overrides must be an object")
        out: dict = {}
        speed = value.get("speed_mps")
        if speed is not None:
            if not isinstance(speed, (int, float)) or speed <= 0:
                raise RpcException(ERROR_INVALID_PARAMS, "overrides.speed_mps must be a positive number")
            out["speed_mps"] = float(speed)
        pattern = value.get("pattern")
        if pattern is not None:
            if pattern not in VALID_PATTERNS:
                raise RpcException(ERROR_INVALID_PARAMS, f"overrides.pattern must be one of {VALID_PATTERNS}")
            out["pattern"] = pattern
        angle = value.get("angle_deg")
        if angle is not None:
            if not isinstance(angle, (int, float)):
                raise RpcException(ERROR_INVALID_PARAMS, "overrides.angle_deg must be a number")
            out["angle_deg"] = float(angle)
        outline_count = value.get("outline_count")
        if outline_count is not None:
            if not isinstance(outline_count, int) or isinstance(outline_count, bool) or outline_count < 0:
                raise RpcException(ERROR_INVALID_PARAMS, "overrides.outline_count must be a non-negative integer")
            out["outline_count"] = outline_count
        return out


class RunHistoryStore:
    """Atomic JSON backing for scheduler run outcomes (FIFO-capped).

    A run is opened with status "started" when a schedule fires, then closed to
    "completed" / "aborted" / "failed" by the /events correlation in
    SchedulerNode._on_event, keyed by run_id. The persistence pattern mirrors
    mower_sessions_recorder/nodes/recorder.py (tempfile + os.replace, lock,
    newest-last with a hard cap) so a crash never corrupts the file.
    """

    MAX_ENTRIES = 500

    def __init__(self, path: str):
        self.path = path
        self._lock = threading.Lock()
        self._runs: list[dict] = []
        self._load()

    def _load(self) -> None:
        try:
            with open(self.path, "r", encoding="utf-8") as f:
                data = json.load(f)
        except FileNotFoundError:
            self._runs = []
            return
        except (OSError, ValueError) as e:
            rospy.logwarn("Failed to load %s, starting empty: %s", self.path, e)
            self._runs = []
            return
        self._runs = list(data.get("runs", []))

    def _save_unlocked(self) -> None:
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        fd, tmp = tempfile.mkstemp(prefix=".run_history.", dir=os.path.dirname(self.path))
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as f:
                json.dump({"version": 1, "runs": self._runs}, f, indent=2)
            os.replace(tmp, self.path)
        except Exception:
            try:
                os.unlink(tmp)
            except OSError:
                pass
            raise

    def open_run(
        self,
        run_id: str,
        schedule_id: str,
        name: str,
        occurrence_utc: datetime.datetime,
        areas: list,
        now_utc: datetime.datetime,
    ) -> None:
        entry = {
            "run_id": run_id,
            "schedule_id": schedule_id,
            "name": name,
            "occurrence_iso": occurrence_utc.astimezone(datetime.timezone.utc).isoformat(),
            "started_iso": now_utc.astimezone(datetime.timezone.utc).isoformat(),
            "area_indices": list(areas),
            "status": "started",
            "reason": "",
        }
        with self._lock:
            self._runs.append(entry)
            # FIFO-evict oldest beyond the cap so the file stays bounded.
            if len(self._runs) > self.MAX_ENTRIES:
                self._runs = self._runs[-self.MAX_ENTRIES :]
            self._save_unlocked()

    def close_run(
        self, run_id: str, status: str, now_utc: datetime.datetime, reason: str = "", extra: Optional[dict] = None
    ) -> bool:
        """Update an open run's outcome. Returns True if a matching run was found."""
        iso = now_utc.astimezone(datetime.timezone.utc).isoformat()
        with self._lock:
            # Search newest-first: a run_id is unique but the latest entry is the
            # live one if ids were ever reused.
            for entry in reversed(self._runs):
                if entry.get("run_id") != run_id:
                    continue
                # Don't downgrade a terminal status (e.g. a late session_completed
                # arriving after an abort already closed the run).
                if entry.get("status") in ("completed", "aborted", "failed"):
                    return True
                entry["status"] = status
                entry["reason"] = reason
                entry["ended_iso"] = iso
                if extra:
                    entry.update(extra)
                self._save_unlocked()
                return True
        return False

    def list_for(self, schedule_id: Optional[str], limit: int) -> list[dict]:
        with self._lock:
            runs = self._runs
            if schedule_id:
                runs = [r for r in runs if r.get("schedule_id") == schedule_id]
            # Newest first, bounded.
            return list(reversed(runs[-limit:])) if limit > 0 else list(reversed(runs))


class ExceptionsStore:
    """Atomic JSON backing for mowing exceptions: manual blocking days, the
    country/region whose public holidays should also block mowing, and
    recurring block-windows (time-of-day ranges that block mowing).

    Shape (version 1):
        {"version": 1, "country": "DE", "subdiv": "BY",
         "blocking_days": ["2026-06-15", ...],
         "timezone": "Europe/Vienna",
         "block_windows": [{"start": "20:00", "end": "08:00", "days": ["MO", ...]}]}

    Holidays are computed with the `holidays` package (offline, no network) and
    cached per year. When the package is unavailable the country setting is kept
    but holiday checks are skipped; manual blocking days always apply.

    Block-windows are evaluated in `timezone` (default UTC). A window with
    end <= start crosses midnight (e.g. 20:00-08:00). An empty `days` list means
    the window is active every day.
    """

    def __init__(self, path: str):
        self.path = path
        self._lock = threading.Lock()
        self._data: dict = {"country": "", "subdiv": "", "blocking_days": [], "timezone": "", "block_windows": []}
        # Per-(country, subdiv, year) cache of computed holiday sets.
        self._holiday_cache: dict = {}
        self._load()

    def _load(self) -> None:
        try:
            with open(self.path, "r", encoding="utf-8") as f:
                data = json.load(f)
        except FileNotFoundError:
            return
        except (OSError, ValueError) as e:
            rospy.logwarn("Failed to load %s, starting empty: %s", self.path, e)
            return
        self._data = {
            "country": str(data.get("country") or ""),
            "subdiv": str(data.get("subdiv") or ""),
            "blocking_days": [str(d) for d in (data.get("blocking_days") or [])],
            "timezone": str(data.get("timezone") or ""),
            "block_windows": [w for w in (data.get("block_windows") or []) if isinstance(w, dict)],
        }

    def _save_unlocked(self) -> None:
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        fd, tmp = tempfile.mkstemp(prefix=".exceptions.", dir=os.path.dirname(self.path))
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as f:
                json.dump({"version": 1, **self._data}, f, indent=2)
            os.replace(tmp, self.path)
        except Exception:
            try:
                os.unlink(tmp)
            except OSError:
                pass
            raise

    def get(self) -> dict:
        with self._lock:
            return dict(self._data)

    def set(self, value: Any) -> dict:
        if not isinstance(value, dict):
            raise RpcException(ERROR_INVALID_PARAMS, "exceptions must be an object")
        country = value.get("country", "")
        subdiv = value.get("subdiv", "")
        if not isinstance(country, str) or not isinstance(subdiv, str):
            raise RpcException(ERROR_INVALID_PARAMS, "country and subdiv must be strings")
        # Validate the country against the holidays package when present, so the
        # UI gets immediate feedback on a bad code instead of silent no-ops.
        if country and _holidays is not None:
            try:
                _holidays.country_holidays(country, subdiv=subdiv or None, years=_utcnow().year)
            except NotImplementedError:
                raise RpcException(ERROR_INVALID_PARAMS, f"unsupported country/subdiv {country!r}/{subdiv!r}")
        days = value.get("blocking_days", [])
        if not isinstance(days, list):
            raise RpcException(ERROR_INVALID_PARAMS, "blocking_days must be a list")
        clean_days = []
        for d in days:
            if not isinstance(d, str):
                raise RpcException(ERROR_INVALID_PARAMS, "blocking_days entries must be strings")
            try:
                clean_days.append(datetime.date.fromisoformat(d[:10]).isoformat())
            except ValueError:
                raise RpcException(ERROR_INVALID_PARAMS, f"invalid blocking day {d!r} (expected YYYY-MM-DD)")
        # Timezone the block windows are interpreted in. Empty is allowed and
        # falls back to UTC at evaluation time, mirroring schedule handling.
        timezone = value.get("timezone", "")
        if not isinstance(timezone, str):
            raise RpcException(ERROR_INVALID_PARAMS, "timezone must be a string")
        if timezone and _resolve_zone(timezone) is None:
            raise RpcException(ERROR_INVALID_PARAMS, f"invalid timezone {timezone!r}")
        block_windows = self._validate_block_windows(value.get("block_windows"))
        with self._lock:
            self._data = {
                "country": country,
                "subdiv": subdiv,
                "blocking_days": sorted(set(clean_days)),
                "timezone": timezone,
                "block_windows": block_windows,
            }
            self._holiday_cache.clear()
            self._save_unlocked()
            return dict(self._data)

    @staticmethod
    def _validate_block_windows(value: Any) -> list:
        if value is None:
            return []
        if not isinstance(value, list):
            raise RpcException(ERROR_INVALID_PARAMS, "block_windows must be a list")
        out: list = []
        for w in value:
            if not isinstance(w, dict):
                raise RpcException(ERROR_INVALID_PARAMS, "block_windows entries must be objects")
            start = w.get("start")
            end = w.get("end")
            if not isinstance(start, str) or not _parse_hhmm(start):
                raise RpcException(ERROR_INVALID_PARAMS, "block_windows[].start must be 'HH:MM'")
            if not isinstance(end, str) or not _parse_hhmm(end):
                raise RpcException(ERROR_INVALID_PARAMS, "block_windows[].end must be 'HH:MM'")
            clean = {"start": start, "end": end}
            wdays = w.get("days")
            if wdays is not None:
                if not isinstance(wdays, list) or not all(d in WEEKDAY_CODES for d in wdays):
                    raise RpcException(ERROR_INVALID_PARAMS, f"block_windows[].days entries must be in {WEEKDAY_CODES}")
                # Empty list == every day; only store a non-empty subset.
                if wdays:
                    clean["days"] = list(wdays)
            out.append(clean)
        return out

    def _zone(self) -> Any:
        """Resolve the configured block-window timezone, defaulting to UTC."""
        tz = _resolve_zone(self._data.get("timezone") or "UTC")
        return tz if tz is not None else datetime.timezone.utc

    def active_block_window(self, now_utc: datetime.datetime) -> bool:
        """True when `now` falls inside any configured recurring block window,
        evaluated in the exceptions timezone (handles midnight-crossing)."""
        with self._lock:
            windows = list(self._data.get("block_windows") or [])
            tz = self._zone()
        if not windows:
            return False
        now_local = now_utc.astimezone(tz)
        now_minutes = now_local.hour * 60 + now_local.minute
        today_code = WEEKDAY_CODES[now_local.weekday()]
        yesterday_code = WEEKDAY_CODES[(now_local.weekday() - 1) % 7]
        for w in windows:
            start = _parse_hhmm(w.get("start", ""))
            end = _parse_hhmm(w.get("end", ""))
            if not start or not end:
                continue
            start_min = start[0] * 60 + start[1]
            end_min = end[0] * 60 + end[1]
            days = w.get("days")
            if start_min < end_min:
                # Same-day window [start, end). The active weekday is today.
                if (not days or today_code in days) and start_min <= now_minutes < end_min:
                    return True
            else:
                # Crosses midnight: active in [start, 24:00) on the window's day
                # and [00:00, end) on the following day. The "day" the window is
                # anchored to is the day it starts on.
                if (not days or today_code in days) and now_minutes >= start_min:
                    return True
                if (not days or yesterday_code in days) and now_minutes < end_min:
                    return True
        return False

    def names_for_range(self, from_date: datetime.date, to_date: datetime.date) -> list:
        """Return [{date, name}] for every public holiday in [from, to].

        Uses the configured country/region and the `holidays` package. Returns
        an empty list when no country is set or the package is unavailable.
        """
        with self._lock:
            country = self._data.get("country")
            subdiv = self._data.get("subdiv")
        if not country or _holidays is None:
            return []
        if to_date < from_date:
            return []
        out: list = []
        cur = from_date
        while cur <= to_date:
            holiday_set = self._holiday_set(country, subdiv, cur.year)
            name = holiday_set.get(cur) if holiday_set else None
            if name:
                out.append({"date": cur.isoformat(), "name": str(name)})
            cur += datetime.timedelta(days=1)
        return out

    def _holiday_set(self, country: str, subdiv: str, year: int):
        """Return (and memoize) the holidays mapping for a (country, subdiv,
        year). Returns an empty dict when the lookup is unsupported."""
        key = (country, subdiv, year)
        with self._lock:
            holiday_set = self._holiday_cache.get(key)
            if holiday_set is not None:
                return holiday_set
        try:
            holiday_set = _holidays.country_holidays(country, subdiv=subdiv or None, years=year)
        except (NotImplementedError, KeyError) as e:
            rospy.logwarn_throttle(3600, "Holiday lookup failed for %s/%s: %s", country, subdiv, e)
            holiday_set = {}
        with self._lock:
            self._holiday_cache[key] = holiday_set
        return holiday_set

    def reason_for(self, local_date: datetime.date) -> Optional[str]:
        """Return SKIP_BLOCKED for a manual blocking day, SKIP_HOLIDAY for a
        public holiday in the configured country/region, or None."""
        iso = local_date.isoformat()
        with self._lock:
            if iso in self._data.get("blocking_days", []):
                return SKIP_BLOCKED
            country = self._data.get("country")
            subdiv = self._data.get("subdiv")
        if not country or _holidays is None:
            return None
        holiday_set = self._holiday_set(country, subdiv, local_date.year)
        if holiday_set and local_date in holiday_set:
            return SKIP_HOLIDAY
        return None


class SchedulerNode:
    def __init__(self) -> None:
        rospy.init_node(NODE_ID)
        event_publisher.init("mower_scheduler")
        path = rospy.get_param("~schedules_path", DEFAULT_PATH)
        self.store = ScheduleStore(path)
        rospy.loginfo("mower_scheduler loaded %d schedule(s) from %s", len(self.store.list_public()), path)
        history_path = rospy.get_param("~run_history_path", DEFAULT_HISTORY_PATH)
        self.history = RunHistoryStore(history_path)
        exceptions_path = rospy.get_param("~exceptions_path", DEFAULT_EXCEPTIONS_PATH)
        self.exceptions = ExceptionsStore(exceptions_path)
        if _holidays is None:
            rospy.logwarn(
                "python3-holidays not available; public-holiday blocking disabled "
                "(manual blocking days still work)"
            )

        self._state_lock = threading.Lock()
        self._state: Optional[RobotState] = None

        # Stop-at-time tracking. _active_run holds the run we dispatched and the
        # UTC epoch at which it must stop; _live_run_id is the run_id the robot
        # reports actively mowing (from mowing.started events). We only abort a
        # mow when both agree, so the scheduler never stops a manual run.
        self._run_lock = threading.Lock()
        self._active_run: Optional[dict] = None
        self._live_run_id: Optional[str] = None
        # Whether the scheduler currently owns automatic_mode for a continuous
        # (24/7) schedule, and the last value it pushed, so it only writes on
        # change and can hand control back when no continuous schedule remains.
        self._continuous_owned = False
        self._continuous_last_mode: Optional[int] = None

        self._action_pub = rospy.Publisher(ACTION_TOPIC, String, queue_size=10)
        self._rpc_response_pub = rospy.Publisher("/xbot/rpc/response", RpcResponse, queue_size=10)
        self._rpc_error_pub = rospy.Publisher("/xbot/rpc/error", RpcError, queue_size=10)

        rospy.Subscriber(ROBOT_STATE_TOPIC, RobotState, self._on_state, queue_size=1)
        rospy.Subscriber("/xbot/rpc/request", RpcRequest, self._on_rpc, queue_size=20)
        rospy.Subscriber(EVENTS_TOPIC, Event, self._on_event, queue_size=50)

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

    def _on_event(self, msg: Event) -> None:
        # Correlate mowing lifecycle events back to the run we dispatched. We
        # care about mowing.started (to learn the live run_id for stop-at-time)
        # and the outcome events (to close run history). Everything else is
        # ignored.
        if msg.type != "mowing.started" and msg.type not in _RUN_OUTCOME_EVENTS:
            return
        details: dict = {}
        if msg.details_json:
            try:
                parsed = json.loads(msg.details_json)
                if isinstance(parsed, dict):
                    details = parsed
            except ValueError:
                return
        run_id = details.get("run_id")
        if not run_id:
            return

        if msg.type == "mowing.started":
            # The robot confirms it is mowing this run; remember it so the tick
            # loop only aborts a run that is genuinely ours and active.
            with self._run_lock:
                self._live_run_id = run_id
            return

        status, reason = _RUN_OUTCOME_EVENTS[msg.type]
        # mowing.aborted to idle is a deliberate user stop, not a failure.
        if msg.type == "mowing.aborted" and details.get("to_idle"):
            status, reason = "aborted", "stopped_by_user"
        self.history.close_run(run_id, status, _utcnow(), reason=reason)
        # The run is over — drop our tracking so a later tick can't abort it and
        # the time_window cursor (advanced separately) is the only resume state.
        with self._run_lock:
            if self._live_run_id == run_id:
                self._live_run_id = None
            if self._active_run and self._active_run.get("run_id") == run_id:
                self._active_run = None

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
        if method == "schedule.history":
            # Both params are optional: no schedule_id => all schedules.
            schedule_id = None
            limit = 50
            if isinstance(params, dict):
                schedule_id = params.get("schedule_id")
                if "limit" in params and isinstance(params["limit"], (int, float)):
                    limit = int(params["limit"])
            if schedule_id is not None and not isinstance(schedule_id, str):
                raise RpcException(ERROR_INVALID_PARAMS, "schedule_id must be a string")
            return self.history.list_for(schedule_id, limit)
        if method == "exceptions.get":
            return self.exceptions.get()
        if method == "exceptions.set":
            value = self._extract_param(params, "exceptions")
            return self.exceptions.set(value)
        if method == "exceptions.holidays":
            return self._handle_holidays(params)
        if method == "mower.start_mowing":
            return self._handle_start_mowing(params)
        raise RpcException(ERROR_INTERNAL, f"Unhandled method: {method}")

    def _handle_holidays(self, params: Any) -> dict:
        if not isinstance(params, dict):
            raise RpcException(ERROR_INVALID_PARAMS, "expected by-name params {from, to}")
        from_raw = params.get("from")
        to_raw = params.get("to")
        try:
            from_date = datetime.date.fromisoformat(str(from_raw)[:10])
            to_date = datetime.date.fromisoformat(str(to_raw)[:10])
        except (ValueError, TypeError):
            raise RpcException(ERROR_INVALID_PARAMS, "from/to must be ISO 'YYYY-MM-DD' dates")
        return {"holidays": self.exceptions.names_for_range(from_date, to_date)}

    def _handle_start_mowing(self, params: Any) -> dict:
        if not isinstance(params, dict):
            raise RpcException(ERROR_INVALID_PARAMS, "expected by-name params {areas, overrides?, duration_minutes?}")
        areas = params.get("areas")
        if not isinstance(areas, list) or not all(isinstance(a, int) and not isinstance(a, bool) for a in areas):
            raise RpcException(ERROR_INVALID_PARAMS, "areas must be a list of integers")
        overrides = ScheduleStore._validate_overrides(params.get("overrides"))
        duration = params.get("duration_minutes", 0)
        if duration is None:
            duration = 0
        if not isinstance(duration, (int, float)) or isinstance(duration, bool) or duration < 0:
            raise RpcException(ERROR_INVALID_PARAMS, "duration_minutes must be a non-negative number")

        now_utc = _utcnow()
        with self._state_lock:
            state = self._state
        # Gate the same way scheduled runs are gated: the mower must be idle and
        # safe, and no block window may be active.
        global_reason = self._global_block_reason(state)
        if global_reason:
            raise RpcException(ERROR_INVALID_PARAMS, f"cannot start mowing — {global_reason}")
        if self.exceptions.active_block_window(now_utc):
            raise RpcException(ERROR_INVALID_PARAMS, "cannot start mowing — a block window is currently active")

        stop_at = (now_utc + datetime.timedelta(minutes=int(duration))).timestamp() if duration else 0.0
        run_id = self._dispatch_run(
            schedule_id="manual",
            name="Manual run",
            areas=areas,
            overrides=overrides,
            occurrence_utc=now_utc,
            now_utc=now_utc,
            stop_at_epoch=stop_at,
            mark_schedule_fired=False,
        )
        return {"run_id": run_id}

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

        with self._state_lock:
            state = self._state

        # 1. Enforce stop-at-time first: if a run we dispatched has passed its
        #    deadline and is still mowing, send it home. This runs every tick
        #    regardless of due candidates.
        self._enforce_stop_at_time(now_utc, state)

        # 2. Enforce block windows (hard stop): if a recurring block window is
        #    active and the robot is mowing, send it home. Runs every tick.
        block_window_active = self.exceptions.active_block_window(now_utc)
        self._enforce_block_windows(block_window_active, state)

        # 3. Continuous (24/7) schedules drive automatic_mode rather than firing
        #    discrete runs; reconcile that ownership each tick.
        self._reconcile_continuous_mode(now_utc, state, block_window_active)

        # 4. Collect discrete due runs: time_area (rrule occurrences) and
        #    time_window (window-start crossings). Both share the firing path.
        candidates = self.store.evaluate_due(now_utc) + self.store.evaluate_windows(now_utc)
        if not candidates:
            return

        # Determine the global block reason once — it applies uniformly to
        # every due candidate, so we annotate them all with the same skip. An
        # active block window gates every candidate the same way.
        global_reason = self._global_block_reason(state)
        if global_reason is None and block_window_active:
            global_reason = SKIP_BLOCK_WINDOW
        if global_reason:
            rospy.loginfo("Skipping %d due schedule(s) — %s", len(candidates), global_reason)
            for cand in candidates:
                self.store.record_skip(cand.schedule["id"], global_reason, now_utc)
                self._emit_skip(cand.schedule, global_reason)
            return

        for cand in candidates:
            schedule = cand.schedule
            # Blocking days / public holidays take precedence over a normal run.
            blocked = self._blocked_reason(cand.occurrence_utc, schedule)
            if blocked:
                rospy.loginfo("Skipping schedule %s — %s", schedule["id"], blocked)
                self.store.record_skip(schedule["id"], blocked, now_utc)
                self._emit_skip(schedule, blocked)
                continue
            if schedule.get("weather", {}).get("skip_if_rain") and getattr(state, "rain_detected", False):
                rospy.loginfo("Skipping schedule %s — rain detected", schedule["id"])
                self.store.record_skip(schedule["id"], SKIP_RAIN, now_utc)
                self._emit_skip(schedule, SKIP_RAIN)
                continue
            self._fire(schedule, cand.occurrence_utc, now_utc, stop_at_epoch=cand.stop_at_epoch)

    def _enforce_block_windows(self, block_window_active: bool, state: Optional[RobotState]) -> None:
        """When a recurring block window is active and the robot is mowing, send
        it home (hard stop). Mirrors _enforce_stop_at_time but applies to any
        active mow — scheduled or manual — because a block window is a global
        "do not mow now" rule. Idempotent: ABORT to dock is harmless to repeat,
        but we only publish while the robot is still mowing."""
        if not block_window_active:
            return
        if state is None or state.current_state in ("IDLE", "DOCKING"):
            return
        rospy.loginfo_throttle(60, "Block window active while mowing — sending home")
        msg = String()
        msg.data = ABORT_ACTION_ID
        self._action_pub.publish(msg)

    def _enforce_stop_at_time(self, now_utc: datetime.datetime, state: Optional[RobotState]) -> None:
        with self._run_lock:
            run = dict(self._active_run) if self._active_run else None
            live_run_id = self._live_run_id
        if not run:
            return
        stop_at = run.get("stop_at_epoch", 0.0)
        if not stop_at or now_utc.timestamp() < stop_at:
            return
        # Only abort if the robot is actively mowing the very run we dispatched.
        # This guarantees we never stop a manual mow or a stale run.
        if live_run_id != run["run_id"]:
            return
        if state is None or state.current_state == "IDLE":
            return
        rospy.loginfo("Stop-at-time reached for run %s — sending home", run["run_id"])
        msg = String()
        msg.data = ABORT_ACTION_ID
        self._action_pub.publish(msg)
        # Clear the deadline so we don't re-publish abort every tick while the
        # robot drives to the dock; _on_event finalises the run on its event.
        with self._run_lock:
            if self._active_run and self._active_run.get("run_id") == run["run_id"]:
                self._active_run["stop_at_epoch"] = 0.0

    def _reconcile_continuous_mode(
        self, now_utc: datetime.datetime, state: Optional[RobotState], block_window_active: bool
    ) -> None:
        """Drive automatic_mode for continuous (24/7) schedules.

        A single enabled continuous schedule, when not gated by a blocking day,
        holiday, rain, an active block window or an off-day, holds automatic_mode
        at AUTO so IdleBehavior keeps re-starting mowing on its own. When gated,
        or when no continuous schedule exists, the scheduler hands automatic_mode
        back (to SEMIAUTO when gated so an in-progress task can still
        finish-and-dock without auto-restarting, or releases ownership entirely
        otherwise).
        """
        continuous = [
            s for s in self.store.snapshot()
            if s.get("enabled") and s.get("mode") == MODE_CONTINUOUS
        ]
        if not continuous:
            # No continuous schedule: release ownership once so manual changes
            # to automatic_mode are respected from here on.
            if self._continuous_owned:
                rospy.loginfo("No continuous schedule remains; releasing automatic_mode ownership")
                self._continuous_owned = False
                self._continuous_last_mode = None
            return

        # An active block window gates every continuous schedule globally — no
        # auto-restart until it ends, regardless of per-schedule settings.
        if block_window_active:
            self._continuous_owned = True
            if self._continuous_last_mode != AUTO_MODE_SEMIAUTO and self._set_automatic_mode(AUTO_MODE_SEMIAUTO):
                self._continuous_last_mode = AUTO_MODE_SEMIAUTO
                rospy.loginfo("Continuous mode set automatic_mode=%d (block window active)", AUTO_MODE_SEMIAUTO)
            return

        # A continuous schedule wants to mow unless it is individually gated by
        # rain (when it opts in), a blocking day or a holiday. If ANY continuous
        # schedule is currently ungated, mowing should proceed (AUTO); we only
        # fall back to SEMIAUTO when every continuous schedule is gated.
        raining = state is not None and getattr(state, "rain_detected", False)
        gated_reason = None
        all_gated = True
        for s in continuous:
            reason = None
            if raining and s.get("weather", {}).get("skip_if_rain"):
                reason = SKIP_RAIN
            if reason is None:
                reason = self._blocked_reason(now_utc, s)
            if reason is None:
                all_gated = False
                break
            gated_reason = reason  # remember a representative reason for logging

        desired = AUTO_MODE_SEMIAUTO if all_gated else AUTO_MODE_AUTO
        self._continuous_owned = True
        if self._continuous_last_mode == desired:
            return
        if self._set_automatic_mode(desired):
            self._continuous_last_mode = desired
            rospy.loginfo(
                "Continuous mode set automatic_mode=%d%s",
                desired,
                f" (all gated: {gated_reason})" if all_gated and gated_reason else "",
            )

    def _set_automatic_mode(self, mode: int) -> bool:
        """Push automatic_mode into mower_logic via dynamic_reconfigure."""
        try:
            rospy.wait_for_service(MOWER_LOGIC_RECONFIGURE, timeout=2.0)
            proxy = rospy.ServiceProxy(MOWER_LOGIC_RECONFIGURE, Reconfigure)
            req = ReconfigureRequest()
            req.config = ReconfigureConfig(ints=[IntParameter(name="automatic_mode", value=int(mode))])
            proxy(req)
            return True
        except (rospy.ROSException, rospy.ServiceException) as e:
            rospy.logwarn_throttle(300, "Failed to set automatic_mode=%d: %s", mode, e)
            return False

    def _blocked_reason(self, occurrence_utc: datetime.datetime, schedule: dict) -> Optional[str]:
        """Return SKIP_BLOCKED / SKIP_HOLIDAY if the occurrence falls on a
        manual blocking day or a public holiday, evaluated in the schedule's
        own timezone, else None."""
        tz = ScheduleStore._zone_for(schedule)
        local_date = occurrence_utc.astimezone(tz).date()
        return self.exceptions.reason_for(local_date)

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

    def _fire(
        self,
        schedule: dict,
        occurrence_utc: datetime.datetime,
        now_utc: datetime.datetime,
        stop_at_epoch: float = 0.0,
    ) -> None:
        """Dispatch a due schedule occurrence and record the firing against the
        schedule (for rrule dedup)."""
        self._dispatch_run(
            schedule_id=schedule["id"],
            name=schedule["name"],
            areas=schedule.get("areas"),
            overrides=schedule.get("overrides"),
            occurrence_utc=occurrence_utc,
            now_utc=now_utc,
            stop_at_epoch=stop_at_epoch,
            mark_schedule_fired=True,
        )

    def _dispatch_run(
        self,
        *,
        schedule_id: str,
        name: str,
        areas: Optional[list],
        overrides: Optional[dict],
        occurrence_utc: datetime.datetime,
        now_utc: datetime.datetime,
        stop_at_epoch: float,
        mark_schedule_fired: bool,
    ) -> str:
        """Hand a full run spec to MowingBehavior via the next_run param block
        and open a run-history entry. Shared by scheduled firings (_fire) and
        manual starts (mower.start_mowing). Returns the run_id.

        An empty area list means "mow every active area" (24/7 / whole-map).
        The `pending` flag is the unambiguous "fresh run" signal: MowingBehavior
        latches the whole block and clears pending on entry, so a resume after a
        charge cycle keeps the in-progress run. The legacy single next_area_index
        stays untouched for map.start_in_area.
        """
        areas = [int(a) for a in (areas or [])]
        overrides = overrides or {}
        fill = _pattern_to_fill(overrides.get("pattern"))
        speed = overrides.get("speed_mps")
        angle = overrides.get("angle_deg")
        outline_count = overrides.get("outline_count")
        # Correlation id for this run, echoed by MowingBehavior into its
        # lifecycle events so _on_event can tie a completion/abort back to this
        # run and update the run-history entry we open below.
        run_id = secrets.token_hex(8)
        rospy.set_param(NEXT_RUN_PARAM + "/area_indices", areas)
        rospy.set_param(NEXT_RUN_PARAM + "/pattern", int(fill) if fill is not None else -1)
        rospy.set_param(NEXT_RUN_PARAM + "/speed_mps", float(speed) if speed is not None else float("nan"))
        rospy.set_param(NEXT_RUN_PARAM + "/angle_deg", float(angle) if angle is not None else float("nan"))
        rospy.set_param(NEXT_RUN_PARAM + "/outline_count", int(outline_count) if outline_count is not None else -1)
        rospy.set_param(NEXT_RUN_PARAM + "/stop_at_epoch", float(stop_at_epoch))
        rospy.set_param(NEXT_RUN_PARAM + "/run_id", run_id)
        rospy.set_param(NEXT_RUN_PARAM + "/pending", True)
        if areas:
            rospy.loginfo("Firing run %s (%s) over area(s) %s", schedule_id, name, areas)
        else:
            rospy.loginfo("Firing run %s (%s) over all active areas", schedule_id, name)
        # Persist before publishing so a crash between the two does not lead
        # to the action firing again on the next tick: the dedup check in
        # evaluate_due() relies on _last_fired_iso being on disk.
        if mark_schedule_fired:
            self.store.record_fire(schedule_id, occurrence_utc, run_id)
        self.history.open_run(run_id, schedule_id, name, occurrence_utc, areas, now_utc)
        # Track the run for stop-at-time enforcement. stop_at_epoch == 0 means
        # "no time limit" (e.g. a time_area schedule with mowing left to run to
        # natural completion is still bounded by duration_minutes; 0 disables).
        with self._run_lock:
            self._active_run = {
                "run_id": run_id,
                "schedule_id": schedule_id,
                "stop_at_epoch": float(stop_at_epoch),
            }
        msg = String()
        msg.data = START_ACTION_ID
        self._action_pub.publish(msg)
        event_publisher.info(
            "schedule.run_started",
            f"Run '{name}' fired",
            {"schedule_id": schedule_id, "name": name, "area_indices": areas, "run_id": run_id},
        )
        return run_id

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
