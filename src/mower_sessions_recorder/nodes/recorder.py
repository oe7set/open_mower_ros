#!/usr/bin/env python3
"""mower_sessions_recorder — records each MOWING run as a session.

Subscribes to mower_logic/current_state for state transitions and to
/xbot_positioning/xb_pose for distance accumulation. On every transition
into MOWING/PAUSED a fresh session is opened; on the first transition out
of those states the session is closed and the full list is published on
xbot_monitoring/mowing_sessions (JSON-serialised), which xbot_monitoring
rebroadcasts to MQTT mowing_sessions/json. The list is also persisted to
disk so it survives container restarts.

Persistent file layout (default at $ROS_HOME/openmower/mowing_sessions.json):

  {
    "version": 1,
    "sessions": [
      { "id": "abc123",
        "start_ts": 1779120000.5,
        "end_ts":   1779123600.1,
        "area_id":  "0",
        "distance_m": 412.7,
        "duration_s": 3599.6 }
    ]
  }
"""

from __future__ import annotations

import json
import math
import os
import secrets
import tempfile
import threading
from typing import Optional

import rospy
from mower_msgs.msg import HighLevelStatus
from std_msgs.msg import String
from xbot_msgs.msg import AbsolutePose

DEFAULT_PATH = os.path.expanduser(
    os.environ.get("ROS_HOME", "~/.ros") + "/openmower/mowing_sessions.json"
)
STATE_TOPIC = "mower_logic/current_state"
POSE_TOPIC = "/xbot_positioning/xb_pose"
PUBLISH_TOPIC = "xbot_monitoring/mowing_sessions"

# States that indicate an active mowing run. Anything else closes the session.
ACTIVE_STATES = {"MOWING", "PAUSED"}

# Cap on retained sessions; older ones get FIFO-evicted on insert.
MAX_SESSIONS = 200


class SessionsStore:
    """Atomic JSON file backing for the closed-sessions list.

    Mirrors the pattern from mower_scheduler/nodes/scheduler.py:ScheduleStore
    so the persistence guarantees are identical (tempfile + os.replace, top-N
    cap, lock around mutations).
    """

    def __init__(self, path: str):
        self.path = path
        self._lock = threading.Lock()
        self._sessions: list[dict] = []
        self._load()

    def _load(self) -> None:
        try:
            with open(self.path, "r", encoding="utf-8") as f:
                data = json.load(f)
        except FileNotFoundError:
            self._sessions = []
            return
        except (OSError, ValueError) as e:
            rospy.logwarn("Failed to load %s, starting empty: %s", self.path, e)
            self._sessions = []
            return
        self._sessions = list(data.get("sessions", []))

    def _save_unlocked(self) -> None:
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        fd, tmp = tempfile.mkstemp(prefix=".sessions.", dir=os.path.dirname(self.path))
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as f:
                json.dump({"version": 1, "sessions": self._sessions}, f, indent=2)
            os.replace(tmp, self.path)
        except Exception:
            try:
                os.unlink(tmp)
            except OSError:
                pass
            raise

    def append(self, session: dict) -> None:
        with self._lock:
            self._sessions.append(session)
            if len(self._sessions) > MAX_SESSIONS:
                self._sessions = self._sessions[-MAX_SESSIONS:]
            self._save_unlocked()

    def snapshot(self) -> list[dict]:
        with self._lock:
            return list(self._sessions)


class SessionsRecorder:
    def __init__(self, store: SessionsStore, publisher: rospy.Publisher):
        self.store = store
        self.publisher = publisher
        # Live session state — None when not mowing.
        self._current: Optional[dict] = None
        self._last_pose: Optional[tuple[float, float]] = None
        self._last_state: Optional[str] = None
        self._lock = threading.Lock()

    # ----- public hooks ----------------------------------------------------

    def on_state(self, msg: HighLevelStatus) -> None:
        state = msg.state_name or ""
        with self._lock:
            entered_active = state in ACTIVE_STATES and self._last_state not in ACTIVE_STATES
            left_active = state not in ACTIVE_STATES and self._last_state in ACTIVE_STATES
            self._last_state = state

            if entered_active:
                self._open_session(area=int(msg.current_area))
            elif left_active and self._current is not None:
                self._close_session()
                # Reset pose tracking so the next session starts fresh.
                self._last_pose = None

    def on_pose(self, msg: AbsolutePose) -> None:
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        with self._lock:
            if self._current is None:
                # Not mowing — keep the last pose so the first MOWING tick has
                # a delta baseline (though we reset _last_pose on session close
                # to avoid teleport-distance from docking moves).
                self._last_pose = (x, y)
                return
            if self._last_pose is not None:
                dx = x - self._last_pose[0]
                dy = y - self._last_pose[1]
                step = math.hypot(dx, dy)
                # Sanity guard: discrete pose jumps (RTK lock acquired,
                # frame switch, etc.) shouldn't poison the distance integral.
                if step < 5.0:
                    self._current["distance_m"] = float(self._current.get("distance_m", 0.0)) + step
            self._last_pose = (x, y)

    def publish_initial(self) -> None:
        # Publish whatever we have on disk so subscribers see the historic
        # sessions immediately at boot (xbot_monitoring rebroadcasts retained).
        self._publish_snapshot()

    # ----- internals -------------------------------------------------------

    def _open_session(self, area: int) -> None:
        now = rospy.Time.now().to_sec()
        self._current = {
            "id": secrets.token_hex(8),
            "start_ts": now,
            "area_id": str(area) if area >= 0 else None,
            "distance_m": 0.0,
        }
        rospy.loginfo("mower_sessions: opened session %s (area=%s)",
                      self._current["id"], self._current["area_id"])

    def _close_session(self) -> None:
        if self._current is None:
            return
        now = rospy.Time.now().to_sec()
        self._current["end_ts"] = now
        self._current["duration_s"] = max(0.0, now - float(self._current["start_ts"]))
        # Round for friendlier display; the App's schema is forgiving with floats.
        self._current["distance_m"] = round(float(self._current.get("distance_m", 0.0)), 2)
        self._current["duration_s"] = round(float(self._current["duration_s"]), 1)
        # Drop None area_id so the JSON object stays minimal.
        if self._current.get("area_id") is None:
            self._current.pop("area_id", None)

        finished = self._current
        self._current = None
        self.store.append(finished)
        rospy.loginfo("mower_sessions: closed session %s duration=%.1fs distance=%.2fm",
                      finished["id"], finished["duration_s"], finished["distance_m"])
        self._publish_snapshot()

    def _publish_snapshot(self) -> None:
        sessions = self.store.snapshot()
        try:
            payload = json.dumps(sessions)
        except (TypeError, ValueError) as e:
            rospy.logerr("mower_sessions: failed to serialise sessions: %s", e)
            return
        msg = String()
        msg.data = payload
        self.publisher.publish(msg)


def main() -> None:
    rospy.init_node("mower_sessions_recorder")
    sessions_path = rospy.get_param("~sessions_path", DEFAULT_PATH)
    store = SessionsStore(sessions_path)
    # latch so xbot_monitoring sees the snapshot even if it subscribes late.
    publisher = rospy.Publisher(PUBLISH_TOPIC, String, queue_size=1, latch=True)
    recorder = SessionsRecorder(store, publisher)

    rospy.Subscriber(STATE_TOPIC, HighLevelStatus, recorder.on_state, queue_size=10)
    rospy.Subscriber(POSE_TOPIC, AbsolutePose, recorder.on_pose, queue_size=20)

    # Allow the publisher latch to settle before we publish the boot snapshot;
    # otherwise late subscribers can miss the first message in some setups.
    rospy.sleep(0.5)
    recorder.publish_initial()

    rospy.loginfo("mower_sessions_recorder online, persisting to %s", sessions_path)
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
