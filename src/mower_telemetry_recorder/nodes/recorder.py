#!/usr/bin/env python3
"""mower_telemetry_recorder — high-rate sample log per mowing session.

While mower_logic is in MOWING / PAUSED the node writes a 4 Hz JSONL stream
of pose + GPS + WLAN + IMU + selected sensor values to a per-session file
under telemetry_path. An index.json keeps O(1) listings and is the basis for
the telemetry.list_sessions / telemetry.get_session RPC methods this node
also serves (RPC pattern is the same as mower_scheduler/scheduler.py).

Persistent layout under telemetry_path (default /data/ros/openmower/telemetry):

  index.json
    {"version": 1,
     "sessions": [
       {"id": "abc123def4567890",
        "start_ts": 1779120000.5,
        "end_ts":   1779123600.1,
        "duration_s": 3599.6,
        "sample_count": 14398,
        "file_size_bytes": 4187232}
     ]}

  <id>.jsonl
    {"ts": 1779120000.245, "x": 12.34, "y": 5.67, ...}
    {"ts": 1779120000.495, ...}
    ...

The frontend correlates a telemetry session with a mowing-session entry
(from mower_sessions_recorder) by overlapping start_ts. Both recorders
generate IDs independently so neither blocks the other.
"""

from __future__ import annotations

import json
import math
import os
import re
import secrets
import tempfile
import threading
import time
from typing import Any, Optional

import rospy
from mower_msgs.msg import HighLevelStatus
from sensor_msgs.msg import Imu
from xbot_msgs.msg import RobotState, SensorDataDouble
from xbot_rpc.msg import RpcError, RpcRequest, RpcResponse
from xbot_rpc.srv import RegisterMethodsSrv, RegisterMethodsSrvRequest

# Default mirrors the OpenMowerOS volume mount; container fallback is the
# same. recorder.launch can override.
DEFAULT_PATH = os.environ.get("TELEMETRY_PATH", "/data/ros/openmower/telemetry")

STATE_TOPIC = "xbot_monitoring/robot_state"
HIGH_LEVEL_TOPIC = "mower_logic/current_state"
IMU_TOPIC = "imu"
SENSOR_DATA_TEMPLATE = "xbot_monitoring/sensors/{sid}/data"

NODE_ID = "mower_telemetry_recorder"
RPC_METHODS = ("telemetry.list_sessions", "telemetry.get_session")

# Active mowing-state names that gate sample recording.
ACTIVE_STATES = {"MOWING", "PAUSED"}

# 4 Hz sampling — matches the App's heatmap zoom budget at sane file sizes
# (a 30 min mow ≈ 7200 samples ≈ 2 MB JSONL).
SAMPLE_HZ = 4.0
SAMPLE_PERIOD_S = 1.0 / SAMPLE_HZ

# Session-IDs are 16-char lowercase hex (secrets.token_hex(8)). The RPC
# guards every id parameter with this pattern to keep get_session paths
# inside telemetry_path.
SESSION_ID_RE = re.compile(r"^[a-f0-9]{16}$")

# Cap retained sessions on disk; older ones get FIFO-evicted along with
# their JSONL file.
MAX_SESSIONS = 200

# Hard cap on samples returned per get_session call. Prevents accidental
# "send me a 10 MB blob over MQTT" requests.
MAX_SAMPLES_PER_RESPONSE = 50_000

# JSON-RPC error codes — same constants as scheduler.py.
ERROR_INVALID_PARAMS = -32602
ERROR_INTERNAL = -32603


class RpcException(Exception):
    """Raised inside an RPC handler to control the JSON-RPC error response."""

    def __init__(self, code: int, message: str):
        super().__init__(message)
        self.code = code
        self.message = message


class TelemetryStore:
    """Atomic JSON-index + per-session JSONL files.

    Index lives at <telemetry_path>/index.json; each closed session has a
    sibling <id>.jsonl. Open sessions are not in the index until close
    completes — a crash mid-recording leaves a half-written JSONL behind
    that the next boot will quietly delete during the index reconcile.
    """

    def __init__(self, root: str):
        self.root = root
        self._lock = threading.Lock()
        self._sessions: list[dict] = []
        self._load()

    def _index_path(self) -> str:
        return os.path.join(self.root, "index.json")

    def _session_path(self, sid: str) -> str:
        return os.path.join(self.root, sid + ".jsonl")

    def _load(self) -> None:
        os.makedirs(self.root, exist_ok=True)
        try:
            with open(self._index_path(), "r", encoding="utf-8") as f:
                data = json.load(f)
            self._sessions = list(data.get("sessions", []))
        except FileNotFoundError:
            self._sessions = []
        except (OSError, ValueError) as e:
            rospy.logwarn("Failed to load telemetry index, starting empty: %s", e)
            self._sessions = []
        # Reconcile: prune index entries whose JSONL is gone.
        existing = []
        for s in self._sessions:
            sid = s.get("id")
            if sid and os.path.exists(self._session_path(sid)):
                existing.append(s)
        self._sessions = existing

    def _save_unlocked(self) -> None:
        os.makedirs(self.root, exist_ok=True)
        fd, tmp = tempfile.mkstemp(prefix=".telemetry-index.", dir=self.root)
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as f:
                json.dump({"version": 1, "sessions": self._sessions}, f, indent=2)
            os.replace(tmp, self._index_path())
        except Exception:
            try:
                os.unlink(tmp)
            except OSError:
                pass
            raise

    def add_session(self, entry: dict) -> None:
        with self._lock:
            self._sessions.append(entry)
            # FIFO eviction including the on-disk file.
            while len(self._sessions) > MAX_SESSIONS:
                evicted = self._sessions.pop(0)
                evicted_path = self._session_path(evicted.get("id", ""))
                try:
                    os.unlink(evicted_path)
                except OSError:
                    pass
            self._save_unlocked()

    def list_sessions(self) -> list[dict]:
        with self._lock:
            return list(self._sessions)

    def session_path_if_exists(self, sid: str) -> Optional[str]:
        path = self._session_path(sid)
        # Defence-in-depth on top of the regex check at the RPC boundary —
        # ensures the resolved path is still inside `root` even if someone
        # somehow snuck a traversal-friendly id through.
        real = os.path.realpath(path)
        if not real.startswith(os.path.realpath(self.root) + os.sep):
            return None
        return path if os.path.exists(real) else None


class TelemetryRecorder:
    """Tracks state, accumulates samples, persists on session close.

    Thread-safety: rospy callbacks run on its internal thread pool; we
    serialise around `_lock` for the hot fields and use the store's own
    lock for index mutations.
    """

    def __init__(self, store: TelemetryStore, sensor_ids: list[str]):
        self.store = store
        self.sensor_ids = sensor_ids
        self._lock = threading.Lock()
        self._last_state: Optional[str] = None

        # Active session state. None when not recording.
        self._session_id: Optional[str] = None
        self._session_start_ts: Optional[float] = None
        self._session_file = None  # type: Optional[Any]
        self._sample_count: int = 0

        # Latest values cached from subscribers; folded into each sample tick.
        self._robot_state: Optional[RobotState] = None
        self._imu_yaw: float = 0.0
        self._imu_pitch: float = 0.0
        self._imu_roll: float = 0.0
        self._sensor_values: dict[str, float] = {}

    # ----- callbacks -------------------------------------------------------

    def on_high_level(self, msg: HighLevelStatus) -> None:
        state_name = msg.state_name or ""
        with self._lock:
            entered_active = state_name in ACTIVE_STATES and self._last_state not in ACTIVE_STATES
            left_active = state_name not in ACTIVE_STATES and self._last_state in ACTIVE_STATES
            self._last_state = state_name

            if entered_active and self._session_id is None:
                self._open_session()
            elif left_active and self._session_id is not None:
                self._close_session()

    def on_robot_state(self, msg: RobotState) -> None:
        with self._lock:
            self._robot_state = msg

    def on_imu(self, msg: Imu) -> None:
        # Quaternion (w, x, y, z) → yaw/pitch/roll (rad). Standard ZYX
        # convention; the App treats them as relative magnitudes for the
        # IMU-erschütterung heatmap so absolute frame doesn't matter much.
        q = msg.orientation
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        with self._lock:
            self._imu_roll = roll
            self._imu_pitch = pitch
            self._imu_yaw = yaw

    def on_sensor_data(self, sensor_id: str, msg: SensorDataDouble) -> None:
        with self._lock:
            self._sensor_values[sensor_id] = float(msg.data)

    # ----- tick / IO -------------------------------------------------------

    def tick(self) -> None:
        """Called from the main loop at SAMPLE_HZ; writes one JSONL line."""
        with self._lock:
            if self._session_id is None or self._session_file is None:
                return
            sample = self._build_sample_unlocked()
            try:
                self._session_file.write(json.dumps(sample, separators=(",", ":")))
                self._session_file.write("\n")
                self._sample_count += 1
            except OSError as e:
                rospy.logerr("telemetry write failed: %s", e)

    def _build_sample_unlocked(self) -> dict:
        rs = self._robot_state
        sample: dict[str, Any] = {"ts": time.time()}
        if rs is not None:
            sample["x"] = float(rs.robot_pose.pose.pose.position.x)
            sample["y"] = float(rs.robot_pose.pose.pose.position.y)
            sample["gps_fix_type"] = int(rs.gps_fix_type)
            sample["gps_satellite_count"] = int(rs.gps_satellite_count)
            sample["gps_pdop"] = float(rs.gps_pdop)
            sample["wifi_dbm"] = int(rs.wifi_signal_dbm)
            # Send the same field name the openmower-app heatmap expects.
            sample["wifi_q"] = float(rs.wifi_link_quality)
        sample["yaw"] = self._imu_yaw
        sample["pitch"] = self._imu_pitch
        sample["roll"] = self._imu_roll
        for sid, val in self._sensor_values.items():
            sample[sid] = val
        return sample

    def _open_session(self) -> None:
        sid = secrets.token_hex(8)
        path = os.path.join(self.store.root, sid + ".jsonl")
        try:
            os.makedirs(self.store.root, exist_ok=True)
            self._session_file = open(path, "w", encoding="utf-8")
        except OSError as e:
            rospy.logerr("telemetry: cannot open %s: %s", path, e)
            self._session_file = None
            return
        self._session_id = sid
        self._session_start_ts = time.time()
        self._sample_count = 0
        rospy.loginfo("telemetry: opened session %s at %s", sid, path)

    def _close_session(self) -> None:
        sid = self._session_id
        start_ts = self._session_start_ts
        f = self._session_file
        sample_count = self._sample_count
        # Reset live state before doing IO so a callback that lands during
        # close doesn't try to write to a stale handle.
        self._session_id = None
        self._session_start_ts = None
        self._session_file = None
        self._sample_count = 0
        if sid is None or f is None or start_ts is None:
            return
        try:
            f.close()
        except OSError as e:
            rospy.logwarn("telemetry: close failed for %s: %s", sid, e)
        end_ts = time.time()
        path = os.path.join(self.store.root, sid + ".jsonl")
        try:
            file_size = os.path.getsize(path)
        except OSError:
            file_size = 0
        entry = {
            "id": sid,
            "start_ts": round(start_ts, 3),
            "end_ts": round(end_ts, 3),
            "duration_s": round(end_ts - start_ts, 1),
            "sample_count": sample_count,
            "file_size_bytes": file_size,
        }
        self.store.add_session(entry)
        rospy.loginfo("telemetry: closed session %s samples=%d size=%d duration=%.1fs",
                      sid, sample_count, file_size, entry["duration_s"])


class TelemetryRpcServer:
    """Implements telemetry.list_sessions and telemetry.get_session.

    Same shape as mower_scheduler/scheduler.py: register methods with the
    central xbot_rpc dispatcher, listen on /xbot/rpc/request, publish the
    result on /xbot/rpc/response (or an error on /xbot/rpc/error).
    """

    def __init__(self, store: TelemetryStore):
        self.store = store
        self._rpc_response_pub = rospy.Publisher("/xbot/rpc/response", RpcResponse, queue_size=20)
        self._rpc_error_pub = rospy.Publisher("/xbot/rpc/error", RpcError, queue_size=20)
        rospy.Subscriber("/xbot/rpc/request", RpcRequest, self._on_rpc, queue_size=20)
        try:
            rospy.wait_for_service("/xbot/rpc/register", timeout=10.0)
            register = rospy.ServiceProxy("/xbot/rpc/register", RegisterMethodsSrv)
            req = RegisterMethodsSrvRequest()
            req.node_id = NODE_ID
            req.methods = list(RPC_METHODS)
            register(req)
            rospy.loginfo("telemetry: registered %d RPC method(s)", len(RPC_METHODS))
        except rospy.ROSException:
            rospy.logwarn("RPC registration service not available; methods will not be routable")

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
        if method == "telemetry.list_sessions":
            return {"sessions": self.store.list_sessions()}
        if method == "telemetry.get_session":
            sid = self._extract_param(params, "id")
            if not isinstance(sid, str) or not SESSION_ID_RE.match(sid):
                raise RpcException(ERROR_INVALID_PARAMS, "id must match ^[a-f0-9]{16}$")
            stride = 1
            if isinstance(params, dict) and "stride" in params:
                try:
                    stride = max(1, int(params["stride"]))
                except (TypeError, ValueError):
                    raise RpcException(ERROR_INVALID_PARAMS, "stride must be a positive integer")
            return self._read_session(sid, stride)
        raise RpcException(ERROR_INTERNAL, f"Unhandled method: {method}")

    def _read_session(self, sid: str, stride: int) -> dict:
        path = self.store.session_path_if_exists(sid)
        if path is None:
            raise RpcException(ERROR_INVALID_PARAMS, f"unknown session id: {sid}")
        samples: list[dict] = []
        truncated = False
        with open(path, "r", encoding="utf-8") as f:
            for i, line in enumerate(f):
                if i % stride != 0:
                    continue
                line = line.strip()
                if not line:
                    continue
                try:
                    samples.append(json.loads(line))
                except ValueError:
                    # Half-written final line on a recorder crash; skip.
                    continue
                if len(samples) >= MAX_SAMPLES_PER_RESPONSE:
                    truncated = True
                    break
        result: dict[str, Any] = {"samples": samples}
        if truncated:
            result["truncated"] = True
        return result

    @staticmethod
    def _extract_param(params: Any, name: str) -> Any:
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


def main() -> None:
    rospy.init_node(NODE_ID)
    telemetry_path = rospy.get_param("~telemetry_path", DEFAULT_PATH)
    sensor_ids: list[str] = rospy.get_param("~telemetry_sensor_ids", [])

    store = TelemetryStore(telemetry_path)
    recorder = TelemetryRecorder(store, sensor_ids)
    rpc_server = TelemetryRpcServer(store)
    # rpc_server's lifecycle is owned by main; reference kept so it isn't GC'd.
    _ = rpc_server

    rospy.Subscriber(HIGH_LEVEL_TOPIC, HighLevelStatus, recorder.on_high_level, queue_size=10)
    rospy.Subscriber(STATE_TOPIC, RobotState, recorder.on_robot_state, queue_size=10)
    rospy.Subscriber(IMU_TOPIC, Imu, recorder.on_imu, queue_size=20)
    for sid in sensor_ids:
        topic = SENSOR_DATA_TEMPLATE.format(sid=sid)
        rospy.Subscriber(topic, SensorDataDouble,
                         (lambda s: lambda msg: recorder.on_sensor_data(s, msg))(sid),
                         queue_size=10)

    rospy.loginfo("mower_telemetry_recorder online, persisting to %s, sensors=%s",
                  telemetry_path, sensor_ids)

    rate = rospy.Rate(SAMPLE_HZ)
    while not rospy.is_shutdown():
        recorder.tick()
        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            break

    # Make sure an open session is closed cleanly on shutdown.
    with recorder._lock:  # noqa: SLF001 — final flush, deliberate
        if recorder._session_id is not None:
            recorder._close_session()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
