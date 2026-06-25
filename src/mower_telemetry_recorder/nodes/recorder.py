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
from xbot_msgs.msg import AbsolutePose, RobotState, SensorDataDouble
from xbot_positioning.msg import KalmanState
from xbot_mqtt.msg import RpcError, RpcRequest, RpcResponse
from xbot_mqtt.srv import RegisterMethodsSrv, RegisterMethodsSrvRequest

# Default mirrors the OpenMowerOS volume mount; container fallback is the
# same. recorder.launch can override.
DEFAULT_PATH = os.environ.get("TELEMETRY_PATH", "/data/ros/openmower/telemetry")

STATE_TOPIC = "xbot_monitoring/robot_state"
HIGH_LEVEL_TOPIC = "mower_logic/current_state"
# Madgwick-filtered IMU (imu_orientation_filter publishes imu/data with a
# usable orientation quaternion + gyro/accel). The bare "imu" topic does not
# exist, which silently left every IMU field at zero.
IMU_TOPIC = "imu/data"
SENSOR_DATA_TEMPLATE = "xbot_monitoring/sensors/{sid}/data"
# Raw, unfiltered GPS pose (antenna position + GPS-derived headings) and the
# raw EKF state. Only recorded when the localisation-debug mode is on; used to
# diagnose antenna-offset / heading (theta) problems by correlating the raw
# antenna fix against the fused centre estimate.
RAW_GPS_TOPIC = "ll/position/gps"
KALMAN_STATE_TOPIC = "xbot_positioning/kalman_state"

NODE_ID = "mower_telemetry_recorder"
RPC_METHODS = ("telemetry.list_sessions", "telemetry.get_session")

# Mowing-state names that gate a mow session by state transition.
ACTIVE_STATES = {"MOWING", "PAUSED"}

# Manual driving has no dedicated high-level state (the joystick is only routed
# in AREA_RECORDING, and app teleop bypasses the FSM via twist_mux), so manual
# sessions are gated by actual MOVEMENT instead of by state. Such a session
# auto-closes after this long without movement, or immediately when the
# AREA_RECORDING mode is left — only a closed session lands in the index and
# shows up in the heatmap.
MANUAL_IDLE_TIMEOUT_S = 30.0
# Per-tick displacement (m) above which the mower counts as moving (≈0.12 m/s at
# 4 Hz). Opens / keeps alive a manual movement session.
MOVE_EPS_M = 0.03

# 4 Hz sampling — matches the App's heatmap zoom budget at sane file sizes
# (a 30 min mow ≈ 7200 samples ≈ 2 MB JSONL).
SAMPLE_HZ = 4.0
SAMPLE_PERIOD_S = 1.0 / SAMPLE_HZ

# Movement-gating: while the mower sits still (e.g. PAUSED waiting for an RTK
# fix) writing 4 Hz produces hundreds of stacked points at one spot, which
# bloats the file and dominates the heatmap. We only write a new sample once the
# mower has moved at least MIN_MOVE_M, or MAX_STATIONARY_PERIOD_S has elapsed (a
# heartbeat so a long pause still leaves a sparse trace), or the mowing state
# changed (so MOWING/PAUSED segment boundaries are preserved exactly).
MIN_MOVE_M = 0.05
MAX_STATIONARY_PERIOD_S = 5.0

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

# Hard cap on JSONL session file size accepted by telemetry.get_session.
# MAX_SAMPLES_PER_RESPONSE only bounds the *output* — without an input bound
# the recorder still has to stream the entire file from disk into memory
# before truncating, which OOMs the broker when a runaway session
# (recorder forgot to close, days of activity) hits multiple hundred MB.
# 200 MB ≈ 30 h at 4 Hz with the current sample shape, well above any
# legitimate single mowing session.
MAX_SESSION_BYTES = 200 * 1024 * 1024

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

    def __init__(self, store: TelemetryStore, sensor_ids: list[str], record_all_states: bool = False):
        self.store = store
        self.sensor_ids = sensor_ids
        # record_all_states only controls whether the raw GPS + EKF debug fields
        # are folded into each sample; it no longer gates session opening (manual
        # sessions are movement-gated, see _update_manual_session_unlocked).
        self.record_all_states = record_all_states
        self._lock = threading.Lock()
        self._last_state: Optional[str] = None

        # Active session state. None when not recording. A session is either a
        # "mow" session (opened/closed by the MOWING/PAUSED state transition) or
        # a "manual" session (opened by movement, closed on idle timeout or when
        # AREA_RECORDING is left). _manual_session distinguishes the two so the
        # state-transition path never closes a movement session and vice versa.
        self._session_id: Optional[str] = None
        self._session_start_ts: Optional[float] = None
        self._session_file = None  # type: Optional[Any]
        self._sample_count: int = 0
        self._manual_session: bool = False
        # ts of the last detected movement, for the manual-session idle timeout.
        self._last_move_ts: Optional[float] = None
        # Reference pose for per-interval movement detection (manual sessions).
        self._move_ref_x: Optional[float] = None
        self._move_ref_y: Optional[float] = None

        # Latest values cached from subscribers; folded into each sample tick.
        self._robot_state: Optional[RobotState] = None
        self._imu_yaw: float = 0.0
        self._imu_pitch: float = 0.0
        self._imu_roll: float = 0.0
        # Raw IMU: orientation quaternion (w,x,y,z), angular velocity (rad/s),
        # linear acceleration (m/s^2). Cached straight off the Imu message.
        self._imu_quat: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
        self._imu_gyro: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._imu_accel: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._sensor_values: dict[str, float] = {}

        # Localisation-debug caches (only populated/written in record_all_states
        # mode). Raw GPS = the unfiltered antenna fix; Kalman = the fused EKF
        # state. None until the first message arrives so missing topics stay out
        # of the sample instead of writing misleading zeros.
        self._raw_gps: Optional[AbsolutePose] = None
        self._kalman: Optional[KalmanState] = None

        # Movement-gating bookkeeping: the pose/ts of the last sample actually
        # written, so a stationary mower (e.g. paused waiting for RTK fix) does
        # not pile up hundreds of near-identical points at one spot.
        self._last_written_x: Optional[float] = None
        self._last_written_y: Optional[float] = None
        self._last_written_ts: Optional[float] = None
        self._last_written_state: Optional[str] = None

    # ----- callbacks -------------------------------------------------------

    def on_high_level(self, msg: HighLevelStatus) -> None:
        state_name = msg.state_name or ""
        with self._lock:
            prev_state = self._last_state
            entered_active = state_name in ACTIVE_STATES and prev_state not in ACTIVE_STATES
            left_active = state_name not in ACTIVE_STATES and prev_state in ACTIVE_STATES
            self._last_state = state_name

            # Mow session: opened/closed by the MOWING/PAUSED state transition.
            # Takes precedence over a manual session (entering MOWING closes any
            # open movement session first so the two never overlap).
            if entered_active:
                if self._session_id is not None and self._manual_session:
                    self._close_session()
                if self._session_id is None:
                    self._open_session(manual=False)
            elif left_active and self._session_id is not None and not self._manual_session:
                self._close_session()

            # Leaving AREA_RECORDING ends a manual session right away (rather than
            # waiting for the idle timeout), so it lands in the heatmap promptly.
            if prev_state == "AREA_RECORDING" and state_name != "AREA_RECORDING":
                if self._session_id is not None and self._manual_session:
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
        av = msg.angular_velocity
        la = msg.linear_acceleration
        with self._lock:
            self._imu_roll = roll
            self._imu_pitch = pitch
            self._imu_yaw = yaw
            self._imu_quat = (q.w, q.x, q.y, q.z)
            self._imu_gyro = (av.x, av.y, av.z)
            self._imu_accel = (la.x, la.y, la.z)

    def on_sensor_data(self, sensor_id: str, msg: SensorDataDouble) -> None:
        with self._lock:
            self._sensor_values[sensor_id] = float(msg.data)

    def on_raw_gps(self, msg: AbsolutePose) -> None:
        # Raw, unfiltered antenna fix + GPS-derived headings (debug only).
        with self._lock:
            self._raw_gps = msg

    def on_kalman_state(self, msg: KalmanState) -> None:
        # Raw EKF state (x, y, theta, vx, vr); only published when
        # xbot_positioning/debug is true (debug only).
        with self._lock:
            self._kalman = msg

    # ----- tick / IO -------------------------------------------------------

    def tick(self) -> None:
        """Called from the main loop at SAMPLE_HZ; manages movement-gated manual
        sessions, then writes one JSONL line when movement-gating allows it."""
        with self._lock:
            self._update_manual_session_unlocked()
            if self._session_id is None or self._session_file is None:
                return
            sample = self._build_sample_unlocked()
            if not self._should_write_unlocked(sample):
                return
            try:
                self._session_file.write(json.dumps(sample, separators=(",", ":")))
                self._session_file.write("\n")
                self._sample_count += 1
                self._last_written_x = sample.get("x")
                self._last_written_y = sample.get("y")
                self._last_written_ts = sample["ts"]
                self._last_written_state = sample.get("state")
            except OSError as e:
                rospy.logerr("telemetry write failed: %s", e)

    def _current_pose_unlocked(self) -> tuple[Optional[float], Optional[float]]:
        rs = self._robot_state
        if rs is None:
            return None, None
        return float(rs.robot_pose.pose.pose.position.x), float(rs.robot_pose.pose.pose.position.y)

    def _update_manual_session_unlocked(self) -> None:
        """Open/close a movement-gated manual session. Manual driving has no
        dedicated high-level state, so we key off actual pose movement: while not
        mowing, real movement opens a session; it auto-closes after
        MANUAL_IDLE_TIMEOUT_S without movement. Leaving AREA_RECORDING closes it
        immediately (handled in on_high_level)."""
        # Never interfere with a state-driven mow session.
        if self._last_state in ACTIVE_STATES:
            return
        if self._session_id is not None and not self._manual_session:
            return

        now = time.time()
        x, y = self._current_pose_unlocked()
        moved = (
            x is not None
            and y is not None
            and self._move_ref_x is not None
            and self._move_ref_y is not None
            and math.hypot(x - self._move_ref_x, y - self._move_ref_y) >= MOVE_EPS_M
        )
        if moved or self._move_ref_x is None:
            # Advance the reference point whenever we move (or on first pose) so
            # MOVE_EPS_M is a per-interval displacement, not cumulative drift.
            self._move_ref_x, self._move_ref_y = x, y
        if moved:
            self._last_move_ts = now

        if self._session_id is None:
            # Open a manual session on the first detected movement.
            if moved:
                self._open_session(manual=True)
                self._last_move_ts = now
        elif self._manual_session:
            # Close it once movement has stopped for the idle timeout.
            if self._last_move_ts is not None and (now - self._last_move_ts) >= MANUAL_IDLE_TIMEOUT_S:
                self._close_session()

    def _should_write_unlocked(self, sample: dict) -> bool:
        """Movement-gate: always write the first sample, on a state change, or
        once the mower has moved MIN_MOVE_M; otherwise throttle a stationary
        mower to a MAX_STATIONARY_PERIOD_S heartbeat."""
        if self._last_written_ts is None:
            return True
        if sample.get("state") != self._last_written_state:
            return True
        x, y = sample.get("x"), sample.get("y")
        if x is None or y is None or self._last_written_x is None or self._last_written_y is None:
            # No pose to compare — fall back to the heartbeat throttle.
            return (sample["ts"] - self._last_written_ts) >= MAX_STATIONARY_PERIOD_S
        moved = math.hypot(x - self._last_written_x, y - self._last_written_y)
        if moved >= MIN_MOVE_M:
            return True
        return (sample["ts"] - self._last_written_ts) >= MAX_STATIONARY_PERIOD_S

    def _build_sample_unlocked(self) -> dict:
        rs = self._robot_state
        sample: dict[str, Any] = {"ts": time.time()}
        if self._last_state is not None:
            sample["state"] = self._last_state
        if rs is not None:
            sample["x"] = float(rs.robot_pose.pose.pose.position.x)
            sample["y"] = float(rs.robot_pose.pose.pose.position.y)
            # Fused heading (theta) straight from the EKF pose carried in
            # RobotState — available on every recording without the debug topic.
            # This is the robust source for the localisation analysis (raw GPS
            # antenna vs fused centre rotated by theta).
            q = rs.robot_pose.pose.pose.orientation
            sample["fused_theta"] = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            sample["fused_heading"] = float(rs.robot_pose.vehicle_heading)
            sample["fused_heading_valid"] = bool(rs.robot_pose.orientation_valid)
            sample["gps_fix_type"] = int(rs.gps_fix_type)
            sample["gps_satellite_count"] = int(rs.gps_satellite_count)
            sample["gps_pdop"] = float(rs.gps_pdop)
            # Reported position accuracy in metres (RTK fix ≈ a few cm).
            sample["gps_accuracy"] = float(rs.robot_pose.position_accuracy)
            sample["wifi_dbm"] = int(rs.wifi_signal_dbm)
            # Send the same field name the openmower-app heatmap expects.
            sample["wifi_q"] = float(rs.wifi_link_quality)
        sample["yaw"] = self._imu_yaw
        sample["pitch"] = self._imu_pitch
        sample["roll"] = self._imu_roll
        # Raw IMU: quaternion (qw,qx,qy,qz), gyro (gx,gy,gz), accel (ax,ay,az).
        qw, qx, qy, qz = self._imu_quat
        gx, gy, gz = self._imu_gyro
        ax, ay, az = self._imu_accel
        sample["qw"], sample["qx"], sample["qy"], sample["qz"] = qw, qx, qy, qz
        sample["gx"], sample["gy"], sample["gz"] = gx, gy, gz
        sample["ax"], sample["ay"], sample["az"] = ax, ay, az
        for sid, val in self._sensor_values.items():
            sample[sid] = val
        if self.record_all_states:
            self._add_debug_fields_unlocked(sample)
        return sample

    def _add_debug_fields_unlocked(self, sample: dict) -> None:
        """Fold the raw GPS antenna fix and the fused EKF state into the sample
        so an antenna-offset / heading (theta) bug can be diagnosed offline:
        plotting the raw antenna track against the EKF centre, and the
        correction vector (raw - ekf) in body frame, pins down whether the
        offset or theta is wrong. Only called in localisation-debug mode."""
        gps = self._raw_gps
        if gps is not None:
            # Raw antenna position in metres from the datum (map frame).
            raw_x = float(gps.pose.pose.position.x)
            raw_y = float(gps.pose.pose.position.y)
            sample["raw_gps_x"] = raw_x
            sample["raw_gps_y"] = raw_y
            sample["gps_motion_heading"] = float(gps.motion_heading)
            sample["gps_vehicle_heading"] = float(gps.vehicle_heading)
            sample["gps_orientation_valid"] = bool(gps.orientation_valid)
            sample["raw_gps_acc"] = float(gps.position_accuracy)
            sample["raw_gps_flags"] = int(gps.flags)
        kf = self._kalman
        if kf is not None:
            sample["ekf_x"] = float(kf.x)
            sample["ekf_y"] = float(kf.y)
            sample["ekf_theta"] = float(kf.theta)
            sample["ekf_vx"] = float(kf.vx)
            sample["ekf_vr"] = float(kf.vr)
        # The antenna correction vector (raw antenna - fused centre). This is the
        # core diagnostic signal: rotated into body frame via ekf_theta it must
        # be the constant body-fixed offset; if it isn't, theta is the culprit.
        if gps is not None and kf is not None:
            sample["gps_dx"] = sample["raw_gps_x"] - sample["ekf_x"]
            sample["gps_dy"] = sample["raw_gps_y"] - sample["ekf_y"]

    def _open_session(self, manual: bool = False) -> None:
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
        self._manual_session = manual
        # Reset movement-gating so the first tick of the new session is written.
        self._last_written_x = None
        self._last_written_y = None
        self._last_written_ts = None
        self._last_written_state = None
        rospy.loginfo("telemetry: opened %s session %s at %s",
                      "manual" if manual else "mow", sid, path)

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
        was_manual = self._manual_session
        self._manual_session = False
        self._last_move_ts = None
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
            "manual": was_manual,
        }
        self.store.add_session(entry)
        rospy.loginfo("telemetry: closed %s session %s samples=%d size=%d duration=%.1fs",
                      "manual" if was_manual else "mow", sid, sample_count, file_size, entry["duration_s"])


class TelemetryRpcServer:
    """Implements telemetry.list_sessions and telemetry.get_session.

    Same shape as mower_scheduler/scheduler.py: register methods with the
    central xbot_mqtt dispatcher, listen on /xbot/rpc/request, publish the
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
            offset = 0
            limit = MAX_SAMPLES_PER_RESPONSE
            if isinstance(params, dict):
                if "stride" in params:
                    try:
                        stride = max(1, int(params["stride"]))
                    except (TypeError, ValueError):
                        raise RpcException(ERROR_INVALID_PARAMS, "stride must be a positive integer")
                # Pagination over the post-stride sequence: skip `offset` selected
                # samples and return at most `limit`, so a large session can be
                # fetched as several small responses instead of one huge payload.
                if "offset" in params:
                    try:
                        offset = max(0, int(params["offset"]))
                    except (TypeError, ValueError):
                        raise RpcException(ERROR_INVALID_PARAMS, "offset must be a non-negative integer")
                if "limit" in params and params["limit"] is not None:
                    try:
                        limit = int(params["limit"])
                    except (TypeError, ValueError):
                        raise RpcException(ERROR_INVALID_PARAMS, "limit must be a positive integer")
                    if limit <= 0:
                        raise RpcException(ERROR_INVALID_PARAMS, "limit must be a positive integer")
                    limit = min(limit, MAX_SAMPLES_PER_RESPONSE)
            return self._read_session(sid, stride, offset, limit)
        raise RpcException(ERROR_INTERNAL, f"Unhandled method: {method}")

    def _read_session(self, sid: str, stride: int, offset: int = 0, limit: int = MAX_SAMPLES_PER_RESPONSE) -> dict:
        path = self.store.session_path_if_exists(sid)
        if path is None:
            raise RpcException(ERROR_INVALID_PARAMS, f"unknown session id: {sid}")
        try:
            size = os.path.getsize(path)
        except OSError as e:
            raise RpcException(ERROR_INTERNAL, f"cannot stat session file: {e}")
        if size > MAX_SESSION_BYTES:
            # The frontend should suggest re-querying with a higher stride; we
            # surface the limit so it can compute a sensible value rather than
            # guess. We deliberately raise INVALID_PARAMS, not INTERNAL — the
            # caller can recover by changing the request.
            raise RpcException(
                ERROR_INVALID_PARAMS,
                f"session file too large ({size} bytes > {MAX_SESSION_BYTES}); "
                f"retry with a higher stride or open the JSONL directly",
            )
        samples: list[dict] = []
        truncated = False
        # selected_index counts samples that pass the stride filter; offset/limit
        # paginate over that sequence. We read line-by-line and stop as soon as
        # we have one more than `limit` selected past the offset, so `truncated`
        # signals the client to request the next page.
        selected_index = 0
        with open(path, "r", encoding="utf-8") as f:
            for i, line in enumerate(f):
                if i % stride != 0:
                    continue
                line = line.strip()
                if not line:
                    continue
                try:
                    parsed = json.loads(line)
                except ValueError:
                    # Half-written final line on a recorder crash; skip.
                    continue
                # This is a valid selected sample; apply pagination on its index.
                idx = selected_index
                selected_index += 1
                if idx < offset:
                    continue
                if len(samples) >= limit:
                    # There is at least one more selected sample beyond this page.
                    truncated = True
                    break
                samples.append(parsed)
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
    # Localisation-debug mode: also record while driving manually / area
    # recording, and capture the raw GPS + EKF fields for offline diagnosis.
    record_all_states: bool = rospy.get_param("~record_all_states", False)

    store = TelemetryStore(telemetry_path)
    recorder = TelemetryRecorder(store, sensor_ids, record_all_states=record_all_states)
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

    if record_all_states:
        rospy.Subscriber(RAW_GPS_TOPIC, AbsolutePose, recorder.on_raw_gps, queue_size=20)
        rospy.Subscriber(KALMAN_STATE_TOPIC, KalmanState, recorder.on_kalman_state, queue_size=20)

    rospy.loginfo("mower_telemetry_recorder online, persisting to %s, sensors=%s, record_all_states=%s",
                  telemetry_path, sensor_ids, record_all_states)

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
