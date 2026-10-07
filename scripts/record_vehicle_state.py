#!/usr/bin/env python3
"""Records the vehicle's true (Gazebo) state alongside PX4's EKF2 estimate and the
actuator commands driving it, then plots them together -- no plugin or flight-controller
code is touched, this only subscribes.

Eight sources, recorded concurrently, all on ONE timebase (see "Clock" below):

  1. Sim clock -- /world/<world>/clock (gz.msgs.Clock), 1 kHz. The authoritative time
     reference. Used to stamp messages that carry no usable header stamp, and to measure
     the recording duration in SIM seconds rather than wall-clock seconds.
  2. Ground truth pose + twist -- /model/<model>/odometry (gz.msgs.Odometry), published at
     1 kHz by gz::sim::systems::OdometryPublisher (declared in models/monospinner/model.sdf).
     Pose is world ENU; BOTH twist components are BODY frame (FLU) -- verified empirically,
     not assumed, see "Frames" below.
  3. Ground truth body rate + specific force -- /model/<model>/imu_gd_truth
     (gz.msgs.IMU), 250 Hz: ImuPhysical's <ground_truth_topic>, i.e. Link::
     WorldAngularVelocity() straight off the ECM (see src/gz_imu_realism_plugin/src/
     ImuPhysical.cc) with no noise, bias or saturation. That is a better angular-rate
     reference than anything differenced from pose, and it is what
     --gt-angular-source=imu selects. NOT the noisy IMU topic PX4 consumes: that one
     carries the configured gyro bias, so it is not ground truth.
  4. Ground truth low-level actuator input -- /<model>/command/motor_speed (gz.msgs.
     Actuators), what ThrustVectorActuator actually receives.
  5. PX4's EKF2 estimate -- /fmu/out/vehicle_odometry (px4_msgs/VehicleOdometry).
  6. Direct controller input to PX4 -- /fmu/in/actuator_motors (px4_msgs/ActuatorMotors).
  7. GPS as PX4 reads it -- /fmu/out/vehicle_gps_position (px4_msgs/SensorGps). This is
     AFTER GZBridge::addGpsNoise() (GZBridge.cpp:742), which layers PX4's own F9P-style
     noise on top of the gz navsat reading, so it is what EKF2 actually fuses. Lat/lon/alt
     are converted to local NED about the world's <spherical_coordinates> origin (read
     from worlds/<world>.sdf at record time and stored in the manifest; gz's default is
     0/0/0 when the block is absent), using PX4's CONSTANTS_RADIUS_OF_EARTH = 6371000 m
     (src/lib/geo/geo.h:55).
  8. Magnetometer as PX4 reads it -- the gz magnetometer topic GZBridge subscribes to
     (GZBridge::subscribeMag(), GZBridge.cpp:228-233), remapped exactly as
     GZBridge::magnetometerCallback() does (x = -y, y = -x, z = z; GZBridge.cpp:403-405)
     and, like PX4, read as gauss. That is PX4's raw sensor_mag, before calibration. No
     PX4 magnetometer topic is exported over uXRCE-DDS (dds_topics.yaml), hence gz.

Clock  --  REQUIRES config/monospinner.params (UXRCE_DDS_SYNCT 0)
----------------------------------------------------------------
Inside PX4, hrt_absolute_time() IS gz simulation time: GZBridge::clockCallback calls
px4_clock_settime() with the gz clock on every message (GZBridge.cpp:331-347) and under
lockstep hrt returns lockstep_scheduler.get_absolute_time() (drv_hrt.cpp:106-115). uORB
timestamps in build/px4_run/log/*.ulg confirm it -- they run from ~8 s, not a wall epoch.

That is NOT what reaches ROS 2. With UXRCE_DDS_SYNCT=1 (PX4's default) the uXRCE-DDS
client adds the client/agent offset to every outgoing timestamp
(src/modules/uxrce_dds_client/dds_topics.h.em:126), so /fmu/out/* arrives stamped in the
AGENT's wall-clock time. Measured: gz truth at t=96 s vs a PX4 estimate at t=1788873294 s.
config/monospinner.params sets UXRCE_DDS_SYNCT 0 to turn that translation off, and
scripts/run_sim.sh applies it at boot via the PX4_PARAM_* hook (rcS:129-134) because the
parameter is reboot_required.

With that override in place `px4_msg.timestamp * 1e-6` and a gz header stamp are DIRECTLY
COMPARABLE. This script checks the overlap at run time and refuses to pretend otherwise.
Do not "fix" a reported mismatch by normalising each series to its own t0 -- that is what
this script used to do, and it silently destroys the alignment, making estimator lag
unmeasurable. Estimator lag is not a nicety here: per CLAUDE.md, beta_cmd = psi_desired -
theta_spin - delta(omega), so a heading estimate that lags rotates the control input
directly.

Frames
------
Handled per-field, because they are NOT uniform:

  quantity          PX4 /fmu/out/vehicle_odometry   Gazebo ground truth
  ----------------  -----------------------------   ----------------------------
  position          NED        (POSE_FRAME_NED)     world ENU  -> NED
  attitude q        body FRD -> NED                 body FLU -> ENU  -> FRD->NED
  velocity          *** NED ***  (VELOCITY_FRAME_   body FLU -> world ENU -> NED
                    NED, set at EKF2.cpp:1694)
  angular_velocity  body FRD (always, per msg doc)  body FLU -> FRD

The velocity row is the trap. PX4's EKF2 publishes velocity in WORLD NED, not body FRD.
An earlier version of this script converted ground-truth velocity to body FRD and overlaid
it on the NED estimate; on a vehicle that spins at tens of rad/s the body-frame components
oscillate at the spin frequency while the NED estimate is smooth, which reads as a
catastrophic estimator failure that is not there. The frame enums are read off each
message and recorded in the manifest, so a future PX4 release changing them shows up as a
loud mismatch rather than a silently wrong plot.

The ENU/FLU -> NED/FRD transform reproduces PX4's own gz_bridge transform exactly
(GZBridge.cpp:598-627 for position/velocity, :934-950 for the quaternion rotation).

Requires: a running gz-sim server with the monospinner model loaded, and PX4 SITL +
MicroXRCEAgent bridging /fmu/* topics to ROS 2 (i.e. scripts/run_sim.sh).

Usage:
  python3 scripts/record_vehicle_state.py --duration 20 --out-dir build/runs
  python3 scripts/record_vehicle_state.py --duration 20 --gt-angular-source odom
  python3 scripts/record_vehicle_state.py --plot-only build/runs/2026-09-08T14-03-11
"""
import argparse
import csv
import datetime
import json
import os
import subprocess
import sys
import threading
import time

import numpy as np

# --- Gazebo side ---
from gz.transport13 import Node as GzNode
from gz.msgs10.clock_pb2 import Clock
from gz.msgs10.odometry_pb2 import Odometry
from gz.msgs10.imu_pb2 import IMU
from gz.msgs10.actuators_pb2 import Actuators
from gz.msgs10.magnetometer_pb2 import Magnetometer

# --- ROS 2 / PX4 side ---
import rclpy
from rclpy.node import Node as RosNode
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from px4_msgs.msg import VehicleOdometry, ActuatorMotors, SensorGps


# PX4's uXRCE-DDS bridge publishes /fmu/* with BEST_EFFORT/VOLATILE -- a default RELIABLE
# rclpy subscription silently receives nothing against it. This is PX4's documented QoS,
# not a guess.
PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=50,
)

# px4_msgs/VehicleOdometry frame enums, mirrored here for readable manifest output.
POSE_FRAME = {0: "UNKNOWN", 1: "NED", 2: "FRD"}
VELOCITY_FRAME = {0: "UNKNOWN", 1: "NED", 2: "FRD", 3: "BODY_FRD"}


# ---------------------------------------------------------------- quaternion helpers ---
def quat_conj(q):
    w, x, y, z = q
    return np.array([w, -x, -y, -z])


def quat_mul(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def quat_rotate(q, v):
    """Rotates v from the frame q maps FROM into the frame q maps TO (Hamilton, gz
    convention: world_vec = q * body_vec * conj(q))."""
    qv = np.array([0.0, *v])
    return quat_mul(quat_mul(q, qv), quat_conj(q))[1:]


def quat_to_euler_xyz(q):
    """Tait-Bryan roll/pitch/yaw (rad) from a Hamilton (w,x,y,z) quaternion. For display
    only -- everything else in this script stays in quaternion/vector form."""
    w, x, y, z = q
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(np.clip(sinp, -1.0, 1.0))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.array([roll, pitch, yaw])


# --- PX4's own ENU/FLU -> NED/FRD transform (GZBridge.cpp:934-950), reproduced exactly ---
Q_FLU_TO_FRD = np.array([0.0, 1.0, 0.0, 0.0])
Q_ENU_TO_NED = np.array([0.0, 0.70711, 0.70711, 0.0])


def gz_pose_to_px4_frame(pos_enu, quat_flu_to_enu):
    """Gazebo world position + body(FLU)->world(ENU) orientation -> PX4 NED position and
    body(FRD)->NED orientation, per GZBridge.cpp:598-616."""
    pos_ned = np.array([pos_enu[1], pos_enu[0], -pos_enu[2]])
    q_frd_to_ned = quat_mul(quat_mul(Q_ENU_TO_NED, quat_flu_to_enu), quat_conj(Q_FLU_TO_FRD))
    return pos_ned, q_frd_to_ned


def enu_vec_to_ned(v_enu):
    """World ENU -> world NED for a position or velocity vector."""
    return np.array([v_enu[1], v_enu[0], -v_enu[2]])


def gz_body_vec_to_frd(v_flu):
    """FLU -> FRD for a body-frame vector, per GZBridge.cpp:618-627."""
    return np.array([v_flu[0], -v_flu[1], -v_flu[2]])


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nsec * 1e-9


# PX4's CONSTANTS_RADIUS_OF_EARTH (src/lib/geo/geo.h:55).
EARTH_RADIUS_M = 6371000.0


def read_world_gps_origin(world_sdf_path):
    """Returns the world's <spherical_coordinates> origin as a dict (lat_deg, lon_deg,
    elevation_m, heading_deg), i.e. the lat/lon/alt gz's navsat reports at the world
    origin. gz falls back to 0/0/0/0 when the block is absent, and so does this."""
    import xml.etree.ElementTree as ET
    origin = {"lat_deg": 0.0, "lon_deg": 0.0, "elevation_m": 0.0, "heading_deg": 0.0,
              "source": "gz default (no <spherical_coordinates> in world)"}
    try:
        sc = ET.parse(world_sdf_path).getroot().find("./world/spherical_coordinates")
    except (OSError, ET.ParseError) as e:
        origin["source"] = f"gz default (could not read {world_sdf_path}: {e})"
        return origin
    if sc is None:
        return origin
    for key, tag in [("lat_deg", "latitude_deg"), ("lon_deg", "longitude_deg"),
                     ("elevation_m", "elevation"), ("heading_deg", "heading_deg")]:
        el = sc.find(tag)
        if el is not None and el.text and el.text.strip():
            origin[key] = float(el.text)
    origin["source"] = world_sdf_path
    return origin


def gps_to_local_ned(lat_deg, lon_deg, alt_m, origin):
    """Lat/lon/alt -> NED metres about the world origin. Equirectangular, which is exact
    to well under a millimetre over the metres a SITL run covers. Assumes the world frame
    is not rotated (<heading_deg> 0); the caller warns otherwise."""
    lat0 = np.radians(origin["lat_deg"])
    north = np.radians(lat_deg - origin["lat_deg"]) * EARTH_RADIUS_M
    east = np.radians(lon_deg - origin["lon_deg"]) * EARTH_RADIUS_M * np.cos(lat0)
    down = -(alt_m - origin["elevation_m"])
    return np.stack([north, east, down], axis=1)


def gz_mag_to_px4_sensor_mag(field):
    """gz Magnetometer field -> PX4 sensor_mag axes, exactly as
    GZBridge::magnetometerCallback() remaps it (GZBridge.cpp:403-405)."""
    return np.array([-field.y, -field.x, field.z])


# ------------------------------------------------------------------- streaming writer ---
class StreamWriter:
    """Appends rows to a CSV as they arrive. Recording a long run into RAM and writing at
    the end means a crash or a Ctrl-C loses the whole run; this keeps whatever was captured
    up to the failure."""

    def __init__(self, path, header):
        self.path = path
        self._f = open(path, "w", newline="")
        self._w = csv.writer(self._f)
        self._w.writerow(header)
        self._lock = threading.Lock()
        self._closed = False
        self.count = 0

    def write(self, row):
        # gz-transport callbacks keep firing on their own threads after the recording
        # window ends -- there is no unsubscribe in the python binding. Without this guard
        # every in-flight callback raises "I/O operation on closed file" into the
        # transport's deserialize handler once close() has run.
        with self._lock:
            if self._closed:
                return
            self._w.writerow(row)
            self.count += 1

    def close(self):
        with self._lock:
            if self._closed:
                return
            self._closed = True
            self._f.close()


class SimClock:
    """Latest simulation time, from /world/<world>/clock.

    Two jobs. (1) Stamp messages with no usable header stamp -- notably the actuator
    command, since PX4's GZMixingInterfaceESC populates only the velocity field and never
    sets a header (GZMixingInterfaceESC.cpp:80-88). The previous version of this script
    stamped those with time.time(), mixing wall clock into an otherwise sim-time dataset.
    (2) Measure the recording window in sim seconds. Under lockstep the real-time factor is
    not 1, so sleeping for N wall-clock seconds captures an unpredictable amount of sim.
    """

    def __init__(self):
        self._t = None
        self._lock = threading.Lock()

    def update(self, t):
        with self._lock:
            self._t = t

    @property
    def t(self):
        with self._lock:
            return self._t

    def wait_for_first(self, timeout=15.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.t is not None:
                return True
            time.sleep(0.05)
        return False


def git_describe(path):
    try:
        return subprocess.check_output(
            ["git", "-C", path, "rev-parse", "HEAD"],
            stderr=subprocess.DEVNULL, text=True).strip()
    except Exception:
        return None


def file_digest(path):
    try:
        import hashlib
        with open(path, "rb") as f:
            return hashlib.sha256(f.read()).hexdigest()[:16]
    except Exception:
        return None


# --------------------------------------------------------------------------- recording ---
def record(args, run_dir, repo_root):
    world, model = args.world, args.model
    clock_topic = f"/world/{world}/clock"
    odom_topic = args.odom_topic or f"/model/{model}/odometry"
    # ImuPhysical's <ground_truth_topic> in models/monospinner/model.sdf.
    imu_topic = f"/model/{model}/imu_gd_truth"
    gz_actuator_topic = f"/{model}/command/motor_speed"
    # The topic GZBridge::subscribeMag() hardcodes (GZBridge.cpp:228-233).
    mag_topic = (f"/world/{world}/model/{model}/link/base_link/sensor/"
                 f"magnetometer_sensor/magnetometer")
    gps_ros_topic = "/fmu/out/vehicle_gps_position"

    sim_clock = SimClock()

    w_odom = StreamWriter(os.path.join(run_dir, "gt_odom.csv"), [
        "t_sim", "x_enu", "y_enu", "z_enu", "qw", "qx", "qy", "qz",
        "vx_flu", "vy_flu", "vz_flu", "wx_flu", "wy_flu", "wz_flu"])
    w_imu = StreamWriter(os.path.join(run_dir, "gt_imu.csv"), [
        "t_sim", "wx_flu", "wy_flu", "wz_flu", "ax_flu", "ay_flu", "az_flu"])
    w_gzact = StreamWriter(os.path.join(run_dir, "gz_actuator_cmd.csv"), [
        "t_sim", "pos0", "pos1", "vel0", "n_position", "n_velocity"])
    w_px4odom = StreamWriter(os.path.join(run_dir, "px4_odom.csv"), [
        "t_sim", "x_ned", "y_ned", "z_ned", "qw", "qx", "qy", "qz",
        "vx", "vy", "vz", "wx_frd", "wy_frd", "wz_frd",
        "pose_frame", "velocity_frame"])
    w_px4in = StreamWriter(os.path.join(run_dir, "px4_actuator_cmd.csv"), [
        "t_sim", "control0", "control1", "control2", "control3"])
    w_gps = StreamWriter(os.path.join(run_dir, "px4_gps.csv"), [
        "t_sim", "lat_deg", "lon_deg", "alt_m", "vn", "ve", "vd", "fix_type"])
    w_mag = StreamWriter(os.path.join(run_dir, "px4_mag.csv"), [
        "t_sim", "mx_ga", "my_ga", "mz_ga"])
    writers = [w_odom, w_imu, w_gzact, w_px4odom, w_px4in, w_gps, w_mag]

    # Frame enums seen on the wire, checked against expectations after the run rather than
    # assumed at parse time.
    seen_frames = set()

    decim = max(1, args.odom_decimate)
    odom_n = [0]

    def on_clock(msg):
        sim_clock.update(msg.sim.sec + msg.sim.nsec * 1e-9)

    def on_odom(msg):
        odom_n[0] += 1
        if odom_n[0] % decim:
            return
        t = stamp_to_sec(msg.header.stamp)
        p, o = msg.pose.position, msg.pose.orientation
        tl, ta = msg.twist.linear, msg.twist.angular
        w_odom.write([t, p.x, p.y, p.z, o.w, o.x, o.y, o.z,
                      tl.x, tl.y, tl.z, ta.x, ta.y, ta.z])

    def on_imu(msg):
        t = stamp_to_sec(msg.header.stamp)
        av, la = msg.angular_velocity, msg.linear_acceleration
        w_imu.write([t, av.x, av.y, av.z, la.x, la.y, la.z])

    def on_gz_actuator(msg):
        # No header stamp from PX4's ESC interface -- fall back to the sim clock. Never
        # time.time(): that would mix wall clock into a sim-time dataset.
        t = stamp_to_sec(msg.header.stamp)
        if t <= 0.0:
            t = sim_clock.t
            if t is None:
                return
        npos, nvel = len(msg.position), len(msg.velocity)
        w_gzact.write([t,
                       msg.position[0] if npos > 0 else float("nan"),
                       msg.position[1] if npos > 1 else float("nan"),
                       msg.velocity[0] if nvel > 0 else float("nan"),
                       npos, nvel])

    def on_mag(msg):
        t = stamp_to_sec(msg.header.stamp)
        w_mag.write([t, *gz_mag_to_px4_sensor_mag(msg.field_tesla)])

    gz_node = GzNode()
    subs = [
        (Clock, clock_topic, on_clock),
        (Odometry, odom_topic, on_odom),
        (IMU, imu_topic, on_imu),
        (Actuators, gz_actuator_topic, on_gz_actuator),
        (Magnetometer, mag_topic, on_mag),
    ]
    for msg_t, topic, cb in subs:
        if not gz_node.subscribe(msg_t, topic, cb):
            sys.exit(f"error: failed to subscribe to {topic}")

    if not sim_clock.wait_for_first():
        sys.exit(f"error: no messages on {clock_topic} after 15 s -- is gz-sim running?")

    # --- ROS 2 / PX4 subscriptions ---
    rclpy.init()
    ros_node = RosNode("vehicle_state_recorder")

    def on_px4_odom(msg):
        seen_frames.add((int(msg.pose_frame), int(msg.velocity_frame)))
        w_px4odom.write([
            msg.timestamp * 1e-6,
            msg.position[0], msg.position[1], msg.position[2],
            msg.q[0], msg.q[1], msg.q[2], msg.q[3],
            msg.velocity[0], msg.velocity[1], msg.velocity[2],
            msg.angular_velocity[0], msg.angular_velocity[1], msg.angular_velocity[2],
            int(msg.pose_frame), int(msg.velocity_frame)])

    def on_px4_input(msg):
        c = msg.control
        w_px4in.write([msg.timestamp * 1e-6, c[0], c[1], c[2], c[3]])

    def on_px4_gps(msg):
        w_gps.write([msg.timestamp * 1e-6, msg.latitude_deg, msg.longitude_deg,
                     msg.altitude_msl_m, msg.vel_n_m_s, msg.vel_e_m_s, msg.vel_d_m_s,
                     int(msg.fix_type)])

    ros_node.create_subscription(VehicleOdometry, "/fmu/out/vehicle_odometry",
                                 on_px4_odom, PX4_QOS)
    ros_node.create_subscription(ActuatorMotors, "/fmu/in/actuator_motors",
                                 on_px4_input, PX4_QOS)
    ros_node.create_subscription(SensorGps, gps_ros_topic, on_px4_gps, PX4_QOS)

    stop_flag = threading.Event()

    def spin_ros():
        while not stop_flag.is_set():
            rclpy.spin_once(ros_node, timeout_sec=0.05)

    spin_thread = threading.Thread(target=spin_ros, daemon=True)
    spin_thread.start()

    t_sim_start = sim_clock.t
    print(f"==> Recording {args.duration} s of SIM time (starting at t_sim={t_sim_start:.3f})")
    print(f"    gz : {odom_topic}")
    print(f"         {imu_topic}")
    print(f"         {gz_actuator_topic}")
    print(f"         {mag_topic}")
    print(f"    ros: /fmu/out/vehicle_odometry, /fmu/in/actuator_motors, {gps_ros_topic}")

    wall_deadline = time.time() + args.duration * args.wall_timeout_factor + 30.0
    try:
        while True:
            now = sim_clock.t
            if now is not None and now - t_sim_start >= args.duration:
                break
            if time.time() > wall_deadline:
                print("WARNING: wall-clock timeout before the sim-time window elapsed "
                      f"(sim advanced {now - t_sim_start:.2f}/{args.duration} s). "
                      "Is the sim paused, or the real-time factor very low?")
                break
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n==> Interrupted -- flushing what was captured so far")

    t_sim_end = sim_clock.t
    stop_flag.set()
    spin_thread.join(timeout=2.0)
    ros_node.destroy_node()
    rclpy.shutdown()
    for w in writers:
        w.close()

    counts = {os.path.basename(w.path): w.count for w in writers}
    print("==> Captured: " + "  ".join(f"{k}={v}" for k, v in counts.items()))

    frames = [{"pose_frame": POSE_FRAME.get(p, str(p)),
               "velocity_frame": VELOCITY_FRAME.get(v, str(v))} for p, v in sorted(seen_frames)]
    check_timebase(run_dir)

    if frames and frames != [{"pose_frame": "NED", "velocity_frame": "NED"}]:
        print(f"WARNING: unexpected PX4 odometry frames {frames} -- this script's ground-"
              "truth conversion assumes pose_frame=NED, velocity_frame=NED "
              "(EKF2.cpp:1687,1694). Check before trusting the overlay.")

    manifest = {
        "recorded_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "sim_time_window": [t_sim_start, t_sim_end],
        "duration_sim_s": (t_sim_end - t_sim_start) if t_sim_end else None,
        "argv": sys.argv,
        "args": vars(args),
        "topics": {
            "clock": clock_topic, "gt_odom": odom_topic, "gt_imu": imu_topic,
            "gz_actuator": gz_actuator_topic,
            "px4_odom": "/fmu/out/vehicle_odometry",
            "px4_actuator": "/fmu/in/actuator_motors",
            "px4_gps": gps_ros_topic, "gz_mag": mag_topic,
        },
        # Stored rather than re-read at plot time, so --plot-only on an old run uses the
        # origin the world actually had when it was recorded.
        "gps_origin": read_world_gps_origin(
            os.path.join(repo_root, "worlds", f"{world}.sdf")),
        "row_counts": counts,
        "px4_odometry_frames_seen": frames,
        "provenance": {
            "px4_git_sha": git_describe(os.path.join(repo_root, "PX4-Autopilot")),
            "model_sdf_sha256_16": file_digest(
                os.path.join(repo_root, "models", "monospinner", "model.sdf")),
            "world_sdf_sha256_16": file_digest(
                os.path.join(repo_root, "worlds", "monospinner_default.sdf")),
            "params_sha256_16": file_digest(
                os.path.join(repo_root, "config", "monospinner.params")),
        },
    }
    with open(os.path.join(run_dir, "manifest.json"), "w") as f:
        json.dump(manifest, f, indent=2)
    print(f"==> Wrote {os.path.join(run_dir, 'manifest.json')}")
    return counts


# ---------------------------------------------------------------------------- plotting ---
def load_csv(path):
    """Reads a StreamWriter CSV into a dict of column arrays. Returns None if the file has
    no data rows -- callers must handle that rather than plot an empty array."""
    if not os.path.exists(path):
        return None
    with open(path) as f:
        rows = list(csv.reader(f))
    if len(rows) < 2:
        return None
    header, data = rows[0], rows[1:]
    arr = np.array(data, dtype=float)
    return {name: arr[:, i] for i, name in enumerate(header)}


def col_stack(d, names):
    return np.stack([d[n] for n in names], axis=1)


def check_timebase(run_dir):
    """Reports whether the gz and PX4 series actually share a timeline.

    A mismatch here is not a subtle degradation -- it is the difference between a
    meaningful overlay and two unrelated plots side by side, and it is silent. The usual
    cause is UXRCE_DDS_SYNCT, see the Clock section of the module docstring.
    """
    gt = load_csv(os.path.join(run_dir, "gt_odom.csv"))
    px = load_csv(os.path.join(run_dir, "px4_odom.csv"))
    if gt is None or px is None:
        return None
    tg, tp = gt["t_sim"], px["t_sim"]
    overlap = min(tg[-1], tp[-1]) - max(tg[0], tp[0])
    if overlap > 0:
        print(f"==> Timebase OK: {overlap:.2f} s of overlap between Gazebo ground truth "
              f"and the PX4 estimate.")
    else:
        print("\n" + "!" * 78)
        print("TIMEBASE MISMATCH -- ground truth and PX4 estimate do not overlap in time.")
        print(f"  gz ground truth : [{tg[0]:.3f}, {tg[-1]:.3f}] s")
        print(f"  PX4 estimate    : [{tp[0]:.3f}, {tp[-1]:.3f}] s")
        print(f"  offset          : {tp[0] - tg[0]:+.3f} s")
        if abs(tp[0]) > 1e8:
            print("  PX4 timestamps look like a Unix wall-clock epoch, so the uXRCE-DDS")
            print("  client is still applying its timesync offset")
            print("  (dds_topics.h.em:126). Set UXRCE_DDS_SYNCT 0 -- it is already in")
            print("  config/monospinner.params -- and RESTART PX4: the parameter is")
            print("  reboot_required, so setting it on a running instance does nothing.")
        print("  The state/error figures below are NOT meaningful until this is fixed.")
        print("!" * 78 + "\n")
    return overlap


def plot_run(run_dir, gt_angular_source):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    gt = load_csv(os.path.join(run_dir, "gt_odom.csv"))
    imu = load_csv(os.path.join(run_dir, "gt_imu.csv"))
    px4 = load_csv(os.path.join(run_dir, "px4_odom.csv"))
    gzact = load_csv(os.path.join(run_dir, "gz_actuator_cmd.csv"))
    px4in = load_csv(os.path.join(run_dir, "px4_actuator_cmd.csv"))
    # Both absent (None) for runs recorded before these sources existed.
    gps = load_csv(os.path.join(run_dir, "px4_gps.csv"))
    mag = load_csv(os.path.join(run_dir, "px4_mag.csv"))
    gps_origin = None
    try:
        with open(os.path.join(run_dir, "manifest.json")) as f:
            gps_origin = json.load(f).get("gps_origin")
    except (OSError, ValueError):
        pass

    fig_dir = os.path.join(run_dir, "figures")
    os.makedirs(fig_dir, exist_ok=True)

    if gt is None or px4 is None:
        print("WARNING: missing ground truth or PX4 odometry -- skipping the state "
              "comparison figure. (Is PX4 publishing? Is the vehicle armed?)")
    else:
        # --- ground truth into PX4's frames, per the Frames table in the docstring ---
        t_gt = gt["t_sim"]
        pos_enu = col_stack(gt, ["x_enu", "y_enu", "z_enu"])
        q_flu_enu = col_stack(gt, ["qw", "qx", "qy", "qz"])
        v_flu = col_stack(gt, ["vx_flu", "vy_flu", "vz_flu"])
        w_flu = col_stack(gt, ["wx_flu", "wy_flu", "wz_flu"])

        n = len(t_gt)
        gt_pos_ned = np.zeros((n, 3))
        gt_q_ned = np.zeros((n, 4))
        gt_vel_ned = np.zeros((n, 3))
        gt_w_frd = np.zeros((n, 3))
        for i in range(n):
            q_i = q_flu_enu[i]
            gt_pos_ned[i], gt_q_ned[i] = gz_pose_to_px4_frame(pos_enu[i], q_i)
            # velocity: body FLU -> world ENU -> world NED, because EKF2 publishes
            # VELOCITY_FRAME_NED (EKF2.cpp:1694). NOT body FRD.
            gt_vel_ned[i] = enu_vec_to_ned(quat_rotate(q_i, v_flu[i]))
            gt_w_frd[i] = gz_body_vec_to_frd(w_flu[i])

        # angular rate reference: imu_gd_truth is Link::WorldAngularVelocity() off the ECM
        # with no noise/bias, so it is true body rate; the odometry twist is
        # pose-differenced at 1 kHz.
        if gt_angular_source == "imu" and imu is not None:
            t_w = imu["t_sim"]
            w_ref_frd = np.stack([gz_body_vec_to_frd(v) for v in
                                  col_stack(imu, ["wx_flu", "wy_flu", "wz_flu"])])
            w_label = "ground truth (IMU, true ECM rate)"
        else:
            t_w, w_ref_frd = t_gt, gt_w_frd
            w_label = "ground truth (odometry, 1 kHz differenced)"

        t_px4 = px4["t_sim"]
        px4_pos = col_stack(px4, ["x_ned", "y_ned", "z_ned"])
        px4_q = col_stack(px4, ["qw", "qx", "qy", "qz"])
        px4_vel = col_stack(px4, ["vx", "vy", "vz"])
        px4_w = col_stack(px4, ["wx_frd", "wy_frd", "wz_frd"])

        gt_rpy = np.array([quat_to_euler_xyz(q) for q in gt_q_ned])
        px4_rpy = np.array([quat_to_euler_xyz(q) for q in px4_q])
        # Unwrap yaw: this body spins continuously by design, so a wrapped yaw plot is a
        # sawtooth that hides the thing you actually want to read off it.
        gt_rpy[:, 2] = np.unwrap(gt_rpy[:, 2])
        px4_rpy[:, 2] = np.unwrap(px4_rpy[:, 2])

        # ONE shared x-axis origin. Both series are already on the sim clock; offsetting by
        # a common t0 is only cosmetic and preserves relative alignment.
        t0 = min(t_gt[0], t_px4[0])

        # --- sensor readings as PX4 sees them (docstring sources 7 and 8) ---
        gps_pos = gps_vel = None
        if gps is not None:
            if gps_origin is None:
                print("WARNING: px4_gps.csv present but manifest.json has no gps_origin -- "
                      "skipping the GPS overlay.")
            else:
                if gps_origin.get("heading_deg", 0.0) != 0.0:
                    print(f"WARNING: world <heading_deg> = {gps_origin['heading_deg']}; the "
                          "GPS -> NED conversion assumes 0, so the GPS overlay is rotated.")
                gps_pos = gps_to_local_ned(gps["lat_deg"], gps["lon_deg"], gps["alt_m"],
                                           gps_origin)
                gps_vel = col_stack(gps, ["vn", "ve", "vd"])
                fixes = sorted(set(gps["fix_type"].astype(int)))
                gps_label = f"GPS (PX4 vehicle_gps_position, fix_type {fixes})"

        # Optional third series per row: (t, values, label) or None.
        rows = [
            ("position (NED, m)", t_gt, gt_pos_ned, t_px4, px4_pos,
             ["x", "y", "z"], "ground truth",
             (gps["t_sim"], gps_pos, gps_label) if gps_pos is not None else None),
            ("attitude (rpy, rad)", t_gt, gt_rpy, t_px4, px4_rpy,
             ["roll", "pitch", "yaw (unwrapped)"], "ground truth", None),
            ("velocity (NED, m/s)", t_gt, gt_vel_ned, t_px4, px4_vel,
             ["vx", "vy", "vz"], "ground truth",
             (gps["t_sim"], gps_vel, gps_label) if gps_vel is not None else None),
            ("angular velocity (body FRD, rad/s)", t_w, w_ref_frd, t_px4, px4_w,
             ["p", "q", "r"], w_label, None),
        ]
        n_rows = len(rows) + (1 if mag is not None else 0)
        fig, axes = plt.subplots(n_rows, 3, figsize=(16, 3 * n_rows), sharex=True)
        for r, (label, tg, g, te, e, comps, glabel, sensor) in enumerate(rows):
            for c in range(3):
                ax = axes[r, c]
                # Sensor first and thin, so the truth/estimate lines stay readable on top.
                if sensor is not None:
                    ts, s, slabel = sensor
                    ax.plot(ts - t0, s[:, c], color="tab:green", lw=0.7, alpha=0.6,
                            label=slabel)
                ax.plot(tg - t0, g[:, c], color="tab:blue", lw=1.2, label=glabel)
                ax.plot(te - t0, e[:, c], color="tab:red", lw=1.2, alpha=0.85,
                        label="PX4 EKF2 estimate")
                ax.set_title(f"{label.split(' (')[0]} {comps[c]}", fontsize=10)
                if c == 0:
                    ax.set_ylabel(label.split("(")[1].rstrip(")"))
                ax.legend(loc="upper right", fontsize=7)

        if mag is not None:
            # No truth/estimate counterpart: this row is the raw reading EKF2 gets for
            # heading. Earth's field is ~0.25-0.65 gauss; a magnitude near the sensor noise
            # floor means there is no heading information in it at all.
            m_xyz = col_stack(mag, ["mx_ga", "my_ga", "mz_ga"])
            norm = np.linalg.norm(m_xyz, axis=1).mean()
            for c, comp in enumerate(["x", "y", "z"]):
                ax = axes[-1, c]
                ax.plot(mag["t_sim"] - t0, m_xyz[:, c], color="tab:green", lw=0.8,
                        label="magnetometer (PX4 sensor_mag axes, raw)")
                ax.axhline(0.0, color="k", lw=0.6, alpha=0.4)
                ax.set_title(f"magnetometer {comp}  (mean |B| = {norm:.3g} gauss)",
                             fontsize=10)
                if c == 0:
                    ax.set_ylabel("gauss")
                ax.legend(loc="upper right", fontsize=7)
        for c in range(3):
            axes[-1, c].set_xlabel("t_sim - t0 (s)")
        fig.suptitle("Vehicle state: ground truth vs PX4 EKF2 estimate\n"
                     "shared simulation clock -- horizontal offsets are real estimator lag")
        fig.tight_layout()
        p = os.path.join(fig_dir, "state.png")
        fig.savefig(p, dpi=130)
        print(f"==> Saved {p}")

        # --- estimator error, only meaningful because the clock is shared ---
        fig, axes = plt.subplots(4, 3, figsize=(16, 11), sharex=True)
        err_rows = [
            ("position error (m)", t_gt, gt_pos_ned, px4_pos, ["x", "y", "z"]),
            ("attitude error (rad)", t_gt, gt_rpy, px4_rpy,
             ["roll", "pitch", "yaw (unwrapped)"]),
            ("velocity error (m/s)", t_gt, gt_vel_ned, px4_vel, ["vx", "vy", "vz"]),
            ("angular velocity error (rad/s)", t_w, w_ref_frd, px4_w, ["p", "q", "r"]),
        ]
        lo, hi = max(t_gt[0], t_px4[0]), min(t_gt[-1], t_px4[-1])
        for r, (label, tg, g, e, comps) in enumerate(err_rows):
            lo_r, hi_r = max(tg[0], t_px4[0]), min(tg[-1], t_px4[-1])
            m = (t_px4 >= lo_r) & (t_px4 <= hi_r)
            tc = t_px4[m]
            for c in range(3):
                ax = axes[r, c]
                if len(tc) < 2:
                    ax.text(0.5, 0.5, "no time overlap", ha="center", va="center",
                            transform=ax.transAxes)
                    continue
                g_i = np.interp(tc, tg, g[:, c])
                d = g_i - e[m, c]
                ax.plot(tc - t0, d, color="tab:purple", lw=1.0)
                ax.axhline(0.0, color="k", lw=0.6, alpha=0.4)
                ax.set_title(f"{label.split(' (')[0]} {comps[c]}  "
                             f"(rms {np.sqrt((d ** 2).mean()):.4g})", fontsize=9)
                if c == 0:
                    ax.set_ylabel(label.split("(")[1].rstrip(")"))
        for c in range(3):
            axes[-1, c].set_xlabel("t_sim - t0 (s)")
        fig.suptitle("Estimator error: ground truth - PX4 EKF2 estimate "
                     "(truth interpolated onto the estimate's timestamps)")
        fig.tight_layout()
        p = os.path.join(fig_dir, "estimator_error.png")
        fig.savefig(p, dpi=130)
        print(f"==> Saved {p}")

    # --- actuator chain: what the controller asked for vs what the plugin received ---
    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    ax = axes[0]
    if px4in is not None:
        t0i = px4in["t_sim"][0]
        for i, lab in enumerate(["control0 (throttle)", "control1 (tilt x)",
                                 "control2 (tilt y)", "control3 (unused)"]):
            ax.plot(px4in["t_sim"] - t0i, px4in[f"control{i}"], lw=1.0, label=lab)
        ax.legend(fontsize=8, ncol=4)
    else:
        ax.text(0.5, 0.5, "no /fmu/in/actuator_motors samples\n"
                          "(is a controller commanding the vehicle?)",
                ha="center", va="center", transform=ax.transAxes)
    ax.set_title("controller -> PX4  (/fmu/in/actuator_motors)")
    ax.set_ylabel("normalised")

    ax = axes[1]
    if gzact is not None:
        t0g = gzact["t_sim"][0]
        for name, lab in [("pos0", "position[0]"), ("pos1", "position[1]"),
                          ("vel0", "velocity[0]")]:
            ax.plot(gzact["t_sim"] - t0g, gzact[name], lw=1.0, label=lab)
        ax.legend(fontsize=8)
        # PX4's ESC interface never populates `position` (GZMixingInterfaceESC.cpp:80-88),
        # while ThrustVectorActuator returns early unless position_size() >= 2
        # (ThrustVectorActuator.cc:157). If that is what the wire shows, say so here
        # rather than let an all-zero physics trace look like a controller problem.
        frac_usable = float((gzact["n_position"] >= 2).mean())
        if frac_usable == 0.0:
            note = ("position[] never populated (n_position < 2 on every message)\n"
                    "ThrustVectorActuator.cc:157 drops ALL of these commands")
        elif frac_usable < 1.0:
            # Two publishers on one topic. The plugin keeps only the most recent message,
            # so whichever publishes faster wins most physics steps -- and PX4's ESC
            # interface runs far faster than a hand-rolled test publisher.
            note = (f"MIXED PUBLISHERS: only {frac_usable * 100:.0f}% of messages have "
                    "position[] populated.\nPX4's ESC interface "
                    "(GZMixingInterfaceESC.cpp:80-88) never sets position[], so those\n"
                    "are dropped at ThrustVectorActuator.cc:157 -- and it out-publishes a "
                    "test exciter,\nso the last-message-wins plugin mostly sees the "
                    "commands that get dropped.")
        else:
            note = None
        if note:
            ax.text(0.5, 0.80, note, ha="center", va="center", transform=ax.transAxes,
                    color="crimson", fontsize=9,
                    bbox=dict(fc="white", ec="crimson", alpha=0.9))
    else:
        ax.text(0.5, 0.5, "no /<model>/command/motor_speed samples",
                ha="center", va="center", transform=ax.transAxes)
    ax.set_title("PX4 -> Gazebo plugin  (/<model>/command/motor_speed)")
    ax.set_xlabel("t_sim - t0 (s)")

    fig.suptitle("Actuator chain")
    fig.tight_layout()
    p = os.path.join(fig_dir, "actuators.png")
    fig.savefig(p, dpi=130)
    print(f"==> Saved {p}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--world", default="monospinner_default")
    ap.add_argument("--model", default="monospinner")
    ap.add_argument("--odom-topic", default=None,
                    help="ground-truth odometry topic; defaults to /model/<model>/odometry, "
                         "matching <odom_topic> in models/monospinner/model.sdf")
    ap.add_argument("--duration", type=float, default=20.0,
                    help="seconds of SIMULATION time to record (not wall clock)")
    ap.add_argument("--odom-decimate", type=int, default=1,
                    help="keep every Nth odometry sample (source is 1 kHz)")
    ap.add_argument("--gt-angular-source", choices=["imu", "odom"], default="imu",
                    help="angular-rate reference: 'imu' is true Link::WorldAngularVelocity() "
                         "off the ECM at 250 Hz (ImuPhysical's imu_gd_truth topic); "
                         "'odom' is pose-differenced at 1 kHz")
    ap.add_argument("--wall-timeout-factor", type=float, default=5.0,
                    help="give up after duration*this + 30 s of wall clock")
    ap.add_argument("--out-dir", default="build/runs",
                    help="parent directory; each run gets a timestamped subdirectory")
    ap.add_argument("--plot-only", default=None,
                    help="skip recording and re-plot an existing run directory")
    args = ap.parse_args()

    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    if args.plot_only:
        plot_run(args.plot_only, args.gt_angular_source)
        return

    stamp = datetime.datetime.now().strftime("%Y-%m-%dT%H-%M-%S")
    run_dir = os.path.join(args.out_dir, stamp)
    os.makedirs(run_dir, exist_ok=True)
    print(f"==> Run directory: {run_dir}")

    record(args, run_dir, repo_root)
    plot_run(run_dir, args.gt_angular_source)
    print(f"\n==> Done. Re-plot without re-flying:\n"
          f"    python3 scripts/record_vehicle_state.py --plot-only {run_dir}")


if __name__ == "__main__":
    main()
