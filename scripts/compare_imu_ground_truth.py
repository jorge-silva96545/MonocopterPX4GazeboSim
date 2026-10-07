#!/usr/bin/env python3
"""Compares ImuPhysical's published IMU reading against ground-truth kinematics
reconstructed purely by subscribing to Gazebo's own topics -- no plugin or
flight-controller code is touched.

Ground truth comes from /world/<world>/dynamic_pose/info (gz.msgs.Pose_V), which the
stock gz-sim-scene-broadcaster-system already publishes (see worlds/monospinner_default.sdf).
This script differentiates that pose sequence -- angular velocity from the quaternion
derivative, linear acceleration from a second position derivative -- to reconstruct the
same physical quantities ImuPhysical reads directly off the ECM
(Link::WorldAngularVelocity()/WorldLinearAcceleration(), see src/gz_imu_realism_plugin/
src/ImuPhysical.cc) before it adds noise/bias.

Caveat: dynamic_pose/info updates at whatever rate the scene broadcaster runs (~50-60 Hz
here, GUI-oriented), far below the 1 kHz physics step or the IMU's own 250 Hz. The
differentiated ground truth is therefore smoother and laggier than what the plugin
actually sees -- treat this as a check on magnitude, sign and general behaviour (spin rate
in the right ballpark, gravity direction correct at rest, etc.), not a sample-for-sample
reference.

Requires a running gz-sim server with the monospinner model already loaded, e.g.:
  gz sim -s -r worlds/monospinner_default.sdf
(standalone, no PX4 needed -- this only talks to Gazebo topics)

Usage:
  python3 scripts/compare_imu_ground_truth.py --duration 8 --output build/imu_ground_truth_check.png
  python3 scripts/compare_imu_ground_truth.py --no-spin   # passive: don't publish anything
"""
import argparse
import os
import sys
import time

import numpy as np

from gz.transport13 import Node
from gz.msgs10.imu_pb2 import IMU
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.actuators_pb2 import Actuators

# sdformat14 1.9 <world><gravity> default (/usr/share/sdformat14/1.9/world.sdf:48).
# worlds/monospinner_default.sdf does not override <gravity>, so this is what the physics
# engine actually uses.
GRAVITY_WORLD = np.array([0.0, 0.0, -9.8])


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nsec * 1e-9


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
    convention: world_vec = q * body_vec * conj(q), matching gz::math::Quaternion::
    RotateVector and ImuPhysical.cc's pose->Rot()/qInv usage)."""
    qv = np.array([0.0, *v])
    return quat_mul(quat_mul(q, qv), quat_conj(q))[1:]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--world", default="monospinner_default")
    ap.add_argument("--model", default="monospinner")
    ap.add_argument("--duration", type=float, default=8.0, help="seconds of data to record")
    ap.add_argument("--no-spin", action="store_true",
                     help="don't publish anything -- purely passive subscription, body "
                          "stays at rest under gravity. Default publishes a steady "
                          "thrust+tilt command so the comparison isn't degenerate; see "
                          "CLAUDE.md's Lockstep trap -- this breaks determinism, it's a "
                          "quick sanity check only, never use it to collect results.")
    ap.add_argument("--thrust-n", type=float, default=4.6,
                     help="commanded thrust in Newtons if spinning (hover is ~4.09 N at "
                          "m_b+m_s+m_p=0.4174 kg per model.sdf)")
    ap.add_argument("--tilt-rad", type=float, default=0.05,
                     help="commanded tilt angle (rad) on both axes if spinning")
    ap.add_argument("--output", default="build/imu_ground_truth_check.png")
    args = ap.parse_args()

    imu_topic = (f"/world/{args.world}/model/{args.model}/link/base_link/sensor/"
                 f"imu_sensor/imu")
    pose_topic = f"/world/{args.world}/dynamic_pose/info"
    actuator_topic = f"/{args.model}/command/motor_speed"

    imu_samples = []   # (t, wx, wy, wz, ax, ay, az)
    pose_samples = []  # (t, x, y, z, qw, qx, qy, qz)

    def on_imu(msg):
        t = stamp_to_sec(msg.header.stamp)
        av, la = msg.angular_velocity, msg.linear_acceleration
        imu_samples.append((t, av.x, av.y, av.z, la.x, la.y, la.z))

    def on_pose(msg):
        t = stamp_to_sec(msg.header.stamp)
        for pose in msg.pose:
            if pose.name == args.model:
                p, o = pose.position, pose.orientation
                pose_samples.append((t, p.x, p.y, p.z, o.w, o.x, o.y, o.z))
                break

    node = Node()
    if not node.subscribe(IMU, imu_topic, on_imu):
        sys.exit(f"error: failed to subscribe to {imu_topic}")
    if not node.subscribe(Pose_V, pose_topic, on_pose):
        sys.exit(f"error: failed to subscribe to {pose_topic}")

    pub = None
    cmd = None
    if not args.no_spin:
        pub = node.advertise(actuator_topic, Actuators)
        cmd = Actuators()
        cmd.position.extend([args.tilt_rad, args.tilt_rad])
        cmd.velocity.extend([args.thrust_n])
        print(f"==> Publishing a steady actuator command on {actuator_topic} "
              f"(thrust={args.thrust_n} N, tilt=({args.tilt_rad}, {args.tilt_rad}) rad) "
              f"-- sanity-check only, see CLAUDE.md's Lockstep trap.")

    print(f"==> Recording {args.duration}s from {imu_topic} and {pose_topic}")
    t_start = time.time()
    while time.time() - t_start < args.duration:
        if pub is not None:
            pub.publish(cmd)
        time.sleep(0.02)

    if pub is not None:
        cmd.velocity[0] = 0.0
        pub.publish(cmd)

    print(f"==> Captured {len(imu_samples)} IMU samples, {len(pose_samples)} pose samples")
    if len(imu_samples) < 10 or len(pose_samples) < 10:
        sys.exit("error: too few samples captured -- is the gz-sim server actually running "
                  "with the monospinner model loaded?")

    # --- Ground truth: differentiate the pose sequence ---
    pose_arr = np.array(sorted(pose_samples, key=lambda r: r[0]))
    t_p = pose_arr[:, 0]
    pos = pose_arr[:, 1:4]
    quat = pose_arr[:, 4:8]  # w, x, y, z

    n = len(t_p)
    gt_t = t_p[1:-1]
    ang_vel_body = np.zeros((n - 2, 3))
    spec_force_body = np.zeros((n - 2, 3))
    for i in range(1, n - 1):
        dt1 = t_p[i] - t_p[i - 1]
        dt2 = t_p[i + 1] - t_p[i]
        if dt1 <= 0 or dt2 <= 0:
            continue
        q_i = quat[i]

        qdot = (quat[i + 1] - quat[i - 1]) / (dt1 + dt2)
        omega_quat = 2.0 * quat_mul(quat_conj(q_i), qdot)
        ang_vel_body[i - 1] = omega_quat[1:]

        v_prev = (pos[i] - pos[i - 1]) / dt1
        v_next = (pos[i + 1] - pos[i]) / dt2
        a_world = 2.0 * (v_next - v_prev) / (dt1 + dt2)
        specific_force_world = a_world - GRAVITY_WORLD
        spec_force_body[i - 1] = quat_rotate(quat_conj(q_i), specific_force_world)

    # --- Measured: straight from the IMU topic ---
    imu_arr = np.array(sorted(imu_samples, key=lambda r: r[0]))
    t_imu = imu_arr[:, 0]
    gyro_meas = imu_arr[:, 1:4]
    accel_meas = imu_arr[:, 4:7]

    t0, t1 = max(gt_t[0], t_imu[0]), min(gt_t[-1], t_imu[-1])
    mask = (t_imu >= t0) & (t_imu <= t1)
    t_cmp = t_imu[mask]
    gyro_meas_cmp = gyro_meas[mask]
    accel_meas_cmp = accel_meas[mask]
    if len(t_cmp) < 2:
        sys.exit("error: no time overlap between IMU and pose samples")

    gyro_gt_cmp = np.stack([np.interp(t_cmp, gt_t, ang_vel_body[:, i]) for i in range(3)],
                            axis=1)
    accel_gt_cmp = np.stack([np.interp(t_cmp, gt_t, spec_force_body[:, i]) for i in range(3)],
                             axis=1)
    t_rel = t_cmp - t_cmp[0]

    for name, arr in [("gyro (measured)", gyro_meas_cmp), ("gyro (ground truth)", gyro_gt_cmp),
                       ("accel (measured)", accel_meas_cmp),
                       ("accel (ground truth)", accel_gt_cmp)]:
        if not np.all(np.isfinite(arr)):
            print(f"WARNING: non-finite values in {name}")

    print("\n--- mean value, per axis ---")
    for label, meas, gt, unit in [("gyro", gyro_meas_cmp, gyro_gt_cmp, "rad/s"),
                                   ("accel", accel_meas_cmp, accel_gt_cmp, "m/s^2")]:
        for i, axis in enumerate("xyz"):
            print(f"  {label}.{axis}: measured={meas[:, i].mean():+.4f} "
                  f"ground_truth={gt[:, i].mean():+.4f} {unit}")

    print("\n--- residual (measured - ground truth), per axis ---")
    for label, meas, gt, unit in [("gyro", gyro_meas_cmp, gyro_gt_cmp, "rad/s"),
                                   ("accel", accel_meas_cmp, accel_gt_cmp, "m/s^2")]:
        resid = meas - gt
        for i, axis in enumerate("xyz"):
            print(f"  {label}.{axis}: mean={resid[:, i].mean():+.4f} "
                  f"std={resid[:, i].std():.4f} {unit}")

    # --- Plot ---
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 3, figsize=(15, 7), sharex=True)
    gyro_labels = ["gyro x", "gyro y", "gyro z"]
    accel_labels = ["accel x", "accel y", "accel z"]
    for i in range(3):
        ax = axes[0, i]
        ax.plot(t_rel, gyro_gt_cmp[:, i], color="tab:blue", lw=1.5, label="ground truth")
        ax.plot(t_rel, gyro_meas_cmp[:, i], color="tab:orange", lw=0.8, alpha=0.8,
                 label="IMU measured")
        ax.set_title(gyro_labels[i])
        ax.set_ylabel("rad/s")
        if i == 0:
            ax.legend(loc="upper right", fontsize=8)

        ax = axes[1, i]
        ax.plot(t_rel, accel_gt_cmp[:, i], color="tab:blue", lw=1.5, label="ground truth")
        ax.plot(t_rel, accel_meas_cmp[:, i], color="tab:orange", lw=0.8, alpha=0.8,
                 label="IMU measured")
        ax.set_title(accel_labels[i])
        ax.set_ylabel("m/s^2")
        ax.set_xlabel("t (s)")

    fig.suptitle("ImuPhysical: real specific force / angular velocity vs. IMU output\n"
                  "(ground truth = differentiated dynamic_pose/info, subscribe-only)")
    fig.tight_layout()
    os.makedirs(os.path.dirname(args.output) or ".", exist_ok=True)
    fig.savefig(args.output, dpi=130)
    print(f"\n==> Saved plot to {args.output}")


if __name__ == "__main__":
    main()
