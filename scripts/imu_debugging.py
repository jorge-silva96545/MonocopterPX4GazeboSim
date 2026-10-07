#!/usr/bin/env python3
"""Plots ImuPhysical's noisy IMU output against its own noise-free ground-truth IMU.

ImuPhysical (src/gz_imu_realism_plugin/src/ImuPhysical.cc) publishes two gz.msgs.IMU
messages per update, with identical header stamps:
  - the noisy reading PX4 consumes, on the <topic> set in models/monospinner/model.sdf;
  - the same body-frame angular velocity and specific force before bias/noise/saturation,
    on <ground_truth_topic> (only when <publish_ground_truth> is true).
Because the stamps match, samples are paired exactly by timestamp, not interpolated.

Passive: subscribes only, publishes nothing, so it does not disturb a lockstepped run.

Requires a running gz-sim server with the monospinner model loaded, e.g.:
  gz sim -s -r worlds/monospinner_default.sdf

Usage:
  python3 scripts/imu_debugging.py --duration 8 --output build/imu_debugging.png
"""
import argparse
import os
import sys
import time

import numpy as np

from gz.transport13 import Node
from gz.msgs10.imu_pb2 import IMU


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nsec * 1e-9


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--world", default="monospinner_default")
    ap.add_argument("--model", default="monospinner")
    ap.add_argument("--duration", type=float, default=8.0, help="seconds of data to record")
    ap.add_argument("--output", default="build/imu_debugging.png")
    args = ap.parse_args()

    # Both must match models/monospinner/model.sdf's ImuPhysical <topic> and
    # <ground_truth_topic>.
    imu_topic = (f"/world/{args.world}/model/{args.model}/link/base_link/sensor/"
                 f"imu_sensor/imu")
    gt_topic = f"/model/{args.model}/imu_gd_truth"

    # stamp -> (wx, wy, wz, ax, ay, az)
    imu_samples = {}
    gt_samples = {}

    def make_callback(store):
        def on_msg(msg):
            av, la = msg.angular_velocity, msg.linear_acceleration
            store[stamp_to_sec(msg.header.stamp)] = (av.x, av.y, av.z, la.x, la.y, la.z)
        return on_msg

    node = Node()
    for topic, store in [(imu_topic, imu_samples), (gt_topic, gt_samples)]:
        if not node.subscribe(IMU, topic, make_callback(store)):
            sys.exit(f"error: failed to subscribe to {topic}")

    print(f"==> Recording {args.duration}s from {imu_topic} and {gt_topic}")
    time.sleep(args.duration)
    # Stop the callbacks before the Node is torn down at interpreter exit; otherwise
    # gz-transport's subscriber thread can abort the process ("terminate called without an
    # active exception").
    for topic in (imu_topic, gt_topic):
        node.unsubscribe(topic)

    print(f"==> Captured {len(imu_samples)} IMU samples, {len(gt_samples)} ground-truth "
          f"samples")
    stamps = sorted(set(imu_samples) & set(gt_samples))
    if len(stamps) < 10:
        sys.exit("error: too few matched samples -- is the gz-sim server running with the "
                  "monospinner model loaded and <publish_ground_truth> set to true?")

    t = np.array(stamps)
    meas = np.array([imu_samples[s] for s in stamps])
    gt = np.array([gt_samples[s] for s in stamps])
    t_rel = t - t[0]

    print("\n--- residual (measured - ground truth), per axis ---")
    resid = meas - gt
    for j, (label, unit) in enumerate([("gyro", "rad/s"), ("accel", "m/s^2")]):
        for i, axis in enumerate("xyz"):
            r = resid[:, 3 * j + i]
            print(f"  {label}.{axis}: mean={r.mean():+.5f} std={r.std():.5f} {unit}")

    # --- Plot ---
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 3, figsize=(15, 7), sharex=True)
    for j, (label, unit) in enumerate([("gyro", "rad/s"), ("accel", "m/s^2")]):
        for i, axis in enumerate("xyz"):
            ax = axes[j, i]
            ax.plot(t_rel, meas[:, 3 * j + i], color="tab:orange", lw=0.8, alpha=0.8,
                    label="IMU measured")
            ax.plot(t_rel, gt[:, 3 * j + i], color="tab:blue", lw=1.5, label="ground truth")
            ax.set_title(f"{label} {axis}")
            ax.set_ylabel(unit)
            if j == 1:
                ax.set_xlabel("t (s)")
            if j == 0 and i == 0:
                ax.legend(loc="upper right", fontsize=8)

    fig.suptitle("ImuPhysical: IMU output vs. noise-free ground truth (body frame)")
    fig.tight_layout()
    os.makedirs(os.path.dirname(args.output) or ".", exist_ok=True)
    fig.savefig(args.output, dpi=130)
    print(f"\n==> Saved plot to {args.output}")


if __name__ == "__main__":
    main()
