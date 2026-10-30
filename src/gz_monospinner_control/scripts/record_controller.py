#!/usr/bin/env python3
"""Records MonospinnerController's debug topic next to Gazebo ground truth, then plots them.

Sources (all on sim time):
  /<model>/controller/debug  gz.msgs.Double_V, one message per control update; index layout
                             in src/MonospinnerController.cc's header comment
  /model/<model>/odometry    gz.msgs.Odometry ground truth: pose world ENU, twist body FLU
                             (see analysis/frames.py)

Writes controller.csv and controller_gt.csv into --run-dir and figures/controller_*.png:
  controller_overview.png        controller input vs truth, commands, estimate age, position
  controller_truth_vs_ekf2.png   PX4 EKF2 vs truth (needs px4_odom.csv from
                                 scripts/record_vehicle_state.py in the same run dir; made by
                                 --plot-only once both recorders have finished)

Usage:
  record_controller.py --duration 25 --run-dir build/runs/<run>
  record_controller.py --plot-only build/runs/<run>
"""
import argparse
import csv
import os
import sys
import threading
import time

import numpy as np

DEBUG_FIELDS = (["t_sim", "t_sample", "age", "phase", "omega", "alpha", "beta", "gamma_dot",
                 "thrust"]
                + [f"{v}_{a}" for v in ("n", "nbar", "w", "wr", "tau") for a in "xyz"]
                + ["singular", "qw", "qx", "qy", "qz"])
GT_FIELDS = ["t_sim", "x", "y", "z", "qw", "qx", "qy", "qz", "wx", "wy", "wz"]


def record(args):
    from gz.msgs10.clock_pb2 import Clock
    from gz.msgs10.double_v_pb2 import Double_V
    from gz.msgs10.odometry_pb2 import Odometry
    from gz.transport13 import Node

    lock = threading.Lock()
    state = {"t": None, "debug": [], "gt": [], "n_odom": 0, "on": True}

    def on_clock(msg):
        with lock:
            state["t"] = msg.sim.sec + msg.sim.nsec * 1e-9

    def on_debug(msg):
        with lock:
            if state["on"] and len(msg.data) == len(DEBUG_FIELDS):
                state["debug"].append(list(msg.data))

    def on_odom(msg):
        with lock:
            state["n_odom"] += 1
            if not state["on"] or state["n_odom"] % args.odom_decimate:
                return
            t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
            p, o, w = msg.pose.position, msg.pose.orientation, msg.twist.angular
            state["gt"].append([t, p.x, p.y, p.z, o.w, o.x, o.y, o.z, w.x, w.y, w.z])

    node = Node()
    node.subscribe(Clock, f"/world/{args.world}/clock", on_clock)
    node.subscribe(Double_V, f"/{args.model}/controller/debug", on_debug)
    node.subscribe(Odometry, f"/model/{args.model}/odometry", on_odom)

    deadline = time.time() + 30
    while state["t"] is None:
        if time.time() > deadline:
            sys.exit("error: no /clock -- is gz running?")
        time.sleep(0.05)
    t0 = state["t"]
    wall_deadline = time.time() + args.duration * args.wall_timeout_factor + 10
    while state["t"] < t0 + args.duration and time.time() < wall_deadline:
        time.sleep(0.05)
    with lock:
        state["on"] = False
        debug, gt = list(state["debug"]), list(state["gt"])

    for name, header, rows in (("controller.csv", DEBUG_FIELDS, debug),
                               ("controller_gt.csv", GT_FIELDS, gt)):
        with open(os.path.join(args.run_dir, name), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(header)
            w.writerows(rows)
    print(f"==> record_controller: {len(debug)} controller rows, {len(gt)} ground-truth rows")


def load(path):
    with open(path) as f:
        r = csv.reader(f)
        header = next(r)
        data = np.array([[float(x) for x in row] for row in r])
    if data.size == 0:
        data = np.zeros((0, len(header)))
    return {h: data[:, i] for i, h in enumerate(header)}


def body_up(qw, qx, qy, qz):
    """Third column of R(q): the body z axis in world, for (w, x, y, z) quaternions."""
    return np.stack([2 * (qx * qz + qw * qy), 2 * (qy * qz - qw * qx),
                     1 - 2 * (qx * qx + qy * qy)], 1)


def plot(run_dir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    c = load(os.path.join(run_dir, "controller.csv"))
    g = load(os.path.join(run_dir, "controller_gt.csv"))
    if len(c["t_sim"]) == 0 or len(g["t_sim"]) == 0:
        sys.exit("error: empty recording")
    t0 = c["t_sim"][0]
    tc, tg = c["t_sim"] - t0, g["t_sim"] - t0

    n_est = np.stack([c["n_x"], c["n_y"], c["n_z"]], 1)
    n_bar = np.stack([c["nbar_x"], c["nbar_y"], c["nbar_z"]], 1)
    n_true_g = body_up(g["qw"], g["qx"], g["qy"], g["qz"])
    n_true = np.stack([np.interp(tc, tg, n_true_g[:, i]) for i in range(3)], 1)
    n_true /= np.linalg.norm(n_true, axis=1, keepdims=True)

    def tilt(n):
        return np.degrees(np.arccos(np.clip(n[:, 2], -1, 1)))

    est_err = np.degrees(np.arccos(np.clip(np.sum(n_est * n_true, 1), -1, 1)))
    trk_err = np.degrees(np.arccos(np.clip(np.sum(n_true * n_bar, 1), -1, 1)))
    flying = c["phase"] >= 1
    # Ground-truth input is used in the same step it is read: age exactly 0.
    source = "ground truth" if np.all(c["age"] == 0.0) else "EKF2"
    src_lbl = "ground truth" if source == "ground truth" else "EKF2, predicted"

    fig, ax = plt.subplots(4, 2, figsize=(16, 16), sharex=True)
    a = ax[0, 0]
    a.plot(tc, tilt(n_true), lw=2, label="true (Gazebo)")
    a.plot(tc, tilt(n_est), "--", label=f"controller input ({src_lbl})")
    a.plot(tc, tilt(n_bar), "k:", label="setpoint")
    a.set_ylabel("tilt [deg]")
    a.legend()
    a = ax[0, 1]
    for i, lbl in ((0, "x (E)"), (1, "y (N)")):
        a.plot(tc, n_true[:, i], lw=2, label=f"n_{lbl} true")
        a.plot(tc, n_est[:, i], "--", label=f"n_{lbl} input")
        a.plot(tc, n_bar[:, i], ":", label=f"n_{lbl} setpoint")
    a.set_ylabel("body up axis, world")
    a.legend(fontsize=8, ncol=3)
    a = ax[1, 0]
    a.plot(tc, trk_err, label="tracking error |n_true vs n_bar|")
    a.plot(tc, est_err, label="input error |n_input vs n_true|")
    a.set_ylabel("angle [deg]")
    a.legend()
    a = ax[1, 1]
    a.plot(tc, c["age"] * 1e3, lw=0.8)
    if np.any(flying):
        mean_age = 1e3 * c["age"][flying].mean()
        a.axhline(mean_age, color="k", ls=":", label=f"mean in flight {mean_age:.1f} ms")
        a.legend()
    a.set_ylabel("estimate age at use [ms]")
    a = ax[2, 0]
    a.plot(tc, np.degrees(c["alpha"]), label="alpha")
    a.set_ylabel("alpha [deg]")
    a2 = a.twinx()
    a2.plot(tc, c["thrust"], color="C1", label="thrust")
    a2.set_ylabel("thrust [N]", color="C1")
    a.legend(loc="upper left")
    a = ax[2, 1]
    for i, k in enumerate("xyz"):
        a.plot(tc, np.interp(tc, tg, g[f"w{k}"]), lw=2, color=f"C{i}", label=f"w{k} true")
        a.plot(tc, c[f"w_{k}"], "--", color=f"C{i}", label=f"w{k} input")
    a.set_ylabel("body rate, FLU [rad/s]")
    a.legend(fontsize=8, ncol=2)
    a = ax[3, 0]
    a.plot(tg, g["z"], label="z (up)")
    a.plot(tg, g["x"], label="x (E)")
    a.plot(tg, g["y"], label="y (N)")
    a.set_ylabel("position, world ENU [m]")
    a.set_xlabel("sim time since controller start [s]")
    a.legend()
    a = ax[3, 1]
    a.plot(tc, c["phase"], label="phase (0 wait, 1/2 takeoff, 3 hover, -1 timeout)")
    a.plot(tc, c["singular"], label="singular")
    a.set_xlabel("sim time since controller start [s]")
    a.legend()
    for a in ax.flat:
        a.grid(alpha=0.3)
    fig.suptitle(f"MonospinnerController (paper Sec. 6), controller input: {source}")
    fig.tight_layout()
    os.makedirs(os.path.join(run_dir, "figures"), exist_ok=True)
    out = os.path.join(run_dir, "figures", "controller_overview.png")
    fig.savefig(out, dpi=100)
    plt.close(fig)
    print(f"==> record_controller: plot {out}")
    if np.any(flying):
        print(f"    in flight ({source} input): input age mean {1e3 * c['age'][flying].mean():.1f} ms, "
              f"max {1e3 * c['age'][flying].max():.1f} ms; input error mean "
              f"{est_err[flying].mean():.2f} deg, max {est_err[flying].max():.2f} deg")


Q_ENU_NED = np.array([0.0, np.sqrt(0.5), np.sqrt(0.5), 0.0])  # 180 deg about (1,1,0)
Q_FRD_FLU = np.array([0.0, 1.0, 0.0, 0.0])                      # 180 deg about x


def quat_mul(a, b):
    """Hamilton product of (N,4) or (4,) (w, x, y, z) quaternions."""
    aw, ax, ay, az = np.moveaxis(np.asarray(a, float), -1, 0)
    bw, bx, by, bz = np.moveaxis(np.asarray(b, float), -1, 0)
    return np.stack([aw * bw - ax * bx - ay * by - az * bz,
                     aw * bx + ax * bw + ay * bz - az * by,
                     aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw], -1)


def plot_estimate(run_dir):
    """PX4 EKF2 (px4_odom.csv, from scripts/record_vehicle_state.py) against Gazebo ground
    truth (controller_gt.csv), in the controller's frames: world ENU, body FLU. EKF2's
    body(FRD)->NED attitude is converted with the inverse of analysis/frames.py:
    gz_quat_to_px4(), as in MonospinnerController.cc."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    px4_path = os.path.join(run_dir, "px4_odom.csv")
    if not os.path.exists(px4_path):
        print(f"==> record_controller: no {px4_path}; skipping the EKF2 figure")
        return
    c = load(os.path.join(run_dir, "controller.csv"))
    g = load(os.path.join(run_dir, "controller_gt.csv"))
    e = load(px4_path)
    t0 = c["t_sim"][0]
    tg, te = g["t_sim"] - t0, e["t_sim"] - t0
    keep = (te >= tg[0]) & (te <= tg[-1])
    te = te[keep]
    e = {k: v[keep] for k, v in e.items()}
    source = "ground truth" if np.all(c["age"] == 0.0) else "EKF2"

    q_true = np.stack([g["qw"], g["qx"], g["qy"], g["qz"]], 1)
    q_ekf = quat_mul(quat_mul(Q_ENU_NED, np.stack([e["qw"], e["qx"], e["qy"], e["qz"]], 1)),
                     Q_FRD_FLU)
    n_true_g = body_up(*q_true.T)
    n_ekf = body_up(*q_ekf.T)
    n_true = np.stack([np.interp(te, tg, n_true_g[:, i]) for i in range(3)], 1)
    n_true /= np.linalg.norm(n_true, axis=1, keepdims=True)
    tilt_err = np.degrees(np.arccos(np.clip(np.sum(n_true * n_ekf, 1), -1, 1)))

    # Spin angle theta_s = 2 atan2(qz, qw) (paper Eq. 55), unwrapped; the heading error is
    # wrapped to +-180 deg.
    spin_true = np.unwrap(2 * np.arctan2(g["qz"], g["qw"]))
    spin_ekf = np.unwrap(2 * np.arctan2(q_ekf[:, 3], q_ekf[:, 0]))
    spin_err = np.degrees(np.angle(np.exp(1j * (spin_ekf - np.interp(te, tg, spin_true)))))

    w_ekf = np.stack([e["wx_frd"], -e["wy_frd"], -e["wz_frd"]], 1)
    # EKF2 position is NED about its own origin; express it as ENU and shift it onto the
    # true start position so both curves share one origin.
    p_ekf = np.stack([e["y_ned"], e["x_ned"], -e["z_ned"]], 1)
    p_true0 = np.array([np.interp(te[0], tg, g[k]) for k in ("x", "y", "z")])
    p_ekf = p_ekf - p_ekf[0] + p_true0

    phase_t = {}
    for ph, lbl in ((1, "takeoff"), (3, "hover")):
        idx = np.where(c["phase"] >= ph)[0]
        if len(idx):
            phase_t[lbl] = c["t_sim"][idx[0]] - t0

    def tilt(n):
        return np.degrees(np.arccos(np.clip(n[:, 2], -1, 1)))

    fig, ax = plt.subplots(4, 2, figsize=(16, 16), sharex=True)
    a = ax[0, 0]
    a.plot(tg, tilt(n_true_g), lw=2, label="true (Gazebo)")
    a.plot(te, tilt(n_ekf), "--", label="PX4 EKF2")
    a.set_ylabel("tilt [deg]")
    a.legend()
    a = ax[0, 1]
    a.plot(te, tilt_err)
    a.set_ylabel("EKF2 tilt error: angle(n_EKF2, n_true) [deg]")
    a = ax[1, 0]
    for i, lbl in ((0, "x (E)"), (1, "y (N)")):
        a.plot(tg, n_true_g[:, i], lw=2, color=f"C{i}", label=f"n_{lbl} true")
        a.plot(te, n_ekf[:, i], "--", color=f"C{i}", label=f"n_{lbl} EKF2")
    a.set_ylabel("body up axis, world ENU")
    a.legend(fontsize=8, ncol=2)
    a = ax[1, 1]
    a.plot(te, spin_err)
    a.set_ylabel("EKF2 spin-angle (heading) error, wrapped [deg]")
    a = ax[2, 0]
    for i, k in enumerate("xyz"):
        a.plot(tg, g[f"w{k}"], lw=2, color=f"C{i}", label=f"w{k} true")
        a.plot(te, w_ekf[:, i], "--", color=f"C{i}", label=f"w{k} EKF2")
    a.set_ylabel("body rate, FLU [rad/s]")
    a.legend(fontsize=8, ncol=3)
    a = ax[2, 1]
    for i, k in enumerate("xyz"):
        a.plot(te, w_ekf[:, i] - np.interp(te, tg, g[f"w{k}"]), color=f"C{i}", label=f"w{k}")
    a.set_ylabel("EKF2 body-rate error, FLU [rad/s]")
    a.legend()
    a = ax[3, 0]
    a.plot(tg, g["z"], lw=2, color="C2", label="z (up) true")
    a.plot(te, p_ekf[:, 2], "--", color="C2", label="z (up) EKF2")
    a.set_ylabel("altitude, world [m]")
    a.set_xlabel("sim time since controller start [s]")
    a.legend()
    a = ax[3, 1]
    for i, k in ((0, "x"), (1, "y")):
        a.plot(tg, g[k], lw=2, color=f"C{i}", label=f"{k} true")
        a.plot(te, p_ekf[:, i], "--", color=f"C{i}", label=f"{k} EKF2")
    a.set_ylabel("horizontal position, world ENU [m]")
    a.set_xlabel("sim time since controller start [s]")
    a.legend()
    for a in ax.flat:
        a.grid(alpha=0.3)
        for lbl, tv in phase_t.items():
            a.axvline(tv, color="grey", ls=":", alpha=0.8)
    fig.suptitle(f"Ground truth vs PX4 EKF2 estimate (controller input: {source}); dotted lines: "
                 + ", ".join(f"{k} at {v:.1f} s" for k, v in phase_t.items()))
    fig.tight_layout()
    os.makedirs(os.path.join(run_dir, "figures"), exist_ok=True)
    out = os.path.join(run_dir, "figures", "controller_truth_vs_ekf2.png")
    fig.savefig(out, dpi=100)
    plt.close(fig)
    print(f"==> record_controller: plot {out}")
    t_hover = phase_t.get("hover", 0.0)
    for a_, b_ in ((t_hover, t_hover + 2), (t_hover + 2, t_hover + 5),
                   (t_hover + 5, t_hover + 10), (t_hover + 10, te[-1])):
        s = (te >= a_) & (te < b_)
        if s.any():
            print(f"    t {a_:5.1f}-{b_:5.1f} s: EKF2 tilt error mean {tilt_err[s].mean():6.2f} "
                  f"max {tilt_err[s].max():6.2f} deg; |heading error| mean "
                  f"{np.abs(spin_err[s]).mean():6.1f} deg; true tilt max "
                  f"{tilt(n_true)[s].max():5.2f} deg")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--duration", type=float, default=25.0, help="sim s to record")
    ap.add_argument("--run-dir", default=None)
    ap.add_argument("--world", default="monospinner_default")
    ap.add_argument("--model", default="monospinner")
    ap.add_argument("--odom-decimate", type=int, default=4)
    ap.add_argument("--wall-timeout-factor", type=float, default=5.0)
    ap.add_argument("--plot-only", default=None, metavar="RUN_DIR")
    args = ap.parse_args()
    if args.plot_only:
        plot(args.plot_only)
        plot_estimate(args.plot_only)
        return
    if not args.run_dir:
        sys.exit("error: --run-dir is required when recording")
    os.makedirs(args.run_dir, exist_ok=True)
    record(args)
    plot(args.run_dir)


if __name__ == "__main__":
    code = 0
    try:
        main()
    except SystemExit as e:
        if isinstance(e.code, str):
            print(e.code, file=sys.stderr)
            code = 1
        else:
            code = e.code or 0
    except BaseException:  # noqa: BLE001
        import traceback
        traceback.print_exc()
        code = 1
    sys.stdout.flush()
    sys.stderr.flush()
    # gz-transport callback threads can abort interpreter teardown (see
    # scripts/wrench_player.py); every file is closed by now.
    os._exit(code)
