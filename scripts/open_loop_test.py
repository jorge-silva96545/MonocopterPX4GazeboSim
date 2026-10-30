#!/usr/bin/env python3
"""Open-loop physics test of ThrustVectorActuator, with PX4 out of the loop.

Launches gz-server alone on worlds/monospinner_default.sdf (the world <include>s the model,
so no PX4 is needed to spawn it), publishes a hand-made command timeline on the plugin's
input topic, records the vehicle's ground-truth state and the commands sent, then kills
gz and plots everything together with the analytic predictions for this test.

Nothing else publishes on the command topic because PX4 is not running, so the plugin
sees exactly what this script sends.

Command message (what ThrustVectorActuator::PreUpdate currently reads, see
src/gz_monospinner_plugin/src/ThrustVectorActuator.cc):
  topic     /<model>/command/motor_speed  (gz.msgs.Actuators)
  position  [alpha, beta]   rad   -> Quaterniond(0, alpha, beta) = Rz(beta) Ry(alpha), paper Eq. 12
  velocity  [T]             N     -> thrust magnitude, used directly as a force
Both fields must be populated: the plugin returns early unless position_size() >= 2 and
velocity_size() >= 1.

Profile (sim seconds):
  settle  T = 0                  vehicle rests on the ground
  thrust  T = (1 + margin) m g   alpha = beta = 0: vertical climb + spin-up from drag torque
  tilt    same T, alpha/beta     only with --alpha-deg != 0: a constant tilt command, applied
                                 once the vehicle is airborne and spinning. alpha/beta are
                                 BODY-frame (the plugin applies them as given; nothing here
                                 compensates for the spin), so the tilt moment turns with the
                                 body and the vehicle cones rather than leaning one way.

Recorded sources (all stamped with sim time):
  /model/<model>/odometry      gz.msgs.Odometry, 1 kHz (OdometryPublisher in model.sdf):
                               pose world ENU, BOTH twist components body FLU
                               (see analysis/frames.py)
  /model/<model>/imu_gd_truth  gz.msgs.IMU, 250 Hz (ImuPhysical ground truth): body rate and
                               specific force, body FLU, no noise or bias
  commands                     logged here as they are published, stamped with /clock

Predictions drawn on the plots (thrust phase, alpha = 0):
  vertical acceleration  a_z = (T - m g) / m = margin * g      after lift-off
  body specific force    f_z = T / m                           while airborne
  steady spin rate       w_z = +sqrt((k_tau / k_f) * T / K_z)  drag torque (+k*T*r, paper
                               Eq. 20/26) balanced by fin drag (Eq. 31-32); the sign is the
                               paper's convention as implemented in the plugin today

Prediction for the tilt phase (second figure, open_loop_tilt.png):
  The rotation about the composite CoM does not depend on the translation (gravity has no
  moment about the CoM, and every torque the plugin applies depends only on the command and
  the body rate), so it is integrated here directly, in the body frame, from the MEASURED
  state at tilt onset, with the torques ThrustVectorActuator applies:
    J w_dot = T d (e_z x r) + k T r + Omega J_pz (w x r) - K_z w_z |w_z| e_z - w x (J w)
    r = (s_alpha c_beta, s_alpha s_beta, c_alpha), Omega = sqrt(T / k_f), k = k_tau / k_f
  J is the composite inertia about the composite CoM and d the height of the rotor link's
  centre of mass above the composite CoM (AddWorldForce applies at the link's CoM), both
  from model.sdf. It is integrated twice, with and without the gyroscopic term, so the
  measured curve shows whether that term is present and has the right sign and size.
  This checks that the plugin applies the intended torques; it is not the paper's own
  simulator (which works about F_B with the Delta_m coupling terms instead).

This is an asynchronous publisher next to a real-time simulation: fine for a sanity check,
not for collecting results (CLAUDE.md, "Lockstep").

Usage:
  python3 scripts/open_loop_test.py                     # margin 0.1, 2 s settle, 5 s thrust
  python3 scripts/open_loop_test.py --margin 0.2 --thrust-s 3
  python3 scripts/open_loop_test.py --alpha-deg 2       # + 1 s of 2 deg tilt at beta = 0
  python3 scripts/open_loop_test.py --alpha-deg 2 --beta-deg 90 --tilt-s 0.5
  python3 scripts/open_loop_test.py --gui
  python3 scripts/open_loop_test.py --plot-only build/runs/<run>
"""
import argparse
import csv
import datetime
import glob
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
import xml.etree.ElementTree as ET

import numpy as np

from gz.msgs10.actuators_pb2 import Actuators
from gz.msgs10.clock_pb2 import Clock
from gz.msgs10.imu_pb2 import IMU
from gz.msgs10.odometry_pb2 import Odometry
from gz.transport13 import Node

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))
from wrench_player import mass_properties, world_gravity  # noqa: E402

PLUGIN_NAME = "monospinner::ThrustVectorActuator"


# --------------------------------------------------------------------------- parameters ---
def plugin_params(sdf_path):
    """The ThrustVectorActuator <plugin> block's numeric parameters, as the plugin reads them."""
    model = ET.parse(sdf_path).getroot().find("model")
    for plugin in model.findall("plugin"):
        if plugin.get("name") == PLUGIN_NAME:
            out = {}
            for child in plugin:
                try:
                    out[child.tag] = float(child.text)
                except (TypeError, ValueError):
                    pass
            return out
    sys.exit(f"error: no <plugin name=\"{PLUGIN_NAME}\"> in {sdf_path}")


def rotor_lever_arm(sdf_path, mp):
    """Height of the rotor link's centre of mass above the composite CoM [m]. The plugin's
    AddWorldForce applies the thrust at that link's CoM, so this is the thrust lever arm."""
    model = ET.parse(sdf_path).getroot().find("model")
    rotor_name = None
    for plugin in model.findall("plugin"):
        if plugin.get("name") == PLUGIN_NAME:
            rotor_name = plugin.findtext("rotor_visual_link")
    xyz = {}
    for link in model.findall("link"):
        lp = [float(v) for v in (link.findtext("pose") or "0 0 0 0 0 0").split()]
        ip = [float(v) for v in (link.find("inertial").findtext("pose") or "0 0 0 0 0 0").split()]
        xyz[link.get("name")] = [lp[i] + ip[i] for i in range(3)]
    if rotor_name not in xyz:
        sys.exit(f"error: rotor link '{rotor_name}' not found in {sdf_path}")
    rotor = xyz[rotor_name]
    if abs(rotor[0]) > 1e-9 or abs(rotor[1]) > 1e-9:
        sys.exit("error: rotor link is off the body z axis; the tilt prediction assumes it is on it")
    base_link_pose = [float(v) for v in (next(
        l for l in model.findall("link") if l.get("name") == "base_link"
    ).findtext("pose") or "0 0 0 0 0 0").split()]
    com_z = base_link_pose[2] + mp["com_offset_base_link_m"][2]
    return rotor[2] - com_z


def stale_plugin_warning():
    """The installed .so is what gz loads; warn if any plugin source is newer than it."""
    so = os.path.join(REPO_ROOT, "install", "gz_plugin", "lib", "libgz_monospinner_plugin.so")
    if not os.path.exists(so):
        return f"{so} does not exist -- run scripts/build_all.sh plugin"
    sources = glob.glob(os.path.join(REPO_ROOT, "src", "gz_monospinner_plugin", "**", "*.*"),
                        recursive=True)
    newer = [s for s in sources if os.path.getmtime(s) > os.path.getmtime(so)]
    if newer:
        names = ", ".join(os.path.relpath(s, REPO_ROOT) for s in newer)
        return (f"installed plugin is OLDER than its sources ({names}); this run tests the old "
                "build. Rebuild with scripts/build_all.sh plugin to test the current code.")
    return None


# ------------------------------------------------------------------------------ gz-server ---
def start_gz(world_file, gui):
    env = dict(os.environ)
    env["GZ_SIM_RESOURCE_PATH"] = ":".join(
        p for p in [os.path.join(REPO_ROOT, "models"), os.path.join(REPO_ROOT, "worlds"),
                    env.get("GZ_SIM_RESOURCE_PATH", "")] if p)
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = ":".join(
        p for p in [os.path.join(REPO_ROOT, "install", "gz_plugin", "lib"),
                    env.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")] if p)
    cmd = ["gz", "sim", "-r", world_file] if gui else ["gz", "sim", "-s", "-r", world_file]
    # Own process group: `gz sim` forks the server (and GUI) as children, and all of them
    # must go down together at the end.
    return subprocess.Popen(cmd, env=env, start_new_session=True)


def stop_gz(proc):
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        proc.wait()
    except ProcessLookupError:
        pass


def gz_already_running(world):
    try:
        out = subprocess.run(["gz", "topic", "-l"], capture_output=True, text=True,
                             timeout=10).stdout
    except subprocess.TimeoutExpired:
        return False
    return f"/world/{world}/clock" in out.split()


# ------------------------------------------------------------------------------ recording ---
class Recorder:
    def __init__(self):
        self.lock = threading.Lock()
        self.t = None
        self.odom, self.imu, self.cmd = [], [], []
        self.recording = False

    def on_clock(self, msg):
        with self.lock:
            self.t = msg.sim.sec + msg.sim.nsec * 1e-9

    def now(self):
        with self.lock:
            return self.t

    def on_odom(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        p, o = msg.pose.position, msg.pose.orientation
        tl, ta = msg.twist.linear, msg.twist.angular
        with self.lock:
            if self.recording:
                self.odom.append([t, p.x, p.y, p.z, o.w, o.x, o.y, o.z,
                                  tl.x, tl.y, tl.z, ta.x, ta.y, ta.z])

    def on_imu(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        av, la = msg.angular_velocity, msg.linear_acceleration
        with self.lock:
            if self.recording:
                self.imu.append([t, av.x, av.y, av.z, la.x, la.y, la.z])

    def log_cmd(self, t, phase, alpha, beta, thrust):
        with self.lock:
            if self.recording:
                self.cmd.append([t, phase, alpha, beta, thrust])


ODOM_HEADER = ["t_sim", "x_enu", "y_enu", "z_enu", "qw", "qx", "qy", "qz",
               "vx_flu", "vy_flu", "vz_flu", "wx_flu", "wy_flu", "wz_flu"]
IMU_HEADER = ["t_sim", "wx_flu", "wy_flu", "wz_flu", "fx_flu", "fy_flu", "fz_flu"]
CMD_HEADER = ["t_sim", "phase", "alpha_rad", "beta_rad", "thrust_n"]


def write_csv(path, header, rows):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)


def run(args, run_dir):
    sdf = os.path.join(REPO_ROOT, "models", args.model, "model.sdf")
    world_file = os.path.join(REPO_ROOT, "worlds", f"{args.world}.sdf")
    mp = mass_properties(sdf, "base_link")
    g = world_gravity(world_file, args.world)
    params = plugin_params(sdf)
    m = mp["mass_kg"]
    thrust = (1.0 + args.margin) * m * g

    warning = stale_plugin_warning()
    if warning:
        print(f"WARNING: {warning}", file=sys.stderr)

    profile = [("settle", args.settle_s, 0.0, 0.0, 0.0),
               ("thrust", args.thrust_s, 0.0, 0.0, thrust)]
    if args.alpha_deg != 0.0:
        if abs(args.alpha_deg) > params.get("alpha_max_deg", 90.0):
            print(f"WARNING: |alpha| = {abs(args.alpha_deg):g} deg exceeds alpha_max_deg = "
                  f"{params['alpha_max_deg']:g} in model.sdf (the plugin does not clamp it yet)",
                  file=sys.stderr)
        profile.append(("tilt", args.tilt_s, math.radians(args.alpha_deg),
                        math.radians(args.beta_deg), thrust))

    with open(os.path.join(run_dir, "manifest.json"), "w") as f:
        json.dump({
            "script": "scripts/open_loop_test.py",
            "created": datetime.datetime.now().isoformat(timespec="seconds"),
            "world": args.world, "model": args.model,
            "margin": args.margin, "g_m_s2": g, "mass_properties": mp,
            "plugin_params": params,
            "rotor_com_above_composite_com_m": rotor_lever_arm(sdf, mp),
            "profile": [{"phase": p, "duration_s": d, "alpha_rad": a, "beta_rad": b,
                         "thrust_n": T} for p, d, a, b, T in profile],
            "command_topic": f"/{args.model}/command/motor_speed",
            "command_encoding": "position=[alpha, beta] rad, velocity=[thrust] N",
            "stale_plugin_warning": warning,
        }, f, indent=2)

    if gz_already_running(args.world):
        sys.exit(f"error: /world/{args.world}/clock already exists -- another gz-server is "
                 "running (stop it, or PX4 may be publishing commands too).")

    rec = Recorder()
    gz = None
    node = Node()
    try:
        print(f"==> Starting gz-server ({'GUI' if args.gui else 'headless'}): {world_file}")
        gz = start_gz(world_file, args.gui)

        node.subscribe(Clock, f"/world/{args.world}/clock", rec.on_clock)
        node.subscribe(Odometry, f"/model/{args.model}/odometry", rec.on_odom)
        node.subscribe(IMU, f"/model/{args.model}/imu_gd_truth", rec.on_imu)
        pub = node.advertise(f"/{args.model}/command/motor_speed", Actuators)

        deadline = time.time() + 60
        while rec.now() is None:
            if gz.poll() is not None:
                sys.exit("error: gz exited during startup")
            if time.time() > deadline:
                sys.exit(f"error: no /world/{args.world}/clock after 60 s")
            time.sleep(0.05)
        time.sleep(1.0)  # let discovery between our publisher and the plugin complete
        print(f"==> gz-server up; mass {m:.4f} kg, g {g:.3f}, thrust {thrust:.4f} N "
              f"({1 + args.margin:.2f} m g)")

        with rec.lock:
            rec.recording = True
        t0 = rec.now()
        period = 1.0 / args.rate_hz
        start = 0.0
        for phase, duration, alpha, beta, T in profile:
            print(f"    t_sim={rec.now() - t0:7.3f}  {phase}: alpha={math.degrees(alpha):g} deg "
                  f"beta={math.degrees(beta):g} deg T={T:.4f} N for {duration:g} s")
            while rec.now() < t0 + start + duration:
                msg = Actuators()
                msg.position.extend([alpha, beta])
                msg.velocity.extend([T])
                pub.publish(msg)
                rec.log_cmd(rec.now(), phase, alpha, beta, T)
                if gz.poll() is not None:
                    sys.exit("error: gz exited during the run")
                time.sleep(period)
            start += duration
        with rec.lock:
            rec.recording = False
    finally:
        stop_gz(gz)
        with rec.lock:
            odom, imu, cmd = list(rec.odom), list(rec.imu), list(rec.cmd)
        write_csv(os.path.join(run_dir, "gt_odom.csv"), ODOM_HEADER, odom)
        write_csv(os.path.join(run_dir, "gt_imu.csv"), IMU_HEADER, imu)
        write_csv(os.path.join(run_dir, "commands.csv"), CMD_HEADER, cmd)
        print(f"==> Recorded {len(odom)} odometry, {len(imu)} imu, {len(cmd)} command rows "
              f"in {run_dir}")
    if not odom:
        sys.exit("error: no odometry recorded -- is OdometryPublisher in the model?")


# ------------------------------------------------------------------------------- plotting ---
def load(path):
    with open(path) as f:
        r = csv.reader(f)
        header = next(r)
        rows = list(r)
    cols = {}
    for i, name in enumerate(header):
        vals = [row[i] for row in rows]
        try:
            cols[name] = np.array(vals, dtype=float)
        except ValueError:
            cols[name] = np.array(vals)
    return cols


def quat_rotate(q, v):
    """Rotate body vectors v (N,3) by unit quaternions q (N,4: w,x,y,z), body -> world."""
    w, x, y, z = q.T
    R = np.stack([
        np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)], -1),
        np.stack([2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)], -1),
        np.stack([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)], -1),
    ], 1)
    return np.einsum("nij,nj->ni", R, v)


def plot_run(run_dir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    with open(os.path.join(run_dir, "manifest.json")) as f:
        man = json.load(f)
    od = load(os.path.join(run_dir, "gt_odom.csv"))
    im = load(os.path.join(run_dir, "gt_imu.csv"))
    cm = load(os.path.join(run_dir, "commands.csv"))

    t0 = cm["t_sim"][0] if len(cm["t_sim"]) else od["t_sim"][0]
    to, ti, tc = od["t_sim"] - t0, im["t_sim"] - t0, cm["t_sim"] - t0

    m, g = man["mass_properties"]["mass_kg"], man["g_m_s2"]
    p = man["plugin_params"]
    thrust_phase = [s for s in man["profile"] if s["phase"] == "thrust"][0]
    T = thrust_phase["thrust_n"]
    t_on = sum(s["duration_s"] for s in man["profile"][:man["profile"].index(thrust_phase)])
    t_off = t_on + thrust_phase["duration_s"]
    tilt_phase = next((s for s in man["profile"] if s["phase"] == "tilt"), None)
    wz_pred = math.sqrt((p["k_tau"] / p["k_f"]) * T / p["K_z"])

    q = np.stack([od["qw"], od["qx"], od["qy"], od["qz"]], 1)
    v_world = quat_rotate(q, np.stack([od["vx_flu"], od["vy_flu"], od["vz_flu"]], 1))
    tilt = np.degrees(np.arccos(np.clip(1 - 2 * (od["qx"] ** 2 + od["qy"] ** 2), -1, 1)))
    spin = np.degrees(np.unwrap(2 * np.arctan2(od["qz"], od["qw"])))  # paper Eq. 55

    # Prediction for the climb: lift-off when the measured height starts rising, then
    # z = z0 + 0.5 a (t - t_lift)^2 with a = (T - m g) / m.
    a_pred = (T - m * g) / m
    z0 = od["z_enu"][to < t_on].mean() if np.any(to < t_on) else od["z_enu"][0]
    rising = np.where((to >= t_on) & (od["z_enu"] > z0 + 1e-3))[0]
    t_lift = to[rising[0]] if len(rising) else t_on
    tz = to[(to >= t_lift) & (to <= t_off)]  # alpha = 0 predictions hold in the thrust phase only

    fig, ax = plt.subplots(4, 2, figsize=(15, 15), sharex=True)
    title = (f"Open-loop test: alpha = beta = 0, T = {T:.3f} N = {1 + man['margin']:.2f} m g  "
             f"(m = {m:.4f} kg)")
    if tilt_phase:
        title += (f"; then alpha = {math.degrees(tilt_phase['alpha_rad']):g} deg, beta = "
                  f"{math.degrees(tilt_phase['beta_rad']):g} deg (body) from t = {t_off:g} s")
    if man.get("stale_plugin_warning"):
        title += "\nWARNING: plugin build older than its sources -- old code was tested"
    fig.suptitle(title)

    a = ax[0, 0]
    a.plot(tc, cm["thrust_n"], label="T commanded")
    a.axhline(m * g, color="k", ls=":", label="m g")
    a.set_ylabel("thrust [N]")
    a.legend()

    a = ax[0, 1]
    a.plot(tc, np.degrees(cm["alpha_rad"]), label="alpha")
    a.plot(tc, np.degrees(cm["beta_rad"]), "--", label="beta")
    a.set_ylabel("tilt command [deg]")
    a.legend()

    a = ax[1, 0]
    for k, lbl in (("x_enu", "x (E)"), ("y_enu", "y (N)"), ("z_enu", "z (U)")):
        a.plot(to, od[k], label=lbl)
    a.plot(tz, z0 + 0.5 * a_pred * (tz - t_lift) ** 2, "k:", label="z predicted")
    a.set_ylabel("position, world ENU [m]")
    a.legend()

    a = ax[1, 1]
    for i, lbl in enumerate(("vx (E)", "vy (N)", "vz (U)")):
        a.plot(to, v_world[:, i], label=lbl)
    a.plot(tz, a_pred * (tz - t_lift), "k:", label=f"vz predicted ({a_pred:.3f} m/s^2)")
    a.set_ylabel("velocity, world ENU [m/s]")
    a.legend()

    a = ax[2, 0]
    a.plot(to, tilt, label="tilt (body z vs world z)")
    a.set_ylabel("tilt [deg]")
    a.legend()

    a = ax[2, 1]
    a.plot(to, spin, label="spin angle theta_s (paper Eq. 55), unwrapped")
    a.set_ylabel("spin [deg]")
    a.legend()

    a = ax[3, 0]
    for k, lbl in (("wx_flu", "wx"), ("wy_flu", "wy"), ("wz_flu", "wz")):
        a.plot(ti, im[k], label=lbl)
    a.axhline(wz_pred, color="k", ls=":", label=f"wz steady predicted (+{wz_pred:.2f})")
    a.set_ylabel("body rate, FLU [rad/s]")
    a.set_xlabel("sim time since start [s]")
    a.legend()

    a = ax[3, 1]
    for k, lbl in (("fx_flu", "fx"), ("fy_flu", "fy"), ("fz_flu", "fz")):
        a.plot(ti, im[k], label=lbl)
    a.axhline(T / m, color="k", ls=":", label=f"T/m airborne ({T / m:.3f})")
    a.axhline(g, color="grey", ls=":", label=f"g at rest ({g:.3f})")
    a.set_ylabel("specific force, body FLU [m/s^2]")
    a.set_xlabel("sim time since start [s]")
    a.legend()

    for a in ax.flat:
        a.grid(True, alpha=0.3)
        a.axvline(t_on, color="r", alpha=0.3)
        if tilt_phase:
            a.axvline(t_off, color="m", alpha=0.3)
    fig.tight_layout()
    out = os.path.join(run_dir, "open_loop_test.png")
    fig.savefig(out, dpi=110)
    plt.close(fig)
    print(f"==> Plot: {out}")

    if tilt_phase:
        plot_tilt(run_dir, man, od, to, cm, tc, tilt_phase, T, t_off, plt)


def rotation_prediction(man, T, alpha, beta, w0, q0, duration, gyro, dt=1e-4):
    """Integrate the body-frame rotation about the composite CoM under a constant command,
    with the torques ThrustVectorActuator applies (see the module docstring). RK4 on w,
    first-order on q (renormalised), from the measured state (w0 body FLU, q0 w,x,y,z)."""
    p = man["plugin_params"]
    i = man["mass_properties"]["inertia_about_com_kg_m2"]
    J = np.diag([i["ixx"], i["iyy"], i["izz"]])
    Jinv = np.linalg.inv(J)
    d = man["rotor_com_above_composite_com_m"]
    k = p["k_tau"] / p["k_f"]
    omega = math.sqrt(T / p["k_f"])
    jpz = p["J_pz"] if gyro else 0.0
    ez = np.array([0.0, 0.0, 1.0])
    r = np.array([math.sin(alpha) * math.cos(beta), math.sin(alpha) * math.sin(beta),
                  math.cos(alpha)])
    tau_const = T * d * np.cross(ez, r) + k * T * r

    def wdot(w):
        tau = (tau_const + omega * jpz * np.cross(w, r) - p["K_z"] * w[2] * abs(w[2]) * ez
               - np.cross(w, J @ w))
        return Jinv @ tau

    n = int(round(duration / dt))
    w = np.array(w0, dtype=float)
    q = np.array(q0, dtype=float)
    ts, ws, qs = [0.0], [w.copy()], [q.copy()]
    for step in range(1, n + 1):
        k1 = wdot(w)
        k2 = wdot(w + 0.5 * dt * k1)
        k3 = wdot(w + 0.5 * dt * k2)
        k4 = wdot(w + dt * k3)
        qw, qx, qy, qz = q
        wx, wy, wz = w  # q_dot = 0.5 q (x) (0, w), body rate (paper Eq. 50)
        q = q + 0.5 * dt * np.array([-qx * wx - qy * wy - qz * wz,
                                      qw * wx + qy * wz - qz * wy,
                                      qw * wy - qx * wz + qz * wx,
                                      qw * wz + qx * wy - qy * wx])
        q /= np.linalg.norm(q)
        w = w + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)
        if step % 10 == 0:
            ts.append(step * dt)
            ws.append(w.copy())
            qs.append(q.copy())
    return np.array(ts), np.array(ws), np.array(qs)


def plot_tilt(run_dir, man, od, to, cm, tc, tilt_phase, T, t_off, plt):
    alpha, beta = tilt_phase["alpha_rad"], tilt_phase["beta_rad"]
    # Onset = the first tilt command actually published (sim time from /clock); the plugin
    # picks it up on its next step, so the true onset can lag this by a few ms.
    tilt_rows = np.where(cm["phase"] == "tilt")[0]
    t_tilt = tc[tilt_rows[0]] if len(tilt_rows) else t_off
    i0 = int(np.searchsorted(to, t_tilt))
    w0 = [od["wx_flu"][i0], od["wy_flu"][i0], od["wz_flu"][i0]]
    q0 = [od["qw"][i0], od["qx"][i0], od["qy"][i0], od["qz"][i0]]
    t_end = to[-1]
    duration = max(0.0, t_end - t_tilt)
    pred = {g: rotation_prediction(man, T, alpha, beta, w0, q0, duration, g)
            for g in (True, False)}

    def tilt_deg(q):
        return np.degrees(np.arccos(np.clip(1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2), -1, 1)))

    win = to >= t_tilt - 0.2
    fig, ax = plt.subplots(4, 1, figsize=(12, 14), sharex=True)
    fig.suptitle(f"Tilt phase: alpha = {math.degrees(alpha):g} deg, beta = "
                 f"{math.degrees(beta):g} deg (body), T = {T:.3f} N\n"
                 "measured (solid) vs rotation integrated from the measured onset state, "
                 "with gyro term (dashed) and without (dotted)")
    labels = (("wx_flu", 0, "wx body [rad/s]"), ("wy_flu", 1, "wy body [rad/s]"),
              ("wz_flu", 2, "wz body [rad/s]"))
    for a, (key, j, ylabel) in zip(ax[:3], labels):
        a.plot(to[win], od[key][win], color="C0", label="measured")
        tp, wp, _ = pred[True]
        a.plot(t_tilt + tp, wp[:, j], "--", color="C3", label="predicted, with gyro")
        tp, wp, _ = pred[False]
        a.plot(t_tilt + tp, wp[:, j], ":", color="k", label="predicted, no gyro")
        a.set_ylabel(ylabel)
        a.legend(loc="upper left")
    a = ax[3]
    tilt_meas = np.degrees(np.arccos(np.clip(1 - 2 * (od["qx"] ** 2 + od["qy"] ** 2), -1, 1)))
    a.plot(to[win], tilt_meas[win], color="C0", label="measured")
    tp, _, qp = pred[True]
    a.plot(t_tilt + tp, tilt_deg(qp), "--", color="C3", label="predicted, with gyro")
    tp, _, qp = pred[False]
    a.plot(t_tilt + tp, tilt_deg(qp), ":", color="k", label="predicted, no gyro")
    a.set_ylabel("tilt (body z vs world z) [deg]")
    a.set_xlabel("sim time since start [s]")
    a.legend(loc="upper left")
    for a in ax:
        a.grid(True, alpha=0.3)
        a.axvline(t_tilt, color="m", alpha=0.4)
    fig.tight_layout()
    out = os.path.join(run_dir, "open_loop_tilt.png")
    fig.savefig(out, dpi=110)
    plt.close(fig)
    print(f"==> Plot: {out}")


# ----------------------------------------------------------------------------------- main ---
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--margin", type=float, default=0.1,
                    help="thrust = (1 + margin) * m * g during the thrust phase (default 0.1)")
    ap.add_argument("--settle-s", type=float, default=2.0, help="sim s at T = 0 first")
    ap.add_argument("--thrust-s", type=float, default=5.0, help="sim s of thrust")
    ap.add_argument("--alpha-deg", type=float, default=0.0,
                    help="tilt magnitude for an extra tilt phase after the thrust phase; "
                         "0 (default) = no tilt phase. A small value such as 2 is a good start")
    ap.add_argument("--beta-deg", type=float, default=0.0,
                    help="tilt direction in the BODY frame for the tilt phase (default 0)")
    ap.add_argument("--tilt-s", type=float, default=1.0, help="sim s of the tilt phase")
    ap.add_argument("--rate-hz", type=float, default=100.0, help="command publish rate")
    ap.add_argument("--gui", action="store_true", help="start the gz GUI as well")
    ap.add_argument("--world", default="monospinner_default")
    ap.add_argument("--model", default="monospinner")
    ap.add_argument("--out-dir", default=os.path.join(REPO_ROOT, "build", "runs"))
    ap.add_argument("--plot-only", metavar="RUN_DIR", help="re-plot an existing run")
    args = ap.parse_args()

    if args.plot_only:
        plot_run(args.plot_only)
        return

    stamp = datetime.datetime.now().strftime("%Y-%m-%dT%H-%M-%S")
    run_dir = os.path.join(args.out_dir, f"{stamp}_open_loop")
    os.makedirs(run_dir, exist_ok=True)
    signal.signal(signal.SIGTERM, lambda s, _f: sys.exit(128 + s))
    run(args, run_dir)
    plot_run(run_dir)


if __name__ == "__main__":
    exit_code = 0
    try:
        main()
    except SystemExit as e:
        if isinstance(e.code, str):
            print(e.code, file=sys.stderr)
            exit_code = 1
        else:
            exit_code = e.code or 0
    except BaseException:  # noqa: BLE001 -- report anything, then exit through os._exit
        import traceback
        traceback.print_exc()
        exit_code = 1
    sys.stdout.flush()
    sys.stderr.flush()
    # Skip interpreter finalization: gz-transport's subscriber threads can call back into
    # Python while it is being torn down and abort the process (same as wrench_player.py).
    # Every file is closed by now.
    os._exit(exit_code)
