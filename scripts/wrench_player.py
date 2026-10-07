#!/usr/bin/env python3
"""Play a force/torque timeline on the monospinner through gz-sim's ApplyLinkWrench system.

Estimator debugging aid: the forces go through the physics engine, so ground truth, the
IMU (ImuPhysical) and every other sensor stay physically consistent -- unlike a kinematic
velocity override (VelocityControl), under which the accelerometer reads free fall.

Requires gz::sim::systems::ApplyLinkWrench attached to the world (scripts/run_wrench_test.sh
does that at runtime through /world/<world>/entity/system/add; no .sdf change).

Frames and conventions (gz Link API, /usr/include/gz/sim8/gz/sim/Link.hh:349-370):
  * force and torque are expressed in WORLD axes (ENU);
  * every force is applied at the vehicle's composite centre of mass, given to
    ApplyLinkWrench as an offset from the base_link origin in the base_link frame, so a
    force never adds a spurious torque. The CoM is computed from models/monospinner/model.sdf.
  * "assist": true adds a vertical force of m*g at the CoM, so the vehicle floats
    (gravity cancelled) and the segment's own force accelerates it. g is the WORLD's
    gravity, read from the world .sdf (SDF default 9.8 when the file sets none) -- not
    standard gravity: 9.80665 against the world's 9.8 left 0.00665 m/s^2 of net climb,
    which over a 20 s hover added 0.13 m/s and ~2 m of unplanned altitude.

Persistent wrenches ADD UP in ApplyLinkWrench (measured: 0.5 mg + 0.6 mg published one
after the other gave 1.1 mg). The player therefore publishes each segment change as a
single delta (new total - previous total) -- one message, no clear/publish gap -- and
clears everything at the end, including on Ctrl-C / SIGTERM.

Timing: segment boundaries are scheduled on the SIMULATION clock, but this process runs
asynchronously next to the simulation, so a boundary can land a millisecond or so late.
Fine for checking the estimator; per CLAUDE.md ("Lockstep") not for collecting results.
The sim time each wrench was actually applied is logged.

Profile JSON format (times in sim seconds, segments are back to back):
  {"segments": [
     {"label": "baseline", "duration": 5},
     {"label": "climb", "duration": 2, "assist": true, "force": [0, 0, 0.4]},
     {"label": "spin up", "duration": 2, "assist": true, "torque": [0, 0, 0.001]}
  ]}

Usage:
  scripts/wrench_player.py --list
  scripts/wrench_player.py --profile lift_hover --out-dir build/runs/<run>
  scripts/wrench_player.py --profile my_profile.json --print-duration
"""
import argparse
import csv
import json
import math
import os
import signal
import sys
import threading
import time
import xml.etree.ElementTree as ET

from gz.msgs10.clock_pb2 import Clock
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.entity_wrench_pb2 import EntityWrench
from gz.transport13 import Node

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def world_gravity(world_sdf_path, world_name):
    """|g| from <world><gravity>; SDF's default is "0 0 -9.8" when the element is absent."""
    root = ET.parse(world_sdf_path).getroot()
    for world in root.iter("world"):
        if world.get("name") == world_name:
            text = world.findtext("gravity")
            gravity = [float(x) for x in text.split()] if text else [0.0, 0.0, -9.8]
            if abs(gravity[0]) > 1e-9 or abs(gravity[1]) > 1e-9:
                sys.exit(f"error: non-vertical gravity {gravity} in {world_sdf_path}; the "
                         "gravity assist assumes world -z.")
            return -gravity[2]
    sys.exit(f"error: no <world name=\"{world_name}\"> in {world_sdf_path}")


# --- mass properties from the SDF ---------------------------------------------------------

def _pose(text):
    v = [float(x) for x in (text or "").split()]
    return (v + [0.0] * 6)[:6]


def mass_properties(sdf_path, base_link):
    """Total mass, CoM (relative to the base_link origin, base_link frame) and inertia about
    the CoM, from the model's links. Assumes every link and inertial pose is unrotated, which
    holds for models/monospinner/model.sdf; refuses otherwise rather than guess."""
    model = ET.parse(sdf_path).getroot().find("model")
    links = []
    base_xyz = None
    for link in model.findall("link"):
        lp = _pose(link.findtext("pose"))
        inertial = link.find("inertial")
        ip = _pose(inertial.findtext("pose"))
        if any(abs(a) > 1e-9 for a in lp[3:] + ip[3:]):
            sys.exit(f"error: link '{link.get('name')}' has a rotated pose; mass_properties() "
                     "assumes unrotated links -- extend it before using this model.")
        inertia = inertial.find("inertia")
        links.append({
            "name": link.get("name"),
            "mass": float(inertial.findtext("mass")),
            "xyz": [lp[i] + ip[i] for i in range(3)],
            "ixx": float(inertia.findtext("ixx")),
            "iyy": float(inertia.findtext("iyy")),
            "izz": float(inertia.findtext("izz")),
        })
        if link.get("name") == base_link:
            base_xyz = lp[:3]
    if base_xyz is None:
        sys.exit(f"error: no link named '{base_link}' in {sdf_path}")

    mass = sum(l["mass"] for l in links)
    com = [sum(l["mass"] * l["xyz"][i] for l in links) / mass for i in range(3)]

    def about_com(axis):
        a, b = {"x": (1, 2), "y": (0, 2), "z": (0, 1)}[axis]
        return sum(l["i" + axis * 2] + l["mass"] * ((l["xyz"][a] - com[a]) ** 2 +
                                                     (l["xyz"][b] - com[b]) ** 2) for l in links)

    return {
        "mass_kg": mass,
        "com_offset_base_link_m": [com[i] - base_xyz[i] for i in range(3)],
        "inertia_about_com_kg_m2": {"ixx": about_com("x"), "iyy": about_com("y"),
                                    "izz": about_com("z")},
        "links": [l["name"] for l in links],
    }


# --- built-in profiles ----------------------------------------------------------------------

def seg(label, duration, force=(0.0, 0.0, 0.0), torque=(0.0, 0.0, 0.0), assist=False):
    return {"label": label, "duration": float(duration), "force": list(force),
            "torque": list(torque), "assist": bool(assist)}


def builtin_profiles(mp, g):
    mg = mp["mass_kg"] * g
    inertia = mp["inertia_about_com_kg_m2"]
    up = 0.1 * mg  # +/-0.1 g vertical: 2 s accelerate + ~2 s brake = 1.96 m/s peak, 3.9 m

    # NEVER EXACTLY STILL IN THE AIR. gz-sim only rewrites a link's velocity/acceleration
    # components in steps where its pose changed (gz-sim 8.15.0 src/systems/physics/
    # Physics.cc: ChangedLinks() 3376-3463, UpdateSim() loop 3674-3760), so ImuPhysical
    # freezes at its last values whenever the body is bit-for-bit at rest -- observed after
    # a symmetric accelerate/brake pair left exactly zero velocity (0.9 g read through a
    # 20 s hover). The climb's brake is therefore 0.5 % shorter than its accelerate phase:
    # the vehicle keeps rising at ~1 cm/s (~0.2-0.5 m over a profile) and its pose changes
    # every step. The descent's brake is 1 % shorter, which turns that into a ~1 cm/s
    # descent, so the vehicle keeps moving until the touchdown phase (net 0.02 g down)
    # sets it gently on the ground. Resting on the ground afterwards can still freeze the
    # IMU at its touchdown value -- treat data after touchdown with care.
    climb = [seg("climb: accelerate up (+0.1 g)", 2, (0, 0, up), assist=True),
             seg("climb: brake (-0.1 g; 0.5 % short, keeps ~1 cm/s climb)", 1.99,
                 (0, 0, -up), assist=True)]
    descend = [seg("descend: accelerate down (-0.1 g)", 2, (0, 0, -up), assist=True),
               seg("descend: brake (+0.1 g; 1 % short, keeps ~1 cm/s descent)", 1.98,
                   (0, 0, up), assist=True),
               seg("touchdown: net 0.02 g down onto the ground", 4, (0, 0, -0.02 * mg),
                   assist=True)]
    baseline = [seg("baseline: on the ground, no wrench", 5)]
    settle = [seg("settle: on the ground, no wrench", 5)]

    def hover(s, label="hover (gravity assist only)"):
        return [seg(label, s, assist=True)]

    # lateral: 0.05 g for 2 s then -0.05 g for 2 s -> 0.98 m/s peak, 1.96 m travel
    side = 0.05 * mg

    def push(ax, name, extra_label="", torque=(0, 0, 0)):
        f = [0.0, 0.0, 0.0]
        f[ax] = side
        return [seg(f"push {name}{extra_label}: accelerate (+0.05 g)", 2, tuple(f), torque,
                    assist=True),
                seg(f"push {name}{extra_label}: brake (-0.05 g)", 2, tuple(-v for v in f),
                    torque, assist=True)]

    # yaw: spin up to 5 rad/s in 2 s, coast, spin down. tau = Izz * omega / T
    omega, t_spin = 5.0, 2.0
    tau_z = inertia["izz"] * omega / t_spin
    spin_up = [seg(f"spin up to {omega:g} rad/s (world z torque)", t_spin,
                   torque=(0, 0, tau_z), assist=True)]
    spin_down = [seg("spin down", t_spin, torque=(0, 0, -tau_z), assist=True)]

    # tilt: 10 deg about a world axis and back. Accelerate for t, brake for t: theta = alpha*t^2
    theta, t_tilt = math.radians(10.0), 0.5

    def tilt(ax, name, i_key, hold):
        tq = [0.0, 0.0, 0.0]
        tq[ax] = inertia[i_key] * theta / t_tilt ** 2
        neg = tuple(-v for v in tq)
        return [seg(f"{name} +10 deg: accelerate", t_tilt, torque=tuple(tq), assist=True),
                seg(f"{name} +10 deg: brake", t_tilt, torque=neg, assist=True),
                seg(f"hold {name} at 10 deg", hold, assist=True),
                seg(f"{name} back to level: accelerate", t_tilt, torque=neg, assist=True),
                seg(f"{name} back to level: brake", t_tilt, torque=tuple(tq), assist=True)]

    roll = lambda hold: tilt(0, "roll (world x)", "ixx", hold)    # noqa: E731
    pitch = lambda hold: tilt(1, "pitch (world y)", "iyy", hold)  # noqa: E731
    push_x = lambda extra="": push(0, "x (east)", extra)          # noqa: E731
    push_y = lambda extra="": push(1, "y (north)", extra)         # noqa: E731

    return {
        "quiet": ("no wrench at all, on the ground (baseline)",
                  [seg("on the ground, no wrench", 20)]),
        "lift_hover": ("climb ~3.9 m, hover 20 s, descend back to the ground",
                       baseline + climb + hover(20) + descend + settle),
        "push_xy": ("climb, lateral 0.05 g pulses along world x then y (~2 m each), descend",
                    baseline + climb + hover(3) + push_x() + hover(3) + push_y() + hover(3)
                    + descend + settle),
        "yaw_spin": ("climb, spin up to 5 rad/s about z, coast 20 s, spin down, descend",
                     baseline + climb + hover(3) + spin_up
                     + [seg(f"coast at {omega:g} rad/s", 20, assist=True)]
                     + spin_down + hover(3) + descend + settle),
        "tilt": ("climb, tilt 10 deg about world x, hold, level; same about y; descend",
                 baseline + climb + hover(3) + roll(5) + hover(3) + pitch(5) + hover(3)
                 + descend + settle),
        "all_dof": ("every DOF in turn: z, x, y, roll, pitch, yaw; then x/y pushes while "
                    "spinning at 5 rad/s; descend",
                    baseline + climb + hover(2)
                    + push_x() + hover(2) + push_y() + hover(2)
                    + roll(3) + hover(2) + pitch(3) + hover(2)
                    + spin_up
                    + [seg(f"coast at {omega:g} rad/s", 2, assist=True)]
                    + push(0, "x (east)", f" while spinning at {omega:g} rad/s")
                    + push(1, "y (north)", f" while spinning at {omega:g} rad/s")
                    + [seg(f"coast at {omega:g} rad/s", 2, assist=True)]
                    + spin_down + hover(2) + descend + settle),
    }


def load_profile(name_or_path, mp, g):
    builtins = builtin_profiles(mp, g)
    if name_or_path in builtins:
        return name_or_path, builtins[name_or_path][0], builtins[name_or_path][1]
    if not os.path.isfile(name_or_path):
        sys.exit(f"error: '{name_or_path}' is neither a built-in profile "
                 f"({', '.join(builtins)}) nor a file.")
    with open(name_or_path) as f:
        raw = json.load(f)
    segments = [seg(s.get("label", f"segment {i}"), s["duration"], s.get("force", (0, 0, 0)),
                    s.get("torque", (0, 0, 0)), s.get("assist", False))
                for i, s in enumerate(raw["segments"])]
    return os.path.abspath(name_or_path), raw.get("description", ""), segments


# --- playback -------------------------------------------------------------------------------

class SimClock:
    def __init__(self):
        self.t = None
        self._event = threading.Event()

    def on_clock(self, msg):
        self.t = msg.sim.sec + msg.sim.nsec * 1e-9
        self._event.set()

    def wait_first(self, timeout):
        return self._event.wait(timeout)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--profile", default="lift_hover",
                    help="built-in profile name (see --list) or path to a JSON profile")
    ap.add_argument("--list", action="store_true", help="list built-in profiles and exit")
    ap.add_argument("--print-duration", action="store_true",
                    help="print the profile's duration in sim seconds and exit")
    ap.add_argument("--out-dir", default=None,
                    help="write wrench_applied.csv and wrench_profile.json here")
    ap.add_argument("--world", default="monospinner_default")
    ap.add_argument("--model", default="monospinner")
    ap.add_argument("--link", default="base_link")
    ap.add_argument("--sdf", default=os.path.join(REPO_ROOT, "models", "monospinner",
                                                  "model.sdf"))
    ap.add_argument("--world-sdf", default=None,
                    help="world file to read <gravity> from (default worlds/<world>.sdf)")
    args = ap.parse_args()

    mp = mass_properties(args.sdf, args.link)
    g = world_gravity(args.world_sdf or os.path.join(REPO_ROOT, "worlds", f"{args.world}.sdf"),
                      args.world)

    if args.list:
        for name, (desc, segments) in builtin_profiles(mp, g).items():
            print(f"{name:12s} {sum(s['duration'] for s in segments):5.1f} s  {desc}")
        return

    profile_id, description, segments = load_profile(args.profile, mp, g)
    total = sum(s["duration"] for s in segments)
    if args.print_duration:
        print(f"{total:.3f}")
        return

    mg = mp["mass_kg"] * g
    offset = mp["com_offset_base_link_m"]

    def total_wrench(s):
        f = list(s["force"])
        if s["assist"]:
            f[2] += mg
        return f, list(s["torque"])

    node = Node()
    clock = SimClock()
    node.subscribe(Clock, f"/world/{args.world}/clock", clock.on_clock)
    pub = node.advertise(f"/world/{args.world}/wrench/persistent", EntityWrench)
    pub_clear = node.advertise(f"/world/{args.world}/wrench/clear", Entity)

    entity = Entity()
    entity.name = f"{args.model}::{args.link}"
    entity.type = Entity.LINK

    def publish_delta(df, dt):
        msg = EntityWrench()
        msg.entity.CopyFrom(entity)
        msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z = df
        msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z = dt
        msg.wrench.force_offset.x, msg.wrench.force_offset.y, msg.wrench.force_offset.z = offset
        pub.publish(msg)

    def clear_all():
        pub_clear.publish(entity)

    def on_signal(signum, _frame):
        raise SystemExit(128 + signum)

    signal.signal(signal.SIGTERM, on_signal)

    if not clock.wait_first(10.0):
        sys.exit(f"error: no message on /world/{args.world}/clock within 10 s -- is gz running?")
    time.sleep(0.5)  # let the publishers' discovery complete before the first wrench

    rows = []
    out_csv = None
    if args.out_dir:
        os.makedirs(args.out_dir, exist_ok=True)
        with open(os.path.join(args.out_dir, "wrench_profile.json"), "w") as f:
            json.dump({
                "profile": profile_id,
                "description": description,
                "frames": "force/torque in WORLD axes (ENU); forces applied at the composite "
                          "CoM; 'assist' adds m*g along world +z at the CoM",
                "mass_properties": mp,
                "g_m_s2": g,
                "segments": segments,
            }, f, indent=2)
        out_csv = os.path.join(args.out_dir, "wrench_applied.csv")

    print(f"==> Profile '{profile_id}': {len(segments)} segments, {total:.1f} s of sim time; "
          f"mass {mp['mass_kg']:.4f} kg, CoM offset {offset[2]:+.4f} m (base_link z)")

    cur_f, cur_t = [0.0] * 3, [0.0] * 3
    t0 = clock.t
    start = 0.0
    try:
        for i, s in enumerate(segments):
            while clock.t < t0 + start:
                time.sleep(0.0005)
            new_f, new_t = total_wrench(s)
            df = [new_f[k] - cur_f[k] for k in range(3)]
            dt = [new_t[k] - cur_t[k] for k in range(3)]
            if any(abs(v) > 0 for v in df + dt):
                publish_delta(df, dt)
            cur_f, cur_t = new_f, new_t
            t_applied = clock.t
            rows.append([f"{t_applied:.4f}", i, s["label"], *cur_f, *cur_t, *offset])
            print(f"    t_sim={t_applied:8.3f}  [{i:2d}] {s['label']}")
            start += s["duration"]
        while clock.t < t0 + start:
            time.sleep(0.0005)
    finally:
        clear_all()
        if clock.t is not None:
            rows.append([f"{clock.t:.4f}", len(segments), "end: all wrenches cleared",
                         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, *offset])
        time.sleep(0.3)  # let the clear go out before the process exits
        if out_csv:
            with open(out_csv, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["t_sim", "segment", "label", "fx_world", "fy_world", "fz_world",
                            "tx_world", "ty_world", "tz_world", "offset_x", "offset_y",
                            "offset_z"])
                w.writerows(rows)
            print(f"==> Wrote {out_csv}")
        node.unsubscribe(f"/world/{args.world}/clock")
    print("==> Profile done; all wrenches cleared")


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
    # Python while it is being torn down, which aborts the process ("terminate called
    # without an active exception", observed once at the end of a 38 s profile, after all
    # output was written). Every file is closed by now, so nothing is lost.
    os._exit(exit_code)
