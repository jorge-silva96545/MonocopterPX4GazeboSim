#!/usr/bin/env bash
# !!! KNOWN LIMITATION -- ACCELEROMETER IS WRONG WHILE VelocityControl IS ATTACHED !!!
# From the moment VelocityControl is attached (even on the ground), ImuPhysical's
# accelerometer reads ~0 m/s^2 (free fall) instead of -9.81 on FRD z: it takes
# Link::WorldLinearAcceleration(), which under a kinematic velocity command still carries
# gravity's per-step acceleration, so specific force = a - g ~ 0. EKF2 then has no tilt
# reference and its altitude, tilt and (through the 63 deg field inclination) heading go
# wrong: ~2 deg tilt / -3..-4 deg yaw at 0.5-3 rad/s, 75 deg tilt at 10 rad/s (runs
# 2026-10-02T11-35-57, T11-28-27, T11-37-44). Treat tilt, altitude and yaw results from
# this script as rig artefacts until that is fixed. Gyro and magnetometer data are fine.
#
# Spin test for the heading estimate: launches the full stack (scripts/run_sim.sh), lets
# EKF2 align with the vehicle still on the ground, lifts it to a hover height, holds it,
# then spins it about body z at a constant rate while scripts/record_vehicle_state.py
# records PX4's EKF2 estimate against Gazebo ground truth and plots it (same figures and
# build/runs/<timestamp>/ layout as scripts/run_ekf2_debugging.sh).
#
# The motion is KINEMATIC, not flown: gz-sim's VelocityControl system
# (gz-sim-velocity-control-system) is attached to the running model at runtime through
# the world's entity/system/add service, so no .sdf file is involved. It overrides the
# dynamics -- no rotor torque, no nutation -- which is what an estimator test wants (an
# exact, known body rate the gyro sees through ImuPhysical's Link::WorldAngularVelocity())
# and what a flight test would not. While attached it also holds the vehicle against
# gravity.
#
# Why lift first: on the ground, contact friction caps the spin at ~1.6 rad/s whatever the
# command (measured with 3.0 rad/s commanded); clear of the ground the commanded rate is
# exact. The climb is a velocity command, not a teleport, so IMU, baro and GNSS all see a
# consistent motion. LIFT_Z=0 skips it (friction-limited spin; the commanded rate is then
# NOT the actual rate -- read it from the gyro plot).
#
# Env:
#   HEADLESS=1      start gz-server without the GUI client (passed through to run_sim.sh).
#   SETTLE_S=5      wall s after PX4 odometry appears, before anything moves (EKF2 aligns
#                   at rest -- CLAUDE.md "Arm before spin-up").
#   LIFT_Z=1.0      hover height [m, ground-truth ENU z of the model origin]; 0 = no lift.
#   LIFT_RATE=0.25  climb rate [m/s].
#   HOLD_S=5        wall s hovering still after the climb, before recording starts.
#   PRE_S=10        s of still-hover baseline at the start of the recording.
#   SPIN_RATE=3.0   body yaw rate [rad/s], positive = counter-clockwise seen from above
#                   (gz FLU body z).
#   SPIN_S=60       s of spinning recorded.
#
# Extra arguments are passed through to record_vehicle_state.py, except --duration, which
# this script sets to PRE_S + SPIN_S. E.g.:
#   HEADLESS=1 SPIN_RATE=3 scripts/run_spin_test.sh
#   SPIN_RATE=10 SPIN_S=90 scripts/run_spin_test.sh --gt-angular-source odom
# Output lands in build/runs/<timestamp>/ (figures/ holds the plots), plus spin_test.json
# with this script's settings and the sim time at which the spin was commanded.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

# record_vehicle_state.py needs rclpy and px4_msgs; neither is on the path in a
# non-interactive shell (see CLAUDE.md). ROS's setup scripts are not `set -u` safe.
set +u
# shellcheck disable=SC1091
[[ -z "${AMENT_PREFIX_PATH:-}" ]] && source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "$REPO_ROOT/install/setup.bash"
set -u

# Must match the world name in worlds/monospinner_default.sdf and the model name
# run_sim.sh spawns (PX4_GZ_MODEL_NAME).
WORLD="monospinner_default"
MODEL="monospinner"
PX4_ODOM_TOPIC="/fmu/out/vehicle_odometry"
# VelocityControl's default topic, /model/<model>/cmd_vel (VelocityControl.cc, gz-sim 8).
CMD_VEL_TOPIC="/model/$MODEL/cmd_vel"
GT_ODOM_TOPIC="/model/$MODEL/odometry"
CLOCK_TOPIC="/world/$WORLD/clock"

SETTLE_S="${SETTLE_S:-5}"
LIFT_Z="${LIFT_Z:-1.0}"
LIFT_RATE="${LIFT_RATE:-0.25}"
HOLD_S="${HOLD_S:-5}"
PRE_S="${PRE_S:-10}"
SPIN_RATE="${SPIN_RATE:-3.0}"
SPIN_S="${SPIN_S:-60}"
DURATION="$(python3 -c "print(float($PRE_S) + float($SPIN_S))")"

# One gz.msgs.Twist on the VelocityControl topic. Body-frame linear/angular velocity; the
# system keeps applying the last command until a new one arrives.
cmd_vel() {  # <linear_z> <angular_z>
  gz topic -t "$CMD_VEL_TOPIC" -m gz.msgs.Twist -p "linear: {z: $1}, angular: {z: $2}"
}

# Ground-truth ENU z of the model origin, from one odometry message.
gt_z() {
  gz topic -e -n 1 -t "$GT_ODOM_TOPIC" | awk '/position/{f=1} f&&/z:/{print $2; exit}'
}

# Current sim time [s] from the world clock.
sim_time() {
  gz topic -e -n 1 -t "$CLOCK_TOPIC" |
    awk '/^sim/{f=1} f&&/sec:/&&!/nsec/{s=$2} f&&/nsec:/{n=$2} f&&/}/{print s + n*1e-9; exit}'
}

# A stack left over from an earlier session would answer every check below: run_sim.sh
# fails to start its own agent ("port 8888 may already be in use"), and this script would
# then spin and record the OLD instance. Refuse instead.
if pgrep -f "px4_sitl_default/bin/px4 " >/dev/null || pgrep -f "gz sim" >/dev/null; then
  echo "error: a PX4 SITL or Gazebo instance is already running; stop it first:" >&2
  pgrep -af "px4_sitl_default/bin/px4 |gz sim" >&2 || true
  exit 1
fi

echo "==> Starting the simulation stack (scripts/run_sim.sh)"
"$REPO_ROOT/scripts/run_sim.sh" &
SIM_PID=$!
REC_PID=""
REC_LOG="$(mktemp)"
cleanup() {
  [[ -n "$REC_PID" ]] && kill "$REC_PID" 2>/dev/null || true
  # run_sim.sh's own trap stops PX4, the agent and Gazebo when it receives SIGTERM.
  kill "$SIM_PID" 2>/dev/null || true
  wait "$SIM_PID" 2>/dev/null || true
  rm -f "$REC_LOG"
}
trap cleanup EXIT

# Wait for an actual MESSAGE, not for the topic to be listed: `ros2 topic list` goes through
# the ROS 2 daemon, which remembers topics from earlier sessions, so it can succeed before
# this session's PX4 has even booted -- and the px4-* client calls below then fail.
# Best-effort QoS to match PX4's uXRCE-DDS publishers.
echo "==> Waiting for a message on $PX4_ODOM_TOPIC"
found=0
deadline=$((SECONDS + 90))
while ((SECONDS < deadline)); do
  if timeout 5 ros2 topic echo --once --qos-reliability best_effort "$PX4_ODOM_TOPIC" \
      >/dev/null 2>&1; then
    found=1
    break
  fi
  if ! kill -0 "$SIM_PID" 2>/dev/null; then
    echo "error: scripts/run_sim.sh exited before $PX4_ODOM_TOPIC was published." >&2
    exit 1
  fi
done
if [[ "$found" != "1" ]]; then
  echo "error: no message on $PX4_ODOM_TOPIC after 90 s -- is PX4 connecting to the" \
    "MicroXRCEAgent?" >&2
  exit 1
fi

echo "==> PX4 odometry is up; letting EKF2 align at rest for ${SETTLE_S} s"
sleep "$SETTLE_S"

# --- stop PX4's online gyro calibration for this session --------------------------------
# While DISARMED, gyro_calibration treats the vehicle as still when the accelerometer is
# steady and gyro variance is low (PX4-Autopilot/src/modules/gyro_calibration/
# GyroCalibration.cpp:175-204), then saves the mean gyro reading as CAL_GYRO*_OFF
# (:198-274). A constant kinematic spin about an axis through the IMU passes both checks,
# so it learns the spin rate as a gyro offset -- applied live (EKF2 then sees ~zero rate
# while the body spins) and persisted, so the next boot starts with a gyro reading
# -SPIN_RATE at rest and EKF2 never initialises. Observed: CAL_GYRO0_ZOFF -0.498, -2.998,
# -9.998 after 0.5/3/10 rad/s runs. A real vehicle never spins while disarmed, so this is
# a test artefact. Stopping the module only affects this PX4 session: it is started again
# on every boot (rcS, `if param compare -s IMU_GYRO_CAL_EN 1`), and no parameter changes.
PX4_BIN_DIR="$REPO_ROOT/PX4-Autopilot/build/px4_sitl_default/bin"
echo "==> Stopping PX4 gyro_calibration for this session"
# The px4-* clients reach instance 0 from run_sim.sh's PX4 working directory.
if ! (cd "$REPO_ROOT/build/px4_run" && "$PX4_BIN_DIR/px4-gyro_calibration" stop); then
  echo "error: could not stop gyro_calibration -- refusing to spin a disarmed vehicle" \
    "(it would save the spin rate as a gyro offset; see comment above)." >&2
  exit 1
fi

# --- attach VelocityControl to the running model --------------------------------------
# entity/system/add needs the entity ID: it accepted {name, type: MODEL} but then attached
# to the wrong entity ("VelocityControl plugin should be attached to a model entity").
MODEL_ID="$(gz model -m "$MODEL" 2>/dev/null | sed -n 's/^Model: \[\([0-9]*\)\].*/\1/p' | head -1)"
if [[ -z "$MODEL_ID" ]]; then
  echo "error: could not resolve the entity ID of model '$MODEL' (gz model -m $MODEL)." >&2
  exit 1
fi
echo "==> Attaching gz::sim::systems::VelocityControl to model '$MODEL' (entity $MODEL_ID)"
reply="$(gz service -s "/world/$WORLD/entity/system/add" \
  --reqtype gz.msgs.EntityPlugin_V --reptype gz.msgs.Boolean --timeout 3000 \
  --req "entity: {id: $MODEL_ID}, plugins: [{name: \"gz::sim::systems::VelocityControl\", \
filename: \"gz-sim-velocity-control-system\", innerxml: \"\"}]")"
if ! grep -q "data: true" <<<"$reply"; then
  echo "error: entity/system/add failed: $reply" >&2
  exit 1
fi
sleep 1
cmd_vel 0 0

# --- lift to hover height --------------------------------------------------------------
if python3 -c "import sys; sys.exit(0 if float($LIFT_Z) > 0 else 1)"; then
  echo "==> Climbing to z=${LIFT_Z} m at ${LIFT_RATE} m/s"
  cmd_vel "$LIFT_RATE" 0
  reached=0
  # Generous wall timeout: twice the nominal climb time plus slack.
  max_polls="$(python3 -c "print(int(2 * float($LIFT_Z) / float($LIFT_RATE) / 0.2) + 50)")"
  for _ in $(seq "$max_polls"); do
    z="$(gt_z)"
    if [[ -n "$z" ]] && python3 -c "import sys; sys.exit(0 if $z >= $LIFT_Z else 1)"; then
      reached=1
      break
    fi
    sleep 0.2
  done
  cmd_vel 0 0
  if [[ "$reached" != "1" ]]; then
    echo "error: vehicle did not reach z=${LIFT_Z} m (last z=${z:-?})." >&2
    exit 1
  fi
  # The climb stops on a polled reading, so it overshoots by roughly one poll interval
  # (~0.25 m at the default rate); the actual height goes into spin_test.json.
  HOVER_Z="$(gt_z)"
  echo "==> At z=${HOVER_Z} m; holding still for ${HOLD_S} s"
else
  echo "==> LIFT_Z=0: spinning on the ground (friction-limited -- commanded rate will NOT" \
    "be the actual rate). Holding for ${HOLD_S} s"
fi
sleep "$HOLD_S"

# --- record: PRE_S still, then SPIN_S spinning -----------------------------------------
echo "==> Recording ${DURATION} s (${PRE_S} s still, then ${SPIN_S} s at ${SPIN_RATE} rad/s)"
# -u: unbuffered, or the "==> Recording" line below only reaches $REC_LOG when the
# recorder exits and the spin lands after the recording window.
python3 -u "$REPO_ROOT/scripts/record_vehicle_state.py" "$@" --duration "$DURATION" \
  > >(tee "$REC_LOG") 2>&1 &
REC_PID=$!

# Time the spin from the moment the recorder actually starts writing, not from launch.
started=0
for _ in $(seq 120); do
  if grep -q "^==> Recording" "$REC_LOG"; then
    started=1
    break
  fi
  if ! kill -0 "$REC_PID" 2>/dev/null; then
    echo "error: record_vehicle_state.py exited before recording started." >&2
    exit 1
  fi
  sleep 0.5
done
if [[ "$started" != "1" ]]; then
  echo "error: record_vehicle_state.py did not start recording within 60 s." >&2
  exit 1
fi
RUN_DIR="$(sed -n 's/^==> Run directory: //p' "$REC_LOG" | head -1)"
T_REC_START="$(sed -n 's/.*starting at t_sim=\([0-9.]*\).*/\1/p' "$REC_LOG" | head -1)"

sleep "$PRE_S"
cmd_vel 0 "$SPIN_RATE"
T_SPIN="$(sim_time)"
echo "==> Spin commanded at t_sim=${T_SPIN} s"

wait "$REC_PID"
REC_PID=""
# Stop the spin before shutdown; harmless if the stack is already going down.
cmd_vel 0 0 || true

if [[ -n "$RUN_DIR" && -d "$RUN_DIR" ]]; then
  cat > "$RUN_DIR/spin_test.json" <<EOF
{
  "script": "scripts/run_spin_test.sh",
  "motion": "kinematic, gz::sim::systems::VelocityControl attached at runtime",
  "settle_s": $SETTLE_S,
  "lift_z_m": $LIFT_Z,
  "lift_rate_m_s": $LIFT_RATE,
  "hover_z_m_actual": ${HOVER_Z:-null},
  "hold_s": $HOLD_S,
  "pre_s": $PRE_S,
  "spin_rate_rad_s": $SPIN_RATE,
  "spin_rate_sense": "body z (gz FLU), positive = counter-clockwise seen from above",
  "spin_s": $SPIN_S,
  "t_sim_record_start": ${T_REC_START:-null},
  "t_sim_spin_command": ${T_SPIN:-null}
}
EOF
  echo "==> Wrote $RUN_DIR/spin_test.json"
else
  echo "warning: could not determine the run directory; spin_test.json not written." >&2
fi
