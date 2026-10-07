#!/usr/bin/env bash
# !!! KNOWN LIMITATION -- THE SIMULATED IMU FREEZES WHEN THE BODY IS EXACTLY AT REST !!!
# gz-sim only rewrites a link's velocity/acceleration components in steps where its pose
# changed (gz-sim 8.15.0 src/systems/physics/Physics.cc: ChangedLinks() 3376-3463 selects
# the moved links, UpdateSim() 3674-3760 writes pose/velocity/acceleration for those only).
# ImuPhysical reads Link::WorldLinearAcceleration()/WorldAngularVelocity(), so whenever the
# vehicle is bit-for-bit still it keeps reporting the values from its last moving step,
# plus noise -- the average is wrong, not constant (observed: 0.9 g through a perfectly
# balanced hover, 1.1 g after touchdown). Harmless in normal flight (never exactly still);
# it matters only at rest. The built-in profiles keep the vehicle drifting ~1 cm/s in the
# air so it never freezes there (see scripts/wrench_player.py), but resting on the ground
# after touchdown can still freeze it: compare gt_imu.csv with the odometry before reading
# EKF2 errors from the last seconds of a run.
#
# Wrench test for the estimator: launches the full stack (scripts/run_sim.sh), lets EKF2
# align at rest, then pushes the vehicle around with real forces and torques
# (scripts/wrench_player.py, through gz-sim's ApplyLinkWrench system) while
# scripts/record_vehicle_state.py records PX4's EKF2 estimate against Gazebo ground truth
# and plots it -- same figures and build/runs/<timestamp>/ layout as
# scripts/run_ekf2_debugging.sh, plus the applied wrench timeline.
#
# The forces go through the physics engine, so the accelerometer stays physically
# consistent (unlike scripts/run_spin_test.sh's kinematic VelocityControl, under which it
# reads free fall). ApplyLinkWrench is attached to the running world through
# /world/<world>/entity/system/add, so no .sdf file is involved.
#
# The vehicle stays DISARMED. PX4 therefore considers it landed throughout (the land
# detector reports landed whenever disarmed, PX4-Autopilot/src/modules/land_detector/
# MulticopterLandDetector.cpp:249,287,296), and EKF2 runs its on-ground logic. That is
# deliberate for a first check; if the estimate does not converge, an armed variant is the
# next step.
#
# Env:
#   HEADLESS=1          start gz-server without the GUI client (passed to run_sim.sh).
#   SETTLE_S=5          wall s after PX4 odometry first arrives, before anything moves.
#   PROFILE=lift_hover  built-in profile (scripts/wrench_player.py --list; all_dof moves
#                       every degree of freedom) or a JSON profile file.
#   TAIL_S=3            extra s recorded after the profile ends.
#   STOP_LAND_DETECTOR=0  1 = stop PX4's land_detector for this session, so EKF2 stops
#                       treating the disarmed vehicle as at rest (see the comment at that
#                       step). Default 0: plain disarmed behaviour.
#
# Extra arguments are passed through to record_vehicle_state.py, except --duration, which
# this script sets to the profile length + TAIL_S. E.g.:
#   HEADLESS=1 PROFILE=yaw_spin scripts/run_wrench_test.sh
#   PROFILE=my_profile.json scripts/run_wrench_test.sh --gt-angular-source odom
# Output lands in build/runs/<timestamp>/ (figures/ holds the plots), plus
# wrench_profile.json and wrench_applied.csv from the player.
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

# Must match the world name in worlds/monospinner_default.sdf.
WORLD="monospinner_default"
PX4_ODOM_TOPIC="/fmu/out/vehicle_odometry"
PX4_BIN_DIR="$REPO_ROOT/PX4-Autopilot/build/px4_sitl_default/bin"

SETTLE_S="${SETTLE_S:-5}"
PROFILE="${PROFILE:-lift_hover}"
TAIL_S="${TAIL_S:-3}"
STOP_LAND_DETECTOR="${STOP_LAND_DETECTOR:-0}"

# Validate the profile before starting anything.
PROFILE_S="$(python3 "$REPO_ROOT/scripts/wrench_player.py" --profile "$PROFILE" --print-duration)"
DURATION="$(python3 -c "print(float($PROFILE_S) + float($TAIL_S))")"

# A stack left over from an earlier session would answer every check below: run_sim.sh
# fails to start its own agent ("port 8888 may already be in use"), and this script would
# then push and record the OLD instance. Refuse instead.
# Patterns are anchored to the start of the command line, so they match the PX4 binary and
# the gz CLI themselves, not any process that merely mentions them (an unanchored "gz sim"
# matched the shell of a command that grepped for it).
STACK_PATTERN='^[^ ]*px4_sitl_default/bin/px4 |^gz sim '
if pgrep -f "$STACK_PATTERN" >/dev/null; then
  echo "error: a PX4 SITL or Gazebo instance is already running; stop it first:" >&2
  pgrep -af "$STACK_PATTERN" >&2 || true
  exit 1
fi

echo "==> Starting the simulation stack (scripts/run_sim.sh)"
"$REPO_ROOT/scripts/run_sim.sh" &
SIM_PID=$!
REC_PID=""
REC_LOG="$(mktemp)"
descendants() {  # every descendant PID of $1, deepest first
  local child
  for child in $(ps -o pid= --ppid "$1" 2>/dev/null); do
    descendants "$child"
    echo "$child"
  done
}
cleanup() {
  [[ -n "$REC_PID" ]] && kill "$REC_PID" 2>/dev/null || true
  # Captured before run_sim.sh exits, after which its children would be reparented.
  local session_pids pid stuck=()
  session_pids="$(descendants "$SIM_PID")"
  # run_sim.sh's own trap stops PX4, the agent and Gazebo when it receives SIGTERM -- but
  # it waits for each of them without a timeout, and it once hung there for >20 min after
  # a complete run. Give it 20 s, then kill whatever this session left behind, so the next
  # run's "already running" check does not trip over orphans.
  kill "$SIM_PID" 2>/dev/null || true
  for _ in $(seq 40); do
    kill -0 "$SIM_PID" 2>/dev/null || break
    sleep 0.5
  done
  for pid in $session_pids "$SIM_PID"; do
    kill -0 "$pid" 2>/dev/null && stuck+=("$pid")
  done
  if ((${#stuck[@]})); then
    echo "warning: the simulation stack did not shut down within 20 s; killing:" >&2
    ps -o pid=,args= -p "${stuck[@]}" >&2 || true
    kill -9 "${stuck[@]}" 2>/dev/null || true
  fi
  wait "$SIM_PID" 2>/dev/null || true
  rm -f "$REC_LOG"
}
trap cleanup EXIT

# Wait for an actual MESSAGE, not for the topic to be listed: `ros2 topic list` goes through
# the ROS 2 daemon, which remembers topics from earlier sessions, so it can succeed before
# this session's PX4 has even booted. Best-effort QoS to match PX4's uXRCE-DDS publishers.
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
# (:198-274). A steady rotation (e.g. yaw_spin's coast phase) passes both checks, so it
# would learn the rotation rate as a gyro offset -- applied live and persisted, breaking
# EKF2 initialisation on the next boot (observed: CAL_GYRO0_ZOFF -9.998 after a 10 rad/s
# spin). Stopping the module only affects this PX4 session; it is started again on every
# boot, and no parameter changes.
echo "==> Stopping PX4 gyro_calibration for this session"
# The px4-* clients reach instance 0 from run_sim.sh's PX4 working directory.
if ! (cd "$REPO_ROOT/build/px4_run" && "$PX4_BIN_DIR/px4-gyro_calibration" stop); then
  echo "error: could not stop gyro_calibration -- refusing to move a disarmed vehicle" \
    "(it could save a rotation rate as a gyro offset; see comment above)." >&2
  exit 1
fi

# --- optional: stop the land detector for this session ----------------------------------
# Disarmed, the land detector reports landed, and at_rest whenever the IMU vibration
# metrics are low (PX4-Autopilot/src/modules/land_detector/LandDetector.cpp:153,244-256) --
# always true for smooth simulated motion. EKF2 then fuses ZERO VELOCITY every 200 ms
# (sigma 0.2 m/s, src/modules/ekf2/EKF/aid_sources/ZeroVelocityUpdate.cpp:48-72) and the
# GYRO READING AS GYRO BIAS (ZeroGyroUpdate.cpp:51-69) while the vehicle moves. With the
# land detector stopped, its last message goes stale after 3 s and EKF2 falls back to
# in_air = armed (false here) and at_rest = false (src/modules/ekf2/EKF2.cpp:2613-2642,
# EKF/common.h:263-270): still the on-ground mode, but without those two fake measurements.
# Only this PX4 session is affected; the module starts again on every boot.
if [[ "$STOP_LAND_DETECTOR" == "1" ]]; then
  echo "==> Stopping PX4 land_detector for this session (STOP_LAND_DETECTOR=1)"
  if ! (cd "$REPO_ROOT/build/px4_run" && "$PX4_BIN_DIR/px4-land_detector" stop); then
    echo "error: could not stop land_detector." >&2
    exit 1
  fi
  # Let its last message go stale (EKF2 ignores it after 3 s) before anything moves.
  sleep 4
fi

# --- attach ApplyLinkWrench to the running world -----------------------------------------
echo "==> Attaching gz::sim::systems::ApplyLinkWrench to world '$WORLD'"
reply="$(gz service -s "/world/$WORLD/entity/system/add" \
  --reqtype gz.msgs.EntityPlugin_V --reptype gz.msgs.Boolean --timeout 3000 \
  --req "entity: {name: \"$WORLD\", type: WORLD}, plugins: [{name: \
\"gz::sim::systems::ApplyLinkWrench\", filename: \"gz-sim-apply-link-wrench-system\", \
innerxml: \"\"}]")"
if ! grep -q "data: true" <<<"$reply"; then
  echo "error: entity/system/add failed: $reply" >&2
  exit 1
fi
sleep 1

# --- record while the profile plays ------------------------------------------------------
echo "==> Recording ${DURATION} s (profile '${PROFILE}' ${PROFILE_S} s + ${TAIL_S} s tail)"
# -u: unbuffered, or the "==> Recording" line below only reaches $REC_LOG when the
# recorder exits and the profile would start after the recording window.
python3 -u "$REPO_ROOT/scripts/record_vehicle_state.py" "$@" --duration "$DURATION" \
  > >(tee "$REC_LOG") 2>&1 &
REC_PID=$!

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

# A player failure must not cut the recording short (set -e would otherwise stop the
# stack while the recorder is still writing): note it, let the recorder finish, report.
player_rc=0
python3 -u "$REPO_ROOT/scripts/wrench_player.py" --world "$WORLD" --profile "$PROFILE" \
  --out-dir "$RUN_DIR" || player_rc=$?

wait "$REC_PID"
REC_PID=""
if [[ "$player_rc" != "0" ]]; then
  echo "error: wrench_player.py exited with code $player_rc -- check the applied wrench" \
    "timeline in $RUN_DIR/wrench_applied.csv before trusting this run." >&2
  exit "$player_rc"
fi
echo "==> Done: $RUN_DIR"
