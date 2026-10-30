#!/usr/bin/env bash
# Closed-loop run: PX4 SITL + EKF2 + the paper's reduced-attitude controller
# (gz_monospinner_control, a gz-sim system) + ThrustVectorActuator.
#
# Nothing outside this package is modified. Everything the stack needs beyond
# scripts/run_sim.sh's defaults is applied for THIS SESSION ONLY:
#
#  1. PX4's own motor output is silenced once PX4 is up: `param set-default
#     SIM_GZ_EC_FUNC1..4 0`. 0 (disabled) is the firmware default, so this drops the x500
#     airframe's runtime default (101..104), notifies the modules using the parameter and
#     saves nothing (PX4-Autopilot/src/lib/parameters/parameters.cpp:550-607) -- the next
#     boot gets 101..104 again. With no output function assigned,
#     GZMixingInterfaceESC::updateOutputs() counts zero active outputs and publishes
#     nothing (PX4-Autopilot/src/modules/simulation/gz_bridge/GZMixingInterfaceESC.cpp:
#     65-90); the script checks that. Without this PX4 publishes velocity-only messages on
#     the SAME topic as the controller, and ThrustVectorActuator skips every step on which
#     PX4's message is the latest (it returns early unless position_size() >= 2).
#     (A PX4_PARAM_SIM_GZ_EC_FUNCn=0 env override does NOT work: `param set` to the
#     firmware default stores nothing, and the airframe's later set-default wins.)
#  2. The vehicle flies DISARMED (PX4 commands nothing, so arming would only add arming
#     checks and auto-disarm logic). As in scripts/run_wrench_test.sh:
#     - gyro_calibration is stopped: disarmed, it would save the spin rate as a gyro
#       offset and persist it (see the comment in run_wrench_test.sh).
#     - land_detector is stopped (STOP_LAND_DETECTOR=1, default): disarmed it reports
#       landed/at rest, and EKF2 then fuses zero velocity and the gyro reading as gyro bias
#       while the vehicle flies. EKF2 still runs its on-ground logic (in_air = armed =
#       false); see run_wrench_test.sh for the source references.
#  3. The controller system is attached to the world (with <model_name>) at runtime through
#     /world/<world>/entity/system/add, so no .sdf file is involved.
#
# Then a setpoint schedule is played on /<model>/controller/setpoint and the run is
# recorded: scripts/record_vehicle_state.py (EKF2 vs ground truth, commands; unchanged)
# and record_controller.py (controller internals, its own plots).
#
# Env:
#   HEADLESS=1            no gz GUI (passed to run_sim.sh)
#   SETTLE_S=5            wall s after PX4 odometry first arrives (EKF2 aligns at rest)
#   STATE_SOURCE=ekf2     controller input: ekf2 (PX4's estimate) or ground_truth (exact
#                         state from Gazebo; EKF2 still runs and is recorded as a passenger)
#   STOP_LAND_DETECTOR=1  0 = leave PX4's land detector running (see 2. above)
#   RECORD_S=25           sim s recorded from the moment the controller is attached
#   SETPOINTS="8:0:5 11:90:5 14:180:5 17:270:5 20:0:0"
#                         "t:precession_deg:nutation_deg ..." -- t in wall s after the
#                         controller is attached (takeoff starts START_DELAY s after it).
#                         SETPOINTS="" plays none: the vehicle stays upright until
#                         setpoints are sent from another terminal with
#                         send_setpoint.sh <precession_deg> <nutation_deg>.
#   START_DELAY=1  TAKEOFF_ACCEL=0.3  TAKEOFF_TIME=1.5  UPDATE_RATE=500
#   KP="0.7 0.7 0.7"  KD="0 0 0"  KI="0 0 0"  P="1 1 1"  PREDICT_ATTITUDE=true
#
# Output: build/runs/<timestamp>/ from record_vehicle_state.py, plus controller.csv,
# controller_gt.csv and figures/controller_overview.png, controller_truth_vs_ekf2.png from
# record_controller.py.
#
# Needs: scripts/build_all.sh px4, plugin, imu_plugin, mag_plugin, and ros (which builds
# this package).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
PKG_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
REPO_ROOT="$(cd "$PKG_DIR/../.." && pwd)"
cd "$REPO_ROOT"

set +u
# shellcheck disable=SC1091
[[ -z "${AMENT_PREFIX_PATH:-}" ]] && source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "$REPO_ROOT/install/setup.bash"
set -u

WORLD="monospinner_default"
MODEL="monospinner"
PX4_ODOM_TOPIC="/fmu/out/vehicle_odometry"
PX4_BIN_DIR="$REPO_ROOT/PX4-Autopilot/build/px4_sitl_default/bin"
PLUGIN_LIB_DIR="$REPO_ROOT/install/gz_monospinner_control/lib"

SETTLE_S="${SETTLE_S:-5}"
STATE_SOURCE="${STATE_SOURCE:-ekf2}"
STOP_LAND_DETECTOR="${STOP_LAND_DETECTOR:-1}"
RECORD_S="${RECORD_S:-25}"
SETPOINTS="${SETPOINTS-8:0:5 11:90:5 14:180:5 17:270:5 20:0:0}"  # set but empty = none
START_DELAY="${START_DELAY:-1}"
TAKEOFF_ACCEL="${TAKEOFF_ACCEL:-0.3}"
TAKEOFF_TIME="${TAKEOFF_TIME:-1.5}"
UPDATE_RATE="${UPDATE_RATE:-500}"
KP="${KP:-0.7 0.7 0.7}"
KD="${KD:-0 0 0}"
KI="${KI:-0 0 0}"
P="${P:-1 1 1}"
PREDICT_ATTITUDE="${PREDICT_ATTITUDE:-true}"

if [[ ! -f "$PLUGIN_LIB_DIR/libgz_monospinner_control.so" ]]; then
  echo "error: $PLUGIN_LIB_DIR/libgz_monospinner_control.so not found --" \
    "run scripts/build_all.sh ros first." >&2
  exit 1
fi

STACK_PATTERN='^[^ ]*px4_sitl_default/bin/px4 |^gz sim '
if pgrep -f "$STACK_PATTERN" >/dev/null; then
  echo "error: a PX4 SITL or Gazebo instance is already running; stop it first:" >&2
  pgrep -af "$STACK_PATTERN" >&2 || true
  exit 1
fi

# Let gz-sim find this package's system library (run_sim.sh prepends its own path and
# keeps the inherited one).
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PLUGIN_LIB_DIR${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"

echo "==> Starting the simulation stack (scripts/run_sim.sh)"
"$REPO_ROOT/scripts/run_sim.sh" &
SIM_PID=$!
REC_PIDS=()
REC_LOG="$(mktemp)"
descendants() {  # every descendant PID of $1, deepest first
  local child
  for child in $(ps -o pid= --ppid "$1" 2>/dev/null); do
    descendants "$child"
    echo "$child"
  done
}
cleanup() {
  local pid
  for pid in "${REC_PIDS[@]}"; do kill "$pid" 2>/dev/null || true; done
  local session_pids stuck=()
  session_pids="$(descendants "$SIM_PID")"
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
  echo "error: no message on $PX4_ODOM_TOPIC after 90 s." >&2
  exit 1
fi
echo "==> PX4 odometry is up; letting EKF2 align at rest for ${SETTLE_S} s"
sleep "$SETTLE_S"

# 1. Silence PX4's ESC output for this session (see header), then check it.
echo "==> Disabling PX4's gz ESC outputs for this session (SIM_GZ_EC_FUNC1..4 -> 0)"
for i in 1 2 3 4; do
  (cd "$REPO_ROOT/build/px4_run" && "$PX4_BIN_DIR/px4-param" set-default "SIM_GZ_EC_FUNC$i" 0)
done
sleep 1
if timeout 2 gz topic -e -t "/$MODEL/command/motor_speed" -n 1 2>/dev/null | grep -q .; then
  echo "error: PX4 is still publishing on /$MODEL/command/motor_speed." >&2
  exit 1
fi
echo "    no PX4 messages on /$MODEL/command/motor_speed"

# 2. Disarmed flight: stop gyro_calibration (always) and land_detector (default).
echo "==> Stopping PX4 gyro_calibration for this session"
if ! (cd "$REPO_ROOT/build/px4_run" && "$PX4_BIN_DIR/px4-gyro_calibration" stop); then
  echo "error: could not stop gyro_calibration -- refusing to spin a disarmed vehicle." >&2
  exit 1
fi
if [[ "$STOP_LAND_DETECTOR" == "1" ]]; then
  echo "==> Stopping PX4 land_detector for this session"
  if ! (cd "$REPO_ROOT/build/px4_run" && "$PX4_BIN_DIR/px4-land_detector" stop); then
    echo "error: could not stop land_detector." >&2
    exit 1
  fi
  sleep 4  # EKF2 ignores the land detector's last message once it is 3 s old
fi

# Recorders first, so the takeoff is captured.
echo "==> Starting recorders (${RECORD_S} s of sim time)"
python3 -u "$REPO_ROOT/scripts/record_vehicle_state.py" --duration "$RECORD_S" \
  > >(tee "$REC_LOG") 2>&1 &
REC_PIDS+=($!)
started=0
for _ in $(seq 120); do
  if grep -q "^==> Recording" "$REC_LOG"; then started=1; break; fi
  if ! kill -0 "${REC_PIDS[0]}" 2>/dev/null; then
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
python3 -u "$SCRIPT_DIR/record_controller.py" --duration "$RECORD_S" --run-dir "$RUN_DIR" &
REC_PIDS+=($!)
sleep 1

# 3. Attach the controller to the model.
innerxml="<model_name>$MODEL</model_name><state_source>$STATE_SOURCE</state_source><update_rate>$UPDATE_RATE</update_rate><start_delay>$START_DELAY</start_delay>"
innerxml+="<takeoff_accel>$TAKEOFF_ACCEL</takeoff_accel><takeoff_time>$TAKEOFF_TIME</takeoff_time>"
innerxml+="<P>$P</P><Kp>$KP</Kp><Kd>$KD</Kd><Ki>$KI</Ki>"
innerxml+="<predict_attitude>$PREDICT_ATTITUDE</predict_attitude>"
echo "==> Attaching monospinner::MonospinnerController to world '$WORLD' (model '$MODEL', state source '$STATE_SOURCE')"
reply="$(gz service -s "/world/$WORLD/entity/system/add" \
  --reqtype gz.msgs.EntityPlugin_V --reptype gz.msgs.Boolean --timeout 5000 \
  --req "entity: {name: \"$WORLD\", type: WORLD}, plugins: [{name: \
\"monospinner::MonospinnerController\", filename: \"libgz_monospinner_control.so\", \
innerxml: \"$innerxml\"}]")"
if ! grep -q "data: true" <<<"$reply"; then
  echo "error: entity/system/add failed: $reply" >&2
  exit 1
fi
T_ATTACH=$SECONDS

# Setpoint schedule (wall-clock seconds after attaching; the sim runs at ~real time).
for sp in $SETPOINTS; do
  IFS=: read -r t_sp prec nut <<<"$sp"
  while ((SECONDS - T_ATTACH < t_sp)); do sleep 0.1; done
  echo "==> t+${t_sp}s setpoint: precession ${prec} deg, nutation ${nut} deg"
  gz topic -t "/$MODEL/controller/setpoint" -m gz.msgs.Vector2d \
    -p "x: $(python3 -c "import math; print(math.radians($prec))"), y: $(python3 -c "import math; print(math.radians($nut))")"
done

for pid in "${REC_PIDS[@]}"; do wait "$pid" || true; done
REC_PIDS=()
# Both recorders are done: re-plot with px4_odom.csv available (truth vs EKF2 figure).
python3 "$SCRIPT_DIR/record_controller.py" --plot-only "$RUN_DIR" || true
echo "==> Run directory: $RUN_DIR"
