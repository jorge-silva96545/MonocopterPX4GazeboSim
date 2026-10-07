#!/usr/bin/env bash
# Launches the full stack (Gazebo + MicroXRCEAgent + PX4 SITL, via scripts/run_sim.sh),
# records PX4's EKF2 estimate against Gazebo ground truth with
# scripts/record_vehicle_state.py, plots it, then shuts everything down.
#
# Env:
#   HEADLESS=1     start gz-server without the GUI client (passed through to run_sim.sh).
#   SETTLE_S=5     sim-independent wall seconds to wait after PX4's odometry first appears,
#                  so EKF2's start-up transient is not the whole recording.
#
# Extra arguments are passed through to record_vehicle_state.py, e.g.:
#   scripts/run_ekf2_debugging.sh --duration 30
#   HEADLESS=1 scripts/run_ekf2_debugging.sh --duration 10 --gt-angular-source odom
# Output lands in build/runs/<timestamp>/ (figures/ holds the plots).
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

PX4_ODOM_TOPIC="/fmu/out/vehicle_odometry"
SETTLE_S="${SETTLE_S:-5}"

echo "==> Starting the simulation stack (scripts/run_sim.sh)"
"$REPO_ROOT/scripts/run_sim.sh" &
SIM_PID=$!
# run_sim.sh's own trap stops PX4, the agent and Gazebo when it receives SIGTERM.
trap 'kill "$SIM_PID" 2>/dev/null || true; wait "$SIM_PID" 2>/dev/null || true' EXIT

echo "==> Waiting for $PX4_ODOM_TOPIC"
found=0
for _ in $(seq 90); do
  if ros2 topic list 2>/dev/null | grep -qx "$PX4_ODOM_TOPIC"; then
    found=1
    break
  fi
  if ! kill -0 "$SIM_PID" 2>/dev/null; then
    echo "error: scripts/run_sim.sh exited before $PX4_ODOM_TOPIC appeared." >&2
    exit 1
  fi
  sleep 1
done
if [[ "$found" != "1" ]]; then
  echo "error: $PX4_ODOM_TOPIC never appeared after 90 s -- is PX4 connecting to the" \
    "MicroXRCEAgent?" >&2
  exit 1
fi

echo "==> PX4 odometry is up; letting EKF2 settle for ${SETTLE_S} s"
sleep "$SETTLE_S"

python3 "$REPO_ROOT/scripts/record_vehicle_state.py" "$@"
