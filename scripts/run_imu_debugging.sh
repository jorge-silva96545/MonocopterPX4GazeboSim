#!/usr/bin/env bash
# Launches Gazebo with the monospinner world, records ImuPhysical's noisy and ground-truth
# IMU topics with scripts/imu_debugging.py, plots them, then shuts Gazebo down.
#
# PX4 is not needed: ImuPhysical publishes both topics on its own, straight from the ECM.
#
# Env:
#   HEADLESS=1   start gz-server without the GUI client.
#
# Extra arguments are passed through to imu_debugging.py, e.g.:
#   scripts/run_imu_debugging.sh --duration 10 --output build/imu_debugging.png
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

WORLD_FILE="$REPO_ROOT/worlds/monospinner_default.sdf"
# Must match ImuPhysical's <ground_truth_topic> in models/monospinner/model.sdf.
GT_TOPIC="/model/monospinner/imu_gd_truth"

export GZ_SIM_RESOURCE_PATH="$REPO_ROOT/models:$REPO_ROOT/worlds${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$REPO_ROOT/install/gz_plugin/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"

# -r starts the simulation unpaused; ImuPhysical publishes nothing while paused.
GZ_ARGS=(-r "$WORLD_FILE")
if [[ "${HEADLESS:-0}" == "1" ]]; then
  GZ_ARGS=(-s "${GZ_ARGS[@]}")
fi

echo "==> Starting Gazebo"
gz sim "${GZ_ARGS[@]}" &
GZ_PID=$!
trap 'kill "$GZ_PID" 2>/dev/null || true; wait "$GZ_PID" 2>/dev/null || true' EXIT

echo "==> Waiting for $GT_TOPIC"
for _ in $(seq 60); do
  if gz topic -l 2>/dev/null | grep -qx "$GT_TOPIC"; then
    break
  fi
  if ! kill -0 "$GZ_PID" 2>/dev/null; then
    echo "error: Gazebo exited before $GT_TOPIC appeared." >&2
    exit 1
  fi
  sleep 1
done
if ! gz topic -l 2>/dev/null | grep -qx "$GT_TOPIC"; then
  echo "error: $GT_TOPIC never appeared -- is <publish_ground_truth> true in model.sdf," \
    "and was the plugin built and installed (scripts/build_all.sh imu_plugin)?" >&2
  exit 1
fi

python3 "$REPO_ROOT/scripts/imu_debugging.py" "$@"
