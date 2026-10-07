#!/usr/bin/env bash
# Launches a standalone Gazebo server with the monospinner world, then attaches PX4 SITL
# to it. PX4's own multicopter controllers are not relevant here -- this only needs the
# vehicle to spawn and PX4 to connect.
#
# Env:
#   HEADLESS=1   start gz-server without the GUI client.
#
# Verified against PX4-Autopilot/Tools/simulation/sitl_multiple_run.sh (px4 binary -i/-d
# invocation -- -d takes the build's etc/ dir, not rootfs/, which holds only .gdbinit and
# gz_env.sh) and PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/px4-rc.gzsim (PX4_GZ_WORLD
# must match the world name in the .sdf -- gz_bridge attaches with `-w "$PX4_GZ_WORLD"` and
# the scene-info readiness check polls /world/${PX4_GZ_WORLD}/scene/info).
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

WORLD_FILE="$REPO_ROOT/worlds/monospinner_default.sdf"
PX4_BUILD_DIR="$REPO_ROOT/PX4-Autopilot/build/px4_sitl_default"
PX4_BIN="$PX4_BUILD_DIR/bin/px4"
PX4_ETC="$PX4_BUILD_DIR/etc"
PARAM_FILE="$REPO_ROOT/config/monospinner.params"

# PX4 writes its runtime state (dataman, eeprom, parameters.bson, logs, test_data) into
# whatever the cwd is at launch. Run from a dedicated, gitignored dir under build/ so none
# of that lands next to source files.
RUN_DIR="$REPO_ROOT/build/px4_run"
mkdir -p "$RUN_DIR"
cd "$RUN_DIR"

export GZ_SIM_RESOURCE_PATH="$REPO_ROOT/models:$REPO_ROOT/worlds${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$REPO_ROOT/install/gz_plugin/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"

if [[ ! -x "$PX4_BIN" ]]; then
  echo "error: $PX4_BIN not found -- run scripts/build_all.sh px4 first." >&2
  exit 1
fi

if ! command -v MicroXRCEAgent >/dev/null 2>&1; then
  echo "error: MicroXRCEAgent not found on PATH -- see the Dockerfile for the" \
    "build-from-source install step." >&2
  exit 1
fi

# containerEnv.DISPLAY (devcontainer.json) is captured once, at container creation, from
# the host's $DISPLAY at that moment. It goes stale -- with no error, just a Qt/xcb crash
# from the GUI client -- if the host's X display number changes afterward (e.g. a
# reconnected remote-desktop session landing on :1 instead of :0), since nothing
# re-syncs it. Detect that here rather than let gz's GUI fail with a cryptic native stack
# trace.
if [[ "${HEADLESS:-0}" != "1" ]]; then
  configured_socket="/tmp/.X11-unix/X${DISPLAY#:}"
  if [[ ! -S "$configured_socket" ]]; then
    available_sockets=(/tmp/.X11-unix/X*)
    if [[ ${#available_sockets[@]} -eq 1 && -S "${available_sockets[0]}" ]]; then
      corrected_display=":${available_sockets[0]##*/X}"
      echo "warning: DISPLAY=$DISPLAY has no matching socket ($configured_socket);" \
        "found exactly one available display, using $corrected_display instead." >&2
      export DISPLAY="$corrected_display"
    else
      echo "error: DISPLAY=$DISPLAY has no matching socket ($configured_socket), and" \
        "there isn't exactly one alternative to fall back to (found:" \
        "${available_sockets[*]:-none}). Set DISPLAY manually, or use HEADLESS=1." >&2
      exit 1
    fi
  fi
fi

GZ_PID=""
PX4_PID=""
AGENT_PID=""

cleanup() {
  echo "==> Shutting down"
  [[ -n "$PX4_PID" ]] && kill "$PX4_PID" 2>/dev/null || true
  [[ -n "$AGENT_PID" ]] && kill "$AGENT_PID" 2>/dev/null || true
  [[ -n "$GZ_PID" ]] && kill "$GZ_PID" 2>/dev/null || true
  [[ -n "$PX4_PID" ]] && wait "$PX4_PID" 2>/dev/null || true
  [[ -n "$AGENT_PID" ]] && wait "$AGENT_PID" 2>/dev/null || true
  [[ -n "$GZ_PID" ]] && wait "$GZ_PID" 2>/dev/null || true
}
trap cleanup SIGINT SIGTERM EXIT

echo "==> Starting Gazebo (world: $WORLD_FILE, headless: ${HEADLESS:-0})"
if [[ "${HEADLESS:-0}" == "1" ]]; then
  gz sim -s -r "$WORLD_FILE" &
else
  gz sim -r "$WORLD_FILE" &
fi
GZ_PID=$!

echo "==> Waiting for gz-server"
for _ in $(seq 1 60); do
  if gz topic -l 2>/dev/null | grep -q '/clock$'; then
    break
  fi
  sleep 0.5
done
if ! gz topic -l 2>/dev/null | grep -q '/clock$'; then
  echo "error: gz-server did not come up (no /clock topic after 30s)" >&2
  exit 1
fi
echo "==> gz-server is up"

echo "==> Starting Micro XRCE-DDS Agent (udp4, port 8888)"
# 8888 matches uxrce_dds_port in
# PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/rcS, the default PX4's uxrce_dds_client
# connects to unless PX4_UXRCE_DDS_PORT overrides it (not set here). Without this agent
# running, PX4's DDS client has nothing to talk to and no /fmu/out/* topics ever reach ROS 2.
MicroXRCEAgent udp4 -p 8888 &
AGENT_PID=$!

sleep 0.5
if ! kill -0 "$AGENT_PID" 2>/dev/null; then
  echo "error: MicroXRCEAgent exited immediately -- port 8888 may already be in use" \
    "(e.g. an instance started by hand in another terminal)." >&2
  exit 1
fi

# Parameter overrides are applied through PX4's own documented env-var hook, not by
# poking a running instance: ROMFS/px4fmu_common/init.d-posix/rcS:129-134 loops over the
# environment and runs `param set "${name#PX4_PARAM_}" "$value"` for every PX4_PARAM_*
# variable. That runs DURING boot, which is the only point at which a parameter marked
# `reboot_required: true` in a module.yaml (UXRCE_DDS_SYNCT is one) can take effect for
# this session. Setting such a parameter on an already-running instance via px4-param
# silently does nothing until the next restart.
# config/monospinner.params holds one `param set <NAME> <VALUE>` per line.
PARAM_ENV=()
if [[ -f "$PARAM_FILE" ]]; then
  while read -r kw op name value; do
    if [[ "$kw" != "param" || "$op" != "set" || -z "$name" || -z "$value" ]]; then
      echo "warning: ignoring unrecognised line in $PARAM_FILE: $kw $op $name $value" >&2
      continue
    fi
    PARAM_ENV+=("PX4_PARAM_${name}=${value}")
  done < <(grep -v -e '^\s*#' -e '^\s*$' "$PARAM_FILE" || true)
fi

if [[ ${#PARAM_ENV[@]} -gt 0 ]]; then
  echo "==> Applying $PARAM_FILE overrides via PX4_PARAM_* (rcS:129-134):"
  printf '    %s\n' "${PARAM_ENV[@]}"
else
  echo "==> No active overrides in $PARAM_FILE"
fi

echo "==> Starting PX4 SITL (standalone, model=monospinner, autostart=4001)"
env \
  PX4_GZ_STANDALONE=1 \
  PX4_SYS_AUTOSTART=4001 \
  PX4_GZ_MODEL_NAME=monospinner \
  PX4_GZ_WORLD=monospinner_default \
  "${PARAM_ENV[@]}" \
  "$PX4_BIN" -i 0 -d "$PX4_ETC" &
PX4_PID=$!

wait "$PX4_PID"
