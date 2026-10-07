#!/usr/bin/env bash
# Fetches pinned external dependencies (monospinner.repos) and initializes PX4's
# submodules. Idempotent: safe to re-run.
#
# Does NOT build anything -- see build_all.sh.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

if [[ ! -d /opt/ros/jazzy ]]; then
  echo "error: /opt/ros/jazzy not found -- this script must run inside the devcontainer." >&2
  exit 1
fi

if ! command -v vcs >/dev/null 2>&1; then
  echo "error: vcstool (vcs) not found -- expected inside the devcontainer." >&2
  exit 1
fi

if [[ ! -d PX4-Autopilot ]]; then
  echo "==> Importing pinned repositories from monospinner.repos"
  vcs import < monospinner.repos
else
  echo "==> PX4-Autopilot already present, skipping vcs import"
fi

if [[ -d PX4-Autopilot/.git ]]; then
  echo "==> Initializing PX4 submodules"
  git -C PX4-Autopilot submodule update --init --recursive
else
  echo "error: PX4-Autopilot/.git missing after import -- vcs import likely failed." >&2
  exit 1
fi

cat <<'EOF'

==> Workspace setup complete.

Next steps:
  1. scripts/build_all.sh        # build PX4 SITL, the gz plugin, and the ROS 2 workspace
  2. scripts/run_sim.sh          # launch Gazebo + PX4 SITL

EOF
