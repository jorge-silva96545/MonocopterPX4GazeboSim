#!/usr/bin/env bash
# Runs once after the devcontainer is created. Handles two things that can't be baked into
# the image at build time because they depend on the host this container happens to run on:
#
# 1. Workspace ownership. The repo is bind-mounted from the host; the host directory's
#    owning UID/GID is whatever the host user's are, which will not generally match the
#    "dev" user created in the Dockerfile. Reconcile it here instead of guessing a UID at
#    build time.
#
# 2. GPU render access. devcontainer.json passes --device=/dev/dri so Gazebo's Sensors
#    system can do headless (or accelerated) rendering. The device nodes are owned by
#    host-specific video/render groups, and the render group's GID is not standardized
#    across distros/hosts (unlike "video", which is almost always 44) -- it depends on
#    driver/udev ordering on the host. Detect the actual GIDs at container start and join
#    them, rather than hardcoding a GID that only matches the machine this was written on.
set -euo pipefail

WORKSPACE_DIR="${1:-$PWD}"

echo "==> postCreate: chown $WORKSPACE_DIR to $(id -un):$(id -gn)"
sudo chown -R "$(id -u):$(id -g)" "$WORKSPACE_DIR"

for dev_node in /dev/dri/card0 /dev/dri/card1 /dev/dri/renderD128; do
  [[ -e "$dev_node" ]] || continue
  gid="$(stat -c '%g' "$dev_node")"
  group_name="$(getent group "$gid" | cut -d: -f1 || true)"
  if [[ -z "$group_name" ]]; then
    group_name="gpu_${gid}"
    echo "==> postCreate: creating group $group_name (gid $gid) for $dev_node"
    sudo groupadd -g "$gid" "$group_name"
  fi
  if ! id -Gn | tr ' ' '\n' | grep -qx "$group_name"; then
    echo "==> postCreate: adding $(id -un) to $group_name (gid $gid)"
    sudo usermod -aG "$group_name" "$(id -un)"
  fi
done

SOURCE_LINE="[[ -f \"$WORKSPACE_DIR/install/setup.bash\" ]] && source \"$WORKSPACE_DIR/install/setup.bash\""
# The Dockerfile sources /opt/ros/jazzy/setup.bash in .bashrc, but that's the base ROS 2
# install only -- it has no notion of px4_msgs (VehicleOdometry, etc.), which lives in this
# repo's own colcon workspace and is only produced by `build_all.sh ros`. Without also
# sourcing the workspace overlay, `ros2 topic echo /fmu/out/...` fails with "message type is
# invalid" in any terminal that didn't manually source install/setup.bash first. Guarded with
# -f (not baked in unconditionally) because install/ doesn't exist until the first ROS build,
# and guarded against duplication so re-running postCreate (container rebuild, "Rerun Post
# Create Command") doesn't pile up repeated lines in .bashrc.
if ! grep -qF "$SOURCE_LINE" "$HOME/.bashrc" 2>/dev/null; then
  echo "==> postCreate: adding workspace overlay ($WORKSPACE_DIR/install/setup.bash) to .bashrc"
  echo "$SOURCE_LINE" >> "$HOME/.bashrc"
fi

echo "==> postCreate done. Open a new terminal (or reconnect) for group membership to take effect."
