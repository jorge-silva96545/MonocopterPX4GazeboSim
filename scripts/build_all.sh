#!/usr/bin/env bash
# Builds PX4 SITL, the standalone Gazebo plugins, and the ROS 2 workspace, in that order.
#
# Usage: build_all.sh [px4|plugin|imu_plugin|mag_plugin|ros]
#   With no argument, builds all five.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

# colcon build needs ament_cmake's CMake modules on the prefix path, which only exist once
# the ROS environment is sourced -- don't assume the invoking shell already did this
# (interactive terminals do, via .bashrc, but a fresh non-interactive/scripted invocation
# won't).
if [[ -z "${AMENT_PREFIX_PATH:-}" ]]; then
  # ROS's setup.bash references unset variables internally and isn't `set -u` safe.
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  set -u
fi

TARGET="${1:-all}"

build_px4() {
  echo "==> Building PX4 SITL"
  make -C PX4-Autopilot px4_sitl
}

build_plugin() {
  echo "==> Building gz_monospinner_plugin"
  cmake -S src/gz_monospinner_plugin -B build/gz_plugin \
    -DCMAKE_INSTALL_PREFIX="$REPO_ROOT/install/gz_plugin" \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
  cmake --build build/gz_plugin
  cmake --install build/gz_plugin
}

build_imu_plugin() {
  echo "==> Building gz_imu_realism_plugin"
  # Installed to the same install/gz_plugin prefix as gz_monospinner_plugin (both are
  # native gz-sim system plugins) so run_sim.sh only needs one GZ_SIM_SYSTEM_PLUGIN_PATH
  # entry to find either.
  cmake -S src/gz_imu_realism_plugin -B build/gz_imu_plugin \
    -DCMAKE_INSTALL_PREFIX="$REPO_ROOT/install/gz_plugin" \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
  cmake --build build/gz_imu_plugin
  cmake --install build/gz_imu_plugin
}

build_mag_plugin() {
  echo "==> Building gz_magnetometer_realism_plugin"
  # Same install/gz_plugin prefix as the other gz-sim system plugins, see build_imu_plugin.
  cmake -S src/gz_magnetometer_realism_plugin -B build/gz_mag_plugin \
    -DCMAKE_INSTALL_PREFIX="$REPO_ROOT/install/gz_plugin" \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
  cmake --build build/gz_mag_plugin
  cmake --install build/gz_mag_plugin
}

build_ros() {
  echo "==> Building ROS 2 workspace"
  # --base-paths src scopes colcon's package discovery to src/. Without it, colcon
  # recursively scans the whole repo root and also picks up PX4-Autopilot/package.xml
  # (PX4's legacy ROS1 catkin manifest, name="px4", buildtool_depend=ament_cmake) as an
  # ament_cmake package. PX4's actual CMakeLists.txt never calls ament_package(), so
  # colcon emits a package.bash/.sh wrapper for it but no local_setup.bash -- breaking
  # `source install/setup.bash` with a "not found: .../px4/share/px4/local_setup.bash"
  # error, since the generic wrapper unconditionally sources it.
  colcon build --symlink-install --base-paths src
}

case "$TARGET" in
  px4)
    build_px4
    ;;
  plugin)
    build_plugin
    ;;
  imu_plugin)
    build_imu_plugin
    ;;
  mag_plugin)
    build_mag_plugin
    ;;
  ros)
    build_ros
    ;;
  all)
    build_px4
    build_plugin
    build_imu_plugin
    build_mag_plugin
    build_ros
    ;;
  *)
    echo "error: unknown target '$TARGET' (expected px4, plugin, imu_plugin, mag_plugin, ros, or no" \
      "argument)" >&2
    exit 1
    ;;
esac

echo "==> Done."
