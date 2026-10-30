# monospinner-sitl

A Gazebo Harmonic + PX4 SITL simulation of a mono-spinner UAV — a single-rotor aerial
vehicle using swashplateless torque modulation for attitude control — built to develop and
test new flight controllers against PX4's own state estimate.

Reference: E. Bernardes, F. Boyer, S. Viollet, *Modelling, control and simulation of a
single rotor UAV with swashplateless torque modulation*, Aerospace Science and Technology,
2023. Preprint: [hal-04139662](https://amu.hal.science/hal-04139662v1).

| Component | Version |
| --- | --- |
| Ubuntu | 24.04 |
| ROS 2 | Jazzy |
| Gazebo | Harmonic (gz-sim 8) |
| PX4-Autopilot | `v1.17.0` |
| px4_msgs | `release/1.17` |
| px4-ros2-interface-lib | `1.6.1` |

PX4 and the ROS 2 dependencies are not vendored: they are pinned in
[monospinner.repos](monospinner.repos) and fetched by `scripts/setup_workspace.sh`. PX4 is
used unmodified.

## Host requirements

- **Linux** with Docker. The container uses `--network=host` (Gazebo Transport and Fast
  DDS discover each other over UDP multicast, which does not cross Docker's bridge
  network), so Docker Desktop on macOS/Windows is not supported.
- **An X11 session** for the Gazebo GUI. Gazebo Harmonic fails to launch under Wayland.
  Headless runs (`HEADLESS=1`) need no display at all, but the devcontainer still
  bind-mounts `/tmp/.X11-unix`: on a host with no X server, `mkdir /tmp/.X11-unix` first.
- **`/dev/dri`** must exist on the host (any Mesa/Intel/AMD GPU, or the NVIDIA driver with
  its DRM device). The container is started with `--device=/dev/dri` and will not start
  without it; remove that line from `.devcontainer/devcontainer.json` (or the
  `docker run` command below) on a machine without a GPU — Gazebo then falls back to
  software rendering.
- About 25 GB of free disk, and a while for the first build: PX4 and its submodules are
  cloned and compiled from source.

If the GUI can't open the display (`could not connect to display`), allow local
connections to your X server on the host:

```bash
xhost +local:      # revoke later with: xhost -local:
```

## Option A — VS Code devcontainer (development)

The repo is bind-mounted into the container; source lives on the host, and **everything
is built and run inside the container**. PX4 bakes absolute paths into its build, so a
`build/` produced on the host does not work in the container or vice versa.

1. Clone the repo and open it in VS Code, then run
   *Dev Containers: Reopen in Container* (or `devcontainer up --workspace-folder .`).
   The `dev` user is remapped to your host UID/GID, so files created in the container stay
   owned by you on the host.
2. In a container terminal:

   ```bash
   scripts/setup_workspace.sh   # fetch PX4-Autopilot, px4_msgs, px4-ros2-interface-lib
   scripts/build_all.sh         # PX4 SITL, the gz-sim plugins, the ROS 2 workspace
   scripts/run_sim.sh           # Gazebo + PX4 SITL + Micro XRCE-DDS Agent
   ```

   Open a new terminal after the first build so `install/setup.bash` is sourced.

## Option B — standalone image

The default target of the [Dockerfile](Dockerfile) fetches the pinned dependencies and
builds everything inside the image, at `/home/dev/monospinner`:

```bash
docker build -t monospinner .

docker run -it --rm \
  --network=host --ipc=host \
  --device=/dev/dri \
  -e DISPLAY="$DISPLAY" -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  monospinner
```

Then, in the container's shell, `scripts/run_sim.sh` (or `HEADLESS=1 scripts/run_sim.sh`).
Nothing is mounted from the host, so changes and recorded runs are lost when the container
exits — use Option A for development.

## Running

| Script | What it does |
| --- | --- |
| `scripts/run_sim.sh` | Gazebo server (+ GUI unless `HEADLESS=1`), Micro XRCE-DDS Agent, PX4 SITL with `config/monospinner.params` applied |
| `src/gz_monospinner_control/scripts/run_closed_loop.sh` | Closed-loop flight with the reduced-attitude controller on PX4's EKF2 estimate; records to `build/runs/<timestamp>/`. Options are documented at the top of the script. |
| `scripts/open_loop_test.py` | Open-loop physics test of the actuator plugin: gz-server only, PX4 out of the loop |

`scripts/build_all.sh <target>` rebuilds a single part: `px4`, `plugin`, `imu_plugin`,
`mag_plugin` or `ros`.

## Layout

```
.devcontainer/       devcontainer definition (uses the Dockerfile's `dev` stage)
Dockerfile           toolchain image (`dev`) and fully built image (default target)
monospinner.repos    pinned external dependencies (vcstool)
config/              PX4 parameter overrides
models/monospinner/  SDF model: single link, rotor modelled as a force
worlds/              simulation world
scripts/             setup / build / run / test tools
src/
  gz_monospinner_plugin/            gz-sim system: thrust-vector actuator physics
  gz_imu_realism_plugin/            gz-sim system: IMU model
  gz_magnetometer_realism_plugin/   gz-sim system: magnetometer model
  gz_monospinner_control/           reduced-attitude controller (gz-sim system + ROS 2)
```
