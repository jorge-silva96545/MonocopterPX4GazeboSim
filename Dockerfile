# Two stages:
#
#   dev      Toolchain only (ROS 2 Jazzy, Gazebo Harmonic, PX4 build deps, Micro XRCE-DDS
#            Agent). Used by .devcontainer/devcontainer.json, which bind-mounts the source
#            and builds it in place.
#
#   (final)  dev + this repo, with the pinned dependencies fetched and everything built.
#            `docker build -t monospinner .` gives an image that can run the simulation
#            straight away -- see README.md.
#
# PX4 bakes absolute paths into its build at configure time, so the final stage builds at a
# fixed path (/home/dev/monospinner) and the result is only valid inside this image.

FROM osrf/ros:jazzy-desktop AS dev

ARG USERNAME=dev
RUN apt-get update && apt-get install -y sudo git wget \
 && useradd -m -s /bin/bash $USERNAME \
 && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME

USER $USERNAME
WORKDIR /tmp/px4setup

ARG PX4_TAG=v1.17.0
RUN wget https://raw.githubusercontent.com/PX4/PX4-Autopilot/${PX4_TAG}/Tools/setup/ubuntu.sh \
 && wget https://raw.githubusercontent.com/PX4/PX4-Autopilot/${PX4_TAG}/Tools/setup/requirements.txt \
 && DEBIAN_FRONTEND=noninteractive bash ubuntu.sh --no-nuttx

RUN sudo apt-get update && sudo apt-get install -y libgz-sim8-dev

RUN sudo apt-get update && sudo apt-get install -y \
    python3-vcstool \
    python3-colcon-common-extensions

# Micro XRCE-DDS Agent, built from source per PX4's ROS 2 setup docs
# (https://docs.px4.io/main/en/ros2/user_guide.html#setup-micro-xrce-dds-agent-client).
RUN sudo apt-get update && sudo apt-get install -y build-essential cmake git \
 && git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/Micro-XRCE-DDS-Agent \
 && cd /tmp/Micro-XRCE-DDS-Agent \
 && mkdir build && cd build \
 && cmake .. \
 && make -j"$(nproc)" \
 && sudo make install \
 && sudo ldconfig /usr/local/lib/ \
 && rm -rf /tmp/Micro-XRCE-DDS-Agent

# Without this, every non-login shell (including scripted/tool invocations, not just
# interactive terminals) starts with no ROS environment, and `colcon build` fails with
# "ament_cmake not found" until sourced manually.
# GZ_SIM_RESOURCE_PATH / GZ_SIM_SYSTEM_PLUGIN_PATH are deliberately not set here:
# scripts/run_sim.sh derives both from the repo location at launch.
RUN echo "source /opt/ros/jazzy/setup.bash" >> "$HOME/.bashrc"


FROM dev

ENV MONOSPINNER_WS=/home/dev/monospinner
WORKDIR $MONOSPINNER_WS
COPY --chown=dev:dev . .

# Fetch the pinned external dependencies (monospinner.repos) and build PX4 SITL, the
# gz-sim plugins and the ROS 2 workspace. Kept as two layers so a failed build does not
# re-download PX4 and its submodules.
RUN scripts/setup_workspace.sh
RUN scripts/build_all.sh

# Workspace overlay (px4_msgs, monospinner_control) on top of the base ROS 2 install.
RUN echo "source $MONOSPINNER_WS/install/setup.bash" >> "$HOME/.bashrc"

CMD ["bash"]
