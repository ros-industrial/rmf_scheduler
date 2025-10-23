FROM ros:jazzy-ros-base

# Install dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        python3-setuptools \
        python3-vcstool \
        python3-rosdep \
        uuid-runtime \
        python3-dev \
        python3-pip \
        coreutils \
        jq \
        ros-jazzy-rmw-cyclonedds-cpp \
  && rm -rf /var/lib/apt/lists/*

# Clone the repository
ENV RMF2_WS=/rmf2_ws
WORKDIR ${RMF2_WS}
COPY . ${RMF2_WS}/src/rmf2_scheduler

# Set up the workspace
WORKDIR ${RMF2_WS}
RUN apt-get update \
  && rosdep install --from-paths src --ignore-src --rosdistro=$ROS_DISTRO -y \
  && rm -rf /var/lib/apt/lists/*

# # Build the workspace
RUN . /opt/ros/$ROS_DISTRO/setup.sh \
  && colcon build

# # Ensure the entrypoint script sources the ROS setup
RUN sed -i '$isource "/rmf2_ws/install/setup.bash"' /ros_entrypoint.sh

# # Ensure proper permissions for entrypoint
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]

