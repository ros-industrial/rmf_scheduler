FROM ros:humble-ros-base

# Install dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        python3-setuptools \
        python3-vcstool \
        python3-rosdep \
        python3-colcon-mixin \
        uuid-runtime \
        python3-dev \
        python3-pip \
        coreutils \
        clang \
        lld \
        jq \
        ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

RUN colcon mixin remove default
RUN colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
RUN colcon mixin update default

RUN pip3 install -U \
        datamodel-code-generator \
        websockets \
        fastapi \
        flask-socketio \
        uvicorn

# Clone the repository
WORKDIR /ros2_ws/src
RUN mkdir rmf_scheduler
COPY ./croncpp_vendor rmf_scheduler/croncpp_vendor
COPY ./ortools_vendor rmf_scheduler/ortools_vendor
COPY ./rmf_notification rmf_scheduler/rmf_notification
COPY ./rmf_scheduler rmf_scheduler/rmf_scheduler
COPY ./rmf_scheduler_plugins rmf_scheduler/rmf_scheduler_plugins
COPY ./rmf_scheduler_ros2 rmf_scheduler/rmf_scheduler_ros2
COPY ./taskflow_vendor rmf_scheduler/taskflow_vendor

# Set up the workspace
WORKDIR /ros2_ws
COPY rmf.repos rmf.repos
RUN vcs import src < rmf.repos
RUN rosdep update
RUN apt-get update && rosdep install --from-paths src --ignore-src --rosdistro=$ROS_DISTRO -y \
  && rm -rf /var/lib/apt/lists/*

# # Build the workspace
ENV CXX=clang++
ENV CC=clang
RUN . /opt/ros/$ROS_DISTRO/setup.sh \
  && colcon build --mixin release lld

# # Ensure the entrypoint script sources the ROS setup
RUN echo 'source /ros2_ws/install/setup.bash' >> /ros_entrypoint.sh

# # Ensure proper permissions for entrypoint
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]

