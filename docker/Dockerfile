FROM ros:humble AS base
ENV ROS_DISTRO=humble
ENV DEBIAN_FRONTEND=noninteractive
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV RCUTILS_COLORIZED_OUTPUT=1

ENV SHELL=/bin/bash
SHELL ["/bin/bash", "-c"]

# Define workspace path
ENV ROS_WS=/ros_ws

# Copy entrypoint
COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

# Set entrypoint
ENTRYPOINT ["/ros_entrypoint.sh"]

RUN apt update \
    && apt install -y \
    git \
    ssh \
    lsb-release \
    wget \
    vim \
    build-essential \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    ros-$ROS_DISTRO-rqt-tf-tree \
    ros-$ROS_DISTRO-rqt-reconfigure \
    ros-$ROS_DISTRO-rqt-common-plugins \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-image-transport \
    ros-$ROS_DISTRO-image-transport-plugins \
    && rm -rf /var/lib/apt/lists/*

# Setup rosdep
RUN apt update \
    # delete file to init rosdep, if file doesnt exist ignore error (||true)
    && rm /etc/ros/rosdep/sources.list.d/20-default.list || true \
    && rosdep init \
    && rosdep update \
    && rm -rf /var/lib/apt/lists/*

RUN echo "PS1='🐳  \[\033[1;36m\]\h \[\033[1;34m\]\W\[\033[0;35m\] \[\033[1;36m\]# \[\033[0m\]'" >> /root/.bashrc

#create workspace
WORKDIR $ROS_WS
RUN source /opt/ros/$ROS_DISTRO/setup.bash && mkdir src && colcon build
RUN echo 'source $ROS_WS/install/setup.bash' >>  /root/.bashrc


FROM base AS ros2_demos

RUN apt update  \
    && apt install -y \
    ros-${ROS_DISTRO}-example-interfaces \
    ros-${ROS_DISTRO}-ros-testing

COPY submodules/ros2_examples $ROS_WS/src/ros2_examples
COPY submodules/ros2_demos $ROS_WS/src/ros2_demos

RUN source /opt/ros/$ROS_DISTRO/setup.bash && colcon build

FROM base AS polygon_demo
COPY pluginlib/polygon_base $ROS_WS/src/polygon_base
COPY pluginlib/polygon_plugins $ROS_WS/src/polygon_plugins
RUN source /opt/ros/$ROS_DISTRO/setup.bash && colcon build --packages-select polygon_base polygon_plugins

FROM base AS pointcloud_demo
COPY pointcloud/pointcloud_demo $ROS_WS/src/pointcloud_demo
COPY pointcloud/pointcloud_segmentation $ROS_WS/src/pointcloud_segmentation
RUN apt update \
 && rosdep update \
 && rosdep install --from-paths ${ROS_WS}/src --ignore-src -r -y \
 && rm -rf /var/lib/apt/lists/*
RUN source /opt/ros/$ROS_DISTRO/setup.bash && colcon build --packages-select pointcloud_demo pointcloud_segmentation

FROM base AS zero_copy_demo
COPY zero_copy_transport/zero_copy_demo $ROS_WS/src/zero_copy_demo
RUN source /opt/ros/$ROS_DISTRO/setup.bash && colcon build --packages-select zero_copy_demo

FROM base AS sync_demo
COPY message_filters/sync_demo $ROS_WS/src/sync_demo
RUN source /opt/ros/$ROS_DISTRO/setup.bash && colcon build --packages-select sync_demo

FROM base AS sensor_calibration_demo
COPY interactive_sensor_calibration/sensor_calibration $ROS_WS/src/sensor_calibration
RUN apt update \
 && rosdep update \
 && rosdep install --from-paths ${ROS_WS}/src --ignore-src -r -y \
 && rm -rf /var/lib/apt/lists/*
RUN source /opt/ros/$ROS_DISTRO/setup.bash && colcon build --packages-select sensor_calibration

FROM base AS gazebo
COPY gazebo/room_sim $ROS_WS/src/room_sim
RUN apt update \
 && rosdep update \
 && rosdep install --from-paths ${ROS_WS}/src --ignore-src -r -y \
 && rm -rf /var/lib/apt/lists/*
RUN source /opt/ros/$ROS_DISTRO/setup.bash && colcon build --packages-select room_sim
