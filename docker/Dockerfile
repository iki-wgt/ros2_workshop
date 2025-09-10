FROM ros:humble AS base
ENV ROS_DISTRO=humble
ENV ROS_WS=/root/ros2_ws
ENV DEBIAN_FRONTEND=noninteractive
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV RCUTILS_COLORIZED_OUTPUT=1

ENV SHELL=/bin/bash
SHELL ["/bin/bash", "-c"]

COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
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



FROM base AS polygon_demo
COPY pluginlib/polygon_base $ROS_WS/src/polygon_base
COPY pluginlib/polygon_plugins $ROS_WS/src/polygon_plugins

WORKDIR $ROS_WS
RUN source /opt/ros/$ROS_DISTRO/setup.bash && colcon build --packages-select polygon_base polygon_plugins
