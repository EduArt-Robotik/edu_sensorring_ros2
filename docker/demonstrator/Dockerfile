#FROM eduartrobotik/eduart-ros-base:jazzy-1.0.1
FROM ros:jazzy-ros-base

ENV USER=user
ENV DEBIAN_FRONTEND=noninteractive

# install packages
USER root
RUN apt update \
    && apt install -y \
    software-properties-common \
    build-essential \
    git \
    sudo \
    libusb-1.0-0 \
    libusb-1.0-0-dev

RUN apt update \
    && apt install -y --fix-missing\
    ros-jazzy-rviz2

#USER $USER

# set up ros workspace
RUN mkdir -p /home/$USER/ros/src/
COPY edu_sensorring_ros2 /home/$USER/ros/src/edu_sensorring_ros2
WORKDIR /home/$USER/ros

# build the ros workspace
RUN bash -c "source /opt/ros/$ROS_DISTRO/setup.bash \
    && colcon build --symlink-install --executor sequential --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && source install/setup.bash"

# add entry point
COPY --chown=$USER edu_sensorring_ros2/docker/demonstrator/entrypoint /home/$USER/
RUN chmod a+x /home/$USER/entrypoint
ENTRYPOINT [ "/home/user/entrypoint" ]