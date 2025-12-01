FROM eduartrobotik/eduart-ros-base:jazzy-1.0.1

ENV USER=user
ENV DEBIAN_FRONTEND=noninteractive

# install packages
USER root
RUN apt-get update \
    && apt-get install -y software-properties-common build-essential git \
    && apt-get install -y cmake build-essential libusb-1.0-0 libusb-1.0-0-dev

USER $USER

# set up ros workspace
RUN mkdir -p /home/$USER/ros/src/
COPY edu_sensorring_ros2 /home/$USER/ros/src/edu_sensorring_ros2
WORKDIR /home/$USER/ros

# build the sensorring ros package
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && source /home/$USER/.bashrc \
    && colcon build --packages-select edu_sensorring_ros2 --symlink-install --executor sequential --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && source install/setup.bash

# add entry point
COPY --chown=$USER edu_sensorring_ros2/docker/entrypoint /home/$USER/
RUN chmod a+x /home/$USER/entrypoint
ENTRYPOINT [ "/home/user/entrypoint" ]