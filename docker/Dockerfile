FROM eduartrobotik/eduart-ros-base:jazzy-1.0.1

ENV USER=user
ENV DEBIAN_FRONTEND=noninteractive

# install mraa lib for UART communication in shield node
USER root
RUN apt-get update \
    && apt-get install -y software-properties-common git build-essential

USER $USER

RUN mkdir -p /home/$USER/eduart_ws/
COPY --chown=$USER eduart_ws/sensorring /home/$USER/eduart_ws/sensorring
WORKDIR /home/$USER/eduart_ws/sensorring
RUN rm build -r \
    && mkdir build \
    && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release && cmake --build . -j && cmake --install . --prefix /home/$USER/sensorring

ENV CMAKE_PREFIX_PATH=/home/$USER/sensorring
 
# set up ros workspace
RUN mkdir -p /home/$USER/ros/src/
COPY ros2_ws/src/sensorring_ros2 /home/$USER/ros/src/sensorring_ros2
WORKDIR /home/$USER/ros

# build EduArt packages nodes
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && source /home/$USER/.bashrc \
    && colcon build --packages-select sensorring_ros2 --symlink-install --executor sequential --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && source install/setup.bash

# add entry point
COPY --chown=$USER ./ros2_ws/src/sensorring_ros2/docker/entrypoint /home/$USER/
RUN chmod a+x /home/$USER/entrypoint
ENTRYPOINT [ "/home/user/entrypoint" ]