FROM eduartrobotik/eduart-ros-base:jazzy-1.0.1

ENV USER=user
ENV DEBIAN_FRONTEND=noninteractive

# install packages
USER root
RUN apt-get update && apt-get install -y \
    software-properties-common \
    build-essential \
    git \
    libusb-1.0-0 \
    libusb-1.0-0-dev \
    ros-jazzy-rviz2

RUN apt-get update && apt-get install -y mesa-utils libgl1-mesa-dri libdrm-dev libdrm-intel1 libdrm2

USER $USER

# get the usbtingo library from github
RUN mkdir -p /home/$USER/eduart_ws/
WORKDIR /home/$USER/eduart_ws

RUN git clone --depth 1 --branch master https://github.com/hannesduske/libusbtingo.git \
    && cd libusbtingo \
    && mkdir build \
    && cd build \
    && cmake .. -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF\
    && cmake --build . -j4 \
    && sudo cmake --install .

# get the sensorring library from github
RUN git clone --depth 1 --branch master https://github.com/EduArt-Robotik/edu_lib_sensorring.git \
    && cd edu_lib_sensorring \
    && mkdir build \
    && cd build \
    && cmake .. -DBUILD_SHARED_LIBS=OFF -DVERBOSE=OFF -DCMAKE_BUILD_TYPE=Release -DUSE_USBTINGO=True\
    && cmake --build . -j4 \
    && sudo cmake --install .

# set up ros workspace
RUN mkdir -p /home/$USER/ros/src/
COPY edu_sensorring_ros2 /home/$USER/ros/src/edu_sensorring_ros2
WORKDIR /home/$USER/ros

# build the ros workspace
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && source /home/$USER/.bashrc \
    && colcon build --symlink-install --executor sequential --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && source install/setup.bash

# add entry point
COPY --chown=$USER edu_sensorring_ros2/docker/entrypoint /home/$USER/
RUN chmod a+x /home/$USER/entrypoint
ENTRYPOINT [ "/home/user/entrypoint" ]