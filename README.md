# EduArt Sensorring Ros2 Wrapper
The software interface of the EduArt Sensorring is the lightweight and framework independent [edu_lib_sensorring](https://github.com/EduArt-Robotik/edu_lib_sensorring) library. This repository is a wrapper of the library to use the Sensorring measurements in Ros2.

>Note: For a Ros1 wrapper see the [edu_sensorring_ros1](https://github.com/EduArt-Robotik/edu_sensorring_ros1) repository.

>Note: The Sensorring library currently only implements the Linux specific SocketCan interface for communication with the sensors. Therefore the Ros2 node is currently also limited to Linux systems.

# Building and running the node

The node can be run either locally on systems with a Ros2 environment or in a docker container that is also included in this repository.

## Running the node locally

To run the node locally you need to have Ros2 installed. Please refer to the official [Ros2 installation guide](https://docs.ros.org/en/jazzy/Installation.html).

### 1. Install the Sensorring library
Use the convenience script to build and install the Sensorring library on your system. Refer to the [edu_lib_sensorring](https://github.com/EduArt-Robotik/edu_lib_sensorring) repository for a detailed description of the build and installation process.

```
mkdir eduart_ws
cd eduart_ws
git clone -b master https://github.com/EduArt-Robotik/edu_lib_sensorring.git
cd edu_lib_sensorring
./cmake/install_release.bash
```
>Note:<br> If the bash script does not execute it may need to be converted to a file with unix style line endings. Run `dos2unix ./cmake/*` for an automatic conversion before executing the script.

### 2. Clone the Sensorring node into your Ros2 workspace
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone -b master https://github.com/EduArt-Robotik/edu_sensorring_ros2.git
```

### 3. Build the node
The node uses the standard Ros2 `colcon build` toolchain.
```
cd ~/ros2_ws
colcon build --packages-select edu_sensorring_ros2 --symlink-install
source ~/ros2_ws/install/setup.bash
```

### 4. Start the node
To start the node use the included launchfile.
```
ros2 launch edu_sensorring_ros2 edu_sensorring_ros2.launch.py
```

>Note:<br> The launchfile launches the node in a namespace that is set by the environment variable `EDU_ROBOT_NAMESPACE`. The default value is `eduard`.

>Note:<br> The launchfile [edu_sensorring_ros2.launch.py](launch/edu_sensorring_ros2.launch.py) uses the parameter set [edu_bot_sensorring_params.yaml](params/edu_bot_sensorring_params.yaml). You likely need to change the parameters to match your hardware configuration. Either create your own launchfile and parameter file or adjust the existing parameter file.


## Running the node in a container

Running the node in a container offers some advantages over natively running the node. It allows you to ...
- ... skip the manual installation of dependencies (e.g. the edu_lib_sensorring)
- ... run the node without installing Ros on your native system
- ... quickly change between different distributions of Ros
- ... autostart the node after boot

To use the container you need to install Docker. Please refer to the official [Docker installation guide](https://docs.docker.com/engine/install/).

### 1. Clone the Sensorring node into your workspace
```
mkdir ~/eduart_ws
cd ~/eduart_ws
git clone -b master https://github.com/EduArt-Robotik/edu_sensorring_ros2.git
```

### 2. Build the container
The container has to be built once after cloning the repository. During the build all dependencies are installed in the container.
```
cd ~/eduart_ws/edu_sensorring_ros2/docker
docker compose build
```

### 3. Start the container
Start the container by running `docker compose up`. The container is configured to restart automatically including after rebooting your system.
```
cd ~/eduart_ws/edu_sensorring_ros2/docker
docker compose up -d
```

You can verify that the container is running by listing all active containers. The list should include the Sensorring container.
```
docker ps
```
>Note:<br>The container uses some environment variables of the host system. Set the following variables according to your setup:<br>- EDU_ROBOT_NAMESPACE: Prefix for the Ros topics and Ros services of the node<br>- RMW_IMPLEMENTATION: Middleware implementation of Ros2 (e.g. "rmw_fastrtps_cpp") <br>- ROS_DOMAIN_ID: Specify a custom domain for the node (e.g. "42")

### 4. Stop the container
To stop the container and remove it from the autostart run `docker compose down`.
```
cd ~/eduart_ws/edu_sensorring_ros2/docker
docker compose down
```

Again you can verify that the container has stopped by listing all active containers. The Sensorring container should no longer be listed.
```
docker ps
```

# Ros interface of the node

## 1. Overview
This is a screenshot of the `rqt_graph` while the edu_sensorring_ros2 node is running. In the example the sensor board 0 has a ToF and a thermal sensor, the boards 1 and 2 only have ToF sensors. The Node publishes one point cloud per tof sensor and two combined point clouds with the measurements from all sensors. The first combined point cloud includes the raw points from all sensors, and the second one includes the transformed points in a common coordinate frame. The default launchfile also includes a static transform publisher that defines a reference frame for the Sensorring measurements.

<img src="doc/images/rqt_graph.png" width="600"/>

## 2. Publisher

### /sensors/tof_sensors/pcl_individual/sensor_*
There is one publisher per sensor board that publishes the point cloud data from its ToF sensor in the sensor coordinate frame. The message type is a [`sensor_msgs/msg/PointCloud2`](https://docs.ros2.org/foxy/api/sensor_msgs/msg/PointCloud2.html). The individual pxl publishers can be used as input for a voxel layer for autonomous navigation.

### /sensors/tof_sensors/pcl_raw
There is one publisher that publishes the point cloud data from all Time-of-Flight sensors of the Sensorring. The message type is a [`sensor_msgs/msg/PointCloud2`](https://docs.ros2.org/foxy/api/sensor_msgs/msg/PointCloud2.html). The combined measurements are **not** transformed into a common coordinate system. The points from the individual sensors can be distinguished by the PointField `sensor_idx` in the point cloud message.

### /sensors/tof_sensors/pcl_transformed
There is one publisher that publishes the point cloud data from all Time-of-Flight sensors of the Sensorring. The message type is a [`sensor_msgs/msg/PointCloud2`](https://docs.ros2.org/foxy/api/sensor_msgs/msg/PointCloud2.html). The combined measurements are transformed into a common coordinate system. The points from the individual sensors can be distinguished by the PointField `sensor_idx` in the point cloud message.

### /sensors/thermal_sensor_*/falsecolor
There is one publisher for each sensor that publishes a falsecolor image of the thermal measurement. The message type is [`sensor_msgs/msg/Image`](https://docs.ros2.org/foxy/api/sensor_msgs/msg/Image.html).

### /sensors/thermal_sensor_*/grayscale
There is one publisher for each thermal sensor that publishes a grayscale image of the thermal measurement. The message type is [`sensor_msgs/msg/Image`](https://docs.ros2.org/foxy/api/sensor_msgs/msg/Image.html).

## 3. Subscriber
There are currently no subscribers in the edu_sensorring_ros2 node.

## 4. Services

### /startThermalCalibration
The service `startThermalCalibration` starts the calibration process of the thermal sensors. The calibration compensates a vignette effect that is apparent in the thermal measurements.

```
ros2 service call /eduard/edu_sensorring_ros2_node/startThermalCalibration edu_sensorring_ros2/srv/StartThermalCalibration "window: 20"
```

>Thermal calibration:<br>1. Run the Sensorring node for 10 to 20 minutes that all sensors reach a steady operating temperature.<br>2. Place an object in front of the thermal sensors that covers the whole sensor area and has a uniform surface and uniform temperature, e.g. a cardboard box. Don't touch the side of your calibration target that is facing the sensor directly before the calibration.<br>3. Call the thermal calibration service. The window size defines how many thermal frames are averaged for the calibration. Recommended values are 20 to 100 frames.<br>4. Wait for the calibration to finish. If the `use_calibration_file` parameter is set to `true` the calibration values are automatically stored in a file and loaded at each start of the node.

### /stopThermalCalibration
The service `stopThermalCalibration` stops the calibration process of the thermal sensors in case the process takes too long.
```
ros2 service call /eduard/edu_sensorring_ros2_node/stopThermalCalibration edu_sensorring_ros2/srv/StopThermalCalibration "stop: true"
```
