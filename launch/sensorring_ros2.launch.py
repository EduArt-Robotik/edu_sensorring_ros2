import os
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, Shutdown

from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
   
    package_path = FindPackageShare('sensorring_ros2')
    parameter_file = PathJoinSubstitution([
      package_path,
      'parameter',
      'edu_bot_sensorring_params.yaml'
    ])

    sensorring = Node(
      package='sensorring_ros2',
      executable='sensorring_ros2_node',
      name='sensorring_ros2_node',
      parameters=[parameter_file],
      #arguments=['--ros-args', '--log-level', 'DEBUG'],
      #prefix='gdbserver localhost:3000',
      #namespace=EnvironmentVariable('EDU_ROBOT_NAMESPACE', default_value="eduard"),
      output='screen',
      on_exit=Shutdown()
    )

    # base_link to base_sensor_ring
    base_link_to_tof_tf = Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      name='base_link_to_base_tof',
      arguments=["--x", "0.0","--y", "0.0","--z", "0.0","--roll",  "0.0","--pitch", "0.0","--yaw",  "0.0", "--frame-id", "base_link","--child-frame-id", "base_sensor_ring"],
      output="screen"
    )

    return LaunchDescription([
        sensorring,
        base_link_to_tof_tf
    ])