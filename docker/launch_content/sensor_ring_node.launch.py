import os
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
   
    package_path = FindPackageShare('sensor_ring')
    parameter_file = PathJoinSubstitution([
      '.',
      'edu_bot_sensor_ring_params.yaml'
    ])

    sensor_ring = Node(
      package='sensor_ring',
      executable='sensor_ring_node',
      name='sensor_ring_node',
      parameters=[parameter_file],
      #prefix='gdbserver localhost:3000',
      #namespace=EnvironmentVariable('EDU_ROBOT_NAMESPACE', default_value="eduard"),
      output='screen'
    )  

    # base_link to base_sensor_ring
    base_link_to_tof_tf = Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      name='base_link_to_base_tof',
      arguments=['0.0','0.0','0.0','0.0','0.0','0.0','base_link','base_sensor_ring']
    )

    return LaunchDescription([
        sensor_ring,
        base_link_to_tof_tf
    ])