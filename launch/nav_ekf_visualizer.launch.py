import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    urdf_file = os.path.join(get_package_share_directory("mint_cart"), "config", "robot_state.urdf")
    rviz_config_file = os.path.join(get_package_share_directory("mint_cart"), "rviz", "ekf_mint.rviz")

    driver_node = Node(
        package='mint_cart',
        executable='ekf_node',
        output='screen',)
        
    robot_state_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[urdf_file],
        )
        
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_node',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom'],
        )
        
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen')
        
    return LaunchDescription([
        driver_node,
        robot_state_node,
        static_tf_node,
        rviz_node,
    ])
