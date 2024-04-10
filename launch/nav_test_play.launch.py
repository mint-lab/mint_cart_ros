from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    bag_dir = '/home/nrf-mint/rosbag2_2024_04_08-16_13_48/rosbag2_2024_04_08-16_13_48_0.db3'
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag_dir, '--topics', '/ublox_gps_node/fix', '/imu/data', '/zed2/zed_node/path_map'],
            output='screen'
        )
    ])
