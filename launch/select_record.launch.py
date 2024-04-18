import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess

package_name = 'mint_cart'
config_file_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'topic_list.txt'
    )

list_txt = open(config_file_path, 'r')
topics = []

for line in list_txt:
    topics.append(line[:-1])

print(topics)

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', 'topics']+topics,
            output='screen'
        )
    ])
