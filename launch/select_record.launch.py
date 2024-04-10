from launch import LaunchDescription
from launch.actions import ExecuteProcess

list_txt = open('/home/nrf-mint/Downloads/mint_ws/src/mint_cart/config/topic_list.txt','r')
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
