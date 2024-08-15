import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

def load_topics(context):
    # Load topics from YAML file
    package_name = 'mint_cart'
    config_file_path = os.path.join(
            get_package_share_directory(package_name),
            'config',
            'topic_list.yaml'
    )
    all_topics = []
    with open(config_file_path, 'r') as file:
        all_topics = yaml.full_load(file)

    camera_model = LaunchConfiguration('camera_model').perform(context)
    use_ahrs = LaunchConfiguration('use_ahrs').perform(context)
    use_rtk_gps = LaunchConfiguration('use_rtk_gps').perform(context)
    use_normal_gps = LaunchConfiguration('use_normal_gps').perform(context)
    use_lidar = LaunchConfiguration('use_lidar').perform(context)
    use_ascender = LaunchConfiguration('use_ascender').perform(context)

    print(f"Camera Model: {camera_model}")
    print(f"Use AHRS: {use_ahrs}")
    print(f"Use RTKGPS: {use_rtk_gps}")
    print(f"Use NormalGPS: {use_normal_gps}")
    print(f"Use Lidar: {use_lidar}")
    print(f"Use Ascender: {use_ascender}")

    record_topics = []
    # Add topics based on the launch arguments
    if camera_model in all_topics: # Ensure camera model is a valid key
        record_topics.extend(all_topics[camera_model])
    if use_ahrs == 'true':
        record_topics.extend(all_topics['ahrs'])
    if use_rtk_gps == 'true':
        record_topics.extend(all_topics['rtk_gps'])
    if use_normal_gps == 'true':
        record_topics.extend(all_topics['normal_gps'])
    if use_lidar == 'true':
        record_topics.extend(all_topics['lidar'])
    if use_ascender == 'true':
        record_topics.extend(all_topics['ascender'])

    print(record_topics)

    return [
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', 'topics'] + record_topics,
            output='screen'
        )
    ]


def generate_launch_description():
    # Declare launch arguments
    camera_model_arg = DeclareLaunchArgument(
        'camera_model', default_value='zed2',
        description="Specify the camera model. e.g. `zed2`, `zed2i`"
    )
    use_ahrs_arg = DeclareLaunchArgument(
        'use_ahrs', default_value='true',
        description="true: Record AHRS data"
    )
    use_rtk_gps_arg = DeclareLaunchArgument(
        'use_rtk_gps', default_value='true',
        description="true: Record RTKGPS data"
    )
    use_normal_gps_arg = DeclareLaunchArgument(
        'use_normal_gps', default_value='true',
        description="true: Record NormalGPS data"
    )
    use_lidar_arg = DeclareLaunchArgument(
        'use_lidar', default_value='false',
        description="true: Record LiDAR data"
    )
    use_ascender_arg = DeclareLaunchArgument(
        'use_ascender', default_value='true',
        description="true: Record Ascender data"
    )

    return LaunchDescription([
        camera_model_arg,
        use_ahrs_arg,
        use_rtk_gps_arg,
        use_normal_gps_arg,
        use_lidar_arg,
        use_ascender_arg,
        OpaqueFunction(function=load_topics)
    ])
