import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
from launch.actions import EmitEvent
from launch.actions import RegisterEventHandler
from launch_ros.events.lifecycle import ChangeState
from launch_ros.events.lifecycle import matches_node_name
from launch_ros.event_handlers import OnStateTransition
from launch.actions import LogInfo
from launch.events import matches_action
from launch.event_handlers.on_shutdown import OnShutdown
from launch_ros import actions
import lifecycle_msgs.msg
import launch

def generate_launch_description():
    
    #myahrs+
    myahrs_config_dir = get_package_share_directory('myahrs_ros2_driver')
    myahrs_config_file = os.path.join(myahrs_config_dir, 'config', 'config.yaml')
    
    #ublox_gps
    ublox_config_file = os.path.join(get_package_share_directory("nmea_navsat_driver"), "config", "ublox_serial_driver.yaml")
    ublox_driver_node = actions.Node(
        package='nmea_navsat_driver',
        executable='nmea_serial_driver',
        output='screen',
        remappings=[("fix", "ublox/fix")],
        parameters=[ublox_config_file, {'port': LaunchConfiguration('port')}])

    #ouster 3d lidar
    ouster_share_dir = get_package_share_directory('ros2_ouster')
    ouster_parameter_file = LaunchConfiguration('params_file')
    ouster_node_name = 'ouster_driver'

    ouster_params_declare = DeclareLaunchArgument('params_file',
                                           default_value=os.path.join(
                                               ouster_share_dir, 'params', 'driver_config.yaml'),
                                           description='FPath to the ROS2 parameters file to use.')
    
    ouster_driver_node = LifecycleNode(package='ros2_ouster',
                                executable='ouster_driver',
                                name=ouster_node_name,
                                output='screen',
                                emulate_tty=True,
                                parameters=[ouster_parameter_file],
                                arguments=['--ros-args', '--log-level', 'INFO'],
                                namespace='/',
                                )
    
    ouster_configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(ouster_driver_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    ouster_activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=ouster_driver_node, goal_state='inactive',
            entities=[
                LogInfo(
                    msg="[LifecycleLaunch] Ouster driver node is activating."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(ouster_driver_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    # TODO make lifecycle transition to shutdown before SIGINT
    shutdown_event = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                EmitEvent(event=ChangeState(
                  lifecycle_node_matcher=matches_node_name(node_name=ouster_node_name),
                  transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVE_SHUTDOWN,
                )),
                LogInfo(
                    msg="[LifecycleLaunch] Ouster driver node is exiting."),
            ],
        )
    )
    
    camera_model = 'zed2i'  

    # ZED Wrapper node
    


    return LaunchDescription([
        Node(
            package='myahrs_ros2_driver',
            executable='myahrs_ros2_driver',
            name='myahrs_ros2_driver',
            output='screen',
            arguments=['/dev/ttyACM1', '115200'],
            parameters=[myahrs_config_file]
        ),

        Node(
            package='nmea_navsat_driver',
            executable='nmea_serial_driver',
            output='screen',
            parameters=[ublox_config_file]
        ),

        ouster_params_declare,
        ouster_driver_node,
        ouster_activate_event,
        ouster_configure_event,
        shutdown_event,

        ublox_driver_node,

        RegisterEventHandler(
                                         event_handler=launch.event_handlers.OnProcessExit(
                                             target_action=ublox_gps_node,
                                             on_exit=[launch.actions.EmitEvent(
                                                 event=launch.events.Shutdown())],
                                         )),

        IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('zed_wrapper'),
            '/launch/include/zed_camera.launch.py'
        ]),
        launch_arguments={
            'camera_model': camera_model
        }.items()
        )
    ])
