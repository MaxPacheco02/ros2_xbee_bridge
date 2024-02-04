import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    main_config_path = os.path.join(
        get_package_share_directory('ros2_xbee_bridge'), 
        'config'
        )
    
    params = os.path.join(main_config_path, 'ros_params.yaml')

    ns = LaunchConfiguration('namespace')
    ns_arg = DeclareLaunchArgument('namespace') 
    
    dev = LaunchConfiguration('dev')
    dev_arg = DeclareLaunchArgument('dev', default_value='/dev/ttyXBEE')
    
    communication = [
        Node(
            package='ros2_xbee_bridge',
            namespace=ns,
            executable='xbee_bridge.py',
            name='xbee_bridge',
            emulate_tty = True,
            output={
                'stdout': 'screen',
                'stderr': 'screen',
            },
            parameters=[
                {'dev': dev},
                params
            ],
            respawn=True,
            respawn_delay=5.0
        ),
    ]

    launch_list = []
    launch_list.append(ns_arg)
    launch_list.append(dev_arg)
    launch_list.extend(communication)

    return LaunchDescription(launch_list)