import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    tdk_nav2_dir = get_package_share_directory('tdk_nav2_manager')
    tdk_slam_dir = '/home/tdk/tdk_slam_ws/src/tdk_slam_manager'

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')

    params_file = os.path.join(
        tdk_nav2_dir,
        'config',
        'tdk_nav2_params.yaml'
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml_file,
        }]
    )

    # Separate lifecycle manager for map_server so it activates independently
    # from the main nav2 lifecycle manager
    lifecycle_manager_map = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server'],
        }]
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true'
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock if true'
        ),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(tdk_slam_dir, 'maps', 'carto_map_0.yaml'),
            description='Full path to pre-scanned map yaml file'
        ),
        map_server_node,
        lifecycle_manager_map,
        nav2_launch,
    ])
