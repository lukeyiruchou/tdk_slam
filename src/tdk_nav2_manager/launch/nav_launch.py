import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    tdk_nav2_dir = get_package_share_directory('tdk_nav2_manager')
    tdk_slam_dir = get_package_share_directory('tdk_slam_manager')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_arg = LaunchConfiguration('map')

    params_file = os.path.join(
        tdk_nav2_dir,
        'config',
        'tdk_nav2_params.yaml'
    )

    # ★ 智慧路徑補全：若傳入純檔名（如 carto_map_4.yaml），自動拼接 package maps 路徑；若為絕對路徑則維持原樣
    map_full_path = PythonExpression([
        f"'{os.path.join(tdk_slam_dir, 'maps')}/' + '{map_arg}' if '/' not in '{map_arg}' else '{map_arg}'"
    ])

    # 1. Map Server 節點
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_full_path,
        }]
    )

    # 2. 引入 Nav2 Navigation 核心（包含 controller, planner, behavior, smoother 等）
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true',
            'use_composition': 'False',  # 使用獨立進程，便於排查各節點
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(tdk_slam_dir, 'maps', 'real_map_0.yaml'),
            description='Full path or filename of pre-scanned map yaml'
        ),
        map_server_node,
        nav2_launch,
    ])