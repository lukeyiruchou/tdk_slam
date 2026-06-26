import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. 取得 tdk_slam_manager 的安裝路徑
    pkg_share = get_package_share_directory('tdk_slam_manager')
    
    # 2. 指向 rviz 資料夾底下的 slam_config.rviz
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'slam_config.rviz')

    # 3. 啟動節點
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        )
    ])