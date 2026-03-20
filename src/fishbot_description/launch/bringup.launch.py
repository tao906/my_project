import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    fishbot_desc_pkg = get_package_share_directory('fishbot_description')
    
    # 1. 包含 Gazebo 启动文件
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fishbot_desc_pkg, 'launch', 'gazebo.launch.py')
        )
    )

    # 2. 【关键修复】：重新开启 EKF 节点，让它来接管断裂的 TF 树
    # ekf_config_path = os.path.join(fishbot_desc_pkg, 'config', 'fishbot_ekf.yaml')
    # robot_localization_node = Node(
    #     package='robot_localization',
    #     executable='ekf_node',
    #     name='ekf_filter_node',
    #     output='screen',
    #     parameters=[ekf_config_path, {'use_sim_time': True}],
    # )

    return LaunchDescription([
        gazebo_launch
        # robot_localization_node # 恢复该节点
    ])