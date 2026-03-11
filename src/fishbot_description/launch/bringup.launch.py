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

    # 2. 运行 pointcloud_to_laserscan 节点
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[{
            'target_frame': 'laser_link', 
            'transform_tolerance': 0.01,
            'min_height': -0.35,          
            'max_height': 1.0,            
            'angle_min': -3.14159,        
            'angle_max': 3.14159,         
            'angle_increment': 0.0087,    
            'scan_time': 0.1,             
            'range_min': 0.12,            
            'range_max': 30.0,            
            'use_inf': True,
            'inf_epsilon': 1.0,
            'use_sim_time': True
        }],
        remappings=[
            ('cloud_in', '/points'),      
            ('scan', '/scan')             
        ]
    )

    # === 新增：EKF 节点 ===
    ekf_config_path = os.path.join(fishbot_desc_pkg, 'config', 'fishbot_ekf.yaml')
    
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': True}],
        remappings=[
             # 将 EKF 输出的里程计话题命名为 /odom_combined (可选，如果不重映射，默认输出到 /odometry/filtered)
             # 为了让 Nav2 直接使用，通常我们不需要重映射输出，只需要 TF
        ]
    )
    # ====================

    return LaunchDescription([
        gazebo_launch,
        pointcloud_to_laserscan_node,
        robot_localization_node # 添加节点
    ])