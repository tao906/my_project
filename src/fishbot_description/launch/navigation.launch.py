import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 获取包路径
    fishbot_pkg = get_package_share_directory('fishbot_description')
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')
    fast_lio_pkg = get_package_share_directory('fast_lio')
    slam_toolbox_pkg = get_package_share_directory('slam_toolbox')

    # 2. 设置文件路径
    slam_map_path = os.path.join(fishbot_pkg, 'maps', '1') 
    nav2_params_path = os.path.join(fishbot_pkg, 'config', 'fishbot_nav2.yaml')
    rviz_config_dir = os.path.join(fishbot_pkg, 'rviz', 'nav2.rviz')

    # 3. 启动 FAST_LIO (局部高频里程计)
    # 【已修复】：补上了配置路径，让它读取咱们调好的 blind: 0.35 的文件！
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_lio_pkg, 'launch', 'mapping.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'True',
            'rviz': 'false',  
            'config_path': os.path.join(fishbot_pkg, 'config'),
            'config_file': 'fastlio_mid360.yaml'
        }.items()
    )

    # 【已修复】：补上建图时用到的桥接坐标系
    tf_odom_to_lidar_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'lidar_odom']
    )

    # 4. 点云转 LaserScan
    pcl_to_scan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/cloud_registered'), 
            ('scan', '/scan')
        ],
        parameters=[{
            'target_frame': 'base_link',  
            'transform_tolerance': 0.05,
            'min_height': 0.20,  # 稍微抬高切片，过滤地面噪点 
            'max_height': 1.5,   
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0087, 
            'scan_time': 0.1,          
            'range_min': 0.2,
            'range_max': 20.0,
            'use_inf': True,
            'use_sim_time': True
        }]
    )

    # 5. 启动 SLAM Toolbox 纯定位模式
    slam_toolbox_loc_node = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox_localization',
        output='screen',
        parameters=[
            os.path.join(slam_toolbox_pkg, 'config', 'mapper_params_localization.yaml'),
            {
                'use_sim_time': True,
                'mode': 'localization',           
                'map_file_name': slam_map_path,   
                'map_start_pose': [0.0, 0.0, 0.0] 
            }
        ]
    )

    # 6. 启动 Nav2 导航核心节点
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fishbot_pkg, 'launch', 'nav2_custom', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'True',       
            'params_file': nav2_params_path, 
        }.items()
    )

    # 7. 启动独立的 RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        tf_odom_to_lidar_odom,    # <-- 补上的 TF
        fast_lio_launch,
        pcl_to_scan_node,         
        slam_toolbox_loc_node,    
        nav2_launch,
        rviz_node
    ])