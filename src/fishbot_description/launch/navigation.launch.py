import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    fishbot_pkg = get_package_share_directory('fishbot_description')
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')
    fast_lio_pkg = get_package_share_directory('fast_lio')
    slam_toolbox_pkg = get_package_share_directory('slam_toolbox')

    slam_map_path = os.path.join(fishbot_pkg, 'maps', '123') 
    nav2_params_path = os.path.join(fishbot_pkg, 'config', 'fishbot_nav2.yaml')
    rviz_config_dir = os.path.join(fishbot_pkg, 'rviz', 'nav2.rviz')

    # 1. 启动 FAST_LIO (局部高频里程计)
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

    # 2. 静态 TF 发布器
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_odom_to_camera_init',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'lidar_odom'],
        output='screen'
    )

    # 3. 启动 pointcloud_to_laserscan 节点
    pct_to_scan_node = Node(
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
            # 🌟 关键修改：同步修改切片高度，避开长方形车头，获取干净的雷达扫面
            'min_height': 0.15,      
            'max_height': 1.0,      
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0087,
            'scan_time': 0.1,      
            'range_min': 0.45,
            'range_max': 20.0,
            'use_inf': True,
            'use_sim_time': True
        }]
    )

    # 4. 启动 SLAM Toolbox 纯定位模式
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

    # 5. 启动 Nav2 导航核心节点
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fishbot_pkg, 'launch', 'nav2_custom', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'True',       
            'params_file': nav2_params_path, 
        }.items()
    )

    # 6. 启动独立的 RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        static_tf_node,
        fast_lio_launch,
        pct_to_scan_node,
        slam_toolbox_loc_node,
        nav2_launch,
        rviz_node
    ])