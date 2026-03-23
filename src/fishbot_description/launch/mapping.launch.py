import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition 
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    fast_lio_pkg = get_package_share_directory('fast_lio')
    slam_toolbox_pkg = get_package_share_directory('slam_toolbox')
    fishbot_desc_pkg = get_package_share_directory('fishbot_description') 
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    show_pcd_arg = DeclareLaunchArgument(
        'show_pcd',
        default_value='false',
        description='Set to true to launch 3D FAST_LIO RViz, false for 2D RViz'
    )
    show_pcd = LaunchConfiguration('show_pcd')

    # 1. 启动 FAST_LIO 
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_lio_pkg, 'launch', 'mapping.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rviz': show_pcd,  
            'config_path': os.path.join(fishbot_desc_pkg, 'config'),
            'config_file': 'fastlio_mid360.yaml'
        }.items()
    )
    # 2. 静态 TF 发布器
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_odom_to_lidar_odom',
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
            # 🌟 关键修改：相对于 base_link，从 0.15m开始切片，完美避开自己的方形车头
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

    # 4. 启动 SLAM Toolbox
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_pkg, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    rviz_config_dir = os.path.join(fishbot_desc_pkg, 'rviz', 'mapping.rviz')
    
    # 5. 启动基础版 RViz2 (2D建图)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_2d',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=UnlessCondition(show_pcd) 
    )

    return LaunchDescription([
        show_pcd_arg,
        static_tf_node,
        fast_lio_launch,
        pct_to_scan_node,
        slam_toolbox_launch,
        rviz_node
    ])