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

    # 2. 桥接坐标系 (odom -> lidar_odom)
    tf_odom_to_lidar_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'lidar_odom']
    )

    # 【核心新增】：使用 EKF 发布稳定准确的 odom -> base_link
    ekf_params_path = os.path.join(fishbot_desc_pkg, 'config', 'fishbot_ekf.yaml')
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_params_path,
            {'use_sim_time': use_sim_time}
        ]
    )

    # 3. 将 3D 点云转换为 2D LaserScan
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
            'min_height': 0.05,        
            'max_height': 1.5,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0087,
            'scan_time': 0.1,
            'range_min': 0.2,
            'range_max': 20.0,
            'use_inf': True,
            'use_sim_time': use_sim_time 
        }]
    )

    # 4. 启动 SLAM Toolbox
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(slam_toolbox_pkg, 'config', 'mapper_params_online_async.yaml'),
            {
                'use_sim_time': use_sim_time,
                'use_sensor_data_qos': True  
            }
        ]
    )

    # 5. 启动 RViz2
    rviz_config_dir = os.path.join(fishbot_desc_pkg, 'rviz', 'mapping.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir], 
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=UnlessCondition(show_pcd) 
    )

    return LaunchDescription([
        show_pcd_arg, 
        tf_odom_to_lidar_odom,
        ekf_node,             # <--- EKF 接管 TF
        fast_lio_launch,
        pcl_to_scan_node,
        slam_toolbox_node,    
        rviz_node
    ])
