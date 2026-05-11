import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition 
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 路径获取
    fast_lio_pkg = get_package_share_directory('fast_lio')
    slam_toolbox_pkg = get_package_share_directory('slam_toolbox')
    fishbot_desc_pkg = get_package_share_directory('fishbot_description') 
    linefit_pkg_dir = get_package_share_directory('linefit_ground_segmentation_ros')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    show_pcd_arg = DeclareLaunchArgument('show_pcd', default_value='false')
    show_pcd = LaunchConfiguration('show_pcd')

    # =========================================================
    # 1. FAST_LIO: 提供高精度里程计和去畸变的点云
    # 输出: /cloud_registered_body (车体系)
    # =========================================================
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

    # =========================================================
    # 2. Filter Node: 订阅 FAST-LIO 的车体系输出，剔除车身遮挡
    # 订阅: /cloud_registered_body
    # 输出: /cloud_filtered
    # =========================================================
    filter_node = Node(
        package='filter',
        executable='filter_node',
        name='pc_filter_node',
        output='screen',
        remappings=[
            ('cloud_in', '/cloud_registered_body'),
            ('cloud_out', '/cloud_filtered')   
        ],
        parameters=[{
            'min_x': -0.35, 'max_x': 0.35,
            'min_y': -0.25, 'max_y': 0.25,
            'min_z': -0.55, 'max_z': 0.10, # 针对雷达光心的Z轴裁剪
            'use_sim_time': use_sim_time
        }]
    )

    # =========================================================
    # 3. IMU 互补滤波: 为系统提供平滑的姿态参考
    # =========================================================
    imu_filter_node = Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='imu_filter',
        remappings=[('imu/data_raw', '/livox/imu'),
                    ('imu/data', '/livox/imu_filtered')
        ],
        parameters=[{'use_mag': False, 'publish_tf': False, 'use_sim_time': use_sim_time}]
    )

    # =========================================================
    # 4. Ground Segmentation: 订阅滤完车体的点云，剥离地面
    # 订阅: /cloud_filtered
    # 输出: /segmentation/obstacle
    # =========================================================
    ground_segmentation_node = Node(
        package='linefit_ground_segmentation_ros',
        executable='ground_segmentation_node',
        name='ground_segmentation',
        remappings=[
            ('input_topic', '/cloud_filtered'),
            ('obstacle_cloud', '/segmentation/obstacle')
        ],
        parameters=[
            os.path.join(linefit_pkg_dir, 'launch', 'segmentation_params.yaml'),
            {'sensor_height': 0.40, 'r_min': 0.1, 'use_sim_time': use_sim_time}
        ]
    )

    # =========================================================
    # 5. Pointcloud to Laserscan: 将干净的障碍物点云投影为 2D 激光
    # 订阅: /segmentation/obstacle
    # 输出: /scan
    # =========================================================
    pct_to_scan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[('cloud_in', '/segmentation/obstacle'), ('scan', '/scan')],
        parameters=[{
            'target_frame': 'base_link',  
            'transform_tolerance': 0.05,
            'min_height': -0.10,         
            'max_height': 1.0,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0087,
            'scan_time': 0.1,
            'range_min': 0.36,
            'range_max': 20.0,
            'use_inf': True,
            'use_sim_time': True 
        }]
    )

    # =========================================================
    # 6. SLAM Toolbox: 最终建图节点
    # =========================================================
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_pkg, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 静态 TF 和 RViz
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_odom_to_lidar_odom',
        arguments=['0','0','0','0','0','0','odom','lidar_odom'],
        output='screen'
    )
    
    rviz_node = Node(
        package='rviz2', 
        executable='rviz2', 
        name='rviz2',
        arguments=['-d', os.path.join(fishbot_desc_pkg, 'rviz', 'mapping.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=UnlessCondition(show_pcd)
    )

    return LaunchDescription([
        show_pcd_arg,
        fast_lio_launch,
        filter_node,
        imu_filter_node,
        ground_segmentation_node,
        pct_to_scan_node,
        slam_toolbox_launch,
        static_tf,
        rviz_node
    ])
