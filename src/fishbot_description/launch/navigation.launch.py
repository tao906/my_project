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
    
    # 🌟 关键修改 1：全部指向本地 fishbot_description/config/ 目录下的配置文件
    seg_params_path = os.path.join(fishbot_pkg, 'config', 'segmentation_params.yaml')
    slam_loc_params_path = os.path.join(fishbot_pkg, 'config', 'slam_localization_params.yaml')
    nav2_params_path = os.path.join(fishbot_pkg, 'config', 'fishbot_nav2.yaml')
    
    slam_map_path = os.path.join(fishbot_pkg, 'maps', '123') 
    rviz_config_dir = os.path.join(fishbot_pkg, 'rviz', 'nav2.rviz')

    # =========================================================
    # 0. IMU 互补滤波 (与建图时一致，保护 FAST-LIO 精度)
    # =========================================================
    imu_filter_node = Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='imu_filter',
        remappings=[
            ('imu/data_raw', '/livox/imu'),                
        ],
        parameters=[{'use_mag': False, 'publish_tf': False, 'use_sim_time': True}]
    )

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

    # =========================================================
    # 2.1 Filter Node (剔除车身遮挡)
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
            'min_z': -0.55, 'max_z': 0.10, 
            'use_sim_time': True
        }]
    )

    # =========================================================
    # 2.2 Ground Segmentation (剥离地面点云)
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
            seg_params_path, # 🌟 关键修改 2：读取中心化参数文件
            {'use_sim_time': True} # 移除了硬编码的 r_min 和 sensor_height
        ]
    )

    # 3. 启动 pointcloud_to_laserscan 节点
    pct_to_scan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/segmentation/obstacle'),  
            ('scan', '/scan')
        ],
        parameters=[{
            'target_frame': 'base_link',  
            'transform_tolerance': 0.05,
            'min_height': -0.10,      
            'max_height': 2.0,      
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

    # 4. 启动 SLAM Toolbox 纯定位模式
    slam_toolbox_loc_node = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox_localization',
        output='screen',
        parameters=[
            slam_loc_params_path, # 🌟 关键修改 3：加载我们刚刚修改过 base_link 和 mode 的自定义定位参数
            {
                'use_sim_time': True,
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
        imu_filter_node,
        fast_lio_launch,
        static_tf_node,
        filter_node,
        ground_segmentation_node,
        pct_to_scan_node,
        slam_toolbox_loc_node,
        nav2_launch,
        rviz_node
    ])