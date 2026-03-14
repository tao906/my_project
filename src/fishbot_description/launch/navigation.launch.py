import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    fishbot_pkg = get_package_share_directory('fishbot_description')
    fast_lio_pkg = get_package_share_directory('fast_lio')
    slam_toolbox_pkg = get_package_share_directory('slam_toolbox')

    # 【关键修改 1】：确保这里的文件名是你建图时保存的前缀，比如 123
    slam_map_path = os.path.join(fishbot_pkg, 'maps', '111') 
    nav2_params_path = os.path.join(fishbot_pkg, 'config', 'fishbot_nav2.yaml')
    
    # 【关键修改 2】：使用我们自己保存的能看 3D 点云的 RViz 配置
    rviz_config_dir = os.path.join(fishbot_pkg, 'rviz', 'mapping.rviz') 

    # 1. 桥接坐标系 (极其重要，缺失会导致 Nav2 瘫痪！)
    tf_odom_to_lidar_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'lidar_odom']
    )

    # 【新增这一段】：把 FAST_LIO 输出的 body 系桥接到 Nav2 需要的 base_link 系
    tf_body_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'body', 'base_link']
    )

    # 2. 启动 FAST_LIO (作为高精度里程计和 3D 环境扫描仪)
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_lio_pkg, 'launch', 'mapping.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'True',
            'rviz': 'false',  
            # 【关键修改 3】：必须加载我们改过外参和话题的配置文件！
            'config_path': os.path.join(fishbot_pkg, 'config'),
            'config_file': 'fastlio_mid360.yaml'
        }.items()
    )

    # 3. 点云转 LaserScan (为 Nav2 提供实时避障线)
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
            'min_height': 0.05,   # 从底盘向上 5cm 开始切
            'max_height': 1.0,   
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
                'mode': 'localization',           # 纯定位模式
                'map_file_name': slam_map_path,   # 指向 123.posegraph 和 123.data
                'map_start_pose': [0.0, 0.0, 0.0] 
            }
        ]
    )

    # 5. 启动 Nav2 核心 (路径规划 + 代价地图)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fishbot_pkg, 'launch', 'nav2_custom', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'True',       
            'params_file': nav2_params_path, 
        }.items()
    )

    # 6. 启动 RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        tf_odom_to_lidar_odom,  # 补上 TF 树
        tf_body_to_base_link,
        fast_lio_launch,
        pcl_to_scan_node,         
        slam_toolbox_loc_node,    
        nav2_launch,
        rviz_node
    ])