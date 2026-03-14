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

    # 获取配置文件路径
    slam_map_path = os.path.join(fishbot_pkg, 'maps', '111') 
    nav2_params_path = os.path.join(fishbot_pkg, 'config', 'fishbot_nav2.yaml')
    rviz_config_dir = os.path.join(fishbot_pkg, 'rviz', 'mapping.rviz') 
    
    # 【新增】：获取 EKF 配置文件路径
    ekf_params_path = os.path.join(fishbot_pkg, 'config', 'fishbot_ekf.yaml')

    # 1. 桥接坐标系：将 map/odom 连接到 lidar_odom (FAST-LIO 的世界原点)
    tf_odom_to_lidar_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'lidar_odom']
    )

    # ⚠️ 注意：这里删除了之前手动发布的 body -> base_link 静态 TF
    # 因为现在我们将使用 EKF 节点来动态、高频地发布 odom -> base_link 之间的 TF

    # 【新增核心节点】：2. 启动 EKF 节点，融合里程计并发布 odom -> base_link 的 TF 桥梁
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_params_path,
            {'use_sim_time': True}
        ]
    )

    # 3. 启动 FAST_LIO (作为高精度里程计和 3D 环境扫描仪)
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_lio_pkg, 'launch', 'mapping.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'True',
            'rviz': 'false',  
            # 必须加载改过外参和话题的配置文件
            'config_path': os.path.join(fishbot_pkg, 'config'),
            'config_file': 'fastlio_mid360.yaml'
        }.items()
    )

    # 4. 点云转 LaserScan (为 Nav2 全局地图提供远距离避障线)
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
                'mode': 'localization',           # 纯定位模式
                'map_file_name': slam_map_path,   
                'map_start_pose': [0.0, 0.0, 0.0] 
            }
        ]
    )

    # 6. 启动 Nav2 核心 (路径规划 + 代价地图)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fishbot_pkg, 'launch', 'nav2_custom', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'True',       
            'params_file': nav2_params_path, 
        }.items()
    )

    # 7. 启动 RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        tf_odom_to_lidar_odom,    # 静态 TF (odom -> lidar_odom)
        ekf_node,                 # 动态高频 TF (odom -> base_link)，修复断树的终极救星！
        fast_lio_launch,          # 3D SLAM 里程计
        pcl_to_scan_node,         # 点云切片
        slam_toolbox_loc_node,    # 2D 栅格定位
        nav2_launch,              # Nav2 导航核心
        rviz_node                 # 可视化界面
    ])