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
    # 注意：这里的 my_map 是 SLAM Toolbox 的地图前缀名，不需要加 .posegraph 或 .data 后缀
    slam_map_path = os.path.join(fishbot_pkg, 'maps', 'my_map') 
    
    # Nav2 的标准配置文件
    nav2_params_path = os.path.join(fishbot_pkg, 'config', 'fishbot_nav2.yaml')
    rviz_config_dir = os.path.join(nav2_bringup_pkg, 'rviz', 'nav2_default_view.rviz')

    # 3. 启动 FAST_LIO (局部高频里程计)
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_lio_pkg, 'launch', 'mapping.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'True',
            'rviz': 'false',  # 强制关闭 FAST_LIO 内部的 RViz
        }.items()
    )

    # 4. 点云转 LaserScan (为 SLAM Toolbox 提供 2D 降维数据)
    pcl_to_scan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            # 将 FAST-LIO 输出的 3D 点云话题重映射到节点输入
            ('cloud_in', '/cloud_registered'), 
            ('scan', '/scan')
        ],
        parameters=[{
            'target_frame': 'base_link',  # 将点云投影到底盘坐标系下
            'transform_tolerance': 0.01,
            'min_height': 0.1,   # 裁剪底盘上方 0.1m 
            'max_height': 1.0,   # 到 1.0m 之间的点云作为 2D 障碍物
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0087, # 约 0.5 度分辨率
            'scan_time': 0.1,          # 雷达扫描周期 10Hz
            'range_min': 0.2,
            'range_max': 20.0,
            'use_inf': True,
        }]
    )

    # 5. 启动 SLAM Toolbox 纯定位模式 (全局重定位，发布 map -> odom)
    slam_toolbox_loc_node = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox_localization',
        output='screen',
        parameters=[
            # 加载系统默认的定位参数
            os.path.join(slam_toolbox_pkg, 'config', 'mapper_params_localization.yaml'),
            {
                'use_sim_time': True,
                'mode': 'localization',           # 强制开启纯定位模式
                'map_file_name': slam_map_path,   # 指向已序列化的地图路径
                'map_start_pose': [0.0, 0.0, 0.0] # 机器人的初始位置估算 [x, y, theta]
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
            # 注意：在 SLAM Toolbox 定位模式下，可能不需要单独传 map yaml 给 nav2_bringup，
            # 因为 slam_toolbox 也会把地图发到 /map 话题。
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
        fast_lio_launch,
        pcl_to_scan_node,         # 新增：3D 转 2D
        slam_toolbox_loc_node,    # 新增：全局定位
        nav2_launch,
        rviz_node
    ])