import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition # 【关键】：重新导入条件判断模块
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    fast_lio_pkg = get_package_share_directory('fast_lio')
    slam_toolbox_pkg = get_package_share_directory('slam_toolbox')
    fishbot_desc_pkg = get_package_share_directory('fishbot_description') 
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # ================= 【声明自定义的 Launch 参数】 =================
    # 声明一个名为 show_pcd 的参数，默认值为 false
    show_pcd_arg = DeclareLaunchArgument(
        'show_pcd',
        default_value='false',
        description='Set to true to launch 3D FAST_LIO RViz, false for 2D RViz'
    )
    show_pcd = LaunchConfiguration('show_pcd')
    # =====================================================================

    # 1. 启动 FAST_LIO (如果 show_pcd 为 true，它内部会自动启动 3D RViz)
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_lio_pkg, 'launch', 'mapping.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rviz': show_pcd,  # 接收参数，为 true 时启动 3D RViz
            'config_path': os.path.join(fishbot_desc_pkg, 'config'),
            'config_file': 'fastlio_mid360.yaml'
        }.items()
    )

    # 2. 桥接坐标系
    tf_odom_to_lidar_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'lidar_odom']
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
            'min_height': 0.05,  # 抬高 5cm 切片，避免地面点变成 2D 噪点       
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
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_pkg, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # ================= 【配置 2D RViz 自动加载路径】 =================
    rviz_config_dir = os.path.join(fishbot_desc_pkg, 'rviz', 'mapping.rviz')
    
    # 5. 启动基础版 RViz2 (2D建图)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir], 
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=UnlessCondition(show_pcd) # 【关键修改】：只有当 show_pcd 为 false 时，才启动这个 2D RViz 节点
    )

    return LaunchDescription([
        show_pcd_arg, 
        tf_odom_to_lidar_odom,
        fast_lio_launch,
        pcl_to_scan_node,
        slam_toolbox_launch,
        rviz_node
    ])