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

    # 2. 设置文件路径
    # 地图路径（如果是建立新地图，可不启动 map_server 或提供空地图）
    default_map_path = os.path.join(fishbot_pkg, 'maps', 'my_map.yaml')
    
    # 导航参数文件路径
    nav2_params_path = os.path.join(fishbot_pkg, 'config', 'fishbot_nav2.yaml')

    # RViz 配置文件路径
    rviz_config_dir = os.path.join(nav2_bringup_pkg, 'rviz', 'nav2_default_view.rviz')

    # 3. 启动 FAST_LIO (替代 AMCL 作为定位里程计和建图模块)
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_lio_pkg, 'launch', 'mapping.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'True',
            # 仿真中直接使用 FAST_LIO 自带或针对 Mid360 的配置
            # 根据你包的实际情况，也可以将其替换为你自行编写的 yaml 配置
        }.items()
    )

    # 4. 启动 Nav2 导航核心节点 (弃用 bringup_launch.py，改用纯 navigation_launch.py 绕过 AMCL)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fishbot_pkg, 'launch', 'nav2_custom', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'True',       
            'map': default_map_path,      
            'params_file': nav2_params_path, 
        }.items()
    )

    # 5. 启动 RViz2
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
        nav2_launch,
        rviz_node
    ])