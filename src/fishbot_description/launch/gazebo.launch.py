import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    fishbot_description_dir = get_package_share_directory('fishbot_description')
    
    # 1. 设置模型路径
    model_path = os.path.join(fishbot_description_dir, 'urdf', 'my_robot.urdf.xacro')
    
    # === 设置 World 文件路径 ===
    world_path = os.path.join(fishbot_description_dir, 'world', 'nav_world.world')

    # 2. 获取机器人描述
    robot_description = ParameterValue(Command(['xacro ', model_path]), value_type=str)

    # 3. 启动 Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True  # 【关键修复】：必须开启仿真时间，与 FAST_LIO 时钟对齐
        }]
    )

    # 4. 启动 Gazebo 仿真环境
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_path}.items()  # 传递 world 参数
    )

    # 5. 在 Gazebo 中生成机器人
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'fishbot',
                   '-x', '0.0', '-y', '0.0', '-z', '0.1'],
        output='screen'
    )

    # 6. 加载控制器
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'fishbot_joint_state_broadcaster'],
        output='screen'
    )

    load_diff_drive_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'fishbot_diff_drive_controller'],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        gazebo,
        spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_diff_drive_controller],
            )
        ),
    ])