import os
import xacro  
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess, AppendEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def generate_launch_description():
    
    # 获取包路径 
    pkg_name = 'plant'
    pkg_share = get_package_share_directory(pkg_name)
    pkg_prefix = get_package_prefix(pkg_name)
    
    # 设置路径
    urdf_path = os.path.join(pkg_share, 'urdf', 'turntable_with_plant.urdf.xacro')
    rviz_path = os.path.join(pkg_share, 'rviz', '2.rviz')
    
    # 模型路径
    plant_model_path = os.path.join(pkg_prefix, "share")

    # 解析 Xacro
    print(f"Parsing Xacro: {urdf_path}")
    doc = xacro.process_file(urdf_path)
    robot_desc = doc.toxml()

    # 配置变量
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # GAZEBO_MODEL_PATH 
    set_gazebo_model_path = AppendEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=plant_model_path
    )

    # 节点定义
    # Robot State Publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc  
        }],
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=(['-d', rviz_path] if os.path.exists(rviz_path) else []),
    )

    # Gazebo Server
    gzserver = ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )
    
    # Gazebo Client
    gzclient = ExecuteProcess(
        cmd=['gzclient'], 
        output='screen'
    )

    # Spawn Entity
    spawner_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'turntable_with_plant',
                   '-topic', '/robot_description'], 
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Controllers
    load_jsb = TimerAction(
        period=4.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster',
                       '--controller-manager', '/controller_manager'],
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        )]
    )

    load_vel = TimerAction(
        period=6.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['turntable_velocity_controller',
                       '--controller-manager', '/controller_manager'],
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        )]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation (Gazebo) clock if true'),
        set_gazebo_model_path, 
        rsp,
        rviz_node,
        gzserver,
        gzclient,
        spawner_entity,
        load_jsb,
        load_vel,
    ])