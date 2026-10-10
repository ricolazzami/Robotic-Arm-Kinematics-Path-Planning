import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_arm_description = FindPackageShare('arm_description')
    pkg_arm_control = FindPackageShare('arm_control')
    pkg_gazebo_ros = FindPackageShare('gazebo_ros')

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' "',
            PathJoinSubstitution([pkg_arm_description, 'urdf', 'robot.urdf.xacro']),
            '"',
        ]
    )
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    controllers_file = PathJoinSubstitution([pkg_arm_control, 'config', 'controllers.yaml'])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gazebo.launch.py'])
        ),
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'decodelabs_ur5e'],
        output='screen',
    )

    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    load_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher_node,
        spawn_entity,
        load_joint_state_broadcaster,
        load_arm_controller,
    ])
