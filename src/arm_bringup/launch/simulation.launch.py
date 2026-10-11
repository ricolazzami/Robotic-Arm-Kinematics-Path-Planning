import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_arm_description = FindPackageShare('arm_description')
    pkg_arm_planning = FindPackageShare('arm_planning')

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' "',
            PathJoinSubstitution([pkg_arm_description, 'urdf', 'robot.urdf.xacro']),
            '"',
        ]
    )
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    rviz_config_file = PathJoinSubstitution([pkg_arm_description, 'rviz', 'view_robot.rviz'])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )

    planning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_arm_planning, 'launch', 'planning.launch.py'])
        ),
    )

    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node,
        planning_launch,
    ])
