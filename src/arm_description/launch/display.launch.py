from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_arm_description = FindPackageShare('arm_description')

    rviz_config_file = PathJoinSubstitution(
        [pkg_arm_description, 'rviz', 'view_robot.rviz']
    )

    robot_description_content = Command(
        [
            FindExecutable(name='xacro'),
            ' ',
            PathJoinSubstitution([pkg_arm_description, 'urdf', 'robot.urdf.xacro']),
        ]
    )

    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='auto',
        description='auto: PointToPointResetPublisher (50Hz) tanpa GUI | manual: joint_state_publisher_gui',
    )
    headless_arg = DeclareLaunchArgument('headless', default_value='false', description='true: skip rviz2')
    mode = LaunchConfiguration('mode')
    is_auto = PythonExpression(["'", mode, "' == 'auto'"])
    is_manual = PythonExpression(["'", mode, "' == 'manual'"])
    not_headless = PythonExpression(["'", LaunchConfiguration('headless'), "' == 'false'"])

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(is_manual),
    )

    point_to_point_publisher_node = Node(
        package='arm_description',
        executable='point_to_point_publisher.py',
        name='point_to_point_reset_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(is_auto),
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': False}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(not_headless),
    )

    return LaunchDescription(
        [
            mode_arg,
            headless_arg,
            joint_state_publisher_gui_node,
            point_to_point_publisher_node,
            robot_state_publisher_node,
            rviz_node,
        ]
    )
