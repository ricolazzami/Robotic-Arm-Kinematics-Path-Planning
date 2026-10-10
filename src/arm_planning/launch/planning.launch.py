from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_arm_planning = FindPackageShare('arm_planning')
    
    ik_solver_node = Node(
        package='arm_planning',
        executable='ik_solver_node',
        name='ik_solver',
        output='screen',
    )
    
    trajectory_planner_node = Node(
        package='arm_planning',
        executable='trajectory_planner_node',
        name='trajectory_planner',
        output='screen',
        parameters=[{
            'velocity_limit': 3.14159,
            'min_waypoints': 10,
        }],
    )
    
    collision_checker_node = Node(
        package='arm_planning',
        executable='collision_checker_node',
        name='collision_checker',
        output='screen',
    )
    
    planning_coordinator_node = Node(
        package='arm_planning',
        executable='planning_coordinator_node',
        name='planning_coordinator',
        output='screen',
        parameters=[{
            'velocity_limit': 3.14159,
        }],
    )
    
    return LaunchDescription([
        ik_solver_node,
        trajectory_planner_node,
        collision_checker_node,
        planning_coordinator_node,
    ])
