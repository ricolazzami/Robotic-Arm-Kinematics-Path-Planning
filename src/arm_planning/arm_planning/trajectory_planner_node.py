#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
import math

class TrajectoryPlannerNode(Node):
    def __init__(self):
        super().__init__('trajectory_planner_node')
        
        self.declare_parameter('velocity_limit', math.pi)
        self.declare_parameter('min_waypoints', 50)
        
        self.velocity_limit = self.get_parameter('velocity_limit').value
        self.min_waypoints = self.get_parameter('min_waypoints').value
        
        self.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                           'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        
        self.subscription = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        
        self.target_subscription = self.create_subscription(
            PoseStamped, '/planning/target_pose', self.target_callback, 10)
        
        self.preview_publisher = self.create_publisher(
            JointTrajectory, '/planning/trajectory_preview', 10)
        
        self.current_joint_state = None
        
        self.get_logger().info('Trajectory Planner Node initialized')

    def joint_state_callback(self, msg):
        self.current_joint_state = msg

    def target_callback(self, msg):
        self.get_logger().info(f'Received target pose: x={msg.pose.position.x}, y={msg.pose.position.y}, z={msg.pose.position.z}')

    def generate_trajectory(self, q_start, q_goal):
        if len(q_start) != len(q_goal):
            self.get_logger().error('Start and goal configs have different sizes')
            return None
        
        n_joints = len(q_start)
        
        max_displacement = max(abs(q_goal[i] - q_start[i]) for i in range(n_joints))
        
        duration = max_displacement / self.velocity_limit
        duration = max(duration, 1.0)
        
        n_waypoints = max(self.min_waypoints, int(duration * 10))
        
        t = np.linspace(0, duration, n_waypoints)
        
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        
        for i in range(n_waypoints):
            alpha = t[i] / duration
            
            # Quintic polynomial interpolation
            # s(a) = 10*a^3 - 15*a^4 + 6*a^5
            # s'(a) = (30*a^2 - 60*a^3 + 30*a^4) / duration
            
            s = 10*alpha**3 - 15*alpha**4 + 6*alpha**5
            ds_da = 30*alpha**2 - 60*alpha**3 + 30*alpha**4
            
            point = JointTrajectoryPoint()
            point.positions = [q_start[j] + s * (q_goal[j] - q_start[j]) for j in range(n_joints)]
            
            ds_dt = ds_da / duration
            point.velocities = [ds_dt * (q_goal[j] - q_start[j]) for j in range(n_joints)]
            
            point.time_from_start = rclpy.duration.Duration(seconds=t[i]).to_msg()
            
            max_vel = max(abs(v) for v in point.velocities)
            if max_vel > self.velocity_limit:
                self.get_logger().error(f'Velocity limit exceeded at waypoint {i}: {max_vel} > {self.velocity_limit}')
                return None
            
            trajectory.points.append(point)
        
        return trajectory

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
