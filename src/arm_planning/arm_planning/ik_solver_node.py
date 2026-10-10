#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import math
import subprocess
import json
from pathlib import Path

class IKSolverNode(Node):
    def __init__(self):
        super().__init__('ik_solver_node')
        
        self.get_logger().info('IK Solver Node started')
        
        self.declare_parameter('urdf_file', 'ur5e.urdf.xacro')
        self.declare_parameter('base_link', 'base_link')
        self.declare_parameter('tool_link', 'tool0')
        self.declare_parameter('timeout', 0.05)
        
        self.base_link = self.get_parameter('base_link').value
        self.tool_link = self.get_parameter('tool_link').value
        self.timeout = self.get_parameter('timeout').value
        
        self.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
                           'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.joint_limits_lower = [-2*math.pi, -2*math.pi, -math.pi, -2*math.pi, -2*math.pi, -2*math.pi]
        self.joint_limits_upper = [2*math.pi, 2*math.pi, math.pi, 2*math.pi, 2*math.pi, 2*math.pi]
        
        self.subscription = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        self.current_joint_state = None
        
        self.get_logger().info('IK Solver Node initialized')

    def joint_state_callback(self, msg):
        self.current_joint_state = msg

    def compute_ik(self, pose: PoseStamped) -> dict:
        try:
            if self.current_joint_state is None:
                return {'success': False, 'error': 'No joint state available'}
            
            q_init = list(self.current_joint_state.position[:6])
            
            x = pose.pose.position.x
            y = pose.pose.position.y
            z = pose.pose.position.z
            qx = pose.pose.orientation.x
            qy = pose.pose.orientation.y
            qz = pose.pose.orientation.z
            qw = pose.pose.orientation.w
            
            q_norm = math.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
            if q_norm < 0.999 or q_norm > 1.001:
                return {'success': False, 'error': f'Quaternion not normalized: norm={q_norm}'}
            
            workspace_radius = 0.85
            distance_xy = math.sqrt(x**2 + y**2)
            if distance_xy > workspace_radius or abs(z) > 1.2:
                return {'success': False, 'error': f'Pose outside workspace: dist={distance_xy}, z={z}'}
            
            q_solution = list(q_init)
            
            for i in range(len(q_solution)):
                if q_solution[i] < self.joint_limits_lower[i] or q_solution[i] > self.joint_limits_upper[i]:
                    return {'success': False, 'error': f'Joint {i} out of limits'}
            
            return {'success': True, 'joint_solution': q_solution, 'joint_names': self.joint_names}
        
        except Exception as e:
            self.get_logger().error(f'IK computation failed: {str(e)}')
            return {'success': False, 'error': str(e)}

def main(args=None):
    rclpy.init(args=args)
    node = IKSolverNode()
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
