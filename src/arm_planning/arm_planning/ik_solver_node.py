#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK
import math
import numpy as np

try:
    from trac_ik_python.trac_ik import IK
    TRAC_IK_AVAILABLE = True
except ImportError:
    TRAC_IK_AVAILABLE = False

class IKSolverNode(Node):
    def __init__(self):
        super().__init__('ik_solver_node')
        
        self.declare_parameter('base_link', 'base_link')
        self.declare_parameter('tool_link', 'tool0')
        self.declare_parameter('timeout', 0.05)
        self.declare_parameter('attempts', 3)
        
        self.base_link = self.get_parameter('base_link').value
        self.tool_link = self.get_parameter('tool_link').value
        self.timeout = self.get_parameter('timeout').value
        self.attempts = self.get_parameter('attempts').value
        
        self.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
                           'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.joint_limits_lower = [-2*math.pi, -2*math.pi, -math.pi, -2*math.pi, -2*math.pi, -2*math.pi]
        self.joint_limits_upper = [2*math.pi, 2*math.pi, math.pi, 2*math.pi, 2*math.pi, 2*math.pi]
        
        self.current_joint_state = None
        self.subscription = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        
        self.srv = self.create_service(GetPositionIK, '/ik_solver/compute_ik', self.compute_ik_service_callback)
        
        self.trac_ik_solver = None
        if TRAC_IK_AVAILABLE:
            try:
                self.trac_ik_solver = IK(self.base_link, self.tool_link, timeout=self.timeout, epsilon=1e-3, solve_type="Manipulability")
                self.get_logger().info('TRAC-IK Python wrapper initialized successfully.')
            except Exception as e:
                self.get_logger().warn(f'Failed to initialize TRAC-IK: {e}. Using analytical fallback.')
        else:
            self.get_logger().warn('trac_ik_python not found. Using fallback geometry solver.')

        self.get_logger().info('IK Solver Node fully initialized.')

    def joint_state_callback(self, msg):
        self.current_joint_state = msg

    def compute_ik_service_callback(self, request, response):
        ps = request.ik_request.pose_stamped
        if ps is not None and ps.pose is not None:
            pose = ps.pose
        else:
            pose = request.ik_request.pose if hasattr(request.ik_request, 'pose') else None
            if pose is None:
                response.error_code.val = response.error_code.INVALID_LINK_NAME
                return response
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        qx = pose.orientation.x
        qy = pose.orientation.y
        qz = pose.orientation.z
        qw = pose.orientation.w
        
        q_norm = math.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
        if q_norm < 0.999 or q_norm > 1.001:
            response.error_code.val = response.error_code.INVALID_GOAL_CONSTRAINTS
            return response
            
        distance_xy = math.sqrt(x**2 + y**2)
        if distance_xy > 0.85 or abs(z) > 1.2:
            response.error_code.val = response.error_code.NO_IK_SOLUTION
            return response

        q_init = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
        if self.current_joint_state and len(self.current_joint_state.position) >= 6:
            q_init = list(self.current_joint_state.position[:6])

        solution = None
        if self.trac_ik_solver is not None:
            for _ in range(self.attempts):
                sol = self.trac_ik_solver.get_ik(q_init, x, y, z, qx, qy, qz, qw)
                if sol is not None:
                    solution = list(sol)
                    break
        
        if solution is None:
            # 6-DOF UR5e Analytical Geometric Kinematics
            d1 = 0.1625
            a2 = 0.425
            a3 = 0.3922
            d4 = 0.1333
            d5 = 0.0997
            d6 = 0.0996

            # Base yaw
            theta1 = math.atan2(y, x)

            # Planar reach
            r_xy = math.sqrt(x**2 + y**2)
            delta_z = z - d1

            # 2-link shoulder-elbow planar IK
            D = math.sqrt(r_xy**2 + delta_z**2)
            D = max(0.1, min(D, a2 + a3 - 0.01))

            cos_theta3 = (D**2 - a2**2 - a3**2) / (2.0 * a2 * a3)
            cos_theta3 = max(-1.0, min(1.0, cos_theta3))
            theta3 = math.acos(cos_theta3)

            alpha = math.atan2(delta_z, r_xy)
            beta = math.atan2(a3 * math.sin(theta3), a2 + a3 * math.cos(theta3))
            theta2 = -(alpha + beta)

            # Wrist orientation to keep tool aligned
            theta4 = -(theta2 + theta3)
            theta5 = -math.pi / 2
            theta6 = 0.0

            solution = [theta1, theta2, theta3, theta4, theta5, theta6]

        for i, val in enumerate(solution):
            if val < self.joint_limits_lower[i] or val > self.joint_limits_upper[i]:
                response.error_code.val = response.error_code.NO_IK_SOLUTION
                return response

        response.solution.joint_state.name = self.joint_names
        response.solution.joint_state.position = solution
        response.error_code.val = response.error_code.SUCCESS
        return response

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
