#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
import math

class PlanningCoordinatorNode(Node):
    def __init__(self):
        super().__init__('planning_coordinator_node')
        
        self.declare_parameter('velocity_limit', math.pi)
        
        self.velocity_limit = self.get_parameter('velocity_limit').value
        self.state = 'IDLE'
        self.emergency_stop_active = False
        self.current_joint_state = None
        
        self.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                           'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        
        self.subscription_target = self.create_subscription(
            PoseStamped, '/planning/target_pose', self.target_callback, 10)
        
        self.subscription_joint_state = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        
        self.subscription_estop = self.create_subscription(
            Bool, '/emergency_stop', self.estop_callback, 10)
        
        self.publisher_trajectory = self.create_publisher(
            JointTrajectory, '/planning/trajectory_preview', 10)
        
        self.action_client = ActionClient(
            self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        
        self.get_logger().info('Planning Coordinator Node initialized')

    def target_callback(self, msg: PoseStamped):
        if self.emergency_stop_active:
            self.get_logger().error('Emergency stop active. Target rejected.')
            return
        
        if self.state != 'IDLE':
            self.get_logger().warn('Not in IDLE state. Ignoring target.')
            return
        
        self.get_logger().info(f'Target received: {msg.pose.position}')
        self.state = 'PLANNING'
        
        # Dummy trajectory for now
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        
        from trajectory_msgs.msg import JointTrajectoryPoint
        point = JointTrajectoryPoint()
        if self.current_joint_state:
            point.positions = list(self.current_joint_state.position[:6])
        else:
            point.positions = [0.0] * 6
        
        traj.points.append(point)
        self.publisher_trajectory.publish(traj)
        
        self.state = 'IDLE'

    def joint_state_callback(self, msg: JointState):
        self.current_joint_state = msg

    def estop_callback(self, msg: Bool):
        if msg.data and not self.emergency_stop_active:
            self.get_logger().fatal('EMERGENCY STOP triggered!')
            self.emergency_stop_active = True
            self.state = 'LOCKED'
        elif not msg.data and self.emergency_stop_active:
            self.get_logger().info('Emergency stop released.')
            self.emergency_stop_active = False
            self.state = 'IDLE'

def main(args=None):
    rclpy.init(args=args)
    node = PlanningCoordinatorNode()
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
