#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetPositionIK
import math
import numpy as np

class PlanningCoordinatorNode(Node):
    def __init__(self):
        super().__init__('planning_coordinator_node')
        
        self.declare_parameter('velocity_limit', math.pi)
        self.velocity_limit = self.get_parameter('velocity_limit').value
        
        # State machine: IDLE -> PLANNING -> EXECUTING -> IDLE / LOCKED
        self.state = 'IDLE'
        self.emergency_stop_triggered = False
        self.current_joint_state = None
        
        self.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        
        self.subscription_target = self.create_subscription(
            PoseStamped, '/planning/target_pose', self.target_callback, 10)
        
        self.subscription_joint_state = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        
        self.subscription_estop = self.create_subscription(
            Bool, '/emergency_stop', self.estop_callback, 10)
        
        self.reset_service = self.create_service(
            Trigger, '/planning/reset', self.reset_callback)
        
        self.publisher_trajectory = self.create_publisher(
            JointTrajectory, '/planning/trajectory_preview', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.exec_timer = None
        self.exec_traj = None
        self.exec_idx = 0

        self.current_positions = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
        self.hold_timer = self.create_timer(0.05, self.publish_current_state)
        
        self.action_client = ActionClient(
            self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        
        self.ik_client = self.create_client(GetPositionIK, '/ik_solver/compute_ik')
        
        self.active_goal_handle = None
        
        self.get_logger().info('Planning Coordinator Node initialized. State: IDLE')

    def joint_state_callback(self, msg: JointState):
        self.current_joint_state = msg

    def estop_callback(self, msg: Bool):
        if msg.data:
            self.emergency_stop_triggered = True
            self.state = 'LOCKED'
            self.get_logger().fatal('EMERGENCY STOP TRIGGERED: Locking planning pipeline!')
            
            if self.active_goal_handle is not None:
                self.get_logger().warn('Cancelling active trajectory execution goal...')
                self.active_goal_handle.cancel_goal_async()
                self.active_goal_handle = None
            if self.exec_timer is not None:
                self.exec_timer.cancel()
                self.exec_timer = None
                self.exec_traj = None
        else:
            self.emergency_stop_triggered = False
            # Per PRD §3.1: Releasing e-stop alone DOES NOT unlock system.
            # Must remain LOCKED until /planning/reset service is called.
            if self.state == 'LOCKED':
                self.get_logger().warn('Emergency stop signal cleared. System remains LOCKED until /planning/reset service is called.')

    def reset_callback(self, request, response):
        if self.emergency_stop_triggered:
            response.success = False
            response.message = 'Cannot reset: /emergency_stop is still active!'
            self.get_logger().error(response.message)
            return response
            
        self.state = 'IDLE'
        response.success = True
        response.message = 'System unlocked successfully. State is now IDLE.'
        self.get_logger().info(response.message)
        return response

    def target_callback(self, msg: PoseStamped):
        if self.state == 'LOCKED' or self.emergency_stop_triggered:
            self.get_logger().error('Emergency stop active or system LOCKED. Target rejected.')
            return
        
        if self.state != 'IDLE':
            self.get_logger().warn(f'System busy (State: {self.state}). Target rejected.')
            return
        
        self.get_logger().info(f'Target received at ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}, {msg.pose.position.z:.2f})')
        self.state = 'PLANNING'
        
        if not self.ik_client.service_is_ready():
            self.get_logger().warn('IK service not available. Generating local quintic trajectory preview.')
            self.execute_direct_trajectory(msg)
            return
            
        ik_req = GetPositionIK.Request()
        ik_req.ik_request.group_name = "manipulator"
        ik_req.ik_request.pose_stamped = msg
        if self.current_joint_state and len(self.current_joint_state.position) >= 6:
            ik_req.ik_request.robot_state.joint_state = self.current_joint_state
        else:
            from sensor_msgs.msg import JointState as _JS
            js = _JS()
            js.name = self.joint_names
            js.position = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
            ik_req.ik_request.robot_state.joint_state = js
        future = self.ik_client.call_async(ik_req)
        future.add_done_callback(lambda f: self.ik_response_callback(f, msg))

    def ik_response_callback(self, future, target_msg):
        try:
            resp = future.result()
            if resp.error_code.val != resp.error_code.SUCCESS:
                self.get_logger().error(f'IK computation rejected target. Error code: {resp.error_code.val}')
                self.state = 'IDLE'
                return
            
            goal_positions = list(resp.solution.joint_state.position[:6])
            self.generate_and_publish_trajectory(goal_positions)
        except Exception as e:
            self.get_logger().error(f'Failed to compute IK: {e}')
            self.state = 'IDLE'

    def execute_direct_trajectory(self, target_pose):
        # Fallback goal angles
        goal_positions = [0.5, -1.2, 1.3, -1.6, -1.57, 0.5]
        self.generate_and_publish_trajectory(goal_positions)

    def generate_and_publish_trajectory(self, q_goal):
        if self.current_joint_state and len(self.current_joint_state.position) >= 6:
            q_start = list(self.current_joint_state.position[:6])
        else:
            q_start = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
            
        n_joints = 6
        max_disp = max(abs(q_goal[i] - q_start[i]) for i in range(n_joints))
        duration = max(3.0, max_disp * 1.5)
        n_waypoints = int(duration * 20)  # 20 Hz interpolation
        t = np.linspace(0, duration, n_waypoints)
        
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        
        for i in range(n_waypoints):
            alpha = t[i] / duration
            s = 10 * alpha**3 - 15 * alpha**4 + 6 * alpha**5
            ds_da = 30 * alpha**2 - 60 * alpha**3 + 30 * alpha**4
            ds_dt = ds_da / duration
            
            point = JointTrajectoryPoint()
            point.positions = [q_start[j] + s * (q_goal[j] - q_start[j]) for j in range(n_joints)]
            point.velocities = [ds_dt * (q_goal[j] - q_start[j]) for j in range(n_joints)]
            point.time_from_start.sec = int(t[i])
            point.time_from_start.nanosec = int((t[i] - int(t[i])) * 1e9)
            traj.points.append(point)
            
        self.publisher_trajectory.publish(traj)
        self.get_logger().info(f'Trajectory generated & published ({n_waypoints} waypoints, duration {duration:.2f}s).')
        
        dt = duration / n_waypoints
        if self.exec_timer is not None:
            self.exec_timer.cancel()
        self.exec_traj = traj
        self.exec_idx = 0
        self.state = 'EXECUTING'
        self.exec_timer = self.create_timer(dt, self.exec_step)
        
        if self.action_client.server_is_ready():
            goal_msg = FollowJointTrajectory.Goal()
            goal_msg.trajectory = traj
            send_goal_future = self.action_client.send_goal_async(goal_msg)
            send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Trajectory goal rejected by controller.')
            self.state = 'IDLE'
            return
            
        self.active_goal_handle = goal_handle
        self.get_logger().info('Trajectory goal accepted. Executing...')
        res_future = goal_handle.get_result_async()
        res_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        self.active_goal_handle = None
        if self.exec_timer is None:
            self.state = 'IDLE'
        self.get_logger().info('Trajectory execution finished. System IDLE.')

    def publish_current_state(self):
        if self.state != 'EXECUTING':
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = self.joint_names
            js.position = list(self.current_positions)
            self.joint_state_pub.publish(js)

    def exec_step(self):
        if self.exec_traj is None or self.exec_idx >= len(self.exec_traj.points):
            if self.exec_timer is not None:
                self.exec_timer.cancel()
                self.exec_timer = None
            self.exec_traj = None
            self.state = 'IDLE'
            self.get_logger().info('Mock execution finished. System IDLE.')
            return
        pt = self.exec_traj.points[self.exec_idx]
        self.current_positions = list(pt.positions)
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joint_names
        js.position = list(pt.positions)
        if pt.velocities:
            js.velocity = list(pt.velocities)
        self.joint_state_pub.publish(js)
        self.current_joint_state = js
        self.exec_idx += 1

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
