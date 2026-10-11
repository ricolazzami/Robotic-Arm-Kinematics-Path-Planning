#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class PointToPointResetPublisher(Node):
    def __init__(self):
        super().__init__('point_to_point_reset_publisher')
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.dt = 0.02
        self.timer = self.create_timer(self.dt, self.timer_cb)

        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint',
        ]

        self.point_a = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.point_b = [1.2, -0.8, 0.6, -0.7, 0.5, 1.57]

        self.t = 0.0
        self.move_duration = 3.0
        self.hold_duration = 0.8
        self.total_cycle = self.move_duration + self.hold_duration

    def timer_cb(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names

        phase = self.t % self.total_cycle

        if phase <= self.move_duration:
            alpha = 0.5 * (1.0 - math.cos(math.pi * (phase / self.move_duration)))
        else:
            alpha = 1.0

        msg.position = [
            a + alpha * (b - a)
            for a, b in zip(self.point_a, self.point_b)
        ]

        self.pub.publish(msg)
        self.t += self.dt


def main():
    rclpy.init()
    node = PointToPointResetPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
