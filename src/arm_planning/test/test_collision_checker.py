import unittest
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from arm_planning.collision_checker_node import CollisionCheckerNode
import rclpy

class TestCollisionChecker(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = CollisionCheckerNode()

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_safe_trajectory(self):
        traj = JointTrajectory()
        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        traj.points.append(point)
        
        valid, idx = self.node.check_trajectory(traj)
        self.assertTrue(valid)
        self.assertEqual(idx, -1)

if __name__ == '__main__':
    unittest.main()
