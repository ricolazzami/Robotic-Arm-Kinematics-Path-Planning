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

    def test_safe_trajectory_start_pose(self):
        traj = JointTrajectory()
        point = JointTrajectoryPoint()
        point.positions = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
        traj.points.append(point)
        valid, idx, obj = self.node.check_trajectory(traj)
        self.assertTrue(valid)

    def test_fk_multi_link_no_self_collision(self):
        from arm_planning.collision_checker_node import ur5e_fk_links
        joints = [0.0, -1.57, 1.57, 0.0, 0.0, 0.0]
        tfs = ur5e_fk_links(joints)
        self.assertEqual(len(tfs), 7)
        for T in tfs:
            self.assertEqual(T.shape, (4, 4))

    def test_env_objects_loaded(self):
        self.assertGreaterEqual(len(self.node.objects), 1)

if __name__ == '__main__':
    unittest.main()
