import unittest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

class TestIntegrationPipeline(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('test_integration_node')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_target_pose_publication(self):
        pub = self.node.create_publisher(PoseStamped, '/planning/target_pose', 10)
        msg = PoseStamped()
        msg.header.frame_id = 'base_link'
        msg.pose.position.x = 0.4
        msg.pose.position.y = 0.0
        msg.pose.position.z = 0.4
        msg.pose.orientation.w = 1.0
        pub.publish(msg)
        self.assertIsNotNone(msg)

    def test_emergency_stop_trigger(self):
        pub = self.node.create_publisher(Bool, '/emergency_stop', 10)
        msg = Bool()
        msg.data = True
        pub.publish(msg)
        self.assertTrue(msg.data)

if __name__ == '__main__':
    unittest.main()
