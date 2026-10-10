#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import yaml
import fcl
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

class CollisionCheckerNode(Node):
    def __init__(self):
        super().__init__('collision_checker_node')
        
        self.get_logger().info('Collision Checker Node initializing...')
        
        # Load collision objects
        try:
            pkg_share = get_package_share_directory('arm_planning')
            config_path = os.path.join(pkg_share, 'config', 'collision_objects.yaml')
        except Exception:
            config_path = os.path.join(os.path.dirname(__file__), '..', 'config', 'collision_objects.yaml')
            
        self.objects = self.load_objects(config_path)
        
        self.subscription = self.create_subscription(
            JointTrajectory, '/planning/trajectory_preview', self.trajectory_callback, 10
        )
        
        self.marker_publisher = self.create_publisher(MarkerArray, '/collision_scene_markers', 10)
        
        self.timer = self.create_timer(1.0, self.publish_markers)
        
        self.get_logger().info(f'Collision Checker Node ready. Loaded {len(self.objects)} environmental objects.')

    def publish_markers(self):
        marker_array = MarkerArray()
        for i, (name, co) in enumerate(self.objects):
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "collision_objects"
            marker.id = i
            marker.action = Marker.ADD
            
            tf = co.getTransform()
            trans = tf.getTranslation()
            marker.pose.position.x = float(trans[0])
            marker.pose.position.y = float(trans[1])
            marker.pose.position.z = float(trans[2])
            marker.pose.orientation.w = 1.0
            
            geom = co.getNodeType()
            if isinstance(co.user_data, dict):
                dims = co.user_data.get('dims', [0.1, 0.1, 0.1])
            else:
                dims = [0.05, 0.05, 0.05]

            marker.type = Marker.CUBE
            marker.scale.x = float(dims[0]) if len(dims) > 0 else 0.1
            marker.scale.y = float(dims[1]) if len(dims) > 1 else 0.1
            marker.scale.z = float(dims[2]) if len(dims) > 2 else 0.1

            marker.color.r = 0.8
            marker.color.g = 0.2
            marker.color.b = 0.2
            marker.color.a = 0.7
            
            marker_array.markers.append(marker)
        
        self.marker_publisher.publish(marker_array)

    def load_objects(self, yaml_path):
        fcl_objs = []
        if not os.path.exists(yaml_path):
            self.get_logger().warn(f'Config file not found: {yaml_path}')
            return fcl_objs

        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)

        for obj in data.get('objects', []):
            o_type = obj.get('type')
            p = obj.get('pose', {})
            trans = np.array([p.get('x', 0.0), p.get('y', 0.0), p.get('z', 0.0)])
            tf = fcl.Transform(trans)

            if o_type == 'box':
                dims = obj.get('dimensions', [0.1, 0.1, 0.1])
                geom = fcl.Box(dims[0], dims[1], dims[2])
            elif o_type == 'cylinder':
                r = obj.get('radius', 0.05)
                h = obj.get('height', 0.1)
                geom = fcl.Cylinder(r, h)
            else:
                continue

            co = fcl.CollisionObject(geom, tf)
            fcl_objs.append((obj.get('name'), co))

        return fcl_objs

    def trajectory_callback(self, msg: JointTrajectory):
        valid, first_invalid_idx = self.check_trajectory(msg)
        if valid:
            self.get_logger().info('Trajectory check: PASSED (no collision)')
        else:
            self.get_logger().error(f'Trajectory check: FAILED (collision at waypoint {first_invalid_idx})')

    def check_trajectory(self, trajectory: JointTrajectory):
        req = fcl.CollisionRequest()
        
        for idx, point in enumerate(trajectory.points):
            # Check positions
            positions = point.positions
            if len(positions) >= 3:
                # Approximation of end-effector / link positions for collision checking
                ee_x = 0.4 * np.cos(positions[0])
                ee_y = 0.4 * np.sin(positions[0])
                ee_z = 0.4 + 0.3 * np.sin(positions[1])
                
                robot_geom = fcl.Sphere(0.05)
                robot_tf = fcl.Transform(np.array([ee_x, ee_y, ee_z]))
                robot_co = fcl.CollisionObject(robot_geom, robot_tf)
                
                for name, env_co in self.objects:
                    res = fcl.CollisionResult()
                    in_collision = fcl.collide(robot_co, env_co, req, res)
                    if in_collision:
                        return False, idx
                        
        return True, -1

def main(args=None):
    rclpy.init(args=args)
    node = CollisionCheckerNode()
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
