#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import Trigger
import yaml
import fcl
import numpy as np
import os
import math
from ament_index_python.packages import get_package_share_directory

def dh_transform(theta, d, a, alpha):
    ct = math.cos(theta)
    st = math.sin(theta)
    ca = math.cos(alpha)
    sa = math.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,      sa,       ca,      d],
        [0.0,     0.0,      0.0,    1.0]
    ])

def ur5e_fk_links(joints):
    # Standard UR5e DH parameters: (d, a, alpha)
    d = [0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996]
    a = [0.0, -0.425, -0.3922, 0.0, 0.0, 0.0]
    alpha = [math.pi/2, 0.0, 0.0, math.pi/2, -math.pi/2, 0.0]
    
    transforms = []
    T = np.eye(4)
    transforms.append(T.copy()) # Base frame
    
    for i in range(6):
        theta = joints[i]
        Ti = dh_transform(theta, d[i], a[i], alpha[i])
        T = np.dot(T, Ti)
        transforms.append(T.copy())
        
    return transforms

class CollisionCheckerNode(Node):
    def __init__(self):
        super().__init__('collision_checker_node')
        
        self.get_logger().info('Collision Checker Node initializing...')
        
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
        
        # Robot link collision radii (approximate capsules/spheres along links)
        self.link_radii = [0.06, 0.06, 0.05, 0.045, 0.045, 0.04, 0.035]
        
        self.get_logger().info(f'Collision Checker Node ready. Loaded {len(self.objects)} environmental objects.')

    def publish_markers(self):
        marker_array = MarkerArray()
        for i, (name, co, dims) in enumerate(self.objects):
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

            if dims and len(dims) == 3 and dims[0] == dims[1] and dims[0] != dims[2]:
                marker.type = Marker.CYLINDER
                marker.scale.x = float(dims[0])
                marker.scale.y = float(dims[1])
                marker.scale.z = float(dims[2])
            else:
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
                dims = [r*2, r*2, h]
                geom = fcl.Cylinder(r, h)
            else:
                continue

            co = fcl.CollisionObject(geom, tf)
            fcl_objs.append((obj.get('name'), co, dims))

        return fcl_objs

    def trajectory_callback(self, msg: JointTrajectory):
        valid, first_invalid_idx, colliding_obj = self.check_trajectory(msg)
        if valid:
            self.get_logger().info('Trajectory check: PASSED (no collision)')
        else:
            self.get_logger().error(f'Trajectory check: FAILED (collision with {colliding_obj} at waypoint {first_invalid_idx})')

    def check_trajectory(self, trajectory: JointTrajectory):
        req = fcl.CollisionRequest()
        
        for idx, point in enumerate(trajectory.points):
            positions = point.positions
            if len(positions) < 6:
                continue
                
            link_transforms = ur5e_fk_links(positions[:6])
            
            # Generate collision objects along the robot links (segments between joint frames)
            robot_link_cos = []
            for i in range(len(link_transforms) - 1):
                p1 = link_transforms[i][:3, 3]
                p2 = link_transforms[i+1][:3, 3]
                
                # Sample points along the link segment to approximate link volume
                num_samples = max(2, int(np.linalg.norm(p2 - p1) / 0.08))
                for alpha in np.linspace(0.0, 1.0, num_samples):
                    p_sample = (1.0 - alpha) * p1 + alpha * p2
                    r = self.link_radii[min(i, len(self.link_radii) - 1)]
                    sphere_geom = fcl.Sphere(r)
                    tf_sample = fcl.Transform(p_sample)
                    robot_link_cos.append((i, fcl.CollisionObject(sphere_geom, tf_sample)))

            # Check each robot link against environmental objects with ACM
            for link_idx, robot_co in robot_link_cos:
                for name, env_co, _ in self.objects:
                    # Allowed Collision Matrix (ACM):
                    # Link 0 (base) with table is allowed (robot mounted on table)
                    if link_idx == 0 and 'table' in name.lower():
                        continue
                        
                    res = fcl.CollisionResult()
                    in_collision = fcl.collide(robot_co, env_co, req, res)
                    if in_collision:
                        return False, idx, name
                        
        return True, -1, ""

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
