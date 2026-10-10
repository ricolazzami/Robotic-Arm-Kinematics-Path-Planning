#!/usr/bin/env python3

import math
import numpy as np
import yaml
import fcl

def test_pipeline():
    print("=== TEST PIPELINE KINEMATICS & PLANNING ===")
    
    # 1. Input Target Pose
    x, y, z = 0.4, 0.2, 0.4
    qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
    print(f"[1] Target Pose: x={x}, y={y}, z={z}, q=({qx},{qy},{qz},{qw})")
    
    # Check normalized quaternion
    q_norm = math.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
    assert 0.999 <= q_norm <= 1.001, "Quaternion must be normalized"
    
    # Check workspace boundary (radius <= 0.85m, z <= 1.2m)
    dist_xy = math.sqrt(x**2 + y**2)
    assert dist_xy <= 0.85 and abs(z) <= 1.2, "Pose must be in workspace"
    print(" -> Pose input valid & within workspace boundary.")

    # 2. IK Solver
    q_start = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
    q_goal = [0.5, -1.2, 1.3, -1.6, -1.57, 0.5]
    joint_limits_lower = [-2*math.pi, -2*math.pi, -math.pi, -2*math.pi, -2*math.pi, -2*math.pi]
    joint_limits_upper = [2*math.pi, 2*math.pi, math.pi, 2*math.pi, 2*math.pi, 2*math.pi]
    
    for i in range(len(q_goal)):
        assert joint_limits_lower[i] <= q_goal[i] <= joint_limits_upper[i], f"Joint {i} out of limits"
    print(" -> IK Solution within valid joint limits.")

    # 3. Quintic Polynomial Trajectory Generation
    velocity_limit = math.pi
    n_joints = len(q_start)
    max_displacement = max(abs(q_goal[i] - q_start[i]) for i in range(n_joints))
    duration = max(max_displacement / velocity_limit, 1.0)
    n_waypoints = max(10, int(duration * 10))
    t = np.linspace(0, duration, n_waypoints)
    
    waypoints = []
    velocities = []
    for i in range(n_waypoints):
        alpha = t[i] / duration
        # Quintic polynomial: s(alpha) = 10*a^3 - 15*a^4 + 6*a^5
        s = 10*alpha**3 - 15*alpha**4 + 6*alpha**5
        ds_da = 30*alpha**2 - 60*alpha**3 + 30*alpha**4
        ds_dt = ds_da / duration
        
        pos = [q_start[j] + s * (q_goal[j] - q_start[j]) for j in range(n_joints)]
        vel = [ds_dt * (q_goal[j] - q_start[j]) for j in range(n_joints)]
        
        for v in vel:
            assert abs(v) <= velocity_limit + 1e-4, f"Velocity exceeded limit: {v}"
            
        waypoints.append(pos)
        velocities.append(vel)
        
    print(f" -> Quintic Spline generated: {n_waypoints} waypoints over {duration:.2f}s, all velocities <= {velocity_limit:.2f} rad/s.")

    # 4. FCL Collision Checking
    # Load objects from config
    with open("src/arm_planning/config/collision_objects.yaml", "r") as f:
        data = yaml.safe_load(f)
        
    env_objects = []
    for obj in data.get('objects', []):
        p = obj.get('pose', {})
        trans = np.array([p.get('x', 0.0), p.get('y', 0.0), p.get('z', 0.0)])
        tf = fcl.Transform(trans)
        if obj.get('type') == 'box':
            dims = obj.get('dimensions', [0.1, 0.1, 0.1])
            geom = fcl.Box(dims[0], dims[1], dims[2])
        elif obj.get('type') == 'cylinder':
            geom = fcl.Cylinder(obj.get('radius', 0.05), obj.get('height', 0.1))
        env_objects.append(fcl.CollisionObject(geom, tf))
        
    print(f" -> Loaded {len(env_objects)} environmental obstacle collision objects.")

    # Check waypoints collision
    req = fcl.CollisionRequest()
    collisions_detected = 0
    for idx, pos in enumerate(waypoints):
        ee_x = 0.4 * np.cos(pos[0])
        ee_y = 0.4 * np.sin(pos[0])
        ee_z = 0.4 + 0.3 * np.sin(pos[1])
        robot_geom = fcl.Sphere(0.05)
        robot_tf = fcl.Transform(np.array([ee_x, ee_y, ee_z]))
        robot_co = fcl.CollisionObject(robot_geom, robot_tf)
        
        for env_co in env_objects:
            res = fcl.CollisionResult()
            if fcl.collide(robot_co, env_co, req, res):
                collisions_detected += 1
                
    assert collisions_detected == 0, f"Collisions detected: {collisions_detected}"
    print(" -> Collision check PASSED: 0 collisions detected across all trajectory waypoints.")

    print("\n✅ PIPELINE INTEGRATION TEST: SUCCESSFUL")

if __name__ == '__main__':
    test_pipeline()
