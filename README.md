# DecodeLabs Project 1: Robotic Arm Kinematics & Path Planning

A digital brain for a 6-DOF UR5e manipulator running on ROS 2. Send a target pose in space (position + orientation), and the system computes the joint angles, builds a smooth trajectory, checks it for collisions, then moves the arm in RViz. Gazebo is intentionally left out to keep things light — RViz + FakeSystem is enough here.

![Architecture](docs/architecture.png)

*Interactive diagram: [docs/architecture.html](docs/architecture.html)*

---

## Contents
1. [Key Features](#key-features)
2. [Tech Stack](#tech-stack)
3. [How It Works (IPO)](#how-it-works-ipo)
4. [Math & Calculations](#math--calculations)
5. [Directory Layout](#directory-layout)
6. [Packages & Nodes](#packages--nodes)
7. [ROS 2 Interfaces](#ros-2-interfaces)
8. [Install & Build](#install--build)
9. [Running the System](#running-the-system)
10. [Headless Test](#headless-test)
11. [Safety Rules](#safety-rules)
12. [Troubleshooting](#troubleshooting)
13. [Author](#author)

---

## Key Features

- **Real robot model — UR5e 6-DOF.** Official meshes and parameters from `ur_description`, plus a table, a red cube, and a blue cylinder on top.
- **TRAC-IK solver.** Give it a target position and the `ik_solver` node returns the six joint angles. Joint limits are checked so no joint gets pushed past what the hardware allows.
- **Smooth motion.** A quintic polynomial keeps position, velocity, and acceleration continuous — no jerky starts or stops.
- **Collision-aware.** Every step along the path is checked with FCL (Flexible Collision Library). If it would hit the table or a block, the whole trajectory is rejected.
- **Managed pipeline.** One node (`planning_coordinator`) runs the full sequence: receive target → check transform → solve IK → build trajectory → check collisions → move. An emergency stop locks everything down.

---

## Tech Stack

| Component | Choice | Purpose |
|---|---|---|
| **Middleware** | ROS 2 Humble | Inter-node communication |
| **Robot** | UR5e Universal Robots | 6-joint arm |
| **Physical model** | URDF/Xacro | Links, visual/collision meshes |
| **Visualization** | RViz2 | Watch the arm move + preview paths |
| **Control** | `ros2_control` FakeSystem | Simulation without real hardware |
| **Collision** | `python-fcl` | Per-waypoint collision checks |
| **Language** | Python 3.10+ (`rclpy`) | Planning nodes |

> Why no Gazebo? It was used at first, but it is heavy and tends to crash on headless Linux. For testing kinematics, trajectories, and collisions, RViz-only is lighter and sufficient.

---

## How It Works (IPO)

**Input** — Publish a `PoseStamped` message to `/planning/target_pose`: coordinates `x, y, z` (meters) plus orientation `qx, qy, qz, qw` (quaternion), in the `base_link` frame.

**Process** — The system runs these steps in order:
1. Check the `base_link` transform is still fresh (age ≤ 1 second).
2. Solve IK with TRAC-IK (up to 3 attempts, 50 ms timeout each).
3. Build a quintic trajectory with at least 60 waypoints (joint speed limit `π rad/s`).
4. Check every waypoint for multi-link collisions using FCL.

**Output** — On success, a `JointTrajectory` goes to `/planning/trajectory_preview` for RViz, plus obstacle markers on `/collision_scene_markers`. On failure you get a clear reason: `IK_NO_SOLUTION`, `COLLISION_DETECTED`, or `OUT_OF_WORKSPACE`.

---

## Math & Calculations

This section covers the formulas behind the pipeline — skip it if you only want to run the robot.

### 1. Quaternion Validation

Quaternions must be normalized for a valid rotation:

$$\sqrt{q_x^2 + q_y^2 + q_z^2 + q_w^2} = 1 \pm 0.001$$

Targets outside this tolerance are rejected.

### 2. Workspace Limits

The UR5e can only reach inside a cylinder. Check before computing:

$$R = \sqrt{x^2 + y^2} \le 0.85~\text{m}, \quad |z| \le 1.2~\text{m}$$

Anything beyond that returns `OUT_OF_WORKSPACE`.

### 3. Forward Kinematics (UR5e DH Parameters)

Each joint has a Denavit-Hartenberg transform:

$$T_i = \begin{bmatrix} \cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\ \sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\ 0 & \sin\alpha_i & \cos\alpha_i & d_i \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

Chain them from base to tool:

$$T_{06} = T_1 T_2 T_3 T_4 T_5 T_6$$

Position `p = (x, y, z)` and rotation `R` come out of this matrix and feed the collision check.

UR5e DH parameters:

| Link | d (m) | a (m) | alpha (rad) |
|---|---|---|---|
| 1 | 0.1625 | 0.0 | pi/2 |
| 2 | 0.0 | -0.425 | 0 |
| 3 | 0.0 | -0.3922 | 0 |
| 4 | 0.1333 | 0.0 | pi/2 |
| 5 | 0.0997 | 0.0 | -pi/2 |
| 6 | 0.0996 | 0.0 | 0 |

### 4. Inverse Kinematics (Geometric Fallback)

When the TRAC-IK Python wrapper is unavailable, an analytic 2-link fallback runs in the radial plane. Base yaw first:

$$\theta_1 = \text{atan2}(y, x)$$

Planar reach and elbow angle:

$$D = \sqrt{r^2 + h^2}, \quad r = \sqrt{x^2+y^2}, \quad h = z - d_1$$

$$\cos\theta_3 = \frac{D^2 - a_2^2 - a_3^2}{2 a_2 a_3}, \quad \theta_3 = \text{arccos}(\cos\theta_3)$$

Shoulder angle and wrist alignment:

$$\alpha = \text{atan2}(h, r), \quad \beta = \text{atan2}(a_3\sin\theta_3, a_2 + a_3\cos\theta_3)$$

$$\theta_2 = -(\alpha + \beta), \quad \theta_4 = -(\theta_2 + \theta_3), \quad \theta_5 = -\pi/2, \quad \theta_6 = 0$$

### 5. Trajectory — Quintic Polynomial Spline

Smooth motion with continuous position, velocity, and acceleration:

$$u = \frac{t}{T}, \quad s(u) = 10u^3 - 15u^4 + 6u^5$$

$$v(t) = \frac{30u^2 - 60u^3 + 30u^4}{T}$$

Joint positions and velocities:

$$q_j(t) = q_{start,j} + s(u)(q_{goal,j} - q_{start,j})$$

$$\dot{q}_j(t) = v(t)(q_{goal,j} - q_{start,j}), \quad |\dot{q}_j| \le \pi~\text{rad/s}$$

Duration and waypoint count:

$$T = \max(3.0, 1.5 \cdot \max_j|q_{goal,j} - q_{start,j}|), \quad N = 20 \cdot T \approx 60$$

### 6. Collision Checking (FCL)

Each link is sampled as small spheres along its segment:

$$p = (1-t)p_i + t p_{i+1}, \quad t \in [0,1], \quad r_i \in \{0.06, 0.05, 0.045, ...\}$$

Each sample is tested against every environment object (table, cube, cylinder):

$$\text{collide} = \text{fcl.collide}(\text{sphere}(r_i), \text{box or cylinder})$$

Allowed Collision Matrix (ACM): `base` vs `table` is allowed (the robot sits on the table), and adjacent links are allowed. Everything else — one colliding waypoint rejects the trajectory with `COLLISION_DETECTED` plus the waypoint index.

### 7. Timing

Simple time parameterization (legacy form):

$$T = \max\left(\frac{\max_j|\Delta q_j|}{\pi}, 1.0\right)$$

Currently used: $T = \max(3.0, 1.5 \cdot \max|\Delta q|)$ so motion stays visible in RViz at 20 Hz interpolation (3 seconds × 20 = 60 points).

---

## Directory Layout

```
Robotic-Arm-Kinematics-Path-Planning/
├── src/
│   ├── arm_description/      # Robot shape & RViz
│   │   ├── launch/display.launch.py
│   │   ├── rviz/view_robot.rviz
│   │   └── urdf/{robot,ur5e,workspace_objects}.urdf.xacro
│   ├── arm_control/          # Fake controller config
│   │   └── config/controllers.yaml
│   ├── arm_planning/         # The robot's brain
│   │   ├── arm_planning/{ik_solver,trajectory_planner,collision_checker,planning_coordinator}_node.py
│   │   ├── config/{kinematics,collision_objects}.yaml
│   │   └── launch/planning.launch.py
│   └── arm_bringup/          # One command to start everything
│       └── launch/simulation.launch.py
├── test_pipeline_e2e.py      # Math test without GUI
├── docs/{architecture.png, architecture.html}
├── PRD.md / architecture.md / instructions.md / TODO.md
```

---

## Packages & Nodes

- **arm_description** — Assembles the Xacro files into a complete robot. `robot.urdf.xacro` connects the UR5e to the table, cube, and cylinder via the chain `world → table → l0 → base_link`. `use_fake_hardware=true` so no Gazebo is needed.
- **arm_control** — Just holds `controllers.yaml` for the fake `JointTrajectoryController`.
- **arm_planning** — All the logic lives here: `ik_solver` (joint angles), `trajectory_planner` (path building), `collision_checker` (FCL checks), `planning_coordinator` (sequencing + e-stop).
- **arm_bringup** — `simulation.launch.py` starts RViz and every planning node at once.

---

## ROS 2 Interfaces

| Topic | Type | From | To | Purpose |
|---|---|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | mock executor | RViz & planning | Current joint angles |
| `/planning/target_pose` | `geometry_msgs/PoseStamped` | you | coordinator | End-effector goal |
| `/planning/trajectory_preview` | `trajectory_msgs/JointTrajectory` | coordinator | RViz | Preview path before moving |
| `/collision_scene_markers` | `visualization_msgs/MarkerArray` | collision_checker | RViz | Red obstacle boxes |
| `/emergency_stop` | `std_msgs/Bool` | safety | coordinator | Emergency trigger |

| Service | Type | Server | Purpose |
|---|---|---|---|
| `/ik_solver/compute_ik` | `moveit_msgs/GetPositionIK` | ik_solver | Compute IK |
| `/planning/reset` | `std_srvs/Trigger` | coordinator | Unlock after e-stop |

| Action | Type | Server | Client |
|---|---|---|---|
| `/arm_controller/follow_joint_trajectory` | `control_msgs/FollowJointTrajectory` | FakeSystem | coordinator |

---

## Install & Build

```bash
micromamba activate ros2_env  # or source /opt/ros/humble/setup.bash
pip install python-fcl scipy numpy pyyaml
colcon build --symlink-install
source install/setup.bash
```

---

## Running the System

**Terminal 1 — Start the simulation (wait for RViz to appear):**
```bash
./run_simulation.sh
# password if asked: kali or ricou
```

**Terminal 2 — Move the robot:**

*Forward:*
```bash
podman exec $(podman ps -q) bash -c "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 topic pub --once /planning/target_pose geometry_msgs/msg/PoseStamped '{header: {frame_id: \"base_link\"}, pose: {position: {x: 0.55, y: 0.00, z: 0.35}, orientation: {w: 1.0}}}'"
```

*Backward:*
```bash
podman exec $(podman ps -q) bash -c "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 topic pub --once /planning/target_pose geometry_msgs/msg/PoseStamped '{header: {frame_id: \"base_link\"}, pose: {position: {x: 0.25, y: 0.00, z: 0.35}, orientation: {w: 1.0}}}'"
```

*Emergency stop + unlock:*
```bash
ros2 topic pub --once /emergency_stop std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /emergency_stop std_msgs/msg/Bool "{data: false}"
ros2 service call /planning/reset std_srvs/srv/Trigger
```

---

## Headless Test

Check the math without opening any GUI:

```bash
micromamba run -p ~/ros2_env python3 test_pipeline_e2e.py
```

Expected result:

```
=== TEST PIPELINE KINEMATICS & PLANNING ===
 -> Pose valid & within workspace
 -> IK within limits
 -> Quintic: 60 waypoints over 3.00s, velocities ≤3.14 rad/s
 -> Loaded 3 obstacles
 -> Collision PASSED: 0 collisions
✅ PIPELINE INTEGRATION TEST: SUCCESSFUL
```

---

## Safety Rules

1. **Emergency stop (`LOCKED`):** `/emergency_stop=True` cancels the active goal, sets state to `LOCKED`, and rejects all new targets. It stays locked even after `False` — call `ros2 service call /planning/reset` to unlock.
2. **Speed limit:** `≤ π rad/s` per joint. Any waypoint over the limit rejects the trajectory outright (never clamped).
3. **Workspace limit:** cylinder `R > 0.85 m` or `|Z| > 1.2 m` → rejected with `OUT_OF_WORKSPACE`.
4. **Collisions:** no bypass exists. If the scene has not loaded, every trajectory is rejected.

---

## Troubleshooting

- **Simulation will not start?** Make sure podman/docker is running, then `chmod +x run_simulation.sh`.
- **`joint_state_publisher_gui` missing?** `sudo apt install ros-humble-joint-state-publisher-gui`
- **`trac_ik_python` import fails?** Normal when `moveit_core` has not been built — the geometric fallback kicks in automatically. For real TRAC-IK: `colcon build --packages-up-to trac_ik_python`.

---

## Author

- **Rico Lazzami** ([@ricolazzami](https://github.com/ricolazzami))
- **Project:** DecodeLabs Internship Project 1
- **License:** Apache-2.0
