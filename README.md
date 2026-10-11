# Robotic Arm Kinematics & Path Planning

Digital brain untuk manipulator UR5e 6-DOF di ROS 2 (RViz-only, tanpa Gazebo biar ringan). Terima target pose SE(3), hitung IK via TRAC-IK, generate trajectory quintic spline, cek collision FCL per-waypoint, eksekusi via `ros2_control` FakeSystem + preview di RViz2.

---

## Daftar Isi
1. [Fitur Utama](#fitur-utama)
2. [Tech Stack](#tech-stack)
3. [Arsitektur Sistem](#arsitektur-sistem)
4. [Model IPO](#model-ipo-input---process---output)
5. [Struktur Direktori](#struktur-direktori-workspace)
6. [Deskripsi Package & Node](#deskripsi-package--node)
7. [ROS 2 Interface](#ros-2-interface-management)
8. [Panduan Instalasi & Build](#panduan-instalasi--build)
9. [Cara Menjalankan](#cara-menjalankan-sistem)
10. [Pengujian Standalone](#pengujian-standalone-end-to-end-test)
11. [Mekanisme Keselamatan](#mekanisme-keselamatan-safety-overrides)
12. [Troubleshooting](#troubleshooting--known-issues)
13. [Author](#author--kontributor)

---

## Fitur Utama

- **UR5e 6-DOF resmi:** `ur_description` v2.14.0 (xacro modular + mesh DAE/STL) + meja/kubus/silinder.
- **TRAC-IK Solver:** Service `/ik_solver/compute_ik` (`moveit_msgs/srv/GetPositionIK`), retry 3x, timeout 50ms, validasi quaternion & workspace silinder `R≤0.85m, |Z|≤1.2m` + joint limits.
- **Trajectory Quintic:** `s(α)=10α³-15α⁴+6α⁵`, ≥10 waypoint, time-parameterized, limit `≤π rad/s`.
- **Collision FCL multi-link:** FK 6-link (DH UR5e) → sphere samples per-link → FCL vs `collision_objects.yaml` + ACM (adjacent & base↔table diizinkan), publish `MarkerArray` ke RViz.
- **Coordinator State Machine:** `IDLE→PLANNING→EXECUTING→IDLE/LOCKED`, e-stop interdependent locking (unlock butuh `/planning/reset` + e-stop `False`), Action Client `FollowJointTrajectory`.

---

## Tech Stack

| Komponen | Teknologi | Keterangan |
|---|---|---|
| **Middleware** | ROS 2 Humble | Topics/Services/Actions |
| **Manipulator** | Universal Robots UR5e | 6-DOF via `ur_description` |
| **Deskripsi** | URDF/Xacro | `ur_macro.xacro` + `ros2_control` FakeSystem |
| **Visualisasi** | RViz2 | Preview + `joint_state_publisher_gui` + markers |
| **Kontrol** | `ros2_control` + `mock_components` | `JointTrajectoryController`, `JointStateBroadcaster` (FakeSystem, tanpa Gazebo) |
| **Collision** | `python-fcl` | Broadphase/narrowphase per-waypoint |
| **Bahasa** | Python 3.10+ | `rclpy` |

> GazeboClassic + `gazebo_ros2_control` sengaja dihapus (berat, OGRE crash headless). Mode ringan = RViz + FakeSystem cukup untuk validasi kinematika.

---

## Arsitektur Sistem

```
┌─────────────────────────────┐
│      USER / TASK NODE       │
│ Publish PoseStamped         │
│ /planning/target_pose       │
└──────────────┬──────────────┘
               ▼
┌──────────────────────────────────────────┐
│ PLANNING LAYER (arm_planning)             │
│ TF2 → TRAC-IK → Quintic → FCL → Coordinator │
│ (on-demand, ~10 Hz)                       │
└──────────────┬───────────────────────────┘
               │ FollowJointTrajectory (Action)
               ▼
┌─────────────────────────────┐
│ MOCK CONTROL (ros2_control) │
│ JointTrajectoryController   │
│ FakeSystem → /joint_states │
└──────────────┬──────────────┘
               ▼
┌─────────────────────────────┐
│ RViz2                       │
│ trajectory_preview + markers│
└─────────────────────────────┘
```

---

## Model IPO (Input - Process - Output)

**Input:** `geometry_msgs/msg/PoseStamped` di `/planning/target_pose` — posisi `(x,y,z)` meter + quaternion `(qx,qy,qz,qw)` normalized `1±0.001`, `frame_id=base_link`.

**Process:** TF2 lookup (timeout 0.5s, umur ≤1s) → TRAC-IK (3 attempts) + joint limits → quintic trajectory (≥10 wp, `≤π rad/s`) → FCL multi-link check per-waypoint + ACM.

**Output:** `/planning/trajectory_preview` (`JointTrajectory`) + `/collision_scene_markers` (`MarkerArray`) ke RViz; `FollowJointTrajectory` goal ke controller. `TRANSFORM_ERROR`/`IK_NO_SOLUTION`/`COLLISION_DETECTED`/`OUT_OF_WORKSPACE` jika gagal.

---

## Struktur Direktori Workspace

```
Robotic-Arm-Kinematics-Path-Planning/
├── src/
│   ├── arm_description/      # URDF/Xacro + RViz
│   │   ├── launch/display.launch.py
│   │   ├── rviz/view_robot.rviz
│   │   └── urdf/{robot,ur5e,workspace_objects}.urdf.xacro
│   ├── arm_control/          # ros2_control config (FakeSystem)
│   │   └── config/controllers.yaml
│   ├── arm_planning/         # IK, trajectory, collision, coordinator
│   │   ├── arm_planning/{ik_solver,trajectory_planner,collision_checker,planning_coordinator}_node.py
│   │   ├── config/{kinematics,collision_objects}.yaml
│   │   └── launch/planning.launch.py
│   └── arm_bringup/          # Entry point
│       └── launch/simulation.launch.py  # RSP + RViz + planning
├── test_pipeline_e2e.py
├── PRD.md / architecture.md / instructions.md / TODO.md
```

---

## Deskripsi Package & Node

- **arm_description:** `robot.urdf.xacro` (rakit UR5e + workspace), `ur5e.urdf.xacro` (`use_fake_hardware=true`, `sim_gazebo=false`), `workspace_objects.xacro` (meja 1.0×0.6×0.05, kubus 0.05, silinder r0.025 h0.1).
- **arm_control:** `controllers.yaml` (`joint_state_broadcaster` + `arm_controller/JointTrajectoryController`).
- **arm_planning:** `ik_solver_node` (TRAC-IK service), `trajectory_planner_node` (quintic), `collision_checker_node` (FCL multi-link + ACM), `planning_coordinator_node` (state machine + e-stop + Action Client).
- **arm_bringup:** `simulation.launch.py` satu command nyalakan semua (tanpa Gazebo).

---

## ROS 2 Interface Management

| Topik | Tipe | Pub | Sub | Fungsi |
|---|---|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | `joint_state_publisher` / FakeSystem | planning, RViz | sudut aktual |
| `/planning/target_pose` | `geometry_msgs/PoseStamped` | User | Coordinator | target EE |
| `/planning/trajectory_preview` | `trajectory_msgs/JointTrajectory` | Coordinator/Planner | RViz, Collision | preview |
| `/collision_scene_markers` | `visualization_msgs/MarkerArray` | Collision Checker | RViz | rintangan 3D |
| `/emergency_stop` | `std_msgs/Bool` | Safety/Operator | Coordinator | trigger LOCKED |

| Service | Tipe | Server | Fungsi |
|---|---|---|---|
| `/ik_solver/compute_ik` | `moveit_msgs/GetPositionIK` | IK Node | IK |
| `/planning/reset` | `std_srvs/Trigger` | Coordinator | unlock LOCKED (butuh e-stop=False) |

| Action | Tipe | Server | Client |
|---|---|---|---|
| `/arm_controller/follow_joint_trajectory` | `control_msgs/FollowJointTrajectory` | `controller_manager` (FakeSystem) | Coordinator |

---

## Panduan Instalasi & Build

```bash
micromamba activate ros2_env  # atau source /opt/ros/humble/setup.bash
pip install python-fcl scipy numpy pyyaml
colcon build --symlink-install
source install/setup.bash
```

---

## Cara Menjalankan Sistem

### 1. Preview Kinematika (RViz + slider)

```bash
ros2 launch arm_description display.launch.py
```

### 2. Planning Pipeline (RViz + FakeSystem)

```bash
ros2 launch arm_bringup simulation.launch.py
# terminal lain:
ros2 topic pub --once /planning/target_pose geometry_msgs/msg/PoseStamped "{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'},
  pose: {position: {x: 0.4, y: 0.2, z: 0.4}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}
}"
# emergency stop + reset:
ros2 topic pub --once /emergency_stop std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /emergency_stop std_msgs/msg/Bool "{data: false}"
ros2 service call /planning/reset std_srvs/srv/Trigger
```

---

## Pengujian Standalone (End-to-End Test)

Tanpa GUI/daemon ROS:

```bash
micromamba run -p ~/ros2_env python3 test_pipeline_e2e.py
```

```
=== TEST PIPELINE KINEMATICS & PLANNING ===
 -> Pose valid & within workspace
 -> IK within limits
 -> Quintic: 10 waypoints over 1.00s, velocities ≤3.14 rad/s
 -> Loaded 3 obstacles
 -> Collision PASSED: 0 collisions
✅ PIPELINE INTEGRATION TEST: SUCCESSFUL
```

---

## Mekanisme Keselamatan (Safety Overrides)

1. **Emergency Stop (LOCKED):** `/emergency_stop=True` → cancel goal aktif, state `LOCKED`, tolak target baru, log FATAL. Tetap `LOCKED` walau e-stop `False` sampai `ros2 service call /planning/reset` dipanggil.
2. **Velocity Limit:** `≤π rad/s` per-joint, validasi saat generation → tolak bukan clamp.
3. **Workspace:** Silinder `R>0.85m` atau `|Z|>1.2m` → `OUT_OF_WORKSPACE`.
4. **Collision:** Tidak ada bypass; scene belum load → tolak semua.

---

## Troubleshooting & Known Issues

- **Gazebo dihapus:** Repo ini RViz-only. Jika butuh fisika, checkout tag `gazebo` lama atau `ros2 launch` Gazebo terpisah.
- **`joint_state_publisher_gui` tidak ada:** `sudo apt install ros-humble-joint-state-publisher-gui` atau `micromamba install -c robostack-staging ros-humble-joint-state-publisher-gui`.
- **`trac_ik_python` import gagal:** Build `src/trac_ik` butuh `moveit_core`; fallback geometri aktif otomatis. Untuk full TRAC-IK: `colcon build --packages-up-to trac_ik_python`.

---

## Author & Kontributor

- **Author:** Rico Lazzami ([@ricolazzami](https://github.com/ricolazzami))
- **Project:** DecodeLabs Internship Project 1
- **Lisensi:** Apache-2.0
