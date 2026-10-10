# Arsitektur Sistem
## DecodeLabs Project 1: Robotic Arm Kinematics & Path Planning

---

## 1. Diagram Arsitektur Keseluruhan

```
                         ┌─────────────────────┐
                         │   USER / TASK NODE   │
                         │  (Input target pose) │
                         └──────────┬───────────┘
                                    │
                          PoseStamped (Topic)
                       /planning/target_pose
                                    │
                                    ▼
┌───────────────────────────────────────────────────────────────────────┐
│                        PLANNING LAYER                                 │
│                                                                       │
│  ┌─────────────┐    ┌──────────────┐    ┌──────────────────────────┐  │
│  │             │    │              │    │                          │  │
│  │  TF2 Buffer │───▶│  TRAC-IK     │───▶│  Trajectory Generator   │  │
│  │  & Lookup   │    │  Solver Node │    │  (Cubic/Quintic Spline) │  │
│  │             │    │              │    │                          │  │
│  └──────┬──────┘    └──────────────┘    └────────────┬─────────────┘  │
│         │                                            │                │
│         │ lookup_transform()                         │ JointTrajectory│
│         │ base_link ← target_frame                   │                │
│         │                                            ▼                │
│         │                              ┌──────────────────────────┐   │
│         │                              │                          │   │
│         │                              │  FCL Collision Checker   │   │
│         │                              │  (Per-waypoint check)    │   │
│         │                              │                          │   │
│         │                              └────────────┬─────────────┘   │
│         │                                           │                 │
│         │                               ✓ Lolos / ✗ Collision         │
│         │                                           │                 │
│  ┌──────┴───────────────────────────────────────────┴──────────────┐  │
│  │                     Planning Coordinator                        │  │
│  │  - Orkestrasikan pipeline TF2 → IK → Trajectory → Collision    │  │
│  │  - Subscribe /emergency_stop                                    │  │
│  │  - Publish preview ke RViz                                      │  │
│  │  - Kirim goal ke Action Server                                  │  │
│  └─────────────────────────────┬───────────────────────────────────┘  │
│                                │                                      │
└────────────────────────────────┼──────────────────────────────────────┘
                                 │
                   FollowJointTrajectory (Action)
              /arm_controller/follow_joint_trajectory
                                 │
                                 ▼
┌───────────────────────────────────────────────────────────────────────┐
│                        CONTROL LAYER                                  │
│                                                                       │
│  ┌─────────────────────────────────────────────────────────────────┐  │
│  │                  ros2_control Controller Manager                │  │
│  │                                                                 │  │
│  │  ┌──────────────────────────────────────────────────────────┐   │  │
│  │  │         JointTrajectoryController                        │   │  │
│  │  │  - Interpolasi trajectory pada rate ≥100 Hz              │   │  │
│  │  │  - Kirim command ke hardware interface                   │   │  │
│  │  │  - Publish feedback (posisi aktual vs target)            │   │  │
│  │  └──────────────────────────┬───────────────────────────────┘   │  │
│  │                             │                                   │  │
│  └─────────────────────────────┼───────────────────────────────────┘  │
│                                │                                      │
│  ┌─────────────────────────────┼───────────────────────────────────┐  │
│  │              gazebo_ros2_control Plugin                         │  │
│  │  - Bridge antara ros2_control hardware interface & Gazebo      │  │
│  │  - PID controller per joint                                    │  │
│  │  - Baca joint state dari simulasi                              │  │
│  └─────────────────────────────┼───────────────────────────────────┘  │
│                                │                                      │
└────────────────────────────────┼──────────────────────────────────────┘
                                 │
                         Torsi / Force command
                                 │
                                 ▼
┌───────────────────────────────────────────────────────────────────────┐
│                       GAZEBO SIMULATION                               │
│                                                                       │
│  ┌────────────┐  ┌────────────┐  ┌─────────────────────────────────┐ │
│  │ UR5e Model │  │ Meja       │  │ Objek Primitif                  │ │
│  │ (6 joints) │  │ (collision)│  │ (Kubus, Silinder — collision)   │ │
│  └────────────┘  └────────────┘  └─────────────────────────────────┘ │
│                                                                       │
│  Physics Engine: ODE                                                  │
│  Gravity: -9.81 m/s² (Z-down)                                        │
│  Step size: 1ms                                                       │
│                                                                       │
└───────────────────────────────────────────────────────────────────────┘
```

---

## 2. Data Flow Detail

### 2.1 Alur Normal (Happy Path)

```
Langkah 1: User Node publish PoseStamped ke /planning/target_pose
     │
     ▼
Langkah 2: Planning Coordinator menerima pose
     │
     ├── Cek: Apakah /emergency_stop aktif? → Ya → TOLAK
     │
     ▼
Langkah 3: TF2 Lookup
     │
     ├── lookup_transform(base_link ← target_frame, timeout=0.5s)
     ├── Validasi umur transform ≤ 1 detik
     ├── Gagal → Return TRANSFORM_ERROR
     │
     ▼
Langkah 4: TRAC-IK Solver
     │
     ├── Input: Pose SE(3) dalam frame base_link
     ├── Output: [θ1, θ2, θ3, θ4, θ5, θ6]
     ├── Validasi: Semua θ dalam joint limits
     ├── Gagal (3x retry) → Return IK_NO_SOLUTION
     │
     ▼
Langkah 5: Trajectory Generation
     │
     ├── Input: Joint config saat ini (dari /joint_states) + Joint config target
     ├── Metode: Cubic spline interpolation
     ├── Output: JointTrajectory (≥10 waypoints, time-parameterized)
     ├── Validasi: Kecepatan setiap joint ≤ π rad/s
     ├── Gagal → Return VELOCITY_LIMIT_EXCEEDED
     │
     ▼
Langkah 6: FCL Collision Check
     │
     ├── Untuk setiap waypoint: FK → collision geometry → cek terhadap scene
     ├── Terapkan Allowed Collision Matrix
     ├── Collision terdeteksi → Return COLLISION_DETECTED + indeks waypoint
     │
     ▼
Langkah 7: Preview (opsional)
     │
     ├── Publish trajectory ke /planning/trajectory_preview (RViz)
     │
     ▼
Langkah 8: Eksekusi
     │
     ├── Kirim FollowJointTrajectory Goal ke Action Server
     ├── Monitor feedback (posisi aktual vs target)
     ├── Tunggu result
     │
     ▼
Langkah 9: Result
     │
     ├── SUCCESSFUL → Log INFO, siap terima target baru
     └── FAILED → Log ERROR, publish status ke /planning/status
```

### 2.2 Alur Emergency Stop

```
/emergency_stop = True (dari sumber eksternal)
     │
     ├──▶ Planning Coordinator:
     │      ├── Cancel goal aktif (jika ada)
     │      ├── Set state = LOCKED
     │      └── Tolak semua target baru
     │
     └──▶ JointTrajectoryController:
            ├── Abort trajectory aktif
            └── Hold posisi sendi saat ini
     
     ... menunggu ...

/emergency_stop = False
     │
     └── Belum cukup. Operator harus panggil:
            /planning/reset (Service Call)
               │
               └── Planning Coordinator:
                      ├── Set state = IDLE
                      └── Siap terima target baru
```

---

## 3. TF2 Transform Tree

```
world
  └── base_link                    (Base robot, terpasang di atas meja)
        ├── shoulder_link
        │     └── upper_arm_link
        │           └── forearm_link
        │                 └── wrist_1_link
        │                       └── wrist_2_link
        │                             └── wrist_3_link
        │                                   └── tool0   (End-effector frame)
        │
        └── base_link_inertia      (Inertia frame, auto-generated)

world
  ├── table_link                   (Static transform, dari launch file)
  ├── object_cube_1                (Dynamic, broadcast oleh scene manager)
  └── object_cylinder_1            (Dynamic, broadcast oleh scene manager)
```

**Konvensi:**
- Transform `world` → `base_link`: static, didefinisikan di launch file.
- Transform `base_link` → `tool0`: dynamic, di-broadcast oleh `robot_state_publisher` berdasarkan `/joint_states`.
- Transform `world` → objek: static atau dynamic tergantung skenario.

---

## 4. Node Graph

```
┌─────────────────────┐     ┌──────────────────────────┐
│  robot_state_        │     │  static_transform_       │
│  publisher           │     │  publisher (launch)      │
│                     │     │                          │
│  Subscribe:          │     │  Broadcast:              │
│   /joint_states     │     │   world→base_link       │
│  Broadcast:          │     │   world→table_link      │
│   TF (dynamic)      │     │                          │
└─────────────────────┘     └──────────────────────────┘

┌─────────────────────┐     ┌──────────────────────────┐
│  planning_           │     │  collision_checker_      │
│  coordinator_node    │     │  node                    │
│                     │     │                          │
│  Subscribe:          │     │  Service Server:         │
│   /planning/target  │     │   /collision/check_      │
│   /emergency_stop   │     │     trajectory           │
│   /joint_states     │     │                          │
│  Publish:            │     │  Maintain:               │
│   /planning/preview │     │   FCL collision scene    │
│   /planning/status  │     │   ACM                    │
│  Action Client:      │     └──────────────────────────┘
│   follow_joint_traj │
│  Service Client:     │     ┌──────────────────────────┐
│   /ik_solver/       │     │  gazebo_ros2_control     │
│     compute_ik      │     │  (Plugin)                │
│   /collision/check  │     │                          │
└─────────────────────┘     │  Action Server:          │
                            │   follow_joint_traj      │
┌─────────────────────┐     │  Publish:                │
│  ik_solver_node      │     │   /joint_states         │
│                     │     │  Controller:             │
│  Service Server:     │     │   JointTrajectory-      │
│   /ik_solver/       │     │     Controller           │
│     compute_ik      │     └──────────────────────────┘
│  Solver:             │
│   TRAC-IK           │
└─────────────────────┘
```

---

## 5. Pemetaan Package → Node

| Package | Node | Tipe | Bahasa |
|---|---|---|---|
| `arm_planning` | `planning_coordinator_node` | Lifecycle Node | Python |
| `arm_planning` | `ik_solver_node` | Node | Python |
| `arm_planning` | `collision_checker_node` | Node | Python |
| `arm_description` | `robot_state_publisher` | Node (bawaan ROS 2) | C++ |
| `arm_control` | `controller_manager` | Node (bawaan ros2_control) | C++ |
| `arm_control` | `gazebo_ros2_control` | Plugin Gazebo | C++ |
| `arm_bringup` | — | Launch file saja | Python |
