# Arsitektur Sistem (RViz-Only State)
## DecodeLabs Project 1: Robotic Arm Kinematics & Path Planning

---

## 1. Diagram Arsitektur Keseluruhan

![Diagram Arsitektur Keseluruhan](docs/architecture.png)

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
│  │  TF2 Buffer │───▶│  TRAC-IK /   │───▶│  Trajectory Generator   │  │
│  │  & Lookup   │    │  6-DOF IK    │    │  (Quintic 60wp 3s)       │  │
│  │             │    │  Fallback    │    │                          │  │
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
│  │  - hold_timer + mock executor                                   │  │
│  │  - Kirim goal ke Action Server / RViz display                   │  │
│  └─────────────────────────────┬───────────────────────────────────┘  │
│                                │                                      │
└────────────────────────────────┼──────────────────────────────────────┘
                                 │
                   FollowJointTrajectory (Action)
              /arm_controller/follow_joint_trajectory
                                 │
                                 ▼
┌───────────────────────────────────────────────────────────────────────┐
│                        RVIZ & FAKE SYSTEM                             │
│                                                                       │
│  ┌─────────────────────────────────────────────────────────────────┐  │
│  │                  ros2_control FakeSystem Hardware               │  │
│  │                                                                 │  │
│  │  ┌──────────────────────────────────────────────────────────┐   │  │
│  │  │         JointTrajectoryController                        │   │  │
│  │  │  - Simulasi posisi tanpa Gazebo/Physics                  │   │  │
│  │  │  - Publish /joint_states                                 │   │  │
│  │  └──────────────────────────────────────────────────────────┘   │  │
│  └─────────────────────────────────────────────────────────────────┘  │
└───────────────────────────────────────────────────────────────────────>
```

---

## 2. TF2 Transform Tree

```
world
  └── table_link                   (Static transform from world)
        ├── l0                     (Mounted on table at offset -0.35, 0, 0.05)
        │     └── base_link        (Robot base)
        │           ├── shoulder_link
        │           │     └── upper_arm_link
        │           │           └── forearm_link
        │           │                 └── wrist_1_link
        │           │                       └── wrist_2_link
        │           │                             └── wrist_3_link
        │           │                                   └── tool0
        │           └── base_link_inertia
        ├── cube_1_link            (On table at y ±0.15 middle, x=0.35)
        └── cylinder_1_link        (On table at y ±0.15 middle, x=0.35)
```

**Konvensi:**
- Transform `world` → `table_link`: static.
- Transform `table_link` → `l0` → `base_link`: static.
- Objects parent `table_link` at `y ±0.15` middle, `collision_objects.yaml` poses `x: 0.35` tables offset.

---

## 3. Node Graph & Eksekusi Demo

- **Coordinator:** `hold_timer` + mock executor aktif.
- **Demo:** `cartesian_abc_demo.py` A(0.40,0,0.35) ↔ B(0.35,0.25,0.30) stopped now manual.
- **Sim running:** `cae28c116ffd`.
