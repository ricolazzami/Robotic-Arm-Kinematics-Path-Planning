# TODO — Roadmap Checklist
## DecodeLabs Project 1: Robotic Arm Kinematics & Path Planning (RViz-Only State)

---

## Fase 0: Setup Workspace ROS 2 & RViz FakeSystem
> Fondasi infrastruktur ROS 2 Humble dengan FakeSystem (Gazebo dihapus).

- [x] Install ROS 2 Humble via RoboStack
- [x] Install dependencies (`ur_description`, `ros2_control`, `rviz2`, `python3-fcl`)
- [x] Workspace colcon build 5 packages OK
- [x] Konfigurasi UR5e `use_fake_hardware="true"`

---

## Fase 1: URDF & TF Chain (`world` → `table_link` → `l0` → `base_link`)
> Definisikan struktur hirarki dan objek workspace.

- [x] Chain hirarki: `world` parent `table_link`, `table_link` parent `l0`, `l0` parent `base_link`
- [x] Workspace objects (meja, cube_1, cylinder_1) parent `table_link` at `y ±0.15` middle
- [x] RViz display launch & visualization

---

## Fase 2: RViz Simulation & FakeSystem Controller
> Eksekusi kendali posisi via FakeSystem hardware interface di RViz.

- [x] Controller manager & joint state broadcaster active (FakeSystem)
- [x] `/joint_states` publisher aktif

---

## Fase 3: IK Solver (TRAC-IK + 6-DOF Analytical Fallback)
> Dari pose SE(3) ke sudut sendi dengan fallback analitik 2-link.

- [x] `ik_solver_node.py` dengan TRAC-IK wrapper dan 6-DOF fallback 2-link
- [x] Validasi joint limits

---

## Fase 4: Trajectory Generation (Quintic 60wp 3s)
> Time-parameterized quintic polynomial trajectory generation.

- [x] `trajectory_planner_node.py` dengan quintic polynomial (60 waypoints, 3s duration)

---

## Fase 5: Collision Checking (FCL) & YAML Pose x0.35
> Validasi FCL dengan `collision_objects.yaml` pose x: 0.35 tables offset.

- [x] `collision_checker_node.py`
- [x] `collision_objects.yaml` objects offset x: 0.35

---

## Fase 6: Integration Pipeline, Coordinator & Manual Demo
> Coordinator hold_timer + mock executor, manual topic demo (`cartesian_abc_demo.py` A↔B stopped manual).

- [x] `planning_coordinator_node.py` (`hold_timer` + mock executor)
- [x] Demo cartesian A(0.40,0,0.35) ↔ B(0.35,0.25,0.30) manual topic mode (sim running cae28c116ffd)

---

## Fase 7: Polish & Validation
> Selesai.

- [x] Pipeline end-to-end terverifikasi di RViz

---

## Catatan Progress

| Fase | Status | Tanggal Mulai | Tanggal Selesai | Catatan |
|---|---|---|---|---|
| 0 | ✅ Selesai | - | 2026-10-11 | ROS 2 Humble + FakeSystem |
| 1 | ✅ Selesai | - | 2026-10-11 | TF Chain world→table_link→l0→base_link |
| 2 | ✅ Selesai | - | 2026-10-11 | RViz FakeSystem control loop |
| 3 | ✅ Selesai | - | 2026-10-11 | TRAC-IK + 6-DOF fallback |
| 4 | ✅ Selesai | - | 2026-10-11 | Quintic 60wp 3s trajectory |
| 5 | ✅ Selesai | - | 2026-10-11 | FCL collision check + yaml pose x0.35 |
| 6 | ✅ Selesai | - | 2026-10-11 | Coordinator hold_timer + mock executor + manual demo A↔B |
| 7 | ✅ Selesai | - | 2026-10-11 | RViz-only validation complete (sim cae28c116ffd) |
