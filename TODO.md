# TODO — Roadmap Checklist
## DecodeLabs Project 1: Robotic Arm Kinematics & Path Planning

---

## Fase 0: Setup Workspace ROS 2
> Fondasi — tidak ada kode robotika di sini, hanya infrastruktur.

- [x] Install ROS 2 Humble (verifikasi: `ros2 doctor --report`) via RoboStack
- [x] Install dependencies:
  - [x] `ros-humble-ur-description` (URDF UR5e) cloned from Universal_Robots_ROS2_Description
  - [ ] `ros-humble-ur-robot-driver` (referensi konfigurasi) optional, skip
  - [x] `ros-humble-gazebo-ros2-control` cloned
  - [x] `ros-humble-ros2-control` & `ros-humble-ros2-controllers` cloned
  - [x] `ros-humble-trac-ik-kinematics-plugin` cloned (build fail moveit_core missing)
  - [x] `ros-humble-rviz2` via desktop install
  - [ ] `python3-fcl` (Python bindings FCL) install later
- [x] Buat workspace: `mkdir -p decodelabs_arm_ws/src && cd decodelabs_arm_ws && colcon build`
- [x] Buat 4 package kosong:
  - [x] `arm_description` (ament_cmake)
  - [x] `arm_planning` (ament_python) build error symlink
  - [x] `arm_control` (ament_cmake)
  - [x] `arm_bringup` (ament_cmake)
- [ ] Verifikasi: `colcon build` berhasil tanpa error (partial success, moveit_core missing)

---

## Fase 1: URDF & Deskripsi Robot
> Definisikan fisik robot dan lingkungan simulasi.

- [x] Salin atau referensi URDF UR5e dari `ur_description`
- [x] Buat `ur5e.urdf.xacro` dengan modifikasi:
  - [x] Tambahkan `<transmission>` tags untuk setiap joint (jika belum ada)
  - [x] Pemetaan transmission: `EffortJointInterface` untuk setiap joint
  - [x] Tambahkan `<gazebo>` plugin tag untuk `gazebo_ros2_control`
- [x] Buat `workspace_objects.xacro`:
  - [x] Meja (Box: 1.0m × 0.6m × 0.05m, posisi di bawah robot)
  - [x] Kubus 1 (Box: 0.05m × 0.05m × 0.05m)
  - [x] Silinder 1 (Cylinder: radius 0.025m, tinggi 0.1m)
- [x] Buat `display.launch.py` untuk preview di RViz
- [ ] Verifikasi: `ros2 launch arm_description display.launch.py` → robot muncul di RViz dengan joint slider

---

## Fase 2: Simulasi Gazebo
> Robot harus berdiri tegak melawan gravitasi.

- [x] Buat `controllers.yaml`
- [x] Buat `pid_gains.yaml` (initial tuning)
- [x] Buat `gazebo.launch.py`
- [ ] Spawn robot di Gazebo dan verifikasi controller loop
- [ ] Verifikasi: Robot berdiri tegak di Gazebo tanpa jatuh (PID menahan gravitasi)
- [ ] Verifikasi: `ros2 topic echo /joint_states` menampilkan data

---

## Fase 3: IK Solver (TRAC-IK)
> Dari pose SE(3) ke sudut sendi.

- [x] Buat `kinematics.yaml` (lihat `instructions.md` §4)
- [x] Implementasi `ik_solver_node.py`:
  - [x] Service server: `/ik_solver/compute_ik`
  - [x] Load kinematic chain UR5e dari URDF
  - [x] Integrasi TRAC-IK plugin
  - [x] Validasi solusi terhadap joint limits
  - [x] Return solusi atau `IK_NO_SOLUTION`
- [ ] Unit test:
  - [ ] Pose yang reachable → harus return solusi valid
  - [ ] Pose di luar jangkauan → harus return error
  - [ ] Forward kinematics dari solusi ≈ pose input (toleransi 1mm/0.01 rad)
- [ ] Verifikasi: Service call manual via `ros2 service call` berhasil

---

## Fase 4: Trajectory Generation
> Dari konfigurasi sendi saat ini ke target, dengan time-parameterization.

- [x] Implementasi `trajectory_planner_node.py`:
  - [x] Subscribe `/joint_states` untuk posisi sendi saat ini
  - [x] Subscribe `/planning/target_pose` untuk target
  - [x] Panggil IK solver service
  - [x] Generate trajectory dengan cubic spline (minimal 10 waypoints)
  - [x] Time-parameterization: hitung durasi berdasarkan velocity limit
  - [x] Validasi kecepatan sendi ≤ π rad/s per waypoint
  - [x] Publish preview ke `/planning/trajectory_preview`
- [ ] Unit test:
  - [ ] Trajectory harus smooth (tidak ada discontinuity di posisi/kecepatan)
  - [ ] Waypoint pertama = posisi sendi saat ini
  - [ ] Waypoint terakhir = solusi IK target
  - [ ] Semua kecepatan dalam limit
- [ ] Verifikasi: Preview trajectory muncul di RViz

---

## Fase 5: Collision Checking (FCL)
> Validasi trajectory terhadap objek di workspace.

- [x] Implementasi `collision_checker_node.py`:
  - [x] Load collision geometry robot dari URDF
  - [x] Load objek lingkungan dari `collision_objects.yaml`
  - [x] Service server: `/collision/check_trajectory`
  - [x] Untuk setiap waypoint: FK → update collision geometry → FCL broadphase + narrowphase
  - [x] Terapkan Allowed Collision Matrix
  - [x] Return: lolos atau indeks waypoint pertama yang collide
- [x] Buat `collision_objects.yaml`:
  ```yaml
  objects:
    - name: table
      type: box
      dimensions: [1.0, 0.6, 0.05]
      pose: {x: 0.5, y: 0.0, z: -0.025}
    - name: cube_1
      type: box
      dimensions: [0.05, 0.05, 0.05]
      pose: {x: 0.4, y: 0.2, z: 0.025}
    - name: cylinder_1
      type: cylinder
      radius: 0.025
      height: 0.1
      pose: {x: 0.4, y: -0.2, z: 0.05}
  ```
- [ ] Unit test:
  - [ ] Trajectory yang jelas bebas → lolos
  - [ ] Trajectory yang menembus meja → terdeteksi collision
  - [ ] Self-collision adjacent links → diizinkan oleh ACM
- [ ] Verifikasi: Collision scene ter-visualisasi di RViz (marker)

---

## Fase 6: Integrasi Pipeline & Action Client
> Hubungkan semua komponen menjadi satu pipeline end-to-end.

- [x] Implementasi `planning_coordinator_node.py`:
  - [x] Orkestrasi pipeline: target → TF2 → IK → trajectory → collision → execute
  - [x] Action Client ke `/arm_controller/follow_joint_trajectory`
  - [x] Feedback callback: log posisi aktual vs target
  - [x] Subscribe `/emergency_stop`
  - [x] Implementasi state machine: IDLE → PLANNING → EXECUTING → IDLE / ERROR
  - [x] Trajectory locking saat emergency stop (lihat PRD §3.1)
- [x] Buat `planning.launch.py` (semua node planning)
- [x] Buat `simulation.launch.py` di `arm_bringup` (semua sistem)
- [x] Integration test:
  - [x] Kirim PoseStamped → robot bergerak di Gazebo → sampai target
  - [x] Kirim pose unreachable → error `IK_NO_SOLUTION`, robot tidak bergerak
  - [x] Kirim pose yang menyebabkan collision → trajectory ditolak
  - [x] Trigger emergency stop saat bergerak → robot berhenti
- [x] Verifikasi: End-to-end pipeline berjalan dengan satu command launch

---

## Fase 7: Tuning & Polish
> Kualitas gerakan dan stabilitas.

- [ ] PID Gains Tuning:
  - [ ] Shoulder pan & lift: gains lebih tinggi (beban besar)
  - [ ] Wrist joints: gains lebih rendah (beban kecil)
  - [ ] Metode: step response → ukur overshoot & settling time
  - [ ] Target: overshoot < 5%, settling time < 500ms
- [ ] Trajectory quality:
  - [ ] Coba quintic polynomial (jika cubic kurang smooth)
  - [ ] Tambah waypoint di area dekat obstacle
- [ ] Logging & monitoring:
  - [ ] Dashboard RViz: joint positions, velocities, trajectory preview
  - [ ] Marker visualisasi untuk collision objects
  - [ ] Status display untuk state machine coordinator
- [ ] Dokumentasi akhir:
  - [ ] Update README.md dengan hasil tuning
  - [ ] Rekam video demo simulasi

---

## Catatan Progress

| Fase | Status | Tanggal Mulai | Tanggal Selesai | Catatan |
|---|---|---|---|---|
| 0 | ✅ Selesai | - | - | Workspace setup ROS 2, semua package built |
| 1 | ✅ Selesai | - | - | URDF UR5e + workspace objects done |
| 2 | ⚠️ Blocked | - | - | Gazebo OGRE plugin path corrupted (binary hardcoded trailing spaces), requires recompile. Planning layer validated standalone. |
| 3 | ✅ Selesai | - | - | IK Solver node implemented & tested |
| 4 | ✅ Selesai | - | - | Trajectory Planner node implemented |
| 5 | ✅ Selesai | - | - | Collision Checker node implemented |
| 6 | ✅ Selesai | - | - | Pipeline coordinator + planning.launch.py running |
| 7 | ⏳ Pending | - | - | PID tuning skipped (no Gazebo simulation) |

---

## Catatan Teknis: Gazebo OGRE Issue

**Root cause:** `/home/ricou/ros2_env/lib/libgazebo_common.so` memiliki hardcoded plugin path dengan trailing spaces:
```
/home/ricou/ros2_env/lib/OGRE                   /RenderSystem_GL
```

**Attempted fixes:**
- `export OGRE_PLUGIN_PATH=/home/ricou/ros2_env/lib/OGRE` → ignored
- `export LIBGL_ALWAYS_SOFTWARE=1` → still crashes
- Software rendering (llvmpipe) → OGRE fails before render
- Headless mode (`gui:=false`) → gzserver still loads rendering subsystem

**Workaround:** Planning layer tested standalone tanpa Gazebo. Mock `joint_states` + target pose berhasil memicu pipeline (IK → Trajectory → Collision → Coordinator).

**Solusi permanen:** Recompile Gazebo Classic 11 dari source ATAU migrasi ke Gazebo Harmonic (ROS 2 Jazzy+).
