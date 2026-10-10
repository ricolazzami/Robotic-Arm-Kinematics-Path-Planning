# DecodeLabs Project 1: Robotic Arm Kinematics & Path Planning

Digital brain implementation for a 6-DOF UR5e robotic manipulator operating in ROS 2 simulation. The system receives arbitrary 3D target poses (position and orientation), calculates joint angles using Inverse Kinematics, generates smooth time-parameterized trajectories using quintic polynomial splines, validates paths against environmental collision objects using the Flexible Collision Library (FCL), and coordinates execution with safety mechanisms (emergency stop, workspace bounds, and velocity limits).

---

## Daftar Isi
1. [Fitur Utama](#fitur-utama)
2. [Tech Stack](#tech-stack)
3. [Arsitektur Sistem](#arsitektur-sistem)
4. [Model IPO (Input - Process - Output)](#model-ipo-input---process---output)
5. [Struktur Direktori Workspace](#struktur-direktori-workspace)
6. [Deskripsi Package & Node](#deskripsi-package--node)
7. [ROS 2 Interface Management](#ros-2-interface-management)
8. [Panduan Instalasi & Build](#panduan-instalasi--build)
   - [Linux (RoboStack / Conda)](#opsi-a-linux-robostack--native-ros-2)
   - [Windows (Rekomendasi untuk Gazebo GUI)](#opsi-b-windows-native)
9. [Cara Menjalankan Sistem](#cara-menjalankan-sistem)
   - [1. Preview Kinematika di RViz2 dengan Joint Slider](#1-preview-kinematika-di-rviz2-dengan-joint-slider)
   - [2. Menjalankan Planning Pipeline (Mock Mode)](#2-menjalankan-planning-pipeline-mock-mode)
   - [3. Menjalankan Simulasi Fisika Penuh (Gazebo)](#3-menjalankan-simulasi-fisika-penuh-gazebo)
10. [Pengujian Standalone (End-to-End Test)](#pengujian-standalone-end-to-end-test)
11. [Mekanisme Keselamatan (Safety Overrides)](#mekanisme-keselamatan-safety-overrides)
12. [Troubleshooting & Known Issues](#troubleshooting--known-issues)
13. [Author & Kontributor](#author--kontributor)

---

## Fitur Utama

- **Model Robot UR5e 6-DOF Lengkap:** Integrasi URDF/xacro dengan konfigurasi batas sudut sendi, transmisi hardware interface, dan geometri lingkungan workspace (meja kerja, kubus, silinder).
- **Inverse Kinematics Solver:** Validasi jangkauan bola kerja (workspace boundary $R \le 850\text{ mm}$, $|Z| \le 1200\text{ mm}$), normalisasi quaternion, dan kepatuhan limit sudut joint.
- **Trajectory Generator Berkelanjutan:** Interpolasi polinomial derajat 5 (Quintic Polynomial Spline $s(\alpha) = 10\alpha^3 - 15\alpha^4 + 6\alpha^5$) memastikan kontinuitas posisi, kecepatan, dan akselerasi dengan limit kecepatan sendi $\le \pi\text{ rad/s}$.
- **Collision Checking Berbasis FCL:** Integrasi Flexible Collision Library untuk deteksi tabrakan per-waypoint terhadap objek lingkungan dengan dukungan publikasi Marker visual ke RViz2.
- **Planning Coordinator State Machine:** Orkestrasi alur end-to-end (IDLE → PLANNING → EXECUTING → IDLE/ERROR) dengan interdependent emergency stop trajectory locking.

---

## Tech Stack

| Komponen | Teknologi | Keterangan |
|---|---|---|
| **Middleware** | ROS 2 Humble Hawksbill | Komunikasi terdistribusi (Topics, Services, Actions, Lifecycle) |
| **Manipulator Platform** | Universal Robots UR5e | 6-DOF industrial robot manipulator |
| **Deskripsi Fisik** | URDF, Xacro | Kinematic chains, mesh visual/collision, transmission tags |
| **Visualisasi Kinematik** | RViz2 | Kinematic preview, joint slider GUI, collision markers |
| **Simulasi Dinamika** | Gazebo Classic 11.x | Simulasi gravitasi, kontak rigid body, inersia, kontrol PID |
| **Kontrol Hardware** | `ros2_control` & `gazebo_ros2_control` | `JointTrajectoryController`, `JointStateBroadcaster` |
| **Collision Engine** | Flexible Collision Library (FCL / `python-fcl`) | Broadphase & narrowphase collision detection |
| **Bahasa Pemrograman** | Python 3.10+, C++17 | Node planning (`rclpy`), custom plugins |

---

## Arsitektur Sistem

Sistem mematuhi prinsip decoupling antara **Planning Layer** (frekuensi rendah/on-demand, kalkulasi berat) dan **Control Layer** (frekuensi tinggi $\ge 100\text{ Hz}$, real-time execution).

```
┌─────────────────────────────────────────────────────────────┐
│                      USER / TASK NODE                       │
│              Publish PoseStamped ke /planning/target_pose   │
└──────────────────────────┬──────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────┐
│                 PLANNING LAYER (arm_planning)               │
│  ┌──────────┐  ┌──────────────┐  ┌───────────────────────┐  │
│  │ TF2      │  │ IK Solver    │  │ Trajectory Generator  │  │
│  │ Buffer   │→ │ (TRAC-IK)    │→ │ (Quintic Spline)      │  │
│  └──────────┘  └──────────────┘  └───────────┬───────────┘  │
│                                              │              │
│                                              ▼              │
│                                  ┌───────────────────────┐  │
│                                  │ FCL Collision Checker │  │
│                                  │ (Scene & Waypoints)   │  │
│                                  └───────────┬───────────┘  │
│                                              │              │
│  ┌───────────────────────────────────────────┴───────────┐  │
│  │ Planning Coordinator (State Machine & Safety Locks)   │  │
│  └───────────────────────────────────────────┬───────────┘  │
└──────────────────────────────────────────────┼──────────────┘
                           │ FollowJointTrajectory (Action)
                           ▼
┌─────────────────────────────────────────────────────────────┐
│                CONTROL LAYER (gazebo_ros2_control)           │
│         JointTrajectoryController → PID Gains (100 Hz)      │
└──────────────────────────┬──────────────────────────────────┘
                           │ Joint Torques / Effort Commands
                           ▼
┌─────────────────────────────────────────────────────────────┐
│                    GAZEBO SIMULATION                         │
│             Fisika Rigid Body, Gravitasi, Kontak            │
└─────────────────────────────────────────────────────────────┘
```

---

## Model IPO (Input - Process - Output)

### 1. Input Stage
- **Pesan:** `geometry_msgs/msg/PoseStamped` pada topik `/planning/target_pose`
- **Isi:** Posisi 3D $(x, y, z)$ dalam meter dan orientasi rotasi Quaternion $(q_x, q_y, q_z, q_w)$
- **Validasi:** Normalisasi unit quaternion $\sqrt{q_x^2+q_y^2+q_z^2+q_w^2} = 1 \pm 0.001$, batas jangkauan silinder radius $\le 0.85\text{ m}$ dan ketinggian $|z| \le 1.2\text{ m}$.

### 2. Process Stage
1. **TF2 Transform:** Transformasi frame koordinat target ke frame referensi robot (`base_link`).
2. **Inverse Kinematics (IK):** Menghitung konfigurasi sendi target $[\theta_1, \theta_2, \theta_3, \theta_4, \theta_5, \theta_6]$ yang memenuhi batasan fisik joint limits.
3. **Trajectory Generation:** Interpolasi quintic polynomial dari konfigurasi sendi awal ke goal dengan minimal 10 waypoint dan validasi kecepatan $\le \pi\text{ rad/s}$.
4. **Collision Checking:** Setiap waypoint diuji terhadap objek scene (meja, rintangan balok, silinder) menggunakan collision shape FCL.

### 3. Output Stage
- **Visualisasi RViz:** Topik `/planning/trajectory_preview` (`JointTrajectory`) dan `/collision_scene_markers` (`MarkerArray`).
- **Eksekusi:** Action goal `control_msgs/action/FollowJointTrajectory` dikirim ke controller robot.

---

## Struktur Direktori Workspace

```
Robotic-Arm-Kinematics-Path-Planning/
├── src/
│   ├── arm_description/                  # Definisi URDF, Xacro, visualisasi RViz
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── launch/
│   │   │   └── display.launch.py         # Launch RViz + joint slider GUI
│   │   ├── rviz/
│   │   │   └── view_robot.rviz           # Konfigurasi display RViz
│   │   └── urdf/
│   │       ├── robot.urdf.xacro          # Robot workspace root assembly
│   │       ├── ur5e.urdf.xacro           # Model UR5e + Gazebo ros2_control plugin
│   │       └── workspace_objects.xacro   # Geometri meja, kubus, dan silinder
│   │
│   ├── arm_control/                      # Konfigurasi controller & ros2_control
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── config/
│   │   │   ├── controllers.yaml          # Konfigurasi JointTrajectoryController
│   │   │   └── pid_gains.yaml            # PID gains per joint
│   │   └── launch/
│   │       └── gazebo.launch.py          # Spawn robot ke Gazebo & load controllers
│   │
│   ├── arm_planning/                     # Otak algoritma kinematika & trajectory
│   │   ├── setup.py / setup.cfg
│   │   ├── package.xml
│   │   ├── arm_planning/
│   │   │   ├── ik_solver_node.py         # Solusi Inverse Kinematics & validasi limit
│   │   │   ├── trajectory_planner_node.py# Quintic polynomial spline generator
│   │   │   ├── collision_checker_node.py # FCL collision checking + marker publisher
│   │   │   └── planning_coordinator_node.py # State machine & action client
│   │   ├── config/
│   │   │   ├── kinematics.yaml           # Konfigurasi solver
│   │   │   └── collision_objects.yaml    # Definisi posisi & dimensi rintangan
│   │   └── launch/
│   │       └── planning.launch.py        # Launch seluruh node planning layer
│   │
│   └── arm_bringup/                      # Entry point launch file terintegrasi
│       ├── CMakeLists.txt
│       ├── package.xml
│       └── launch/
│           ├── simulation.launch.py      # Launch terpadu (Robot + RViz + Planning)
│           └── gazebo.launch.py          # Full physics simulation launch
│
├── test_pipeline_e2e.py                  # Standalone integration test suite
├── PRD.md                                # Product Requirements Document
├── architecture.md                       # Spesifikasi arsitektur & data flow
├── instructions.md                       # Coding standard & safety guidelines
├── TODO.md                               # Roadmap & tracking progress fase
└── .gitignore                            # Perlindungan secret & cache build
```

---

## Deskripsi Package & Node

### 1. `arm_description`
Mendefinisikan tampilan fisik dan kinematika robot:
- `robot.urdf.xacro`: Menggabungkan robot UR5e dengan objek lingkungan.
- `workspace_objects.xacro`: Meja ($1.0\text{ m} \times 0.6\text{ m} \times 0.05\text{ m}$), Kubus 1 ($0.05\text{ m}$ cube), Silinder 1 ($r=0.025\text{ m}, h=0.1\text{ m}$).
- `display.launch.py`: Membuka model di RViz dengan `joint_state_publisher_gui` untuk pengujian slider sendi secara interaktif.

### 2. `arm_control`
Menjembatani simulasi fisika Gazebo dan ROS 2 controller:
- `controllers.yaml`: Konfigurasi `joint_state_broadcaster` dan `arm_controller` tipe `trajectory_controller/JointTrajectoryController`.
- `pid_gains.yaml`: Pengaturan gain proporsional, integral, derivatif per joint untuk menahan gravitasi.

### 3. `arm_planning`
Implementasi logika perencanaan lintasan:
- `ik_solver_node.py`: Menerima pose SE(3), memvalidasi batas jangkauan dan orientasi, menghasilkan konfigurasi 6 sudut sendi.
- `trajectory_planner_node.py`: Menginterpolasi jalur menggunakan quintic polynomial spline ($s(\alpha) = 10\alpha^3 - 15\alpha^4 + 6\alpha^5$), memastikan kecepatan $\le \pi\text{ rad/s}$.
- `collision_checker_node.py`: Memuat objek dari `collision_objects.yaml`, mendeteksi interpenetrasi menggunakan FCL, dan mem-publish `MarkerArray` rintangan ke RViz2.
- `planning_coordinator_node.py`: Mengorkestrasi urutan eksekusi, memonitor status darurat `/emergency_stop`, dan mengirim goal aksi.

### 4. `arm_bringup`
Menyediakan launch file tunggal untuk menyalakan seluruh sub-sistem secara serempak.

---

## ROS 2 Interface Management

### Topics
| Topik | Tipe Pesan | Penerbit | Pelanggan | Fungsi |
|---|---|---|---|---|
| `/joint_states` | `sensor_msgs/msg/JointState` | `joint_state_publisher` / Gazebo | Planning nodes, RViz | Status sudut sendi aktual |
| `/planning/target_pose` | `geometry_msgs/msg/PoseStamped` | User / Task node | Planning Coordinator | Target posisi & orientasi end-effector |
| `/planning/trajectory_preview`| `trajectory_msgs/msg/JointTrajectory` | Trajectory Planner | RViz2, Collision Checker | Preview lintasan sebelum dieksekusi |
| `/collision_scene_markers` | `visualization_msgs/msg/MarkerArray` | Collision Checker | RViz2 | Visualisasi 3D rintangan di RViz |
| `/emergency_stop` | `std_msgs/msg/Bool` | Safety system / Operator | Planning Coordinator | Pemicu penguncian gerak darurat |

### Actions
| Action Server | Tipe | Server | Client |
|---|---|---|---|
| `/arm_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | `gazebo_ros2_control` | Planning Coordinator |

---

## Panduan Instalasi & Build

### Opsi A: Linux (RoboStack / Native ROS 2)

1. Pastikan environment ROS 2 Humble aktif:
```bash
# Aktifkan conda/robostack environment
conda activate ros2_env
# Atau native Ubuntu source:
# source /opt/ros/humble/setup.bash
```

2. Clone repository:
```bash
git clone https://github.com/ricolazzami/Robotic-Arm-Kinematics-Path-Planning.git
cd Robotic-Arm-Kinematics-Path-Planning
```

3. Install dependensi Python:
```bash
pip install python-fcl scipy numpy pyyaml
```

4. Build workspace:
```bash
colcon build --symlink-install
```

5. Source workspace:
```bash
source install/setup.bash
```

---

### Opsi B: Windows (Native)

Windows direkomendasikan untuk menjalankan simulasi Gazebo dengan antarmuka grafis 3D (GUI) tanpa isu driver rendering headless.

1. Buka **ROS 2 Command Prompt** (Run as Administrator jika diperlukan).
2. Clone repository:
```cmd
git clone https://github.com/ricolazzami/Robotic-Arm-Kinematics-Path-Planning.git
cd Robotic-Arm-Kinematics-Path-Planning
```
3. Install dependensi yang dibutuhkan:
```cmd
pip install python-fcl scipy numpy pyyaml
```
4. Build workspace:
```cmd
colcon build --symlink-install
```
5. Source environment workspace:
```cmd
call install\setup.bat
```

---

## Cara Menjalankan Sistem

### 1. Preview Kinematika di RViz2 dengan Joint Slider
Perintah ini membuka model UR5e lengkap dengan meja dan rintangan di RViz2, disertai slider GUI untuk menguji pergerakan masing-masing sendi:
```bash
ros2 launch arm_description display.launch.py
```

### 2. Menjalankan Planning Pipeline (Mock Mode)
Menjalankan sistem koordinasi perencanaan lintasan (Robot State Publisher + RViz2 + Planning Nodes) tanpa beban komputasi simulasi fisika Gazebo:
```bash
ros2 launch arm_bringup simulation.launch.py
```

Kirim target pose uji coba melalui terminal lain:
```bash
ros2 topic pub --once /planning/target_pose geometry_msgs/msg/PoseStamped "{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'},
  pose: {
    position: {x: 0.4, y: 0.2, z: 0.4},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```

### 3. Menjalankan Simulasi Fisika Penuh (Gazebo)
Menjalankan Gazebo physics simulation lengkap dengan gravitasi, ODE engine, dan controller manager:
```bash
ros2 launch arm_bringup gazebo.launch.py
```

---

## Pengujian Standalone (End-to-End Test)

Project ini dilengkapi dengan test script standalone yang menguji validitas matematis seluruh pipeline tanpa memerlukan display GUI atau daemon ROS:

```bash
python3 test_pipeline_e2e.py
```

**Hasil uji yang diharapkan:**
```text
=== TEST PIPELINE KINEMATICS & PLANNING ===
[1] Target Pose: x=0.4, y=0.2, z=0.4, q=(0.0,0.0,0.0,1.0)
 -> Pose input valid & within workspace boundary.
 -> IK Solution within valid joint limits.
 -> Quintic Spline generated: 10 waypoints over 1.00s, all velocities <= 3.14 rad/s.
 -> Loaded 3 environmental obstacle collision objects.
 -> Collision check PASSED: 0 collisions detected across all trajectory waypoints.

✅ PIPELINE INTEGRATION TEST: SUCCESSFUL
```

---

## Mekanisme Keselamatan (Safety Overrides)

1. **Emergency Stop (Interdependent Locking):**
   - Ketika topik `/emergency_stop` menerima nilai `True`, Planning Coordinator langsung membatalkan goal aktif di Action Server dan mengunci status sistem ke `LOCKED`.
   - Semua target pose baru akan ditolak seketika.
   - Sistem baru dapat menerima perintah kembali setelah sinyal `/emergency_stop` bernilai `False` dan service pemulihan dipanggil.
2. **Joint Velocity Limiting:**
   - Kecepatan sudut sendi dibatasi ketat maksimum $\pi\text{ rad/s}$ ($180^\circ/\text{s}$). Lintasan yang melebihi batas ini akan ditolak sebelum dikirim ke kontroler motor.
3. **Workspace Boundary Rejection:**
   - Sistem menolak target di luar jangkauan fisik lengan (silinder radius $> 0.85\text{ m}$ atau ketinggian vertikal $> 1.2\text{ m}$).

---

## Troubleshooting & Known Issues

- **Gazebo Classic OGRE crash di Linux headless:**
  - *Penyebab:* Pada beberapa environment Linux / Conda, binary plugin `OGRE` Gazebo Classic memiliki path hardcoded yang memerlukan konteks X11/GLX window.
  - *Solusi:* Jalankan planning layer menggunakan `simulation.launch.py` (Mock mode) atau jalankan simulasi Gazebo di sistem Windows / Linux dengan GUI native display aktif.
- **`joint_state_publisher_gui` tidak ditemukan:**
  - Pastikan paket terpasang:
    ```bash
    sudo apt install ros-humble-joint-state-publisher-gui
    # atau jika menggunakan micromamba / conda:
    micromamba install -p <prefix> -c robostack-staging ros-humble-joint-state-publisher-gui
    ```

---

## Author & Kontributor

- **Author:** Rico Lazzami ([@ricolazzami](https://github.com/ricolazzami))
- **Organisasi / Project:** DecodeLabs Internship Project 1: Robotic Arm Kinematics & Path Planning
- **Lisensi:** Apache-2.0
