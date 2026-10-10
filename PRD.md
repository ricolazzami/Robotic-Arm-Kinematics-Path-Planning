# PRD — Product Requirements Document
## DecodeLabs Project 1: Robotic Arm Kinematics & Path Planning

---

## 1. Ringkasan Produk

Sistem ini adalah **otak digital** untuk manipulator UR5e 6-DOF yang berjalan sepenuhnya dalam simulasi. Sistem menerima target pose, menghitung solusi kinematika balik, memvalidasi lintasan terhadap tabrakan, dan mengeksekusi gerakan melalui simulasi fisika Gazebo.

---

## 2. Model IPO (Input → Process → Output)

### 2.1 Input Stage — Target Pose SE(3)

**Apa yang masuk ke sistem:**

| Input | Tipe Data | Deskripsi |
|---|---|---|
| Posisi target (x, y, z) | `float64[3]` | Koordinat translasi dalam meter, relatif terhadap `base_link` |
| Orientasi target (qx, qy, qz, qw) | `float64[4]` | Quaternion rotasi end-effector |
| Frame referensi | `string` | Frame ID tempat target didefinisikan (default: `base_link`) |
| Constraint (opsional) | `moveit_msgs/msg/Constraints` | Batasan orientasi atau posisi selama gerakan |

**Format pesan ROS 2:** `geometry_msgs/msg/PoseStamped`

**Validasi input:**
- Posisi harus berada dalam workspace boundary UR5e (radius jangkauan ~850mm dari base).
- Quaternion harus ternormalisasi: \$\sqrt{qx^2 + qy^2 + qz^2 + qw^2} = 1 \pm 0.001\$
- `header.stamp` harus terisi (tidak boleh zero-time).
- `header.frame_id` harus terdaftar di TF tree.

---

### 2.2 Process Stage — IK Solver & Collision Checking

Proses terbagi menjadi **4 tahap sekuensial**. Jika satu tahap gagal, seluruh pipeline berhenti dan mengembalikan error ke caller.

#### Tahap 1: Transform Lookup (TF2)

```
Input pose (frame arbitrary) → lookup_transform → Pose dalam frame `base_link`
```

- Gunakan TF2 buffer dengan temporal validation (lihat `instructions.md` §3).
- Jika transform tidak tersedia atau kedaluwarsa (>1 detik) → **TOLAK**, kembalikan `TRANSFORM_ERROR`.

#### Tahap 2: Inverse Kinematics (TRAC-IK)

```
Pose SE(3) dalam base_link → TRAC-IK → Konfigurasi joint [θ1, θ2, θ3, θ4, θ5, θ6]
```

- Solver: **TRAC-IK** (wajib, KDL dilarang).
- Timeout per request: 50ms.
- Maks percobaan: 3.
- Jika tidak ada solusi → **TOLAK**, kembalikan `IK_NO_SOLUTION`.
- Validasi solusi terhadap joint limits UR5e.

#### Tahap 3: Trajectory Generation

```
Joint config awal + Joint config target → Interpolasi → Time-parameterized trajectory
```

- Interpolasi menggunakan **cubic spline** atau **quintic polynomial** untuk memastikan kontinuitas posisi, kecepatan, dan akselerasi.
- Jumlah waypoint minimum: 10 titik per trajectory.
- Time-parameterization: setiap waypoint memiliki timestamp relatif, dengan kecepatan sendi tidak melebihi limit UR5e.
- Output: `trajectory_msgs/msg/JointTrajectory` yang berisi array `JointTrajectoryPoint` (positions, velocities, accelerations, time_from_start).

#### Tahap 4: Collision Checking (FCL)

```
Trajectory waypoints → FCL collision scene → Validasi per-waypoint
```

- Collision scene berisi:
  - **Self-collision geometry** robot (dari URDF collision tags).
  - **Objek lingkungan**: meja (Box), kubus (Box), silinder (Cylinder).
- Setiap waypoint di-forward-kinematics-kan, lalu dicek tabrakannya terhadap scene.
- Jika ada **satu waypoint saja** yang bertabrakan → **TOLAK seluruh trajectory**, kembalikan `COLLISION_DETECTED` beserta indeks waypoint yang bermasalah.
- Allowed Collision Matrix (ACM) diterapkan sesuai `instructions.md` §5.

---

### 2.3 Output Stage — Eksekusi ke Motor

**Apa yang keluar dari sistem:**

| Output | Tipe Data | Tujuan |
|---|---|---|
| Trajectory command | `control_msgs/action/FollowJointTrajectory` (Goal) | Dikirim ke `gazebo_ros2_control` JointTrajectoryController |
| Feedback real-time | `FollowJointTrajectory.Feedback` | Posisi sendi aktual vs. target selama eksekusi |
| Result | `FollowJointTrajectory.Result` | Status akhir: `SUCCESSFUL`, `GOAL_TOLERANCE_VIOLATED`, dll. |
| Preview visualisasi | `trajectory_msgs/msg/JointTrajectory` | Dikirim ke RViz untuk preview sebelum eksekusi |

**Alur eksekusi:**
1. Planning Node mengirim goal ke Action Server via `FollowJointTrajectory`.
2. `JointTrajectoryController` menginterpolasi trajectory dan mengirim perintah PID ke setiap joint Gazebo pada rate ≥100 Hz.
3. PID controller di Gazebo menerapkan torsi untuk mengatasi gravitasi dan inersia simulasi.
4. Feedback dikirim kembali ke Planning Node secara periodik.
5. Setelah trajectory selesai, result dikembalikan.

---

## 3. Persyaratan Keselamatan (Safety Overrides)

### 3.1 Emergency Stop — Interdependent Trajectory Locking

Mekanisme penguncian lintasan darurat yang **wajib** diimplementasikan:

```
┌──────────────────────────────────────────────────────┐
│              EMERGENCY STOP PROTOCOL                  │
│                                                      │
│  Trigger: /emergency_stop = True                     │
│                                                      │
│  1. Planning Node:                                   │
│     ├─ Cancel goal aktif di Action Server            │
│     ├─ LOCK: Tolak semua target pose baru            │
│     └─ Log FATAL                                     │
│                                                      │
│  2. Controller:                                      │
│     ├─ Hentikan interpolasi trajectory               │
│     └─ Tahan posisi sendi saat ini (hold position)   │
│                                                      │
│  3. Unlock:                                          │
│     ├─ /emergency_stop = False                       │
│     ├─ Operator harus kirim /planning/reset (Service)│
│     └─ Sistem kembali ke state IDLE                  │
└──────────────────────────────────────────────────────┘
```

**Aturan locking:**
- Saat emergency stop aktif, **semua node planning terkunci secara interdependen** — tidak ada satu pun yang boleh menerima atau memproses target baru.
- Unlock membutuhkan **dua kondisi**: e-stop dinonaktifkan DAN service `/planning/reset` dipanggil. Ini mencegah sistem langsung bergerak saat e-stop dilepas secara tidak sengaja.

### 3.2 Joint Velocity Limit

- Kecepatan sendi maksimal: **π rad/s** (180°/s) pada mode simulasi.
- Validasi dilakukan di Planning Layer saat trajectory generation.
- Jika ada waypoint yang melanggar → trajectory ditolak, bukan di-clamp.

### 3.3 Workspace Boundary

- Bounding box workspace didefinisikan dalam konfigurasi (YAML).
- Target pose di luar boundary → ditolak dengan `OUT_OF_WORKSPACE`.
- Default boundary: silinder dengan radius 850mm dan tinggi 1200mm dari base.

### 3.4 Collision Override

- **Tidak ada mode bypass collision.** Collision check selalu aktif.
- Jika collision scene belum ter-load → sistem menolak semua trajectory (fail-safe).

---

## 4. Metrik Keberhasilan

| Metrik | Target | Cara Ukur |
|---|---|---|
| IK Success Rate | ≥ 95% untuk pose dalam workspace | Counter sukses/gagal di IK node |
| Trajectory Execution Accuracy | Posisi akhir ≤ 1mm dari target | Bandingkan `/joint_states` akhir vs. FK dari solusi IK |
| End-to-End Latency | ≤ 500ms dari terima pose sampai mulai eksekusi | Timestamp di Planning Node |
| Collision False Negative | 0% | Tidak ada eksekusi yang menyebabkan collision di Gazebo tanpa terdeteksi |
| Emergency Stop Response | ≤ 100ms dari trigger sampai motion berhenti | Timestamp di log |

---

## 5. Batasan & Asumsi

- Sistem berjalan **sepenuhnya dalam simulasi** — tidak ada koneksi ke hardware robot fisik.
- Gazebo Classic 11.x digunakan (bukan Gazebo Fortress/Garden).
- Gripper/end-effector tidak dimodelkan di fase ini — fokus pada pergerakan sendi.
- Satu robot, satu workspace, tanpa multi-robot coordination.
