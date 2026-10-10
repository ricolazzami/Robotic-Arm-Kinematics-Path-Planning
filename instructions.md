# Panduan Coding — DecodeLabs Robotic Arm Workspace

> Dokumen ini adalah **system prompt** untuk seluruh pengembangan di workspace ini.
> Setiap kontributor (manusia maupun AI) **WAJIB** mematuhi aturan di bawah.

---

## 1. Prinsip Arsitektur: Decoupling Planning vs Control

Sistem terbagi menjadi **dua lapisan yang tidak boleh digabung**:

| Lapisan | Tanggung Jawab | Frekuensi | Bahasa |
|---|---|---|---|
| **Planning Layer** | Terima target pose SE(3), panggil IK solver, hasilkan trajectory | ~10 Hz (on-demand) | Python / C++ |
| **Control Layer** | Eksekusi trajectory ke motor (PID loop via `gazebo_ros2_control`) | ≥ 100 Hz (real-time) | C++ (plugin) |

### Aturan Decoupling

- Planning Layer **TIDAK BOLEH** menulis langsung ke topik `/joint_commands` atau sejenisnya.
- Planning Layer **HANYA** berkomunikasi ke Control Layer melalui **Action Server** `control_msgs/action/FollowJointTrajectory`.
- Control Layer **TIDAK BOLEH** menjalankan logika IK atau collision checking.
- Semua pertukaran data antar-lapisan harus melalui interface ROS 2 yang terdefinisi (Topics, Services, Actions).

---

## 2. Manajemen Interface ROS 2

### 2.1 Topics

| Topik | Tipe Pesan | Publisher | Subscriber | Catatan |
|---|---|---|---|---|
| `/joint_states` | `sensor_msgs/msg/JointState` | `gazebo_ros2_control` | Planning Node, RViz | Read-only bagi Planning |
| `/planning/target_pose` | `geometry_msgs/msg/PoseStamped` | User/Task Node | Planning Node | Target dalam frame `base_link` |
| `/planning/trajectory_preview` | `trajectory_msgs/msg/JointTrajectory` | Planning Node | RViz | Hanya untuk visualisasi |

**Aturan:**
- Semua pesan `PoseStamped` **WAJIB** mengisi field `header.frame_id` dan `header.stamp`.
- Dilarang menggunakan `rospy` (ROS 1). Gunakan `rclpy` atau `rclcpp` saja.
- QoS profile untuk `/joint_states`: `SensorDataQoS` (best-effort, volatile).

### 2.2 Services

| Service | Tipe | Server | Kegunaan |
|---|---|---|---|
| `/ik_solver/compute_ik` | `moveit_msgs/srv/GetPositionIK` | IK Solver Node | Hitung solusi IK untuk satu pose |
| `/collision/check_trajectory` | Custom srv | Collision Node | Validasi trajectory terhadap collision scene |

**Aturan:**
- Service call **WAJIB** memiliki timeout maksimal **2 detik**. Jika timeout tercapai, node harus log `WARN` dan mengembalikan status gagal.
- Jangan gunakan service untuk streaming data — gunakan topic.

### 2.3 Action Servers

| Action | Tipe | Server | Client |
|---|---|---|---|
| `/arm_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | `gazebo_ros2_control` (JointTrajectoryController) | Planning Node |

**Aturan:**
- Planning Node bertindak sebagai **Action Client**.
- Sebelum mengirim goal baru, **WAJIB** cek apakah goal sebelumnya sudah selesai atau di-cancel. Dilarang mengirim goal bertumpuk tanpa penanganan eksplisit.
- Implementasikan **feedback callback** untuk memonitor progres eksekusi (posisi sendi aktual vs. target).
- Jika `result_code != SUCCESSFUL`, log level `ERROR` dan publish status ke `/planning/status`.

---

## 3. Penanganan TF2 dengan Temporal Buffer

### Konfigurasi Buffer

```python
# Konfigurasi standar — JANGAN ubah tanpa review
BUFFER_DURATION = rclpy.duration.Duration(seconds=10.0)  # 10 detik history
TRANSFORM_TIMEOUT = rclpy.duration.Duration(seconds=0.5)  # Timeout lookup
```

### Aturan TF2

1. **Selalu gunakan `lookup_transform()` dengan parameter `timeout`.**
   Jangan pernah memanggil `lookup_transform()` tanpa timeout — ini bisa menyebabkan blocking tak terbatas.

2. **Validasi temporal:** Sebelum menggunakan transform result, cek umur data:
   ```python
   transform = tf_buffer.lookup_transform('base_link', 'tool0', rclpy.time.Time(), timeout=TRANSFORM_TIMEOUT)
   
   age = node.get_clock().now() - rclpy.time.Time.from_msg(transform.header.stamp)
   if age > rclpy.duration.Duration(seconds=1.0):
       node.get_logger().warn(f'Transform data kedaluwarsa: {age.nanoseconds / 1e9:.2f}s')
       # JANGAN gunakan transform ini untuk IK — bisa berbahaya
       return None
   ```

3. **Frame convention:**
   - Frame dasar robot: `base_link`
   - Frame end-effector: `tool0`
   - Frame dunia: `world`
   - Semua frame custom WAJIB menggunakan prefix namespace, contoh: `decodelabs/target_frame`

4. **Dilarang** mem-broadcast static transform dari kode node. Gunakan file URDF atau `static_transform_publisher` dari launch file.

---

## 4. Solver IK: TRAC-IK (Wajib)

### Aturan Utama

- **DILARANG** menggunakan KDL default solver. KDL memiliki success rate rendah (~60%) dibanding TRAC-IK (~99%).
- Konfigurasi solver di `kinematics.yaml`:

```yaml
# kinematics.yaml — konfigurasi solver untuk UR5e
manipulator:
  kinematics_solver: trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.05  # 50ms maks per request
  kinematics_solver_attempts: 3
```

- Jika TRAC-IK gagal setelah `kinematics_solver_attempts` percobaan, node **WAJIB**:
  1. Log `ERROR` dengan detail pose target yang gagal.
  2. Mengembalikan status `UNREACHABLE` ke caller.
  3. **TIDAK BOLEH** fallback ke solver lain secara diam-diam.

### Validasi Solusi IK

- Setiap solusi IK **WAJIB** dicek terhadap joint limits UR5e sebelum dikirim ke trajectory.
- Joint limits referensi (UR5e):

| Joint | Min (rad) | Max (rad) |
|---|---|---|
| shoulder_pan | -2π | 2π |
| shoulder_lift | -2π | 2π |
| elbow | -π | π |
| wrist_1 | -2π | 2π |
| wrist_2 | -2π | 2π |
| wrist_3 | -2π | 2π |

---

## 5. Collision Checking dengan FCL

### Aturan

- Collision scene **WAJIB** di-update setiap kali ada perubahan pada objek di workspace (meja, kubus, silinder).
- Objek collision didefinisikan sebagai geometri primitif (`Box`, `Cylinder`, `Sphere`) — **bukan** mesh kompleks, kecuali untuk robot body sendiri.
- Setiap trajectory point **WAJIB** lolos collision check sebelum dikirim ke Action Server.
- Allowed Collision Matrix (ACM):
  - Self-collision antara link yang bersebelahan (adjacent links) → **diizinkan** (sudah ditangani oleh URDF joint limits).
  - Collision `base_link` ↔ `table` → **diizinkan** (robot terpasang di atas meja).
  - Semua collision lainnya → **DILARANG** (harus memicu trajectory rejection).

---

## 6. Konvensi Kode

### Penamaan

- Package ROS 2: `snake_case` (contoh: `arm_planning`, `arm_description`)
- Node name: `snake_case` (contoh: `trajectory_planner_node`)
- Topic/Service/Action: `snake_case` dengan leading `/` (contoh: `/planning/target_pose`)
- File Python: `snake_case.py`
- File C++: `snake_case.cpp` / `snake_case.hpp`

### Struktur Node

Setiap node **WAJIB** mengikuti pola berikut:

```python
import rclpy
from rclpy.node import Node

class NamaNode(Node):
    def __init__(self):
        super().__init__('nama_node')
        # 1. Deklarasi parameter
        # 2. Inisialisasi publisher/subscriber/service/action
        # 3. Inisialisasi TF2 buffer & listener
        # 4. Timer (jika perlu)
    
    def destroy_node(self):
        # Cleanup resources
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = NamaNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

### Logging

- Gunakan logger bawaan ROS 2 (`self.get_logger()`). **Dilarang** menggunakan `print()` untuk output operasional.
- Level minimum untuk production: `INFO`.
- Semua error yang berhubungan dengan safety (collision, joint limit violation, transform kedaluwarsa) → `ERROR`.

### Launch Files

- Gunakan format Python (`*.launch.py`), **bukan** XML atau YAML.
- Semua parameter yang bisa berubah **WAJIB** dideklarasikan sebagai `LaunchArgument` dengan nilai default.

---

## 7. Aturan Keselamatan (Safety)

1. **Emergency Stop:** Setiap node planning WAJIB subscribe ke `/emergency_stop` (`std_msgs/msg/Bool`). Jika `True`:
   - Cancel semua goal aktif di Action Server.
   - Hentikan publikasi trajectory baru.
   - Log `FATAL`.

2. **Joint Velocity Limit:** Kecepatan sendi TIDAK BOLEH melebihi 180°/s (π rad/s) pada mode simulasi. Validasi ini dilakukan di Planning Layer sebelum pengiriman trajectory.

3. **Workspace Boundary:** Definisikan bounding box workspace yang diizinkan. Semua target pose di luar boundary → tolak dengan `OUT_OF_WORKSPACE`.
