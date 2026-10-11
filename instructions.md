# Panduan Coding — DecodeLabs Robotic Arm Workspace (RViz-Only State)

---

## 1. TF Frames & Tree Convention
- Chain: `world` → `table_link` → `l0` → `base_link` → `tool0`.
- Meja parent: `world`.
- Objek workspace parent: `table_link` at `y ±0.15` middle.
- `collision_objects.yaml` poses offset `x: 0.35`.

---

## 2. IK Solver & Fallback
- Primary: TRAC-IK via Python bindings.
- Fallback: 6-DOF geometric analytical solver (2-link planar IK shoulder-elbow + spherical wrist alignment) jika TRAC-IK tidak tersedia atau time out.

---

## 3. Trajectory Generator
- Quintic polynomial interpolation (60 waypoints, 3 detik).
- Velocity limit: π rad/s.

---

## 4. Collision Checking & ACM
- Framework: Python FCL (`fcl.CollisionObject`).
- Allowed Collision Matrix (ACM):
  - Adjacent links: Allowed.
  - `l0` / `base_link` ↔ `table_link`: Allowed.
  - Semua link robot ↔ obstacles (`cube_1`, `cylinder_1`, unallowed parts): Rejected.

---

## 5. Safety & Coordinator
- **Emergency Stop:** Subscribe `/emergency_stop`. Trajectory lock sampai `/planning/reset` service dipanggil.
- **Velocity Limit:** Maximum joint speed π rad/s.
- **Workspace Boundary:** Target pose divalidasi dalam reachable zone robot.
- **Coordinator:** `hold_timer` + mock executor untuk simulasi interaktif RViz.
- **Manual Demo:** Target dapat dikirim secara manual via `/planning/target_pose` (titik A(0.40, 0, 0.35) ↔ B(0.35, 0.25, 0.30)).
- **Simulation container:** `cae28c116ffd`.
