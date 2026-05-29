# Optimal real-hardware deployment plan

## Context

The Gazebo / Ignition simulation pipeline (perception → Gemini → MoveIt → closed-loop gripper → place) works end-to-end. This branch documents the **architecturally-optimal** deployment path for moving the pipeline to the real Yahboom ROSMaster X3 Plus, assuming no deadline pressure.

A separate `pragmatic` branch documents and develops the deadline-shaped path. The two branches share `main` as a common ancestor and do not diverge in any source file *yet* — this branch is currently planning-only.

### Hardware target

- **Compute**: Jetson Orin NX Super (arm64, JetPack 6.x, Ubuntu 22.04, ROS 2 Humble).
- **Camera**: Orbbec Astra Pro+ (mounted on chassis as URDF describes).
- **Robot**: Yahboom ROSMaster X3 Plus (Mecanum base + 5-DOF arm + 1-DOF gripper, STM32-on-USB).
- **Prior ROS 2 hardware work**: none. Stock Yahboom drivers are ROS 1 Noetic.

### Intended outcome

Production-grade, citable, reusable ROS 2 hardware integration suitable for follow-up projects, publication, or productization. No heuristic gripper tuning. No sim/hw drift. Reproducible builds.

---

## O1. C++ `ros2_control` SystemInterface plugin

- New package: `yahboom_hardware_interface`. C++ `hardware_interface::SystemInterface` plugin implementation.
- Reverse-engineer Yahboom's UART protocol from `Rosmaster_Lib`'s Python source (documented packet format: `0xFF` header, length byte, function code, payload, checksum). Port to C++ using `libserial` or `boost::asio`. No Python, no GIL, real-time safe.
- `read()` / `write()` cycles synced with `controller_manager`'s update loop (100–200 Hz).
- Exposes proper hardware interfaces:
  - **4 wheel joints**: velocity command, position + velocity state.
  - **5 arm joints**: position command, position + **effort** state.
  - **1 grip joint**: position command, position + **effort** state. ← the key signal that obsoletes the gripper-tuning heuristics.
- Plugin registration via `pluginlib`. URDF integration via `<hardware><plugin>yahboom_hardware/RosmasterX3PlusSystem</plugin></hardware>` inside a xacro conditional.

## O2. Single-arg sim/hardware/mock selection in URDF

One xacro argument: `mode := sim|hw|mock`.

- `sim` → `gz_ros2_control/GazeboSimSystem`.
- `hw`  → `yahboom_hardware/RosmasterX3PlusSystem`.
- `mock` → `mock_components/GenericSystem` (for CI without Gazebo or hardware).

One bringup launch with `mode:=` arg, no diverging launch trees, no config drift between sim and hardware.

## O3. Effort-based grasp (kills the heuristic close-loop entirely)

The position-error / delta-movement heuristics (`err > 0.018` etc.) currently in `gemini_pick_place_executor.py::_close_gripper_until_contact` are proxies for "the servo can't move because something is in the way." The real signal is **servo current draw** (effort). With effort exposed as a state interface:

- Drop the per-step delta/err logic.
- Two-stage close:
  1. Position-control to (Gemini-measured object width + small clearance) — fast, open-loop.
  2. Close the last ~5 mm with effort-bounded position control — stop when `effort > effort_threshold`.
- Implement as a custom `ForceLimitedJointTrajectoryController` subclassing `joint_trajectory_controller`, or as a separate `EffortGraspController` plugin.
- Result: ~10× faster close, much more robust to servo behavior changes or partial occlusion, no Gazebo-vs-real threshold drift.

## O4. Camera integration done properly

- Use the `OrbbecSDK_ROS2` submodule (already wired on `main` at v1.5.15), but pin to a release tag rather than a floating ref.
- System dependencies (libusb, libuvc, OrbbecSDK shared lib) installed via a `Dockerfile` so the Orin build is reproducible.
- **Eye-on-base extrinsic calibration** with `easy_handeye2` and a ChArUco target — print, capture ~20 poses, solve for `camera_link → base_link`. Result encoded as a `<joint type="fixed">` in URDF.
- Replace any ad-hoc `static_transform_publisher` in launch files.

## O5. Lifecycle nodes throughout

- Convert bridge node, `perception_bridge`, `gemini_robotics_bridge`, and the Orbbec driver to `LifecycleNode`.
- Sequenced bringup: `configure` (open serial, check camera health, verify Gemini service reachable) → `activate` (start publishing/subscribing). Catches bring-up failures *before* downstream nodes start subscribing to topics that won't appear.

## O6. Diagnostics + EKF + tf2

- `diagnostic_aggregator` reporting: serial link health, motor temperatures (X3 Plus exposes them), camera FPS, IMU sample rate, Gemini service latency.
- `robot_localization` EKF fusing wheel odometry + IMU (the STM32 has a 9-DOF IMU; expose it via the SystemInterface).
- Stable, well-defined `odom → base_link → arm_link5 → camera_link` chain with no manual `static_transform_publisher` patches.

## O7. Reproducibility + CI

- `Dockerfile` pinned to `nvcr.io/nvidia/l4t-ros:r36.x-humble` or equivalent JetPack 6 + Humble base.
- CI on x86 with `mode:=mock` exercises the executor end-to-end (no Gazebo, no hardware). Catches executor logic regressions in seconds, not minutes.
- HIL test rig (Orin + just the arm + USB camera, no chassis) for periodic validation if budget allows.

---

## Cost / payoff

| | |
|---|---|
| **Estimated effort** | 6–10 weeks of focused work |
| C++ SystemInterface | 2–3 weeks (protocol RE + driver impl + URDF/xacro plumbing + debugging) |
| Effort-based grasp controller | 1 week |
| Camera + hand-eye calibration | 1 week |
| Lifecycle + diagnostics + EKF | 1–2 weeks |
| Docker + CI | 1 week |
| **Payoff** | Production-grade, citable, reusable; no sim/hw drift; no heuristic gripper tuning |

---

## Reuse from current main

Every URDF, MoveIt config, perception bridge, Gemini bridge, and pick-executor file on `main` is reused unchanged in this path. Only the hardware interface layer (currently `mock_components/GenericSystem` or `gz_ros2_control`) is replaced. The pragmatic branch's Python bridge becomes a reference implementation for protocol semantics during the C++ port.

## Status

Planning only. No source code changes on this branch yet. If/when this path is pursued, the first concrete deliverable is O1 (the C++ SystemInterface), with the protocol mapping informed by the pragmatic branch's Python prototype if that branch has progressed in parallel.
