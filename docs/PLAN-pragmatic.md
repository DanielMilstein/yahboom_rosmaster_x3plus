# Pragmatic real-hardware deployment plan

## Context

The Gazebo / Ignition simulation pipeline (perception → Gemini → MoveIt → closed-loop gripper → place) works end-to-end. This branch holds the **deadline-shaped** deployment of the pipeline to the real Yahboom ROSMaster X3 Plus, sized for a thesis timeline (~1–2 weeks of focused work on hardware).

A separate `optimal` branch documents the production-grade alternative for follow-up work or publication. The two branches share `main` and do not paint each other into a corner — every source artifact produced here would be reused as a reference implementation if the optimal path is later pursued.

### Hardware target

- **Compute**: Jetson Orin NX Super (arm64, JetPack 6.x, Ubuntu 22.04, ROS 2 Humble).
- **Camera**: Orbbec Astra Pro+ (mounted on chassis as URDF describes).
- **Robot**: Yahboom ROSMaster X3 Plus (Mecanum base + 5-DOF arm + 1-DOF gripper, STM32-on-USB).
- **Prior ROS 2 hardware work**: none. Stock Yahboom drivers are ROS 1 Noetic, ship with the robot image.

### Intended outcome

Working thesis demo: drive → perceive → Gemini grasp pose → pick → verify_show → verify_pick → place → return, on the real robot. No production polish. Heuristics OK where they save weeks.

---

## Architecture choice

**Python action server, no `ros2_control` plugin.** MoveIt's `moveit_simple_controller_manager` calls `FollowJointTrajectory` action servers directly — it does not require a `hardware_interface::SystemInterface` plugin underneath. By hosting the action servers ourselves and wrapping Yahboom's existing `Rosmaster_Lib` Python serial library, we skip ~2–3 weeks of C++ plugin development and protocol reverse-engineering.

The cost is: no real-time guarantees, polled joint states (~15 Hz), no effort feedback (so the gripper close loop keeps using the position-error heuristic with re-tuned thresholds).

---

## P0 — offline prep (no robot needed)

1. Restore the `OrbbecSDK_ROS2` submodule on `main` (done — commit `806c2d4`).
2. Parameterize `use_gazebo` and `use_sim_time` as launch args in `src/yahboom_rosmaster/gemini_pick_place_executor/launch/executor.launch.py` (currently hard-coded `true`). Default stays `true` so simulation keeps working unchanged; pass `use_gazebo:=false use_sim_time:=false` on the Orin.
3. Scaffold a new package `yahboom_rosmaster_hw_bridge` (Python, `ament_python`):
   - `arm_action_server.py` — hosts `/arm_controller/follow_joint_trajectory` (`FollowJointTrajectory` action). Interpolates trajectory points, calls `Rosmaster_Lib.set_uart_servo_angle()`.
   - `gripper_action_server.py` — mirrors the above for `/gripper_controller/follow_joint_trajectory`.
   - `joint_state_publisher.py` — polls `get_uart_servo_angle()` at 15 Hz, publishes `/joint_states`.
   - `base_driver.py` — subscribes `/cmd_vel`, calls `Rosmaster_Lib.set_car_motion()`. Publishes `/odom` from `get_motion_data()` and `/imu/data_raw` from `get_imu_attitude()`.
   - All stubbed today, no serial calls until Phase 1.

## P1 — base on hardware

1. SSH to Orin. Confirm `Rosmaster_Lib` installed: `pip3 show Rosmaster-Lib`. If missing, install from Yahboom's wheels.
2. Find the STM32 serial device: `ls -l /dev/serial/by-id/` (probably `usb-Yahboom_...` or similar). Add a udev rule to symlink as `/dev/myserial` so the bridge isn't tied to `ttyUSB0/1` enumeration order.
3. Smoke-test `Rosmaster_Lib` from a plain Python REPL on the Orin: spin one wheel, read IMU. **No ROS yet.**
4. Wire the bridge package's `base_driver.py` to `Rosmaster_Lib`. Verify `/odom`, `/imu/data_raw` populated. Drive with `teleop_twist_keyboard`.

## P2 — arm + gripper

1. YAML mapping: MoveIt joint names (`arm_joint1`…`arm_joint5`, `grip_joint`) → Yahboom servo IDs.
2. Wire `arm_action_server.py` and `gripper_action_server.py` to `set_uart_servo_angle()`. Linearly interpolate trajectory points to ~10 ms ticks, send each as a servo command with the planned duration. MoveIt's `moveit_simple_controller_manager` calls these action servers directly — no ros2_control plumbing required.
3. `joint_state_publisher.py` polls servo positions at 15 Hz (matches the existing `joint_state_broadcaster` rate in sim).
4. Hand-test: rviz MotionPlanning panel → plan + execute to the `up` SRDF pose.

## P3 — camera

1. Build `OrbbecSDK_ROS2` on the Orin (`colcon build --packages-select orbbec_camera` + system deps).
2. Launch the Astra Pro+ driver. Confirm topics — likely `/camera/color/image_raw`, `/camera/depth/image_raw`, `/camera/color/camera_info`.
3. Two options to wire `perception_bridge.py`:
   - **Easier**: add remappings to the bringup launch so Orbbec topics get renamed to `/cam_1/...`.
   - **Cleaner**: change the defaults in `perception_bridge.py` (lines 36–38) from `/cam_1/...` to `/camera/...`. Keep the param overrides for sim.

## P4 — re-tune the gripper close loop on real hardware

The current `err > 0.018 / delta < 0.040` thresholds were calibrated to Gazebo. Real servos will show a different noise floor and contact onset signature. Empirically:

1. Run the close loop with the `info`-level step telemetry already in place.
2. Eyeball `err` / `delta` for ~20 quiet steps (free motion, no object) → that's the noise floor on real hardware.
3. Eyeball the same on the contact step → set the thresholds 1.5–2× above the noise floor.
4. Existing Fix D (step-action-timeout = soft contact, committed `7d67459`) is the safety net while tuning.

Expect ~30 min of iteration.

## P5 — end-to-end on the robot

1. Full task: drive → perceive → Gemini grasp pose → pick → verify_show → verify_pick → place → return.
2. If perception accuracy is off, calibrate camera extrinsics with a printed ArUco target — quick eye-on-base solve, hard-code the result in URDF.

---

## Files that change in Phase 0

- **Modify**: `src/yahboom_rosmaster/gemini_pick_place_executor/launch/executor.launch.py` — parameterize `use_gazebo` (hard-coded `"true"` in the `MoveItConfigsBuilder` mappings) and surface it as a launch arg; existing `use_sim_time` arg already exists and is fine.
- **New package**: `src/yahboom_rosmaster/yahboom_rosmaster_hw_bridge/` (Python `ament_python`):
  - `package.xml`, `setup.py`, `setup.cfg`, `resource/yahboom_rosmaster_hw_bridge`.
  - `yahboom_rosmaster_hw_bridge/__init__.py`.
  - `yahboom_rosmaster_hw_bridge/arm_action_server.py` (stub).
  - `yahboom_rosmaster_hw_bridge/gripper_action_server.py` (stub).
  - `yahboom_rosmaster_hw_bridge/joint_state_publisher.py` (stub).
  - `yahboom_rosmaster_hw_bridge/base_driver.py` (stub).
  - `launch/hw_bridge.launch.py`.
  - `config/servo_map.yaml`.

## Verification (Phase 0)

- `colcon build --packages-select gemini_pick_place_executor yahboom_rosmaster_hw_bridge` succeeds.
- Existing sim run still works unchanged: `ros2 launch gemini_pick_place_executor executor.launch.py execute:=true drive_axes:=xy` (no new args needed; defaults preserve old behavior).
- Sim run with explicit args also works: `... use_gazebo:=true use_sim_time:=true` produces identical behavior.
- New bridge stubs are importable (`python3 -c "from yahboom_rosmaster_hw_bridge import arm_action_server"`).
- `git submodule status` shows OrbbecSDK_ROS2 at the pinned commit, no errors (inherited from `main`).

## Open items (can resolve later, don't block Phase 0)

- `Rosmaster_Lib` install state on the Orin — confirm next SSH session: `pip3 show Rosmaster-Lib && python3 -c "import Rosmaster_Lib; print(Rosmaster_Lib.__file__)"`. If missing, grab the wheel from Yahboom's distribution. Bridge stubs import-guard the dependency so Phase 0 doesn't need it installed.

## Status

`pragmatic` branch — active development target. Phase 0 work begins next.
