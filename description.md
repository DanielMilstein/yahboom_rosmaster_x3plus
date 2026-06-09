# Gemini Pick-and-Place on the Yahboom ROSMaster X3 Plus

**Audience:** engineers integrating this system into another workflow. Assumes ROS 2 literacy; assumes no prior knowledge of this repo.

---

## 1. What this is

An autonomous, language-directed pick-and-place system on a mobile manipulator. You give it a natural-language task — the default is *"put the red can in the blue bin"* — and it:

1. Looks at the scene through an RGB-D camera.
2. Asks **Gemini Robotics ER** (`gemini-robotics-er-1.6-preview`) to find the target object and destination as image pixels.
3. Projects those pixels into 3-D robot coordinates using the depth image and the TF tree.
4. Drives the Mecanum base sideways/forward until the grasp is kinematically reachable.
5. Picks the object with a MoveIt 2-planned top-down grasp and a closed-loop gripper that stops on contact.
6. Asks Gemini to visually verify the object is actually in the gripper (retries up to 5 times if not).
7. Places it at the destination, returns the base to its starting position, and stows the arm.

The same code runs in two environments, selected by launch arguments:

- **Simulation** — Gazebo (Ignition Fortress) with ros2_control, on any dev machine.
- **Real hardware** — Jetson Orin NX Super + Orbbec Astra Pro+ RGB-D camera + Yahboom ROSMaster X3 Plus (Mecanum base, 5-DOF arm, 1-DOF gripper, STM32 motor controller on USB serial).

Platform: **ROS 2 Humble**, Ubuntu 22.04 (arm64 on the robot, x86_64 for sim).

---

## 2. Architecture

Five layers, each a separate package, communicating only through ROS interfaces:

```
┌─────────────────────────────────────────────────────────────────┐
│ ORCHESTRATION   gemini_pick_place_executor                      │
│   The state machine. Owns the whole pick-place sequence.        │
└──────┬──────────────────┬──────────────────┬────────────────────┘
       │ ROS services     │ moveit_py        │ topics
┌──────▼───────┐  ┌───────▼────────┐  ┌──────▼───────────────────┐
│ REASONING    │  │ PLANNING       │  │ PERCEPTION               │
│ gemini_      │  │ x3plus_moveit_ │  │ perception_bridge.py     │
│ robotics_    │  │ config         │  │ (in x3plus_moveit_config)│
│ bridge       │  │ (MoveIt 2)     │  │ pixel → 3-D projection   │
└──────────────┘  └───────┬────────┘  └──────┬───────────────────┘
                          │ FollowJointTrajectory actions          │ images
┌─────────────────────────▼─────────────────────────────────▼─────┐
│ HARDWARE / SIM                                                   │
│   sim: Gazebo + ros2_control + mecanum_drive_controller          │
│   hw:  yahboom_rosmaster_hw_bridge (serial) + OrbbecSDK_ROS2     │
└──────────────────────────────────────────────────────────────────┘
```

### Package inventory (the ones that matter for integration)

| Package | Role |
|---|---|
| `gemini_pick_place_executor` | Orchestrator node + the launch file that configures the entire behavior (~77 parameters). |
| `gemini_robotics_bridge` | Wraps the Gemini API behind two ROS services. The only component that talks to the internet. |
| `x3plus_moveit_config` | MoveIt 2 config (URDF/SRDF/kinematics), `perception_bridge.py` (pixel→3-D), `publish_tabletop_scene.py` (collision scene), and the top-level bringup launches for both sim and hardware. |
| `yahboom_rosmaster_hw_bridge` | **Hardware only.** Single Python process that owns the STM32 serial link: base drive, odometry, IMU, joint states, and the arm/gripper trajectory action servers. Also hosts `camera.launch.py` for the Orbbec driver. |
| `yahboom_rosmaster_description` | URDF/xacro. The `use_gazebo` xacro arg switches between sim plugins and bare hardware description. Camera mount pose lives here. |
| `yahboom_rosmaster_gazebo` | Gazebo worlds and sim bringup. |
| `yahboom_rosmaster_msgs` | Custom service definitions (`GeminiPickPlace.srv`, `GeminiVerifyPick.srv`). |
| `OrbbecSDK_ROS2` | Git submodule, pinned. Vendor driver for the Astra Pro+ (v1 SDK branch — the Astra Pro+ is "limited maintenance" in v2). |

Other packages in the repo (`yahboom_rosmaster_navigation`, `_localization`, `_docking`, `arm_moveit_demo`, `mecanum_drive_controller`) are either sim-support or earlier experiments and are not part of the pick-place pipeline.

---

## 3. Anatomy of one run

The executor (`gemini_pick_place_executor.py`, node `gemini_pick_place_executor`) auto-starts a single run when it receives its first camera image (`auto_start:=true`, the default). One run, in order:

1. **Stow for perception** — arm moves to a named SRDF pose that keeps it out of the camera's view.
2. **Perceive** — grabs the latest RGB frame from `/perception_bridge/debug_image` and calls the `/gemini_pick_place` service with the task string + image. Gemini returns JSON with target/destination pixels and bounding boxes.
3. **Project** — publishes each pixel to `/perception_bridge/pixel`; `perception_bridge` looks up depth at (or near) that pixel, projects through the camera intrinsics, transforms into `base_footprint`, and answers on `/perception_bridge/selected_point_base`. Object height is estimated from the bounding box.
4. **Drive to feasible** — if the grasp point is outside the arm's sweet spot (`sweet_x`/`sweet_y` ± reach window), searches a bounded grid of base offsets (`base_search_dx/dy_range_m`), checks IK at each candidate, and drives the base (closed-loop on odometry) to the best one. The perceived points are dead-reckoned through the base motion.
5. **Pick** — MoveIt plans a top-down grasp (orientation-constrained, with tilt fallbacks), descends, then closes the gripper with a **closed-loop contact detector**: small position steps, watching position error and movement deltas against thresholds; stops on contact instead of crushing.
6. **Verify** — lifts, moves to a "show" pose, snapshots the gripper, and calls `/gemini_verify_pick` ("is the object held?"). On failure: release, retry from step 2, up to `max_pick_attempts` (5).
7. **Place** — carries (arm in `up` pose) to the destination, descends, opens.
8. **Return** — drives back to the run-start odometry pose (`return_after_place:=true`), restows the arm. On any failure, `reset_on_failure:=true` opens the gripper, drives back, and parks the arm.

A **dry run** (`execute:=false`, the default) does steps 1–3 only and publishes RViz markers of the projected points on `/gemini_pick_place/debug_markers` — no motion. This is the recommended first integration test.

---

## 4. Integration surface

These are the stable interfaces another service can consume. Everything below was verified against source.

### 4.1 ROS services (callable standalone, without the executor)

**`/gemini_pick_place`** — `yahboom_rosmaster_msgs/srv/GeminiPickPlace`

```
# Request
string task                 # natural-language instruction
sensor_msgs/Image image     # RGB frame to analyze
---
bool success                # service-level success (API reachable, JSON parsed)
bool accepted               # Gemini accepted the task as doable in this image
float32 confidence
string result_json          # pixel coordinates + boxes for target & destination
string error_message
string log_path             # on-disk log of prompt/response for debugging
```

**`/gemini_verify_pick`** — `yahboom_rosmaster_msgs/srv/GeminiVerifyPick`

```
# Request
string target_label
sensor_msgs/Image image
---
bool success
bool picked_up              # the actual verdict
float32 confidence
string reason
string log_path
string error_message
```

Both are served by `gemini_robotics_bridge.py`. If your workflow only needs "find object X in this image" or "is the gripper holding X", you can call these directly and skip the executor entirely.

### 4.2 Pixel→3-D projection (topic pair)

`perception_bridge.py` is a stateless projection utility usable by any node:

- **Publish** `geometry_msgs/PointStamped` to `/perception_bridge/pixel` with `point.x = u`, `point.y = v` (pixel coordinates).
- **Receive** `geometry_msgs/PointStamped` on `/perception_bridge/selected_point_base` — the 3-D point in `base_footprint`, computed from the registered depth image and TF.

It also republishes the RGB stream on `/perception_bridge/debug_image` (this is the frame the executor sends to Gemini, so pixels round-trip consistently).

### 4.3 Motion interfaces

| Interface | Type | Notes |
|---|---|---|
| `/arm_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | Joints `arm_joint1..arm_joint5`. Same contract in sim (ros2_control) and hardware (bridge). |
| `/gripper_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | Joint `grip_joint`, range −1.54 rad (open) to 0.0 (closed). |
| `/joint_states` | `sensor_msgs/JointState` | 15 Hz from servo readback on hardware. Filtered to `/moveit_joint_states` for MoveIt. |
| `/cmd_vel` | `geometry_msgs/Twist` | **Hardware bridge** input for teleop and external nodes. Body-frame Mecanum velocities. |
| `/cmd_vel_stamped` | `geometry_msgs/TwistStamped` | **Hardware bridge** input used by the executor's base drive. Same effect as `/cmd_vel`. |
| `mecanum_drive_controller/cmd_vel` | `geometry_msgs/TwistStamped` | **Sim controller** input. The executor launch selects sim vs hardware drive topics automatically from `use_gazebo`. |
| `/odom`, `/imu/data_raw` | `nav_msgs/Odometry`, `sensor_msgs/Imu` | 30 Hz from the hardware bridge (odometry is integrated body velocity — relative, zeroed at bridge start). |

### 4.4 Camera topics (hardware)

`/camera/color/image_raw`, `/camera/depth/image_raw` (hardware-registered to color), `/camera/color/camera_info` — from the Orbbec driver via `camera.launch.py` (`depth_registration:=true`, `align_mode:=HW`). In sim the equivalents are under `/cam_1/...` with overridden intrinsics; `perception_bridge` is parameterized for both.

### 4.5 Environment / external dependencies

- **`GEMINI_API_KEY`** — required by `gemini_robotics_bridge`. Optional `GEMINI_API_KEY_2`, `GEMINI_API_KEY_3` enable automatic key rotation on quota errors.
- Internet access from wherever `gemini_robotics_bridge` runs (it's the only networked component — it can run on a different machine than the robot if they share a ROS domain).
- Python deps: `src/yahboom_rosmaster/gemini_robotics_bridge/requirements-gemini.txt` (Google GenAI SDK).
- Hardware only: `Rosmaster_Lib` v3.3.9 (ships preinstalled on the Yahboom Jetson image) and the Yahboom udev rule that symlinks the STM32 as `/dev/myserial`.

### 4.6 Retargeting the task

The task is just a string parameter. To make the robot do something else with the same scene grammar:

```bash
ros2 launch gemini_pick_place_executor executor.launch.py \
  task:="put the green block in the red cup" execute:=true ...
```

No retraining, no code change — Gemini handles the grounding. Objects must be visible, within the camera's depth range (≥ 0.6 m), and within the arm's reach envelope after base search.

---

## 5. How to run it

### 5.1 Build

```bash
cd <workspace_root>
colcon build --base-paths src --symlink-install
source install/setup.bash   # or setup.zsh
python3 -m pip install -r src/yahboom_rosmaster/gemini_robotics_bridge/requirements-gemini.txt
export GEMINI_API_KEY="..."
```

(`--base-paths src` keeps colcon from scanning a workspace-local venv.)

### 5.2 Simulation

```bash
# Terminal 1 — Gazebo + MoveIt + perception + collision scene + RViz
ros2 launch x3plus_moveit_config gazebo_moveit.launch.py

# Terminal 2 — Gemini service
ros2 run gemini_robotics_bridge gemini_robotics_bridge.py

# Terminal 3 — the executor (runs once, on first image)
ros2 launch gemini_pick_place_executor executor.launch.py execute:=true
```

### 5.3 Real hardware (on the Jetson)

```bash
# Terminal 1 — bridge + camera + MoveIt + perception + RViz
ros2 launch x3plus_moveit_config hardware_moveit.launch.py \
  enable_arm_execution:=true enable_gripper_execution:=true

# Terminal 2 — Gemini service
ros2 run gemini_robotics_bridge gemini_robotics_bridge.py

# Terminal 3 — executor, pointed at hardware
ros2 launch gemini_pick_place_executor executor.launch.py \
  execute:=true use_gazebo:=false use_sim_time:=false
```

**Safety gates:** `enable_arm_execution` and `enable_gripper_execution` default to **false** — the bridge rejects trajectory goals until you pass them as true (or `ros2 param set` at runtime). Bring the stack up gated first, confirm `/joint_states` tracks the physical arm (flex a joint by hand), then enable.

**Recommended first run:** `execute:=false` (perception-only dry run). Verify the debug markers in RViz land where the physical objects are before allowing motion.

### 5.4 Verifying each layer independently

- Camera only: `ros2 launch yahboom_rosmaster_hw_bridge camera.launch.py`, then `ros2 topic hz /camera/color/image_raw`.
- Perception only: `ros2 launch x3plus_moveit_config perception_hw.launch.py`, then the pixel→3-D round trip from §4.2.
- Base only: `ros2 launch yahboom_rosmaster_hw_bridge hw_bridge.launch.py`, then `teleop_twist_keyboard` on `/cmd_vel`.
- Gemini only: call `/gemini_pick_place` with any image (§4.1).

---

## 6. Configuration

All behavior tuning lives in **`src/yahboom_rosmaster/gemini_pick_place_executor/launch/executor.launch.py`**, in the `FORWARDED_PARAMS` list — that file is the source of truth; every entry is overridable as a launch argument. Grouped by concern:

| Group | Representative params | What they control |
|---|---|---|
| Task | `task`, `execute`, `auto_start` | What to do and whether to actually move. |
| Grasp geometry | `pick_lift_m`, `grasp_z_fraction_from_top`, `gripper_tip_offset_xyz`, `table_z_m` | Where on the object to grab and how high to lift. |
| Gripper close-loop | `close_grip_step_size_rad`, `close_grip_position_error_threshold_rad` (0.018), `close_grip_movement_threshold_rad` (0.040), `close_grip_timeout_s` | Contact detection. **Tuned in Gazebo; expect re-tuning on real servos** (noise floor differs). |
| Base drive envelope | `base_search_dx_range_m` `[0, 0.17]`, `base_search_dy_range_m` `[-0.28, 0.23]`, `drive_axes`, `drive_max_lin_speed_mps` | How far the base may roam. Currently set to a specific physical workspace — **adjust to yours**. |
| Reach | `sweet_x` (0.18), `sweet_y`, `reach_window_*` | The arm's preferred grasp zone relative to `base_footprint`. |
| Verification | `verify_pick_with_gemini`, `verify_pick_required`, `max_pick_attempts` | Pick-verify-retry behavior. |
| Recovery | `return_after_place`, `restow_after_place`, `reset_on_failure`, `failure_reset_pose_named` | End-of-run and failure behavior. All default to safe/true. |
| Planning | `planning_time`, `velocity_scale` (0.3), `accel_scale` | MoveIt planner budget and speed caps. |

Hardware bridge configuration: `src/yahboom_rosmaster/yahboom_rosmaster_hw_bridge/config/servo_map.yaml` maps each URDF joint to a servo bus ID and a radians↔degrees linear calibration. **This file is calibrated to one specific robot.** If a servo is replaced or the arm re-assembled, re-derive it with `scripts/probe_servos.py` (sweeps each servo ID so you can observe which joint moves) and verify by hand-flexing joints while watching `/joint_states`.

---

## 7. Constraints, assumptions, and known gaps

Things that will bite an integrator who doesn't know them:

1. **Base-drive topics are backend-specific — let the launch pick them.** The executor's `cmd_vel_topic` / `odom_topic` defaults point at the sim controller; `executor.launch.py` overrides them to the hardware bridge's `/cmd_vel_stamped` and `/odom` when `use_gazebo:=false`. If you run the executor node directly (`ros2 run`) instead of via the launch file, you must set those two parameters yourself or base drive will silently no-op on hardware.
2. **One process owns the serial port.** `/dev/myserial` (the STM32) can be held by exactly one process. Never run the bridge twice, and don't run a `Rosmaster` Python REPL while the bridge is up — the second opener fails or corrupts the stream.
3. **Odometry is relative.** "Return to origin" means the odom pose captured at run start. Restarting the bridge zeroes the integrator. There is no map frame, no localization, no absolute anchor.
4. **Camera depth floor ≈ 0.6 m.** The Astra Pro+ returns invalid depth (0.0) closer than that. Objects must sit in the 0.6 m+ band; `perception_bridge` searches a small radius around the requested pixel for valid depth but cannot conjure depth that isn't there.
5. **Camera pose comes from the URDF.** The `astra_joint` origin in `yahboom_rosmaster_description/urdf/robots/rosmaster_x3_plus.urdf.xacro` must match the physical mount (currently 19° downward pitch). If projections are systematically offset, this transform is the first suspect. There is no hand-eye calibration step.
6. **The collision scene is minimal.** `publish_tabletop_scene.py` publishes one hardcoded table box. MoveIt does not know about the robot's own chassis or anything else in the room — keep clutter out of the arm's envelope.
7. **Gemini is a runtime dependency.** Every run makes 1–2+ API calls (pick + verify per attempt). Expect seconds of latency per call, quota limits (key rotation helps), and nondeterminism — the same scene can yield slightly different pixels run to run. All prompts/responses are logged to disk (`log_dir`) for postmortems.
8. **Trajectory timing on hardware is approximate.** The bridge executes waypoints over serial sequentially and can run slower than the planned trajectory. If MoveIt aborts with execution-duration complaints, raise `trajectory_execution.allowed_execution_duration_scaling` in `hardware_moveit.launch.py`.
9. **Out-of-range servo commands clamp silently** to the servo's 0–180° span. A plan that exceeds `servo_map.yaml` range stops short physically; the symptom is the *next* plan failing its start-state tolerance check.
10. **No tests, no CI.** Verification is by running sim or hardware. Treat any change to the executor or bridge as untested until you've seen a full run.

---

## 8. Where to look when something breaks

| Symptom | First place to look |
|---|---|
| Executor never starts | Is an image arriving on `/perception_bridge/debug_image`? (`auto_start` waits for the first frame.) |
| `/joint_states` all zeros | `servo_map_path` wrong/empty, or serial reads failing — check bridge logs; bridge falls back to 0.0 placeholders. |
| Projected points offset from reality | Camera URDF pose (§7.5), then depth registration (`depth_registration:=true` must be on). |
| Arm goals rejected | `enable_arm_execution` still false, or bridge in stub mode (no serial). |
| Gemini service errors | `GEMINI_API_KEY` unset, quota exhausted (check `log_path` in the response), or no network. |
| Gripper crushes / never detects contact | Close-loop thresholds (§6) still Gazebo-tuned — re-tune against real servo telemetry. |
| Base doesn't move on hardware | Executor started without the launch file's topic overrides (§7.1), or `enable_base_drive:=false`. |

For deeper background: `docs/PLAN-pragmatic.md` (deployment rationale and phase history) and `src/yahboom_rosmaster/README.md` (the original sim-only debug flow).
