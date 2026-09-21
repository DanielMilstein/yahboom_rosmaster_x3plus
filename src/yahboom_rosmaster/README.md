# yahboom_rosmaster X3 Plus #
![OS](https://img.shields.io/ubuntu/v/ubuntu-wallpapers/jammy)
![ROS_2](https://img.shields.io/ros/v/humble/rclcpp)



## Hardware pick-and-place (current flow)

The executor runs the full autonomous mission on the real robot: perceive
(Gemini + plane ranging + lidar wall reference), drive, pick, verify with
Gemini, drive to the destination, place, and return to start.

### Known-good hardware command

```bash
ros2 launch gemini_pick_place_executor executor.launch.py \
  execute:=true use_gazebo:=false use_sim_time:=false \
  lidar_drive_correction:=true place_wall_aim:=true \
  task:="put the white cube in the grey container at the right of the 3d printer" \
  drive_axes:=xy drive_max_lin_speed_mps:=0.05 reperceive_after_drive:=true \
  plane_ranging:=true table_z_source:=perception table_z_m:=0.16 \
  wall_camera_check:=true wall_camera_autocal:=false \
  print_y_m:=0.05 wall_drive_gate:=true joint_limit_margin_rad:=0.05 \
  grasp_z_offset_m:=0.016 pick_z_safety_m:=0.015 drive_timeout_sec:=35.0 \
  grasp_tilt_first:=true grasp_roll_offset_rad:=0.0 bed_collision_clearance_m:=0.012 \
  verify_pick_with_gemini:=true bed_collision:=true \
  base_search_dx_range_m:="[0.0, 0.49]" base_search_dy_range_m:="[-0.15, 0.1]" \
  base_search_step_m:=0.03 ik_search_timeout_sec:=0.2
```

### Per-print arguments (change these for every new print job)

- `print_y_m` — the slicer's y coordinate of the object's CENTER, in
  **meters** (slicer millimeters / 1000). This activates the slicer-referenced
  x mode: the object's forward position is derived from the live lidar wall
  fit + `bed_offset_x_m` (wall face -> bed front border, taped once: 0.155)
  + `print_y_m` — no per-print tape measuring.

### Object size: what adapts automatically vs. what to pass

The pipeline measures the object fresh at every perception — **height** (bbox
top/bottom depth projection; grasp descent = 0.5 x measured height),
**grasp width** (bbox edges; grip command = measured - 5 mm clearance,
clamped to [0.005, 0.060] m), and the empty-grasp gate scales with the
measured width. Nothing about the object's size is hard-coded.

Three launch args encode size assumptions; for an object other than the
30 mm cube, adjust them. Example for a 40x40x40 mm cube:

```bash
object_half_depth_m:=0.02        # center -> near face, used in the slicer x chain (default 0.015)
plane_object_half_height_m:=0.02 # mid-height plane for plane ranging (default 0.015)
print_y_m:=<slicer y in meters>  # per print job, as always
```

`object_half_depth_m` matters most (it shifts the slicer-derived x target);
`plane_object_half_height_m` only affects the vision x that the slicer mode
overrides anyway. `object_height_fallback_m` (0.04) is only used when the
height measurement fails.

### Utility scripts

```bash
# Drive the base back to odom (0,0,yaw 0) after killing a run mid-mission.
# Only valid while yahboom_bridge_node has kept running (restarting the
# bridge re-zeros odometry at the robot's current spot).
ros2 run gemini_pick_place_executor return_to_origin.py

# Interactive arm-pose teach pendant ('2 -0.1' jogs a joint, 'p' prints the
# pose as a launch arg + SRDF snippet). Bringup up, executor down.
ros2 run gemini_pick_place_executor teach_pose.py
```

To watch the camera over SSH: `sudo apt install ros-humble-web-video-server`,
`ros2 run web_video_server web_video_server`, then open
`http://<jetson-ip>:8080` in a browser.

### Executor parameter reference

Every parameter `gemini_pick_place_executor.py` declares, grouped by what it
controls. Defaults are the node's own defaults; where `executor.launch.py`
forwards a different default it is noted as *(launch: value)*. Parameters
marked **node-only** are not exposed as launch arguments, so set them with
`--ros-args -p name:=value` when running the script directly, or add them to
`FORWARDED_PARAMS` in the launch file.

#### Mission and I/O

| Parameter | Default | Description |
|---|---|---|
| `task` | `put the red can in the blue bin` | Natural-language instruction sent to Gemini. Name the object and the destination. |
| `execute` | `false` | `true` moves the robot. `false` runs perception only and publishes RViz markers. |
| `auto_start` | `true` | Start the mission as soon as the first image arrives. **node-only.** |
| `image_topic` | `/perception_bridge/debug_image` | Camera image fed to Gemini. **node-only.** |
| `gemini_service` | `/gemini_pick_place` | Gemini planning service (returns target/destination pixels and boxes). **node-only.** |
| `pixel_topic` | `/perception_bridge/pixel` | Pixel requests published to the perception bridge for 3D projection. **node-only.** |
| `base_point_topic` | `/perception_bridge/selected_point_base` | Projected 3D points returned by the bridge, in `base_footprint`. **node-only.** |
| `marker_topic` | `/gemini_pick_place/debug_markers` | RViz `MarkerArray` output. **node-only.** |
| `scan_topic` | `/scan` | Lidar scan used for the wall reference, drive audit and wall collision object. |
| `joint_states_topic` | `/joint_states` | Joint readback for the closed-loop gripper close and pose correction. **node-only.** |
| `cmd_vel_topic` | `mecanum_drive_controller/cmd_vel` | Base velocity output. The launch file sets it per backend (`/cmd_vel_stamped` on hardware). **node-only.** |
| `odom_topic` | `mecanum_drive_controller/odom` | Odometry for closed-loop drives. The launch file sets it per backend (`/odom` on hardware). **node-only.** |
| `gripper_action_topic` | `/gripper_controller/follow_joint_trajectory` | Direct controller action used by the per-step gripper close (bypasses MoveIt's lagging scene monitor). **node-only.** |
| `arm_action_topic` | `/arm_controller/follow_joint_trajectory` | Direct controller action for the start-collision recovery move. |
| `verify_pick_service` | `/gemini_verify_pick` | Gemini service that judges whether the object is in the gripper. **node-only.** |
| `service_timeout_sec` | `10.0` | Timeout for each Gemini service call. **node-only.** |
| `project_timeout_sec` | `3.0` | Timeout for each pixel-to-3D projection round trip. **node-only.** |
| `project_attempts` | `4` | Projection retries per pixel (no Gemini re-call) on NaN depth or timeout. |
| `perception_attempts` | `3` | Full perception passes (each re-calls Gemini) before failing. |
| `perception_retry_delay_sec` | `1.0` | Pause between perception passes. |

#### Diagnostics

| Parameter | Default | Description |
|---|---|---|
| `ik_probe` | `false` | Diagnostic mode: instead of the mission, sweep x at `ik_probe_z` against all orientation candidates and log which solve IK. Maps the 5-DOF arm's reachable boundary. |
| `ik_probe_z` | `0.166` | Height of the IK probe sweep, in `base_footprint`. |
| `lidar_audit` | `true` | ICP-match the lidar scan before/after every base drive and log true motion vs. odometry. Observe-only unless `lidar_drive_correction` is set. |
| `empty_grasp_freeze_sec` | `0.0` | Hold the arm at a failed (empty) grasp for this long so the scene can be tape-measured against the logged fingertip FK. |

#### MoveIt groups and named poses

| Parameter | Default | Description |
|---|---|---|
| `arm_group_name` | `arm_group` | MoveIt planning group for the arm. **node-only.** |
| `gripper_group_name` | `grip_group` | MoveIt planning group for the gripper. **node-only.** |
| `end_effector_link` | `arm_link5` | Link whose pose IK targets. Its origin sits at the fingertips in this URDF. |
| `home_named` | `up` | SRDF pose the arm returns to at the end of a run. **node-only.** |
| `gripper_open_named` / `gripper_closed_named` | `open` / `close` | SRDF gripper states. **node-only.** |
| `carry_pose_named` | `up` | SRDF pose held while driving with payload. `up` keeps the gripper above nearby obstacles. |
| `stow_joint_values` | `[-1.5708, 1.0, -0.5, 0.0, 0.0]` | Joint pose that folds the arm out of the camera's view for perception. **node-only.** |
| `stow_for_perception` | `true` | Move to the stow pose before every perception pass. |
| `restow_after_place` | `true` | Return to the stow pose after a successful place. |
| `stow_settle_sec` | `0.3` | Wait after a stow before capturing an image. **node-only.** |
| `verify_show_pose_named` | `show` | SRDF pose that presents the gripper to the camera for verification. `none` skips the show move. |
| `verify_show_joints_rad` | `[0.0]` | Taught 5-joint show pose (from `teach_pose.py`). When exactly 5 values are given it overrides the named pose and is planned with collision checking. A 1-element list means unset. |
| `failure_reset_pose_named` | `up` | Pose to park the arm at after a failed run when `reset_on_failure` is set. |
| `start_collision_recovery` | `true` | If the initial stow cannot plan because the arm woke up in a model self-collision, send a direct trajectory to `up` and re-plan once. |

#### Planning and IK

| Parameter | Default | Description |
|---|---|---|
| `planning_time` | `5.0` | OMPL planning budget per pose, seconds. |
| `velocity_scale` / `accel_scale` | `0.3` / `0.3` | MoveIt velocity and acceleration scaling factors. |
| `ik_timeout_sec` | `4.0` | IK timeout for the actual grasp/place poses. |
| `ik_search_timeout_sec` | `0.3` | Shorter IK timeout used while scanning base-offset candidates. |
| `use_orientation_constraint` | `true` | Add an orientation constraint to pose goals (tolerances below). |
| `position_tolerance_m` | `0.01` | Position tolerance of pose goals. |
| `orientation_xy_tol_rad` | `0.1` *(launch: 0.3)* | Orientation tolerance about the x and y axes. |
| `orientation_z_tol_rad` | `3.14` | Orientation tolerance about z (effectively free yaw about the tool axis). |
| `top_down_yaw` | `0.0` | Yaw of the placeholder top-down orientation used for the initial pose; the planner then iterates over orientation candidates. |
| `approach_pitch_below_rad` | `1.0472` | Declared but currently unused. **node-only.** |
| `arm_base_offset_x_m` | `0.09825` | x of the arm's yaw column (`arm_joint1`) in `base_footprint`, from the URDF. Puts candidate grasp yaws on the 5-DOF arm's reachable manifold. |
| `joint_limit_margin_rad` | `0.15` | Reject IK solutions with joints 1-4 within this margin of the ±1.5708 servo limits, where proprioception is unreliable. |
| `gripper_tip_offset_xyz` | `[0.0, 0.0, 0.0]` | Fingertip position in the `arm_link5` frame. Zero because the URDF's `arm_link5` origin already sits at the fingertips; ±0.09 both landed picks one gripper-length off. |

#### Grasp geometry

| Parameter | Default | Description |
|---|---|---|
| `grasp_clearance_m` | `0.005` | Subtracted from the measured object width to get the grip command. **node-only.** |
| `min_grasp_width_m` / `max_grasp_width_m` | `0.005` / `0.060` | Clamp range for the grip width command. **node-only.** |
| `default_grasp_width_m` | `0.045` | Grip width when the bbox width measurement fails. **node-only.** |
| `object_height_fallback_m` | `0.10` *(launch: 0.04)* | Object height when the bbox top/bottom projection fails. |
| `grasp_z_fraction_from_top` | `0.5` | How far down the object the fingertip descends: 0 = top, 0.5 = mid, 1 = bottom. |
| `grasp_z_offset_m` | `0.0` | Extra descent below the perceived target z (positive = lower). Short objects only expose their top surface; hardware uses ~0.016. |
| `grasp_roll_offset_rad` | `0.0` | Wrist (joint 5) roll added to every IK-solved grasp to seat the jaws in the grasping plane. |
| `grasp_tilt_first` | `false` | Try near-horizontal side-grasp orientations before top-down. Needed at the arm's forward reach limit. |
| `grasp_engage_depth_m` | `0.03` | For side grasps only: advance the fingertip target this far along the approach so the object body lands between the fingers rather than at the tips. About one object depth. |
| `pick_lift_m` / `place_lift_m` | `0.06` / `0.06` | Vertical lift after grasp and pre-place height above the destination. |
| `pose_correction_iters` | `2` | After an arm move, compare FK of the joint readback to the target and re-target with the error subtracted, up to this many times. 0 disables. |
| `pose_correction_tol_m` | `0.012` | Fingertip error above which a correction pass runs. |

#### Table floor and printer-bed collision

| Parameter | Default | Description |
|---|---|---|
| `table_z_source` | `perception` | `perception` uses the projected bbox-bottom z as the table height; `param` uses `table_z_m`. |
| `table_z_m` | `0.14` | Table surface height above `base_footprint`. Also the plane for plane ranging. Stale values are dangerous in both directions. |
| `pick_z_safety_m` | `0.10` | The pick fingertip never goes below `table_z + pick_z_safety_m`. Hardware runs use ~0.015. |
| `bed_collision` | `false` | Add a printer-bed slab to the planning scene so planning rejects poses that dip the gripper servo or claws below the bed. Hardware passes `true`. |
| `bed_collision_halfwidth_m` | `0.30` | Half-width (y) of the bed slab. |
| `bed_collision_depth_m` | `0.40` | Depth (x) of the bed slab. |
| `bed_collision_thickness_m` | `0.03` | Thickness of the bed slab. |
| `bed_collision_clearance_m` | `0.012` | The slab top sits this far below the bed surface. Sized with the gripper mesh margin so the proven grasp band stays feasible. |

#### Perception ranging and lidar wall reference

| Parameter | Default | Description |
|---|---|---|
| `pixel_refine` | `true` | Refine Gemini's bbox by segmenting the bright object inside it. Falls back to the raw bbox when segmentation fails. |
| `plane_ranging` | `false` | Derive target x/y by intersecting the bbox pixel ray with the table plane instead of trusting depth. Immune to depth dropouts on white objects. Hardware passes `true`. |
| `plane_object_half_height_m` | `0.015` | Half-height of the object; the bbox-center ray is intersected at mid-height. |
| `lidar_offset_x_m` | `0.10478` | `base_link` to `laser_link` x offset from the URDF; scan points are shifted by it. |
| `lidar_min_range_m` / `lidar_max_range_m` | `0.25` / `2.5` | Scan points outside this range are ignored. |
| `wall_to_target_x_m` | `-1.0` | Taped wall-face to object-near-face distance. When ≥ 0, the vision x is gated against `wall_x + wall_to_target_x_m`. Negative disables. |
| `wall_ref_tol_m` | `0.06` | Warn when the vision x disagrees with the wall-referenced x by more than this. |
| `wall_ref_override` | `false` | Replace the vision x with `wall_x + wall_to_target_x_m`. Only valid for a fixed taped placement. |
| `print_y_m` | `-1.0` | Slicer y of the object center, in meters. ≥ 0 activates slicer-referenced x mode (see "Per-print arguments"). |
| `bed_offset_x_m` | `0.155` | Wall face to bed front edge (the slicer's y=0 line). Taped once. |
| `object_half_depth_m` | `0.015` | Object center to near face, subtracted in the slicer x chain. |
| `wall_camera_check` | `false` | Ask Gemini for the front wall in the same image, plane-range it, and log the camera-vs-lidar wall x delta. |
| `wall_camera_autocal` | `false` | Subtract that delta from the object's plane-ranged x, cancelling shared camera systematics. Requires `wall_camera_check`. |
| `lidar_wall_collision` | `true` | Publish the lidar-detected front wall as a MoveIt collision box before each pick attempt. |
| `lidar_wall_height_m` | `0.13` | Height of the wall collision box. |
| `lidar_wall_base_z_m` | `0.0` | Base z of the wall box. The arena walls stand on the drive surface. |

#### Base drive and search

| Parameter | Default | Description |
|---|---|---|
| `enable_base_drive` | `true` | Allow the chassis to move. `false` picks only from the current spot. |
| `drive_axes` | `y_only` | Which base axes may move: `xy`, `x_only`, `y_only`. Hardware uses `xy` because the Astra's 0.6 m minimum depth range forces perceiving from afar. |
| `drive_mode` | `auto` | `auto` uses odometry when available and falls back to open-loop; `closed_loop` and `open_loop` force one. |
| `drive_odom_wait_sec` | `1.0` | How long to wait for the first odometry message. **node-only.** |
| `drive_kp` | `1.5` | Proportional gain of the closed-loop drive. **node-only.** |
| `drive_max_lin_speed_mps` | `0.10` | Linear speed cap. Hardware runs use 0.05. |
| `drive_max_ang_speed_rps` | `0.3` | Angular speed cap for yaw corrections. |
| `drive_position_tol_m` | `0.01` | Closed-loop drive stops within this position error. |
| `drive_yaw_tol_rad` | `0.01` | Closed-loop yaw tolerance. |
| `drive_timeout_sec` | `15.0` *(launch: 10.0)* | Per-drive timeout. Hardware runs use 35. |
| `drive_settle_sec` | `0.3` | Pause after a drive before the next step. **node-only.** |
| `drive_abort_divergence_m` | `0.10` | Abort a closed-loop drive when the position error grows this much past its best value (catches inverted or mis-scaled odometry). 0 disables. |
| `sweet_x` / `sweet_y` | `0.18` / `0.0` | Legacy drive target: the base-frame point the object is driven to when it lies outside the reach window. |
| `reach_window_x_min` / `reach_window_x_max` | `0.10` / `0.25` | Legacy reach window in x; the base drives if the target is outside it. |
| `reach_window_y_half` | `0.05` | Legacy reach window half-width in y. |
| `base_search_dx_range_m` | `[-0.30, 0.0]` *(launch: [0.0, 0.17])* | Range of forward base offsets scanned for an IK-reachable grasp. Hardware widens to `[0.0, 0.49]`. |
| `base_search_dy_range_m` | `[-0.30, 0.30]` *(launch: [-0.28, 0.23])* | Range of lateral base offsets scanned. |
| `base_search_step_m` | `0.03` | Grid step of the base offset scan. |
| `base_search_order` | `min_reach` | Candidate ordering: `min_reach` prefers the offset that leaves the target at `base_search_ideal_reach_m` from the arm column; `min_drive` prefers the smallest base motion. |
| `base_search_ideal_reach_m` | `0.33` | Horizontal target-to-arm-column distance the `min_reach` ordering aims for. |
| `collision_preflight_candidates` | `5` | Number of ordered IK-reachable base offsets kept for the serialized bed-collision preflight. |
| `reperceive_after_drive` | `true` | Re-run perception after the approach drive. On hardware only useful together with `reperceive_min_target_x_m`. |
| `reperceive_min_target_x_m` | `0.62` | With re-perception on, cap the first drive so the target stays at least this far ahead (outside the camera's minimum range). |
| `wall_drive_gate` | `false` | Cap every forward drive so the chassis front stays clear of the lidar-known front wall. Hardware passes `true`. |
| `chassis_front_x_m` | `0.13` | Chassis front edge from `base_footprint`. |
| `wall_stop_clearance_m` | `0.03` | Gap kept between the chassis front and the wall. |
| `return_after_place` | `true` | Drive back to the run-start odometry pose after placing. |
| `reset_on_failure` | `true` | After a failed run: open the gripper, drive back to the start pose, park the arm at `failure_reset_pose_named`. `false` leaves the robot in place for debugging. |

#### Lidar scan-match drive correction

| Parameter | Default | Description |
|---|---|---|
| `lidar_match_gate_m` | `0.08` | ICP correspondence gate. |
| `lidar_drive_correction` | `false` | Apply one follow-up drive covering the shortfall measured by the scan match. Hardware passes `true`. |
| `lidar_correction_tol_m` | `0.01` | Mismatch below this is not corrected. |
| `lidar_correction_max_m` | `0.05` | Corrections above this are skipped as suspect. |
| `lidar_yaw_correction` | `true` | Also rotate out the measured heading drift. |
| `lidar_yaw_tol_rad` | `0.015` | Yaw error below this is not corrected. |
| `lidar_yaw_max_rad` | `0.12` | Yaw corrections above this are skipped as suspect. |

#### Gripper close and grasp verification

| Parameter | Default | Description |
|---|---|---|
| `close_grip_step_size_rad` | `0.05` | Increment of the closed-loop gripper close. |
| `close_grip_settle_time_s` | `0.10` *(launch: 0.3)* | Wait after each step before reading back the joint. |
| `close_grip_position_error_threshold_rad` | `0.018` | Commanded-minus-actual error that signals contact. Sits just above free-motion noise. |
| `close_grip_movement_threshold_rad` | `0.040` | Minimum actual movement between steps before stall detection applies. |
| `close_grip_extra_grip_step_rad` | `0.06` *(launch: 0.03)* | Extra clamp applied after contact. |
| `close_grip_hold_position_offset_rad` | `0.0` | Offset added to the held position after clamping. |
| `close_grip_timeout_s` | `35.0` *(launch: 50.0)* | Budget for the full close loop (~31 steps at ~1 s each). |
| `grasp_empty_tol_rad` | `0.15` | Empty-grasp gate: fail the pick if the jaws closed this much further than the stop expected from the measured width. |
| `grasp_empty_min_close_rad` | `-0.20` | The empty gate also requires the final grip joint to be at or above this (near fully shut; open is -1.54, closed 0.0). Guards against a noisy width estimate discarding a real grasp. |
| `verify_pick_with_gemini` | `true` | After the lift, show the gripper to the camera and ask Gemini whether the object is held. |
| `verify_pick_required` | `true` | `true` fails the pick on a negative verdict; `false` only logs it. |
| `max_pick_attempts` | `3` *(launch: 5)* | Pick retries (reset gripper, restow, re-perceive) before the run fails. |

#### Destination point

| Parameter | Default | Description |
|---|---|---|
| `destination_point_source` | `box_bias` | `box_bias` picks a point inside Gemini's destination box using the fractions below; `point` uses Gemini's standalone destination point. **node-only.** |
| `destination_box_y_fraction` / `destination_box_x_fraction` | `0.5` / `0.5` | Position inside the destination box (0..1 from top-left). **node-only.** |
| `destination_z_source` | `target` | `target` places at the picked object's z; `fixed` uses `destination_z_fixed`; anything else keeps the projected destination z. |
| `destination_z_fixed` | `0.20` | Place height when `destination_z_source` is `fixed`. |
| `destination_z_max` | `0.25` | Place height is clamped to this. |
| `place_wall_aim` | `false` | Best-effort place toward a container beyond the wall: keep vision y, clamp x into `[wall + place_wall_clear_m, drive budget + place_max_reach_x_m]`. Hardware passes `true`. |
| `place_max_reach_x_m` | `0.38` | Fingertip forward reach the place IK reliably solves at lift height. |
| `place_wall_clear_m` | `0.06` | Minimum object-center distance past the wall face when dropping. |

## Gemini Robotics pick-and-place debug flow (marker-only)

The original debug flow asks Gemini Robotics for image-space
target/destination points, projects those pixels into `base_footprint` through
the perception bridge, and publishes RViz markers without commanding motion
(`execute:=false`).

### Build

From the workspace root:

```bash
cd /home/daniel/yahboom_rosmaster_x3plus
colcon build --base-paths src --packages-select \
  yahboom_rosmaster_msgs \
  gemini_robotics_bridge \
  gemini_pick_place_executor \
  --symlink-install
source install/setup.zsh
```

Use `--base-paths src` so `colcon` does not scan a workspace-local `venv/`.

### Install Gemini SDK

```bash
python3 -m pip install -r src/yahboom_rosmaster/gemini_robotics_bridge/requirements-gemini.txt
export GEMINI_API_KEY="your_api_key_here"
```

### Start the required nodes

Start the camera/perception stack first, including
`ros2 launch x3plus_moveit_config gazebo_moveit.launch.py `

```bash
/perception_bridge/debug_image
/perception_bridge/pixel
/perception_bridge/selected_point_base
```

Then start the Gemini service:

```bash
source install/setup.zsh
export GEMINI_API_KEY="your_api_key_here"
ros2 run gemini_robotics_bridge gemini_robotics_bridge.py
```

In another terminal, run the debug executor:

```bash
source install/setup.zsh
ros2 run gemini_pick_place_executor gemini_pick_place_executor.py
```

The executor defaults to:

```bash
task="put the red can in the blue bin"
image_topic=/perception_bridge/debug_image
marker_topic=/gemini_pick_place/debug_markers
execute=false
```

### RViz

Add a `MarkerArray` display for:

```bash
/gemini_pick_place/debug_markers
```

Marker colors:

- red sphere: target object 3D point
- blue sphere: destination 3D point
- gray line: target-to-destination relationship
- yellow/cyan arrows: candidate lift directions

### Tune the destination point

Gemini returns both a destination point and, usually, a destination box. The
executor defaults to `destination_point_source=box_bias`, which uses a tunable
point inside the destination box instead of the standalone Gemini point. This is
useful when the standalone point lands on a bin wall.

Default center of the destination box:

```bash
ros2 run gemini_pick_place_executor gemini_pick_place_executor.py
```

Bias lower/deeper in the image:

```bash
ros2 run gemini_pick_place_executor gemini_pick_place_executor.py --ros-args \
  -p destination_box_y_fraction:=0.6 \
  -p destination_box_x_fraction:=0.5
```

Use Gemini's original destination point for comparison:

```bash
ros2 run gemini_pick_place_executor gemini_pick_place_executor.py --ros-args \
  -p destination_point_source:=point
```

### One-shot Gemini test client

The local helper script at `~/gemini-test.py` grabs one image, calls
`/gemini_pick_place`, prints the response, and writes an overlay PNG with
Gemini's boxes/points drawn on the exact image sent to the service:

```bash
source /home/daniel/yahboom_rosmaster_x3plus/install/setup.bash
python3 ~/gemini-test.py
```

Look for:

```bash
overlay_path: /home/daniel/.ros/gemini_robotics_bridge/<run_id>/gemini_overlay.png
```

The Gemini bridge logs each request under:

```bash
/home/daniel/.ros/gemini_robotics_bridge/
```

Useful files in each run directory:

```bash
request_response.json
attempt_1.json
scene_<hash>.png
gemini_overlay.png
```

Gcode for the end gcode of the printer (Prusa XL)

```gcode
G0 Z{max(340, max_layer_z)} ; max_layer_z = [max_layer_z] / 340 -> 14cm de la mesa
M190 R30     ; wait for bed to cool to 30C

M77 ; stop print timer
```
