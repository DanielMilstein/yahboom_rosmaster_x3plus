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
