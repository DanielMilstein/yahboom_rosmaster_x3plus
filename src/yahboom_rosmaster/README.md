# yahboom_rosmaster #
![OS](https://img.shields.io/ubuntu/v/ubuntu-wallpapers/noble)
![ROS_2](https://img.shields.io/ros/v/jazzy/rclcpp)

Automatic Addison support for the ROSMASTER X3 mecanum wheel robot robot by Yahboom - ROS 2

![ROSMASTER X3 in Gazebo](https://automaticaddison.com/wp-content/uploads/2024/11/gazebo-800-square-mecanum-controller.gif)

![ROSMASTER X3 in RViz](https://automaticaddison.com/wp-content/uploads/2024/11/rviz-800-square-mecanum-controller.gif)

## Gemini Robotics pick-and-place debug flow

This flow is currently debug-only. It asks Gemini Robotics for image-space
target/destination points, projects those pixels into `base_footprint` through
the perception bridge, and publishes RViz markers. It does not command robot
motion.

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
