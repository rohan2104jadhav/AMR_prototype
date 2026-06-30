# AMR_prototype — Multi-Robot Autonomous Navigation & Object Pursuit (Gazebo Harmonic)

A ROS 2 Humble workspace built around **`nav_bot`**: an Autonomous Mobile Robot (AMR) stack on **Gazebo Harmonic** (`gz sim`, formerly Ignition) featuring SLAM mapping, Nav2-based autonomous navigation, AMCL localization on saved maps, multi-robot spawning, and a YOLOv8-driven **object pursuit** node that detects a target object and autonomously navigates the robot to it.

This is the Gazebo Harmonic / multi-robot evolution of the earlier `nav_bot-gazebo_classic` project — it replaces Gazebo Classic plugins with `ros_gz_bridge` topic bridging and adds Nav2, AMCL, multi-robot namespacing, and perception-driven autonomy.

## What it does

1. Spawns one or more differential-drive robots into a Gazebo Harmonic world.
2. Builds a map of the environment using **SLAM Toolbox** (or loads a previously saved one).
3. Localizes the robot on a saved map using **AMCL** + Nav2's `map_server`.
4. Plans and executes paths to goals using the full **Nav2** stack (planner, controller, behavior server, BT navigator).
5. Optionally runs an **object pursuit** node: YOLOv8 detects a target object class in the robot's RGB-D camera feed, estimates its 3D position, transforms it into the map frame, and sends it to Nav2 as a navigation goal — so the robot autonomously drives toward whatever it's told to look for (defaults to `chair`, fully reconfigurable).

```
                         ┌────────────────────────┐
                         │   Gazebo Harmonic (gz)  │
                         │  world + robot model    │
                         └───────────┬─────────────┘
                                     │ ros_gz_bridge
                 ┌───────────────────┼────────────────────┐
                 ▼                   ▼                    ▼
            /scan (lidar)   /depth_camera/* (RGB-D)   /odom, /tf
                 │                   │                    │
                 ▼                   ▼                    │
          SLAM Toolbox      object_detection.py            │
        (mapping) or         (YOLOv8 + depth → 3D)         │
          AMCL (localize)            │                     │
                 │                   ▼                     │
                 │           NavigateToPose goal            │
                 └──────────────────►│◄────────────────────┘
                                     ▼
                              Nav2 stack
                 (planner_server, controller_server,
                  behavior_server, bt_navigator)
                                     │
                                     ▼
                            /diff_cont/cmd_vel → robot motion
```

## Key Features

- **Gazebo Harmonic (`gz sim`) simulation** — six included worlds: `empty_world`, `obstacle_world`, `office_world`, `small_house_world`, `warehouse`, `warehouse_1` (the warehouse world includes a full NXT-style racking/shelf asset library).
- **Multi-robot support** — `config/robots.yaml` defines named robots with spawn poses and an `enabled` flag; `spawn_robot.launch.py` spawns each one namespaced (e.g. `bot1/`, `bot3/`) with its own bridge, TF tree, and SLAM instance, so robots don't collide on topic names.
- **SLAM Toolbox** mapping (`config/slam_toolbox.yaml`, `config/mapper_params_online_async.yaml`).
- **Nav2 autonomous navigation** — full stack (`navigation_launch.py`) and **AMCL localization** on a saved map (`localization_launch.py`), each adapted from the standard `nav2_bringup` templates with per-namespace parameter rewriting.
- **YOLOv8 object pursuit** (`scripts/object_detection.py`) — detects a configurable target class, computes its 3D position from the aligned RGB-D camera, transforms it to the `map` frame via TF2, applies a standoff distance so the robot doesn't drive into the object, and sends/cancels Nav2 goals as the target moves. Ships with the stock COCO `yolov8n.pt` model out of the box; swap in a custom-trained model via a launch parameter.
- **2D LiDAR + RGB-D depth camera** sensors (`multi_lidar.xacro`, `multi_camera.xacro`, `camera.xacro`, `depth_camera.xacro`) bridged into ROS 2 via `ros_gz_bridge`.
- **Multi-robot keyboard teleop** (`scripts/multi_robot_teleop.py`) — drive all robots together or individually (`1+W/S` for bot1, `3+W/S` for bot3, etc.).
- Example pre-built map (`map/my_map_serial.*`, `map/new_map_*`) ready to use with the localization launch file.

## Prerequisites

- Ubuntu 22.04 + ROS 2 Humble
- **Gazebo Harmonic** (`gz-sim8`), not Gazebo Classic
- Python packages: `ultralytics` (YOLOv8), `opencv-python`, `numpy`

```bash
sudo apt install ros-humble-ros-gz-sim ros-humble-ros-gz-bridge ros-humble-ros-gz-image \
  ros-humble-nav2-bringup ros-humble-slam-toolbox \
  ros-humble-robot-state-publisher ros-humble-joint-state-publisher ros-humble-xacro \
  ros-humble-tf2-ros ros-humble-cv-bridge ros-humble-vision-msgs ros-humble-message-filters

pip install ultralytics opencv-python --break-system-packages
```

## Build

```bash
mkdir -p ~/amr_ws/src
cd ~/amr_ws
git clone https://github.com/rohan2104jadhav/AMR_prototype.git
# repo already contains a `src/nav_bot` package, so either symlink/copy it
# into amr_ws/src, or just build directly from the cloned repo root since
# it already follows the standard colcon workspace layout (src/, map/ at root)
cd AMR_prototype
colcon build --packages-select nav_bot
source install/setup.bash
```

## Usage

### Single-robot SLAM mapping + navigation (Gazebo + Nav2 + SLAM)

```bash
ros2 launch nav_bot start.launch.py
```

This reads `config/robots.yaml`, spawns every robot with `enabled: true`, and for each one launches: `robot_state_publisher`, the Gazebo spawner, a namespaced `ros_gz_bridge`, SLAM Toolbox, the full Nav2 node set (`controller_server`, `planner_server`, `behavior_server`, `bt_navigator`) with a per-namespace lifecycle manager, and RViz. By default, `bot1` and `bot3` are enabled in `config/robots.yaml`.

### Multi-robot bring-up (Gazebo only + manual robot spawning)

```bash
ros2 launch nav_bot multi_robot_main.launch.py
```

Starts Gazebo Harmonic and RViz, then spawns each enabled robot from `config/robots.yaml` (via `spawn_robot.launch.py`) with its own namespaced bridge, TF, and SLAM Toolbox instance, staggered with a 5-second startup delay to let Gazebo stabilize first.

### Drive the robot(s)

```bash
ros2 run nav_bot multi_robot_teleop.py
```

Keyboard control: `W/S`/`A/D`/`Q/E` drive all enabled robots together; prefix with `1` or `3` (e.g. `1` then `W`) to control a single robot; `SPACE` stops all; `X` quits.

### Localize on a saved map (AMCL)

```bash
ros2 launch nav_bot localization_launch.py map:=<path/to/map.yaml> use_sim_time:=true
```

Defaults to the included example map at `map/my_map_serial.yaml` if you don't override the `map` argument. Launches `map_server` + `amcl` + a lifecycle manager.

### Navigation stack only

```bash
ros2 launch nav_bot navigation_launch.py use_sim_time:=true params_file:=src/nav_bot/config/nav2_params.yaml
```

Launches `controller_server`, `smoother_server`, `planner_server`, `behavior_server`, `bt_navigator`, `waypoint_follower`, `velocity_smoother`, and their lifecycle manager — standard `nav2_bringup`-style launch file, adapted for this package.

### Object pursuit (YOLOv8 → Nav2 goal)

```bash
ros2 run nav_bot object_detection.py --ros-args -p target_class:=chair
```

With the SLAM/Nav2 stack and Gazebo running, this subscribes to the synchronized RGB-D camera topics, runs YOLOv8 detection each frame, computes the target's position in the `map` frame, and sends it to Nav2's `navigate_to_pose` action — automatically re-sending or canceling goals as the target moves or the previous goal fails. View live detections with:

```bash
ros2 run rqt_image_view rqt_image_view /yolo/detection_image
```

To use a custom-trained model instead of stock COCO:

```bash
ros2 run nav_bot object_detection.py --ros-args \
  -p model_path:=$HOME/yolo_dataset/runs/detect/train/weights/best.pt \
  -p target_class:=cube
```

## Repository Structure

```
map/                              # example pre-built map + SLAM Toolbox serialized pose graph
├── my_map_serial.pgm / .yaml       # occupancy grid map (for AMCL localization)
├── my_map_save.data / .posegraph   # SLAM Toolbox serialization (for resuming a mapping session)
└── new_map_*                       # second example map

src/nav_bot/
├── description/
│   ├── bot.urdf.xacro              # entry point: includes core, control, lidar, camera
│   ├── bot_core.xacro / robot_core.xacro
│   ├── gz_control.xacro            # Gazebo Harmonic ros2_control plugin binding
│   ├── multi_lidar.xacro           # namespace-aware LiDAR sensor
│   ├── multi_camera.xacro          # namespace-aware RGB-D camera
│   ├── camera.xacro / depth_camera.xacro
│   └── inertial_macros.xacro
├── launch/
│   ├── start.launch.py             # single-call full bring-up: Gazebo + bridge + SLAM + Nav2 (per robots.yaml)
│   ├── multi_robot_main.launch.py  # Gazebo + RViz + staggered multi-robot spawn
│   ├── spawn_robot.launch.py       # spawns one namespaced robot + bridge + SLAM
│   ├── navigation_launch.py        # Nav2 stack only (adapted from nav2_bringup)
│   ├── localization_launch.py      # map_server + AMCL (adapted from nav2_bringup)
│   ├── rsp.launch.py               # robot_state_publisher only
│   └── joystick.launch.py          # joy + teleop_twist_joy
├── scripts/
│   ├── object_detection.py         # YOLOv8 + depth → Nav2 goal (object pursuit node)
│   ├── multi_robot_teleop.py       # keyboard teleop for multiple namespaced robots
│   └── utils.py                    # bridge-yaml namespacing, RViz config templating
├── config/
│   ├── robots.yaml                 # robot names, spawn poses, enabled flags
│   ├── gz_bridge.yaml              # ros_gz_bridge topic template (namespaced per-robot at runtime)
│   ├── nav2_params.yaml / nav2_params1.yaml
│   ├── slam_toolbox.yaml / mapper_params_online_async.yaml
│   ├── twist_mux.yaml, joystick.yaml, my_controllers.yaml
│   └── *.rviz                      # RViz display configs (single + multi-robot)
├── worlds/                         # empty_world, obstacle_world, office_world,
│                                    # small_house_world, warehouse, warehouse_1 (.sdf)
├── models/                         # AWS RoboMaker residential + warehouse Gazebo assets
└── package.xml / CMakeLists.txt
```

## Notes

- This package targets **Gazebo Harmonic**, not Gazebo Classic — sensor and control data flow through `ros_gz_bridge` (`config/gz_bridge.yaml`), not native Gazebo ROS plugins.
- `package.xml` still has placeholder `maintainer`/`license` fields (`TODO: License declaration`) — fill these in before any external use or distribution given the bundled AWS RoboMaker model assets.
- `localization_launch.py` currently defaults the `map` argument to an absolute path (`/home/rohan/cobotx/map/my_map_serial.yaml`); override it with `map:=<your path>` when running on a different machine.
- The `models/` directory contains AWS RoboMaker's residential and warehouse asset packs (furniture, shelving, racks) used to dress out the included worlds — these are large; consider Git LFS or pruning unused assets if repo size becomes a concern.
- `object_detection.py` assumes `use_sim_time:=true` is set consistently across every node (robot_state_publisher, SLAM Toolbox, Nav2) — mismatched clocks are the most common cause of TF extrapolation warnings here.
- A YOLOv8n weights file (`yolov8n.pt`) is included at the repo root; `ultralytics` will also auto-download it if missing.

## License

`package.xml` license field is currently unset — add one (e.g. Apache-2.0, matching the related `nav_bot-gazebo_classic` repo) before treating this as reusable/distributable.