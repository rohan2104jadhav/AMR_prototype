# AI Coding Agent Instructions for nav_bot

## Project Overview
**nav_bot** is a ROS 2 differential-drive robot simulation package built on Gazebo Classic. It demonstrates robot description, simulation, and visualization using modern ROS 2 tooling.

## Architecture

### Core Components
- **Robot Description (URDF/Xacro)**: `src/nav_bot/description/` defines the robot model
  - `robot.urdf.xacro`: Main entry point that includes core and control definitions
  - `robot_core.xacro`: Geometric structure (chassis, wheels), links, joints, and materials
  - `gazebo_control.xacro`: Simulation plugins—**critical**: defines differential drive plugin with wheel_separation and wheel_diameter parameters
  - `inertial_macros.xacro`: Reusable macros for inertial properties

- **Launch System**: `src/nav_bot/launch/` orchestrates startup
  - `rsp.launch.py`: Robot State Publisher—processes URDF xacro, publishes `/robot_description` topic and TF tree
  - `launch_sim.launch.py`: Main entry point—chains RSP, Gazebo, entity spawner, and RViz
  
- **Configuration**: `src/nav_bot/config/` contains RViz display configurations (`drive_bot.rviz`)
- **Simulation World**: `src/nav_bot/worlds/empty.world` (Gazebo Classic format)

### Data Flows
1. **URDF Parsing**: `rsp.launch.py` processes `robot.urdf.xacro` via xacro library → publishes to `/robot_description` topic
2. **Robot Spawning**: `launch_sim.launch.py` invokes `spawn_entity.py` subscribing to `/robot_description` → spawns model in Gazebo
3. **Control**: Gazebo diff_drive plugin publishes to `/odom` topic, broadcasts `odom→base_link` transform
4. **Visualization**: RViz consumes `/robot_description` and TF broadcasts for visualization

## Build & Execution

### Build System (CMake/colcon)
```bash
cd /home/rohan/articubot
colcon build --packages-select nav_bot
source install/setup.bash
```

### Launching the Simulation
```bash
ros2 launch nav_bot launch_sim.launch.py
```
This starts Gazebo server/client, spawns the robot, launches RSP, and opens RViz.

### Key ROS 2 Topics
- `/robot_description`: URDF XML string (Latched topic)
- `/cmd_vel`: Input twist commands (cmd_vel geometry_msgs/Twist) for differential drive
- `/odom`: Odometry messages from Gazebo diff_drive plugin
- `/tf` and `/tf_static`: TF tree broadcasts (odom→base_link, base_link→wheels)

## Project Conventions

### Naming
- Package name: `nav_bot` (consistent across package.xml, CMakeLists.txt, launch arguments)
- Link names: Snake case with semantic meaning (e.g., `base_link`, `left_wheel`, `right_wheel`)
- Joint names: Semantic with `_joint` suffix (e.g., `left_wheel_joint`)

### Xacro Patterns
- Include pattern: Use `<xacro:include>` to modularize URDF
- Macros: Encapsulate repetitive geometry (inertial properties macro in `inertial_macros.xacro`)
- Gazebo overrides: Separate from core geometry—visual properties in `gazebo_control.xacro`
- Constants: Physics parameters (wheel_separation, wheel_diameter, masses) defined in Gazebo plugin section

### ROS 2 Best Practices in This Codebase
- Simulation time enabled via `use_sim_time` launch arg (defaults to true in launch_sim)
- Robot state publisher publishes TF tree without external bridge (Gazebo Classic handles ROS 2 natively)
- Launch files use Python API with `get_package_share_directory()` for portability

## Critical Integration Points

### Gazebo Diff Drive Plugin
Located in `gazebo_control.xacro`:
- **Left/Right Wheel Joints**: Must match joint names in URDF (`left_wheel_joint`, `right_wheel_joint`)
- **Parameters**:
  - `wheel_separation`: 0.35 m (affects turning radius)
  - `wheel_diameter`: 0.1 m (affects speed calculations)
  - `max_wheel_torque`: 200 N⋅m
  - `max_wheel_acceleration`: 10.0 rad/s²
- **Output Topics**: `/odom`, `/tf` (set `publish_odom_tf=true`)

### URDF-Gazebo Coupling
- `gazebo reference="chassis"` elements must match `<link>` names in core URDF
- Physical dimensions in URDF (box size, cylinder length/radius) affect collision behavior
- Modify inertial properties in `robot_core.xacro` (mass values); Gazebo renders them via plugin

## Common Tasks

### Adding a New Link/Joint
1. Edit `robot_core.xacro`: Add `<link>` and `<joint>` with proper `parent`/`child` references
2. If actuated: Register joint in `gazebo_control.xacro` plugin (e.g., add to diff_drive joint list)
3. Update visual/collision geometries and inertial macros
4. Rerun `colcon build` and verify in RViz

### Modifying Physics Simulation
1. Edit `gazebo_control.xacro` for diff_drive parameters or add new plugins
2. Or modify link mass/dimensions in `robot_core.xacro` (affects inertial properties)
3. Rebuild and relaunch with `ros2 launch nav_bot launch_sim.launch.py`

### Debugging TF Tree
```bash
ros2 run tf2_tools view_frames
# Generates /tmp/frames.pdf showing current TF hierarchy
```

## External Dependencies
- **ROS 2 packages**: `robot_state_publisher`, `gazebo_ros`, `rviz2`, `launch_ros`
- **Gazebo plugins**: `libgazebo_ros_diff_drive.so`, `libgazebo_ros_init.so`, `libgazebo_ros_factory.so`
- **Python libraries**: `xacro`, `ament_index_python`

## Testing & Linting
Package uses standard ROS 2 linting (ament_lint_common) via `find_package(ament_lint_auto)` in CMakeLists.txt. No custom tests currently defined—add in `src/nav_bot/test/` if needed.
