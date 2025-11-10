# barista_robot_description

ROS 2 package containing the URDF description, Gazebo simulation, and RViz visualization for the Barista Robot.

## Robot Description

The Barista Robot is a differential drive mobile robot designed for serving coffee. It features:

- **Cylindrical base body**: 356mm diameter, 155mm height
- **Two motorized wheels**: 35.2mm radius, positioned at 300mm separation
- **Articulated caster wheels**: 3-DOF (yaw, roll, pitch) front and back casters for stability
- **Four standoff rods**: 20mm diameter, 220mm height, connecting base to tray
- **Cup holder tray**: 320mm diameter, 90mm height, cylindrical platform
- **Hokuyo URG laser scanner**: Mounted at the front of the robot for obstacle detection

## Package Structure

```
barista_robot_description/
├── CMakeLists.txt
├── package.xml
├── README.md
├── urdf/
│   └── barista_robot_model.urdf       # Complete robot URDF description
├── xacro/
│   ├── barista_robot_model.urdf.xacro # Parameterized robot model (Checkpoint 7)
│   └── laser_scanner.xacro            # Laser scanner macro with namespace support
├── meshes/
│   └── hokuyo_urg_04lx.dae           # Laser scanner 3D mesh
├── launch/
│   ├── barista_urdf.launch.py        # Single robot launch (Checkpoint 7)
│   └── barista_two_robots.launch.py  # Multi-robot simulation (Checkpoint 8)
├── rviz/
│   ├── urdf_vis.rviz                 # Single robot RViz config (Checkpoint 7)
│   └── two_robots.rviz               # Multi-robot RViz config (Checkpoint 8)
└── worlds/
    └── barista_empty.world           # Gazebo empty world file
```

## Features

### Checkpoint 7: Xacro Parameterization
- **Xacro-based robot model** with parameterized components
- **Laser scanner macro** for modular sensor integration
- **Launch argument support** for conditional laser inclusion
- **Single robot simulation** with proper namespace configuration

### Checkpoint 8: Multi-Robot Simulation
- **Dual robot support** (Rick and Morty) in the same simulation
- **Separate namespaces** for each robot to avoid topic/node conflicts
- **Independent TF trees** with `frame_prefix` for each robot
- **World frame coordination** using static transforms
- **Color differentiation**: Rick (red) and Morty (blue) in both Gazebo and RViz
- **Multi-robot RViz visualization** with proper TF prefixes and laser scan displays
- **Orbit camera view** configured for optimal multi-robot observation

### URDF/Xacro Model
- Complete robot description with accurate dimensions and inertial properties
- Properly calculated inertia tensors for all components
- 3-DOF articulated caster wheels for realistic physics simulation
- Gazebo material properties and friction coefficients
- Conditional color properties based on robot name

### Gazebo Plugins (Namespace-aware)
1. **Differential Drive Controller** (`libgazebo_ros_diff_drive.so`)
   - Subscribes to: `/<robot_name>/cmd_vel` (geometry_msgs/Twist)
   - Publishes to: `/<robot_name>/odom` (nav_msgs/Odometry)
   - Publishes TF: `<robot_name>/odom` → `<robot_name>/base_footprint`
   - Wheel separation: 0.3m
   - Wheel diameter: 0.0704m

2. **Joint State Publisher** (`libgazebo_ros_joint_state_publisher.so`)
   - Publishes all joint states to `/<robot_name>/joint_states`
   - Update rate: 30 Hz

3. **Laser Scanner** (`libgazebo_ros_ray_sensor.so`)
   - Publishes to: `/<robot_name>/scan` (sensor_msgs/LaserScan)
   - Scan range: 180° (-90° to +90°)
   - Range: 0.1m to 5.0m
   - 200 samples
   - Update rate: 100 Hz
   - Frame: `<robot_name>/laser_scanner_frame`

### RViz Configuration
**Single Robot (urdf_vis.rviz):**
- Robot model visualization
- LaserScan display with rainbow intensity coloring
- Fixed frame: `base_footprint`
- Properly configured for real-time simulation

**Multi-Robot (two_robots.rviz):**
- Two robot models with separate TF prefixes (rick, morty)
- Color-coded laser scans (Rick: red, Morty: blue)
- Complete TF tree visualization with world frame
- Axes display for world coordinate reference
- Fixed frame: `world`
- Orbit camera view for optimal multi-robot perspective

## Building the Package

```bash
# Navigate to your workspace
cd ~/ros2_ws

# Build the package
colcon build --packages-select barista_robot_description

# Source the workspace
source install/setup.bash
```

## Usage

### Checkpoint 7: Single Robot Simulation

Launch Gazebo with a single robot and RViz visualization:

```bash
ros2 launch barista_robot_description barista_urdf.launch.py
```

This will:
1. Start Gazebo with an empty world
2. Spawn the barista robot at position (0, 0, 0.2)
3. Launch RViz with robot model and laser scan visualization
4. Start all Gazebo plugins (differential drive, laser scanner, joint state publisher)

**Control the robot:**

```bash
# Move forward
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"

# Rotate in place
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}"

# Stop
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

### Checkpoint 8: Multi-Robot Simulation (Rick and Morty)

Launch Gazebo with two robots and multi-robot RViz visualization:

```bash
ros2 launch barista_robot_description barista_two_robots.launch.py
```

This will:
1. Start Gazebo with an empty world
2. Spawn Rick (red robot) at position (0, 0, 0.05)
3. Spawn Morty (blue robot) at position (1, 1, 0.05)
4. Create static transforms: `world` → `rick/odom` and `world` → `morty/odom`
5. Launch RViz with both robots, color-coded laser scans, and TF tree visualization
6. Start all Gazebo plugins with proper namespacing for each robot

**Control Rick:**

```bash
# Move Rick forward
ros2 topic pub /rick/cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"

# Rotate Rick
ros2 topic pub /rick/cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}"
```

**Control Morty:**

```bash
# Move Morty forward
ros2 topic pub /morty/cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"

# Rotate Morty
ros2 topic pub /morty/cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}"
```

### Monitor Topics

**Single robot topics:**

```bash
# List all topics
ros2 topic list

# Echo odometry
ros2 topic echo /odom

# Echo laser scan data
ros2 topic echo /scan

# Echo joint states
ros2 topic echo /joint_states
```

**Multi-robot topics:**

```bash
# List all topics
ros2 topic list

# Echo Rick's odometry
ros2 topic echo /rick/odom

# Echo Morty's odometry
ros2 topic echo /morty/odom

# Echo Rick's laser scan
ros2 topic echo /rick/scan

# Echo Morty's laser scan
ros2 topic echo /morty/scan

# View TF tree
ros2 run tf2_tools view_frames
```

## Robot Specifications

### Physical Properties

| Component | Mass (kg) | Dimensions |
|-----------|-----------|------------|
| Base body | 10.0 | r=0.178m, h=0.155m |
| Left/Right wheel | 0.5 each | r=0.0352m, l=0.0206m |
| Front/Back caster | 0.001 each | r=0.025m (sphere) |
| Standoff rods | 0.2 each | r=0.01m, h=0.22m |
| Cup holder tray | 2.0 | r=0.16m, h=0.09m |
| Laser scanner | 0.16 | 50×50×70mm (approx) |

### Joint Configuration

- **Motorized wheels**: Continuous rotation joints with axis [0, 1, 0]
- **Caster articulation**:
  - Yaw joint: Axis [0, 0, 1] for rotation around vertical
  - Roll joint: Axis [1, 0, 0] for lateral tilt
  - Pitch joint: Axis [0, 1, 0] for forward/backward tilt
- **All other joints**: Fixed

### Coordinate Frames

**Single Robot:**
- `base_footprint`: Ground projection of robot center
- `base_link`: Robot body center (at wheel axis height)
- `left_wheel` / `right_wheel`: Wheel centers
- `front_yaw_link`, `front_roll_link`, `front_pitch_link`: Front caster DOFs
- `back_yaw_link`, `back_roll_link`, `back_pitch_link`: Back caster DOFs
- `cup_holder_tray`: Tray platform
- `laser_scanner`: Laser scanner mounting point
- `laser_scanner_frame`: Laser ray origin point

**Multi-Robot (with namespacing):**
- `world`: Global reference frame for multi-robot coordination
- `<robot_name>/odom`: Robot's odometry frame (connected to world via static transform)
- `<robot_name>/base_footprint`: Ground projection of robot center
- `<robot_name>/base_link`: Robot body center (at wheel axis height)
- `<robot_name>/left_wheel` / `<robot_name>/right_wheel`: Wheel centers
- `<robot_name>/front_yaw_link`, `<robot_name>/front_roll_link`, `<robot_name>/front_pitch_link`: Front caster DOFs
- `<robot_name>/back_yaw_link`, `<robot_name>/back_roll_link`, `<robot_name>/back_pitch_link`: Back caster DOFs
- `<robot_name>/cup_holder_tray`: Tray platform
- `<robot_name>/laser_scanner`: Laser scanner mounting point
- `<robot_name>/laser_scanner_frame`: Laser ray origin point

**TF Tree Structure (Multi-Robot):**
```
world
├── rick/odom (static)
│   └── rick/base_footprint
│       └── rick/base_link
│           ├── rick/left_wheel
│           ├── rick/right_wheel
│           ├── rick/front_yaw_link → ... → rick/front_pitch_link
│           ├── rick/back_yaw_link → ... → rick/back_pitch_link
│           ├── rick/cup_holder_tray
│           └── rick/laser_scanner
│               └── rick/laser_scanner_frame
└── morty/odom (static)
    └── morty/base_footprint
        └── morty/base_link
            ├── morty/left_wheel
            ├── morty/right_wheel
            ├── morty/front_yaw_link → ... → morty/front_pitch_link
            ├── morty/back_yaw_link → ... → morty/back_pitch_link
            ├── morty/cup_holder_tray
            └── morty/laser_scanner
                └── morty/laser_scanner_frame
```

## Dependencies

- ROS 2 Humble
- Gazebo Classic
- gazebo_ros_pkgs
- robot_state_publisher
- tf2_ros (for static transform publishers)
- rviz2
- xacro

## Notes

### Checkpoint 7 (Single Robot)
- The robot uses both URDF and xacro formats
- All inertial properties are calculated using proper physics formulas
- The differential drive plugin is configured for `/cmd_vel` topic control
- Laser scanner provides 180° front-facing coverage
- Caster wheels use 3-DOF articulation for realistic ground contact physics

### Checkpoint 8 (Multi-Robot)
- **Namespace isolation**: Each robot operates in its own namespace to prevent topic/node conflicts
- **TF frame prefixing**: The `frame_prefix` parameter ensures all TF frames are properly namespaced
- **World frame coordination**: Static transforms connect both robots to a common `world` frame
- **Color differentiation**: Conditional xacro properties assign different colors based on robot_name
  - Rick: Red (Gazebo/Red material)
  - Morty: Blue (Gazebo/Blue material)
- **Synchronized spawning**: TimerAction delays ensure proper initialization order
- **Laser visualization**: RViz displays laser scans in different colors using FlatColor transformer
- **Independent control**: Each robot accepts commands on its own `/robot_name/cmd_vel` topic
- **Scalability**: The launch file pattern can be extended to spawn additional robots by following the same namespace and TF prefix approach

## Checkpoints Summary

**Checkpoint 7: Xacro Conversion and Parameterization**
- Converted URDF to xacro format with modular components
- Created reusable laser scanner macro
- Implemented launch arguments for conditional features
- Single robot simulation with namespace support

**Checkpoint 8: Multi-Robot Simulation**
- Implemented dual robot spawning (Rick and Morty)
- Configured proper namespace and TF frame isolation
- Added world frame with static transform coordination
- Created color-coded visualization in Gazebo and RViz
- Developed comprehensive multi-robot RViz configuration
