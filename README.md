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
├── meshes/
│   └── hokuyo_urg_04lx.dae           # Laser scanner 3D mesh
├── launch/
│   └── barista_urdf.launch.py        # Launch Gazebo, robot, and RViz
├── rviz/
│   └── urdf_vis.rviz                 # RViz configuration with LaserScan display
└── worlds/
    └── barista_empty.world           # Gazebo empty world file
```

## Features

### URDF Model
- Complete robot description with accurate dimensions and inertial properties
- Properly calculated inertia tensors for all components
- 3-DOF articulated caster wheels for realistic physics simulation
- Gazebo material properties and friction coefficients

### Gazebo Plugins
1. **Differential Drive Controller** (`libgazebo_ros_diff_drive.so`)
   - Subscribes to: `/cmd_vel` (geometry_msgs/Twist)
   - Publishes to: `/odom` (nav_msgs/Odometry)
   - Publishes TF: `odom` → `base_footprint`
   - Wheel separation: 0.3m
   - Wheel diameter: 0.0704m

2. **Joint State Publisher** (`libgazebo_ros_joint_state_publisher.so`)
   - Publishes all joint states to `/joint_states`
   - Update rate: 30 Hz

3. **Laser Scanner** (`libgazebo_ros_ray_sensor.so`)
   - Publishes to: `/scan` (sensor_msgs/LaserScan)
   - Scan range: 180° (-90° to +90°)
   - Range: 0.1m to 5.0m
   - 200 samples
   - Update rate: 100 Hz

### RViz Configuration
- Robot model visualization
- LaserScan display with rainbow intensity coloring
- Fixed frame: `base_footprint`
- Properly configured for real-time simulation

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

### Launch Complete Simulation

Launch Gazebo with the robot and RViz visualization:

```bash
ros2 launch barista_robot_description barista_urdf.launch.py
```

This will:
1. Start Gazebo with an empty world
2. Spawn the barista robot at position (0, 0, 0.2)
3. Launch RViz with robot model and laser scan visualization
4. Start all Gazebo plugins (differential drive, laser scanner, joint state publisher)

### Control the Robot

Publish velocity commands to move the robot:

```bash
# Move forward
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"

# Rotate in place
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}"

# Stop
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

### Monitor Topics

View available topics:

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

- `base_footprint`: Ground projection of robot center
- `base_link`: Robot body center (at wheel axis height)
- `left_wheel` / `right_wheel`: Wheel centers
- `front_yaw_link`, `front_roll_link`, `front_pitch_link`: Front caster DOFs
- `back_yaw_link`, `back_roll_link`, `back_pitch_link`: Back caster DOFs
- `cup_holder_tray`: Tray platform
- `laser_scanner`: Laser scanner mounting point
- `laser_scanner_frame`: Laser ray origin point

## Dependencies

- ROS 2 Humble
- Gazebo Classic
- gazebo_ros_pkgs
- robot_state_publisher
- rviz2
- xacro

## Notes

- The robot uses a URDF file (not xacro) with all values explicitly defined
- All inertial properties are calculated using proper physics formulas
- The differential drive plugin is configured for `/cmd_vel` topic control
- Laser scanner provides 180° front-facing coverage
- Caster wheels use 3-DOF articulation for realistic ground contact physics
