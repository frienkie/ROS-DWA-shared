# ROS2 DWA Shared Control Package

This package contains a ROS2 implementation of the Dynamic Window Approach (DWA) for shared control of a TurtleBot3 robot.

## Overview

The package implements a shared control system where human commands are combined with autonomous obstacle avoidance using the DWA algorithm. The system can operate in two modes:
- **Human control mode (0)**: Direct human control without autonomous assistance
- **Shared control mode (1)**: Human commands are modified by the DWA algorithm to avoid obstacles

## Key Changes from ROS1 to ROS2

### 1. Import Changes
- `rospy` → `rclpy`
- `tf.transformations` → `tf_transformations`
- Added QoS profiles for publishers/subscribers

### 2. Node Structure
- Converted from procedural to object-oriented approach using `rclpy.node.Node`
- Implemented proper ROS2 lifecycle management
- Added proper error handling and cleanup

### 3. Launch Files
- Converted from XML to Python format
- Updated to use ROS2 launch system
- Changed `rviz` to `rviz2`

### 4. Build System
- Updated `package.xml` to format 3
- Converted `CMakeLists.txt` to use `ament_cmake`
- Added `setup.py` for Python package installation

## Installation

1. **Install Dependencies**:
```bash
sudo apt install ros-humble-tf-transformations
pip3 install openpyxl simpleaudio pynput numpy
```

2. **Build the Package**:
```bash
cd ~/ros2_ws
colcon build --packages-select shared_core
source install/setup.bash
```

## Usage

### Running the DWA Node

```bash
# Run with parameter 1 (balanced cost weights)
ros2 run shared_core dwa_node --param 1

# Run with parameter 2 (obstacle avoidance priority)
ros2 run shared_core dwa_node --param 2

# Run with parameter 3 (human command priority)
ros2 run shared_core dwa_node --param 3
```

### Launch Files

```bash
# Launch with RViz2 and filter node
ros2 launch shared_core remote.launch.py model:=burger
```

### Topics

**Subscribed Topics:**
- `/odom` (nav_msgs/Odometry): Robot odometry
- `/scan` (sensor_msgs/LaserScan): Raw laser scan data
- `/filtered_scan` (sensor_msgs/LaserScan): Filtered laser scan data
- `/cmd_vel_human` (geometry_msgs/Twist): Human velocity commands

**Published Topics:**
- `/cmd_vel` (geometry_msgs/Twist): Final velocity commands
- `/min_d` (std_msgs/Float32): Minimum distance to obstacles
- `~/line_list` (visualization_msgs/Marker): Robot trajectory visualization
- `~/line_list_human` (visualization_msgs/Marker): Human trajectory visualization
- `visualization_marker` (visualization_msgs/Marker): Goal marker

## Configuration

### Cost Function Weights

The system uses three cost components that can be weighted differently:

1. **To Human Cost** (`to_human_cost_gain`): How much to follow human commands
2. **Speed Cost** (`speed_cost_gain`): How much to maintain desired speed
3. **Obstacle Cost** (`obs_cost_gain`): How much to avoid obstacles

**Parameter Settings:**
- `--param 1`: Balanced weights (1.0, 2.0, 1.0)
- `--param 2`: Obstacle avoidance priority (1.0, 1.0, 2.0)
- `--param 3`: Human command priority (2.0, 1.0, 1.0)

### Robot Parameters

Key robot parameters can be modified in the `Config` class:
- `max_speed`: Maximum linear velocity (m/s)
- `max_yawrate`: Maximum angular velocity (rad/s)
- `robot_radius`: Robot collision radius (m)
- `goal_radius`: Goal reaching threshold (m)

## Features

### 1. Dynamic Window Approach
- Implements the DWA algorithm for local path planning
- Considers robot dynamics and constraints
- Evaluates multiple trajectories and selects optimal one

### 2. Shared Control
- Combines human commands with autonomous obstacle avoidance
- Maintains human intent while ensuring safety
- Smooth transition between control modes

### 3. Visualization
- Real-time trajectory prediction visualization
- Goal marker display
- Human and robot trajectory comparison

### 4. Data Logging
- Automatic rosbag recording
- Excel data export for analysis
- Distance and time tracking

### 5. Interactive Control
- Keyboard input for stopping recording ('y' key)
- Sound feedback for goal completion
- Configurable goal points

## Troubleshooting

### Common Issues

1. **Import Errors**:
   - Ensure all Python dependencies are installed
   - Check that the package is properly built and sourced

2. **Topic Connection Issues**:
   - Verify that required topics are being published
   - Check QoS settings if using different reliability policies

3. **TF Transform Issues**:
   - Ensure tf2 is running and transforms are available
   - Check frame IDs in visualization markers

### Debug Mode

To run with debug output:
```bash
ros2 run shared_core dwa_node --param 1 --ros-args --log-level debug
```

## File Structure

```
shared_core/
├── src/
│   ├── dwa_ros2.py              # Main DWA node
│   ├── distancetime_ros2.py     # Utility functions
│   └── filter.py                # Laser scan filter
├── launch/
│   └── remote.launch.py         # ROS2 launch file
├── shared_core/
│   ├── __init__.py
│   ├── dwa_ros2.py              # Package version
│   └── distancetime_ros2.py     # Package version
├── resource/
│   └── shared_core              # Package resource file
├── package.xml                  # ROS2 package manifest
├── CMakeLists.txt              # Build configuration
├── setup.py                    # Python package setup
└── README_ROS2.md             # This file
```

## Contributing

When modifying the code:
1. Follow ROS2 coding standards
2. Update documentation for new features
3. Test with different parameter configurations
4. Ensure proper error handling

## License

This package is based on the original ROS1 implementation by Connor McGuile and is provided under the same license terms. 