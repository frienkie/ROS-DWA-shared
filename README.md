# ROS‑DWA‑shared

## Overview
ROS implementation of the Dynamic Window Approach (DWA) local planner for robot navigation. Given a global path and costmaps, it outputs velocity commands to steer a mobile base, avoiding obstacles in real time.

## Environment
- Ubuntu 20.04  
- ROS Noetic  

## Installation and Build
```bash
cd ~/catkin_ws/src
git clone https://github.com/frienkie/ROS-DWA-shared.git
cd ~/catkin_ws
catkin build ROS-DWA-shared -DCMAKE_BUILD_TYPE=Release
```

## Usage
```bash
rosrun ROS-DWA-shared dwa_remote.py
```
This will launch the DWA-shared controller.

## Running Demo with Simulator
If using TurtleBot3 in Gazebo:
```bash
# Install TurtleBot3 packages
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ~/catkin_ws
export TURTLEBOT3_MODEL=burger
```

## Topics
- Subscribed: `scan`, `human_vel`
- Published: `/cmd_vel`

## Parameters
Refer to `dynamic_reconfigure` for tuning:  
- Robot config: `acc_lim_x`, `max_vel_x`, `max_vel_theta`
- Scoring: `path_distance_bias`, `goal_distance_bias`, `obstacle_cost_scaling`, …  
- Forward simulation: `sim_time`, `sim_granularity`, …  

## How it works
1. Sample possible velocities within acceleration limits.  
2. Simulate forward trajectories.  
3. Score trajectories based on obstacle proximity, path adherence, goal approaching, and speed.  
4. Send the highest-scoring velocity command.  
5. Repeat continuously.

## References
- D. Fox, W. Burgard, S. Thrun, *The dynamic window approach to collision avoidance*, IEEE RAM (1997)  

## License
BSD‑3‑Clause (same as ROS navigation stack)
