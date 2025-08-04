#!/usr/bin/env python3

# Author: Connor McGuile
# Feel free to use in any way.

# A custom Dynamic Window Approach implementation for use with Turtlebot.
# Obstacles are registered by a front-mounted laser and stored in a set.
# If, for testing purposes or otherwise, you do not want the laser to be used,
# disable the laserscan subscriber and create your own obstacle set in main(),
# before beginning the loop. If you do not want obstacles, create an empty set.
# Implentation based off Fox et al.'s paper, The Dynamic Window Approach to 
# Collision Avoidance (1997).
# ROS2 Version

import rclpy
from rclpy.node import Node
import math
import numpy as np
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker
from distancetime_ros2 import *
import json
import signal
import sys
from pynput import keyboard  # for keyboard input detect
import simpleaudio as sa
import threading
import argparse
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

parser = argparse.ArgumentParser(description="设置参数")

# 添加参数
parser.add_argument("--param", type=int, required=True, help="参数值")

# 解析参数
args = parser.parse_args()  # 解析参数

# =====================
# Global State Variables
# =====================
# Trajectory storage for robot and human
list_x: list = []           # Robot predicted trajectory X
list_y: list = []           # Robot predicted trajectory Y
list_x_human: list = []     # Human predicted trajectory X
list_y_human: list = []     # Human predicted trajectory Y

# Shared control and state
angle_robot: float = 0.0    # Robot's current angle (for cost)
human_angle: float = 0.0    # Human's current angle (for cost)
xh: float = 0.0             # Human trajectory X (final point)
yh: float = 0.0             # Human trajectory Y (final point)
yici: int = 1               # Control flag for goal switching

# Human command and input
human: Twist = Twist()      # Human command (Twist message)
human_r: float = float('inf') # Human radius (for cost)
inputkey: int = 0           # 0: human, 1: shared
write: int = 0              # Flag for writing/recording

class Config:
    """
    Configuration parameters for the robot, DWA, and cost functions.
    All parameters are initialized here for clarity and maintainability.
    """
    def __init__(self):
        # --- Robot parameters ---
        self.max_speed: float = 0.20  # [m/s]
        self.min_speed: float = 0.0   # [m/s]
        self.max_yawrate: float = 0.6  # [rad/s]
        self.max_accel: float = 2.5   # [m/ss]
        self.max_dyawrate: float = 3.2  # [rad/ss]
        self.robot_radius: float = 0.12  # [m]

        # --- DWA resolution and timing ---
        self.v_reso: float = 0.04      # [m/s]
        self.yawrate_reso: float = 0.04  # [rad/s]
        self.dt: float = 0.4           # [s]
        self.predict_time: float = 2.4 # [s]
        self.showpredict_time: float = 3.0  # [s]
        self.showdt: float = 1.0
        self.goal_radius: float = 0.4

        # --- Cost function weights (set by param) ---
        if args.param == 1:
            self.to_human_cost_gain = 1.0  # lower = detour
            self.speed_cost_gain = 2.0     # lower = faster
            self.obs_cost_gain = 1.0       # lower = fearless
        elif args.param == 2:
            self.to_human_cost_gain = 1.0
            self.speed_cost_gain = 1.0
            self.obs_cost_gain = 2.0
        elif args.param == 3:
            self.to_human_cost_gain = 2.0
            self.speed_cost_gain = 1.0
            self.obs_cost_gain = 1.0
        else:
            self.to_human_cost_gain = 1.0
            self.speed_cost_gain = 1.0
            self.obs_cost_gain = 1.0

        # --- State tracking ---
        self.x: float = 0.0
        self.y: float = 0.0
        self.th: float = 0.0
        self.goalX: float = 0.0
        self.goalY: float = 0.0
        self.prev_x: float = 0.0
        self.prev_y: float = 0.0
        self.distance: float = 0.0
        self.first_time: bool = True
        self.mindect: float = 0.12
        self.maxdect: float = 1.2
        self.prev_x0: float = 0.0
        self.prev_y0: float = 0.0
        self.distance0: float = 0.0
        self.xy: set = set()
        self.rate = 20.0  # Hz

    def assignOdomCoords(self, msg: Odometry) -> None:
        """
        Odometry callback: updates robot's position and orientation.
        """
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        rot_q = msg.pose.pose.orientation
        (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
        self.th = theta
        self.xy.add((self.x, self.y))
        if self.first_time:
            self.prev_x = self.x
            self.prev_y = self.y
            self.prev_x0 = self.x
            self.prev_y0 = self.y
            self.first_time = False
        self.distance0 = math.sqrt((self.x - self.prev_x0) ** 2 + (self.y - self.prev_y0) ** 2)
        odom_callback(self)

    def goalCB(self, msg: Point) -> None:
        """
        Callback for receiving goal coordinates from Rviz.
        """
        self.goalX = msg.x
        self.goalY = msg.y

class Obstacles():
    def __init__(self):
        # Set of coordinates of obstacles in view
        self.obst = set()
        self.obs1 = set()
        self.minx = 3.5
        self.deltx = 0.15
        self.index = 0
        self.pattern = 0

    def assignObs(self, msg, config):
        deg = len(msg.ranges)   # Number of degrees - varies in Sim vs real world
        self.obst = set()   # reset the obstacle set to only keep visible objects

        for i in range(len(msg.ranges)):
            if msg.ranges[i]>config.maxdect:
                continue
            elif np.isnan(msg.ranges[i]):
                distance = config.mindect
            else:
                distance = msg.ranges[i]

            #degree and rad
            deg = (360)/len(msg.ranges)
            rad = (2*math.pi)/len(msg.ranges)
            objTheta_rad = rad * i
            objTheta_deg = deg * i

            #local(robot)
            obsX_robo = distance * math.cos(abs(objTheta_rad))
            obsY_robo = distance * math.sin(abs(objTheta_rad))

            #global
            obsX = obsX_robo * math.cos(config.th) - obsY_robo * math.sin(config.th) + config.x
            obsY = obsX_robo * math.sin(config.th) + obsY_robo * math.cos(config.th) + config.y

            # all obastacle data
            self.obst.add((obsX,obsY))
        if config.distance0>self.deltx:
            if self.index == 0:
                self.obs1 = self.obst.copy()
            self.index = (self.index + 1) % 1
            config.prev_x0=config.x
            config.prev_y0=config.y
        if self.pattern==1:
            self.obst= self.obs1 | self.obst

    def assignObs1(self, msg, config):
        self.minx=3.5
        for i in range(len(msg.ranges)):
            if msg.ranges[i]>config.maxdect or msg.ranges[i] == float('Inf'):
                distance=config.maxdect
            elif msg.ranges[i]<config.mindect:
                distance=config.mindect
            elif np.isnan(msg.ranges[i]):
                distance = 0.12
            else:
                distance = msg.ranges[i]
            if self.minx>distance:
                self.minx=distance

# Model to determine the expected position of the robot after moving along trajectory
def motion(x, u, dt):
    # motion model
    # x = [x(m), y(m), theta(rad), v(m/s), omega(rad/s)]
    x[0] += u[0] * math.cos(x[2]) * dt
    x[1] += u[0] * math.sin(x[2]) * dt
    x[2] += u[1] * dt

    return x

# Determine the dynamic window from robot configurations
def calc_dynamic_window(x, config):
    # Dynamic window from robot specification
    Vs = [config.min_speed, config.max_speed,
          -config.max_yawrate, config.max_yawrate]

    # Dynamic window from motion model
    Vd = [x[3] - config.max_accel * config.dt,
          x[3] + config.max_accel * config.dt,
          x[4] - config.max_dyawrate * config.dt,
          x[4] + config.max_dyawrate * config.dt]

    #  [vmin, vmax, yawrate min, yawrate max]
    dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
          max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

    return dw

class MarkerLine:
    """
    Helper class for managing a visualization_msgs/Marker LINE_STRIP.
    Allows easy point addition, clearing, and marker retrieval for publishing.
    """
    def __init__(self, color=(0.0, 1.0, 0.0, 1.0), frame_id: str = "odom", scale: float = 0.1):
        self.marker = Marker()
        self.marker.header.frame_id = frame_id
        self.marker.type = Marker.LINE_STRIP
        self.marker.action = Marker.ADD
        self.marker.scale.x = scale
        self.marker.scale.y = scale
        self.marker.scale.z = scale
        self.marker.color.r = color[0]
        self.marker.color.g = color[1]
        self.marker.color.b = color[2]
        self.marker.color.a = color[3]
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0
        self.marker.pose.position.x = 0.0
        self.marker.pose.position.y = 0.0
        self.marker.pose.position.z = 0.0
        self.clear()

    def add_point(self, x: float, y: float, z: float = 0.0) -> None:
        """Add a point to the marker line."""
        pt = Point()
        pt.x = x
        pt.y = y
        pt.z = z
        self.marker.points.append(pt)

    def clear(self) -> None:
        """Clear all points from the marker line."""
        self.marker.points = []

    def get_marker(self) -> Marker:
        """Return the underlying Marker object for publishing."""
        return self.marker

def simulate_trajectory(
    xinit: np.ndarray,
    v: float,
    y: float,
    config: Config,
    marker_line: MarkerLine = None,
    out_list: list = None
) -> int:
    """
    Simulate and record a predicted trajectory (robot or human).
    Optionally, add points to a MarkerLine for visualization.
    Stores trajectory in out_list if provided.
    Returns the number of points in the trajectory.
    """
    x = np.array(xinit)
    time = 0
    if out_list is not None:
        out_list.clear()
    if marker_line:
        marker_line.clear()
    while time <= config.showpredict_time:
        x = motion(x, [v, y], config.dt)
        if out_list is not None:
            out_list.append(x[0])
        if marker_line:
            marker_line.add_point(x[0], x[1])
        time += config.dt
    if out_list is not None:
        return len(out_list)
    return 0

# Calculate a trajectory sampled across a prediction time
def calc_trajectory(xinit, v, y, config):
    x = np.array(xinit)
    traj = np.array(x)  # many motion models stored per trajectory
    time = 0

    while time <= config.predict_time:
        # store each motion model along a trajectory
        x = motion(x, [v, y], config.dt)
        traj = np.vstack((traj, x))
        time += config.dt # next sample

    return traj

def calc_angle_fromtraj(v, y, config):
    x = np.array([config.x,config.y,config.th,v,y])
    time = 0
    while time <= config.predict_time:
        # store each motion model along a trajectory
        x = motion(x, [v, y], config.dt)
        time += config.dt # next sample

    return x[0],x[1]

# Calculate trajectory, costings, and return velocities to apply to robot
def calc_final_input(x, u, dw, config, ob):
    xinit = x[:]
    max_cost = 0.0
    max_u = u
    max_u[0] = 0.0 #no way can be chosen
    max_u[1] = human.angular.z
    global angle_robot
    # evaluate all trajectory with sampled input in dynamic window
    for v in np.arange(dw[0], dw[1]+config.v_reso, config.v_reso):
        for w in np.arange(dw[2], dw[3]+config.yawrate_reso, config.yawrate_reso):
            w = np.round(w, 2)

            traj = calc_trajectory(xinit, v, w, config)

            to_human_cost = (1-calc_to_human_cost(v,w,config,traj,4)) * config.to_human_cost_gain

            speed_cost = config.speed_cost_gain *(1-abs(human.linear.x - v)/(config.max_speed-config.min_speed))

            ob_cost = calc_obstacle_cost(traj, ob, config) * config.obs_cost_gain

            if np.isinf(ob_cost):
                continue

            final_cost = to_human_cost + speed_cost + ob_cost

            # search minimum trajectory     ##最大代价
            if max_cost <= final_cost:
                max_cost = final_cost
                max_u = [v, w]

    simulate_trajectory(xinit, max_u[0], max_u[1], config)
    simulate_trajectory(xinit, human.linear.x, human.angular.z, config)
    return max_u

# Calculate obstacle cost inf: collision, 0:free
def calc_obstacle_cost(traj, ob, config):
    skip_n = 1
    minr = config.maxdect
    
    # Loop through every obstacle in set and calc Pythagorean distance
    # Use robot radius to determine if collision
    for ii in range(1, len(traj[:, 1]), skip_n):
        for i in ob.copy():
            ox = i[0]           ##障害物
            oy = i[1]
            dx = traj[ii, 0] - ox    ##轨迹
            dy = traj[ii, 1] - oy

            r = math.sqrt(dx**2 + dy**2)  ##距离

            if r <= config.robot_radius:
                return float("-Inf")  # collision

            if minr >= r:
                minr = r

    return (minr-config.robot_radius)/(config.maxdect-config.robot_radius)

def calc_to_human_cost( v, w,config,traj,n):
    global human_angle
    global xr,yr,xh,yh
    if n==1:## previous radius cal method
        if w==0:
            robot_r=float("inf")
        else:
            robot_r=v/w

        if (robot_r< config.v_reso/config.max_yawrate)and w>0:
            robot_r=config.v_reso/config.max_yawrate-0.01
        if (robot_r> -config.v_reso/config.max_yawrate)and w<0:
            robot_r=-config.v_reso/config.max_yawrate+0.01
        if v==0:
            if w>0:
                robot_r=config.v_reso/config.max_yawrate-0.01
            if w<0:
                robot_r=-config.v_reso/config.max_yawrate+0.01
        
        cost = abs(1/human_r - 1/robot_r)/(2*(1/(config.v_reso/config.max_yawrate-0.01)))
    elif n==3:# final point distance method
        xr,yr=traj[-1,0],traj[-1,1]
        cost=math.sqrt((xh-xr)**2 + (yh-yr)**2)/(2*(config.max_speed-config.min_speed)*config.showpredict_time)
    elif n==4:# w minus directly
        robot_angle=w
        cost = abs(human_angle-robot_angle)/(config.max_yawrate*2)
    elif n==5:
        cost=abs(math.atan2(w,v)-human_r)/math.pi

    return cost

# Begin DWA calculations
def dwa_control(x, u, config, ob):
    # Dynamic Window control
    dw = calc_dynamic_window(x, config)
    u = calc_final_input(x, u, dw, config, ob)
    return u

def atGoal(config):
    # check at goal
    if math.sqrt((config.x - config.goalX)**2 + (config.y - config.goalY)**2) \
        <= config.goal_radius:
        return 1
    return 0

def atGoal0(config):
    # check at goal
    if config.x>=4:
        return 1
    return 0

def share1(vel_msg,config):# human command get 获取人类指令
    global human
    global human_r
    global inputkey
    global human_angle
    global xh,yh

    human.linear.x=vel_msg.linear.x
    human.angular.z=vel_msg.angular.z
    
    if human.linear.x<0.0:
        human.linear.x=0.0
    human_angle=human.angular.z
    if human.linear.x<=0.001 and human.linear.x>=-0.001:
        human.linear.x = 0.0
    human_r=math.atan2(human.angular.z,human.linear.x)
    xh,yh=calc_angle_fromtraj(human.linear.x,human.angular.z,config)

class RandomNumberGenerator:
    """
    Generates a sequence of numbers for randomized goal selection.
    """
    def __init__(self):
        self.numbers = list(range(1, 2))  # Can be adjusted for more goals
        self.index = 0
        self.toggle = False
        self.shuffled = True
        self.finished = False

    def get_next(self) -> int:
        global yici
        if not self.shuffled:
            import random
            random.shuffle(self.numbers)
            self.shuffled = True
        if self.index < len(self.numbers):
            current_number = self.numbers[self.index]
            self.index += 1
        else:
            if not self.finished:
                current_number = 1
                yici = 0
                self.finished = True
            else:
                current_number = 1
                yici = 0
        return current_number

def change_goal(config: Config, n: int) -> None:
    """
    Change the robot's goal coordinates based on the provided index.
    """
    global yici
    if n == 0:
        config.goalX = -4.0
        config.goalY = 10.0
    elif n == 1:
        config.goalX = -3.8
        config.goalY = 8.5
    elif n == 2:
        config.goalX = -4.0
        config.goalY = -3.0
    elif n == 3:
        config.goalX = -3.7
        config.goalY = 5.9
    elif n == 4:
        config.goalX = -6.0
        config.goalY = 6.0
    elif n == 5:
        config.goalX = 6.0
        config.goalY = 6.0
    if yici == 1:
        print(n)

def listen_key() -> None:
    """
    Listen for keyboard input in a separate thread. Sets the global 'write' variable to 1 when 'y' is pressed.
    """
    global write
    from pynput import keyboard
    def on_press(key):
        global write
        try:
            if key.char == 'y':
                write = 1
                print("Key 'y' detected. Variable 'write' set to 1.")
        except AttributeError:
            pass
    with keyboard.Listener(on_press=on_press) as listener:
        listener.join()

class DWANode(Node):
    def __init__(self):
        super().__init__('dwa_node')
        
        # QoS profiles
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Initialize configuration and obstacles
        self.config = Config()
        self.obs = Obstacles()
        self.rand = RandomNumberGenerator()
        
        # Initialize state
        self.x = np.array([self.config.x, self.config.y, self.config.th, 0.0, 0.0])
        self.u = np.array([0.0, 0.0])
        self.speed = Twist()
        
        # Initialize marker lines
        self.marker_line = MarkerLine(color=(0.0, 1.0, 0.0, 1.0))
        self.marker_line_human = MarkerLine(color=(0.0, 0.0, 1.0, 1.0))
        
        # Setup subscriptions
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.config.assignOdomCoords, qos_profile)
        self.scan_sub1 = self.create_subscription(
            LaserScan, 'scan', self.obs.assignObs1, qos_profile)
        self.scan_sub = self.create_subscription(
            LaserScan, '/filtered_scan', self.obs.assignObs, qos_profile)
        self.human_sub = self.create_subscription(
            Twist, '/cmd_vel_human', self.share1_callback, qos_profile)
        
        # Setup publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.min_d_pub = self.create_publisher(Float32, '/min_d', 10)
        self.line_pub = self.create_publisher(Marker, '~/line_list', 10)
        self.line_human_pub = self.create_publisher(Marker, '~/line_list_human', 10)
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)
        
        # Setup timer
        self.timer = self.create_timer(1.0/self.config.rate, self.timer_callback)
        
        # Initialize other variables
        self.start_time = self.get_clock().now()
        self.counter = StringMessageCounter()
        self.file_value = start_rosbag()
        
        # Start keyboard listener
        threading.Thread(target=listen_key, daemon=True).start()
        
        # Set initial goal
        change_goal(self.config, self.rand.get_next())
        goal_sphere(self.config)
        
        self.get_logger().info("DWA Node initialized")

    def share1_callback(self, msg):
        """Callback for human commands"""
        share1(msg, self.config)

    def timer_callback(self):
        """Main control loop"""
        global inputkey, yici, write
        
        if inputkey == 1:
            self.u = dwa_control(self.x, self.u, self.config, self.obs.obst)
            self.x[0] = self.config.x
            self.x[1] = self.config.y
            self.x[2] = self.config.th
            self.x[3] = self.u[0]
            self.x[4] = self.u[1]
            self.speed.linear.x = self.x[3]
            self.speed.angular.z = self.x[4]
            simulate_trajectory(self.x, self.x[3], self.x[4], self.config, self.marker_line, list_x)
            self.line_pub.publish(self.marker_line.get_marker())
            simulate_trajectory(self.x, human.linear.x, human.angular.z, self.config, self.marker_line_human, list_x_human)
            self.line_human_pub.publish(self.marker_line_human.get_marker())
        else:
            self.speed.linear.x = human.linear.x
            self.speed.angular.z = human.angular.z
            
        if yici > 0:
            self.marker_pub.publish(markers)
            
        self.cmd_vel_pub.publish(self.speed)
        
        min_d_msg = Float32()
        min_d_msg.data = self.obs.minx
        self.min_d_pub.publish(min_d_msg)
        
        if atGoal(self.config) == 1 or write == 1:
            if yici > 0:
                self.get_logger().info("YOU have arrive the goal point")
                current_time = self.get_clock().now()
                elapsed_time = (current_time - self.start_time).nanoseconds / 1e9
                save(elapsed_time, self.config.distance, self.counter.count_time, inputkey, args.param, "map")
                self.get_logger().info(f"distance in this time: {self.config.distance:.2f} m")
                self.get_logger().info(f"hit time: {self.counter.send_count}")
                with open(f'/home/frienkie/cood/test{self.file_value}.txt', 'w') as f:
                    json.dump(list(self.config.xy), f)
                stop_rosbag()
                change_goal(self.config, self.rand.get_next())
                goal_sphere(self.config)
                play_celebration_sound()

def main(args=None):
    rclpy.init(args=args)
    
    print("which map is used now?")
    chizu = input()
    print("human is 0,share is 1")
    inputs = input()
    if inputs == "0":
        inputkey = 0
    elif inputs == "1":
        inputkey = 1
    else:
        print("input error,run as human")
        inputkey = 0
    
    if chizu == "4":
        # Configure for specific map
        pass
    
    node = DWANode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 