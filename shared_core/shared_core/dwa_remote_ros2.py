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

from quopri import _Input
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
from . import distancetime0_ros2
import json
import signal
import sys
from pynput import keyboard  # for keyboard input detect
import simpleaudio as sa
import threading
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# 统一管理全局变量
human = Twist()
human_r = float("inf")
inputkey = 0
write = 0
yici = 1
human_angle = 0

# 记录控制变量
RECORD = True  # 设置为 True 时记录 rosbag 和使用 save，False 时不使用

# 参数设置（保留原有的参数逻辑）
param_value = 1  # 默认参数值，可以根据需要修改

class Config():
    # simulation parameters

    def __init__(self):
        # robot parameter
        #NOTE good params:
        #NOTE 0.55,0.1,1.0,1.6,3.2,0.15,0.05,0.1,1.7,2.4,0.1,3.2,0.18
        self.max_speed = 0.20  # [m/s]
        self.min_speed = 0.0  # [m/s]
        self.max_yawrate = 0.6  # [rad/s]

        self.max_accel = 2.5  # [m/ss]
        self.max_dyawrate = 3.2  # [rad/ss]
        ##################################################33
        self.v_reso = 0.04  # [m/s]
        self.yawrate_reso = 0.04  # [rad/s]
        #######################################################
        self.dt = 0.4  # [s]
        self.predict_time = 2.4  # [s]
        self.showpredict_time = 3.0  # [s]
        self.showdt = 1.0
        self.goal_radius=0.4
        #########################################
        # self.speed_cost_gain = 1.5 
        # self.obs_cost_gain = 1.0
        # self.to_human_cost_gain =0.5
        if param_value == 1:
            self.to_human_cost_gain = 1.0 #lower = detour
            self.speed_cost_gain = 2.0 #lower = faster
            self.obs_cost_gain = 1.0 #lower z= fearless
        if param_value == 2:
            self.to_human_cost_gain = 1.0 #lower = detour
            self.speed_cost_gain = 1.0 #lower = faster
            self.obs_cost_gain = 2.0 #lower z= fearless
        if param_value == 3:
            self.to_human_cost_gain = 2.0 #lower = detour
            self.speed_cost_gain = 1.0 #lower = faster
            self.obs_cost_gain = 1.0 #lower z= fearless
        #############################
        self.robot_radius = 0.12  # [m]
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.goalX = 0.0
        self.goalY = 0.0
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.distance = 0.0
        self.first_time=True
        self.mindect=0.12
        self.maxdect=1.2
        self.prev_x0= 0.0
        self.prev_y0= 0.0
        self.distance0=0.0
        self.xy = set()
        self.rate = 20.0  # Hz

    # Callback for Odometry
    def assignOdomCoords(self, msg):
        # X- and Y- coords and pose of robot fed back into the robot config
        
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        rot_q = msg.pose.pose.orientation
        (roll,pitch,theta) = \
            euler_from_quaternion ([rot_q.x,rot_q.y,rot_q.z,rot_q.w])
        self.th = theta
        self.xy.add((self.x,self.y))
        if self.first_time:  # 只在第一次执行
            self.prev_x = self.x
            self.prev_y = self.y
            self.prev_x0 =self.x
            self.prev_y0 =self.y
            self.first_time = False  # 之后不会再执行
        self.distance0=math.sqrt((self.x - self.prev_x0)**2 + (self.y - self.prev_y0)**2)
        distancetime0_ros2.odom_callback(self)

    # Callback for attaining goal co-ordinates from Rviz Publish Point #rviz中设置目标点
    def goalCB(self,msg):
        self.goalX = msg.x
        self.goalY = msg.y

class Obstacles():
    def __init__(self):
        # Set of coordinates of obstacles in view
        self.obst = set()
        self.obs1 = set()
        self.obs_last=set()
        self.minx = 3.5
        self.deltx = 0.15
        self.index = 0
        self.pattern = 0

    def assignObs(self, msg, config):
        deg = len(msg.ranges)   # Number of degrees - varies in Sim vs real world
        self.obst = set()   # reset the obstacle set to only keep visible objects

        for i in range(len(msg.ranges)):
            if msg.ranges[i]>config.maxdect or msg.ranges[i] == float('Inf'):
                continue
            elif np.isnan(msg.ranges[i]):
                distance = config.mindect
            elif msg.ranges[i]<config.mindect:
                continue
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

        if len(self.obs_last) - len(self.obst)>40:
            self.obst=self.obs_last | self.obst
        else:
            self.obs_last=self.obst.copy()

    def assignObs1(self, msg, config):
        self.minx=3.5
        for i in range(len(msg.ranges)):
            if msg.ranges[i]>config.maxdect or msg.ranges[i] == float('Inf'):
                distance=config.maxdect
            elif msg.ranges[i]<config.mindect:
                distance=config.maxdect
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

# 移除未使用的全局变量

def simulate_trajectory(
    xinit,
    v,
    y,
    config,
    marker_line=None,
    out_list=None
):
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
    if n==4:# w minus directly
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

def share1(vel_msg,config):# human command get 获取人类指令
    global human
    global human_r
    global inputkey
    global human_angle

    human.linear.x=vel_msg.linear.x
    human.angular.z=vel_msg.angular.z
    
    if human.linear.x<0.0:
        human.linear.x=0.0
    human_angle=human.angular.z
    if human.linear.x<=0.001 and human.linear.x>=-0.001:
        human.linear.x = 0.0
    human_r=math.atan2(human.angular.z,human.linear.x)

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

def change_goal(config,n):
    global yici
    if n==0:
        config.goalX=-4.0
        config.goalY=10.0
    if n==1:
        config.goalX=-3.8
        config.goalY=8.5
    if n==2:
        config.goalX=-4.0
        config.goalY=-3.0
    if n==3:
        config.goalX=-3.7
        config.goalY=5.9
    if n==4:
        config.goalX=-6.0
        config.goalY=6.0
    if n==5:
        config.goalX=6.0
        config.goalY=6.0
    if yici==1:
        print(n)

def signal_handler(signal, frame):
    print("\nCtrl + C  is pressed,exit sys")
    sys.exit(0)  # 退出程序

def listen_key():
    global write

    def on_press(key):
        global write
        try:
            if key.char == 'y':  # 检测键盘输入 y
                write = 1
                print("Key 'y' detected. Variable 'write' set to 1.")
        except AttributeError:
            pass

    # 启动监听器
    with keyboard.Listener(on_press=on_press) as listener:
        listener.join()

class DWARemoteNode(Node):
    def __init__(self):
        super().__init__('dwa_remote_node')
        
        # QoS profiles
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        # Initialize configuration and obstacles
        self.config = Config()
        self.obs = Obstacles()
        
        # Initialize state
        self.x = np.array([self.config.x, self.config.y, self.config.th, 0.0, 0.0])
        self.u = np.array([0.0, 0.0])
        self.speed = Twist()
        
        # Setup subscriptions
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.config.assignOdomCoords, qos_profile)
        self.scan_sub1 = self.create_subscription(
            LaserScan, '/scan', self.obs.assignObs1, qos_profile)
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
        if RECORD:
            self.file_value = distancetime0_ros2.start_rosbag()
        else:
            self.file_value = None
        
        # Start keyboard listener
        threading.Thread(target=listen_key, daemon=True).start()
        
        # Initialize MarkerLine objects for trajectory visualization
        self.marker_line = MarkerLine()
        self.marker_line_human = MarkerLine(color=(0.0, 0.0, 1.0, 1.0)) # Blue for human
        
        self.get_logger().info("DWA Remote Node initialized")

    def share1_callback(self, msg):
        """Callback for human commands"""
        share1(msg, self.config)

    def timer_callback(self):
        """Main control loop"""
        global inputkey, yici, write, markers
        
        if inputkey == 1:
            self.u = dwa_control(self.x, self.u, self.config, self.obs.obst)
            self.x[0] = self.config.x
            self.x[1] = self.config.y
            self.x[2] = self.config.th
            self.x[3] = self.u[0]
            self.x[4] = self.u[1]
            self.speed.linear.x = self.x[3]
            self.speed.angular.z = self.x[4]
            # 轨迹可视化，和dwa_ros2.py一致，使用MarkerLine对象
            self.line_pub.publish(self.marker_line.get_marker())
            self.line_human_pub.publish(self.marker_line_human.get_marker())
        else:
            # if 0 then do directly
            self.speed.linear.x = human.linear.x
            self.speed.angular.z = human.angular.z
        
        if yici > 0:
            self.marker_pub.publish(distancetime0_ros2.markers)
        
        self.cmd_vel_pub.publish(self.speed)
        
        min_d_msg = Float32()
        min_d_msg.data = self.obs.minx
        self.min_d_pub.publish(min_d_msg)
        
        if write == 1 or self.config.x > 2.0:
            if yici > 0:
                self.get_logger().info("YOU have arrive the goal point")
                current_time = self.get_clock().now()
                elapsed_time = (current_time - self.start_time).nanoseconds / 1e9
                self.get_logger().info(f"run time: {elapsed_time:.2f} s")
                self.get_logger().info(f"distance in this time: {self.config.distance:.2f} m")
                
                if RECORD:
                    distancetime0_ros2.save(elapsed_time, self.config.distance, inputkey, param_value)
                    distancetime0_ros2.stop_rosbag()
                
                distancetime0_ros2.play_celebration_sound()
                yici = 0

def main(args=None):
    rclpy.init(args=args)
    global inputkey
    print(__file__ + " start!!")
    print(f"RECORD mode: {RECORD}")
    print(f"Parameter value: {param_value}")
    print("human is 0,share is 1")
    inputs = input()
    if inputs == "0":
        inputkey = 0
    elif inputs == "1":
        inputkey = 1
    else:
        print("input error,run as human")
        inputkey = 0

    node = DWARemoteNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 