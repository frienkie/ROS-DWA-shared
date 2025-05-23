#!/usr/bin/env python

# Author: Connor McGuile
# Feel free to use in any way.

# A custom Dynamic Window Approach implementation for use with Turtlebot.
# Obstacles are registered by a front-mounted laser and stored in a set.
# If, for testing purposes or otherwise, you do not want the laser to be used,
# disable the laserscan subscriber and create your own obstacle set in main(),
# before beginning the loop. If you do not want obstacles, create an empty set.
# Implentation based off Fox et al.'s paper, The Dynamic Window Approach to 
# Collision Avoidance (1997).
import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker
from distancetime import *
import json
import signal
import sys
from pynput import keyboard  # for keyboard input detect
import threading



# 添加参数

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

        self.to_human_cost_gain = 1.0 #lower = detour
        self.speed_cost_gain = 2.0 #lower = faster
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
        self.r = rospy.Rate(20)

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
        odom_callback(self)
        # print("当前now: %.2f , %.2f", self.prev_x,self.prev_y)



    # Callback for attaining goal co-ordinates from Rviz Publish Point #rviz中设置目标点
    def goalCB(self,msg):
        self.goalX = msg.point.x
        self.goalY = msg.point.y

class getObstacles():
    def __init__(self):
        # Set of coordinates of obstacles in view
        self.obst = set()
        self.obs1 = set()
        # self.obs2 = set()
        self.minx = 3.5
        self.deltx = 0.15
        self.index = 0

    def generate_points(self,start, end, step=0.01):
        points = []
        distance = np.linalg.norm([end.x - start.x, end.y - start.y])  # 计算线段长度
        num_points = int(distance / step)  # 计算要生成的点数
        points.append((round(end.x, 3), round(end.y, 3)))
        for i in range(num_points):  # +1 确保终点包含在内
            if num_points==0:
                break
            x = start.x + (end.x - start.x) * (i / num_points)
            y = start.y + (end.y - start.y) * (i / num_points)
            points.append((round(x, 3), round(y, 3)))  # 保留三位小数，提高 set 兼容性

        return points
    def callback(self,msg):
        # rospy.loginfo("Received Obstacles Message")
        self.obst = set()
        for segment in msg.segments:
            p1, p2 = segment.first_point, segment.last_point
            segment_points = self.generate_points(p1, p2)
            self.obst.update(segment_points)  # 存入 set 集合，自动去重
            # rospy.loginfo(f"Segment ({p1.x}, {p1.y}) -> ({p2.x}, {p2.y}) added {len(segment_points)} points")

    def assignObs(self, msg, config):

        deg = len(msg.ranges)   # Number of degrees - varies in Sim vs real world
        # print("Laser degree length {}".format(deg))
        self.obst = set()   # reset the obstacle set to only keep visible objects
        self.minx=min(msg.ranges)
        if self.minx>3.5:
            self.minx=3.5
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
            # elif self.index == 1:
            #     self.obs2 = self.obst.copy()
            self.index = (self.index + 1) % 1
            config.prev_x0=config.x
            config.prev_y0=config.y
        #self.obst= self.obs1 | self.obst

    def assignObs1(self, msg, config):

        deg = len(msg.ranges)   # Number of degrees - varies in Sim vs real world
        # print(deg)
        # print("Laser degree length {}".format(deg))
        self.obs1 = set()   # reset the obstacle set to only keep visible objects
        self.minx=min(msg.ranges)
        
        if self.minx>3.5:
            self.minx=3.5
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
            self.obs1.add((obsX,obsY))
        # if config.distance-config.prev_distance>self.deltx:
        #     if self.index == 0:
        #         self.obs1 = self.obst.copy()
        #     # elif self.index == 1:
        #     #     self.obs2 = self.obst.copy()
        #     self.index = (self.index + 1) % 1
        #     config.prev_distance=config.distance
        # self.obst= self.obs1 | self.obst


# Model to determine the expected position of the robot after moving along trajectory
def motion(x, u, dt):
    # motion model
    # x = [x(m), y(m), theta(rad), v(m/s), omega(rad/s)]
    # x[2] += u[1] * dt
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

list_x=[]
list_y=[]
line_num=0
list_x_human=[]
list_y_human=[]
line_num_human=0


def show_trajectory(xinit, v, y, config):
    global line_num
    x = np.array(xinit)
    time = 0
    list_x.clear()
    list_y.clear()

    while time <= config.showpredict_time:
        # store each motion model along a trajectory
        x = motion(x, [v, y], config.dt)
        list_x.append(x[0])
        list_y.append(x[1])
        time += config.dt # next sample
    line_num=len(list_x)

def show_trajectory_human(xinit, v, y, config):
    global line_num_human
    x = np.array(xinit)
    time = 0
    list_x_human.clear()
    list_y_human.clear()

    while time <= config.showpredict_time:
        # store each motion model along a trajectory
        x = motion(x, [v, y], config.dt)
        list_x_human.append(x[0])
        list_y_human.append(x[1])
        time += config.dt # next sample
    line_num_human=len(list_x_human)

angle_robot=0.0
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


################################################################################



# Calculate trajectory, costings, and return velocities to apply to robot
def calc_final_input(x, u, dw, config, ob):
    # global list_x,list_y,line_num
    xinit = x[:]######
    max_cost = 0.0
    max_u = u
    max_u[0] = 0.0 #no way can be chosen
    max_u[1] = human.angular.z
    global angle_robot
    # evaluate all trajectory with sampled input in dynamic window
    for v in np.arange(dw[0], dw[1]+config.v_reso, config.v_reso):
        for w in np.arange(dw[2], dw[3]+config.yawrate_reso, config.yawrate_reso):
            w = np.round(w, 2)
            if (v==0.0 and w!=0.0):
                continue

            traj = calc_trajectory(xinit, v, w, config)
            
            # calc costs with weighted gains
            to_human_cost = (1-calc_to_human_cost(v,w,config,traj,5)) * config.to_human_cost_gain

            speed_cost = config.speed_cost_gain *(1-abs(human.linear.x - v)/(config.max_speed-config.min_speed))

            ob_cost = calc_obstacle_cost(traj, ob, config) * config.obs_cost_gain

            if np.isinf(ob_cost):
                continue

            final_cost = to_human_cost + speed_cost + ob_cost
            # if (v==0.0 and w==0.0):
            #     print(final_cost)

            # search minimum trajectory     ##最大代价
            if max_cost <= final_cost:
                max_cost = final_cost
                max_u = [v, w]
    if max_u[0]==0.0:
        max_u[1]=human.angular.z
    show_trajectory(xinit, max_u[0], max_u[1], config)
    show_trajectory_human(xinit, human.linear.x, human.angular.z, config)
    return max_u
#################################################################################



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


    return (minr-config.mindect)/(config.maxdect-config.mindect)

############################################################################333
def calc_to_human_cost( v, w,config,traj,n):
    global human_angle
    global xr,yr,xh,yh
    if n==1:## previous radius cal method
        if w==0:
            robot_r=float("inf")
        else:
            robot_r=v/w
        
        #cost =abs(np.tanh(robot_r/10)-np.tanh(human_r/10))

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
##################################################################################

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


human_angle=0
xh=0
yh=0
yici=1

def share1(vel_msg,config):# human command get 获取人类指令
    global human
    global human_r
    global inputkey
    global human_angle
    global xh,yh

    

    human.linear.x=vel_msg.linear.x
    
    # if human.linear.x<-0.08:
    #     human.linear.x=-0.08

    if human.linear.x<0.0:
        human.linear.x=0.0
    # if vel_msg.angular.z == 0.0 and str(vel_msg.angular.z) == '-0.0' :
    #     vel_msg.angular.z=0.0
    human.angular.z=vel_msg.angular.z
    human_angle=human.angular.z

    if human.linear.x<=0.001 and human.linear.x>=-0.001:
        human.linear.x = 0.0
    
    # human_angle=cal_angle(human.linear.x,human.angular.z)
    # print(human.linear.x,human.angular.z)
    human_r=math.atan2(human.angular.z,human.linear.x)
    # print(human_r)
    xh,yh=calc_angle_fromtraj(human.linear.x,human.angular.z,config)




marker =  Marker()
marker.header.frame_id = "odom"
marker.type = marker.LINE_STRIP
marker.action = marker.ADD
    # marker scale
marker.scale.x = 0.1
marker.scale.y = 0.1
marker.scale.z = 0.1

    # marker color #green
marker.color.a = 1.0
marker.color.r = 0.0
marker.color.g = 1.0
marker.color.b = 0.0

    # marker orientaiton
marker.pose.orientation.x = 0.0
marker.pose.orientation.y = 0.0
marker.pose.orientation.z = 0.0
marker.pose.orientation.w = 1.0

    # marker position  ###no influence in line
marker.pose.position.x = 0.0
marker.pose.position.y = 0.0
marker.pose.position.z = 0.0

h_marker =  Marker()
h_marker.header.frame_id = "odom"
h_marker.type = h_marker.LINE_STRIP
h_marker.action = h_marker.ADD
    # marker scale
h_marker.scale.x = 0.1
h_marker.scale.y = 0.1
h_marker.scale.z = 0.1

    # marker color #green
h_marker.color.a = 1.0
h_marker.color.r = 0.0
h_marker.color.g = 0.0
h_marker.color.b = 1.0

    # marker orientaiton
h_marker.pose.orientation.x = 0.0
h_marker.pose.orientation.y = 0.0
h_marker.pose.orientation.z = 0.0
h_marker.pose.orientation.w = 1.0

    # marker position  ###no influence in line
h_marker.pose.position.x = 0.0
h_marker.pose.position.y = 0.0
h_marker.pose.position.z = 0.0

def line(num):
    # marker line points
    marker.points = []

    for i in range(num):
        exec(f"line_point{i} = Point()") 
        exec(f"line_point{i}.x = list_x[{i}]")
        exec(f"line_point{i}.y = list_y[{i}]")
        exec(f"line_point{i}.z = 0.0")
        exec(f"marker.points.append(line_point{i})")

def h_line(num):
    # marker line points
    h_marker.points = []

    for i in range(num):
        exec(f"line_point{i} = Point()") 
        exec(f"line_point{i}.x = list_x_human[{i}]")
        exec(f"line_point{i}.y = list_y_human[{i}]")
        exec(f"line_point{i}.z = 0.0")
        exec(f"h_marker.points.append(line_point{i})")

class RandomNumberGenerator:
    def __init__(self):
        # self.numbers = list(range(4, 6))  # 数字列表
        self.numbers = list(range(1, 2))
        self.index = 0  # 当前索引
        self.toggle = False  # 控制交替返回数字和 0
        self.shuffled = True  # 是否已打乱数字列表
        self.finished = False  # 标志数字列表是否已完全遍历

    def get_next(self):
        global yici

        # 如果尚未打乱数字列表，进行随机排列
        if not self.shuffled:
            random.shuffle(self.numbers)
            self.shuffled = True

        # 如果数字列表尚未完全遍历
        if self.index < len(self.numbers):
            # if self.toggle:
            #     current_number = 0  # 返回间隔 0
            #     yici = 1  # 间隔 0 时 yici 为 1
            # else:
                current_number = self.numbers[self.index]  # 返回当前数字
                self.index += 1
            # self.toggle = not self.toggle
        else:
            # 所有数字生成完
            if not self.finished:
                current_number = 1  # 最后的间隔 0
                yici = 0  # 此时 yici 为 0
                self.finished = True  # 标志遍历结束
            else:
                current_number = 1  # 持续返回 0
                yici = 0  # 列表完全遍历完后 yici 为 0

        return current_number



def change_goal(config,n):
    global yici
    if n==0:
        config.goalX=-3.6
        config.goalY=-3.0
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

human=Twist()
human_r=float("inf")
inputkey=0
write=0

def signal_handler(signal, frame):
    print("\nCtrl + C  is pressed,exit sys")
    sys.exit(0)  # 退出程序

# 绑定 SIGINT 信号（Ctrl + C）到 signal_handler
signal.signal(signal.SIGINT, signal_handler)

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

def main():
    global write
    global yici

    print(__file__ + " start!!")
    print("human is 0,share is 1")
    inputs=input()
    # inputs="1"
    if inputs=="0":
            inputkey=0
    elif inputs=="1":
            inputkey=1
    else:
            rospy.loginfo("input error,run as human")
            inputkey = 0
    # robot specification
    rand=RandomNumberGenerator()
    config = Config()
    # position of obstacles
    obs = getObstacles()
    # if inputs=="0":
    counter = StringMessageCounter()

    # model_name = "turtlebot3_burger"
    model_name = "turtlebot3"
    # 指定目标位置和方向 (四元数)
    target_position = [-3.6, -3.0, 0.0]  # x, y, z
    target_orientation = [0.0, 0.0, 0.707,  0.707]  # x, y, z, w
    # target_position = [0.0, 0.0, 0.0]  # x, y, z
    # target_orientation = [0.0, 0.0, 0.0, 0.0]  # x, y, z, w
    set_robot_position(model_name, target_position, target_orientation)

    threading.Thread(target=listen_key, daemon=True).start()

    subOdom = rospy.Subscriber("/odom", Odometry, config.assignOdomCoords)
    # subLaser = rospy.Subscriber("/scan", LaserScan, obs.assignObs, config)
    # subLaser = rospy.Subscriber("/filtered_scan", LaserScan, obs.assignObs1, config)
    subLaser = rospy.Subscriber("/filtered_scan", LaserScan, obs.assignObs, config)
    # obssub = rospy.Subscriber("/raw_obstacles", Obstacles, obs.callback)

    sub_hum = rospy.Subscriber("/cmd_vel_human",Twist,share1,config,queue_size=1)

    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    x_value_pub = rospy.Publisher('/min_d', Float32, queue_size = 1)
    # sub_obs = rospy.Subscriber("/gazebo/base_collision",Contact,StringMessageCounter.callbackobs,queue_size=10)
    pub_line = rospy.Publisher('~line_list', Marker, queue_size=10)
    pub_line_human = rospy.Publisher('~line_list_human', Marker, queue_size=10)
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    
    speed = Twist()
    change_goal(config,rand.get_next())
    goal_sphere(config)
    # initial state [x(m), y(m), theta(rad), v(m/s), omega(rad/s)]
    x = np.array([config.x, config.y, config.th, 0.0, 0.0])
    # initial linear and angular velocities
    u = np.array([0.0, 0.0])

    # file_value=start_rosbag()
    print("You can press y to stop and save rosbag when you need.")
    # start_time = rospy.get_time()
    # runs until terminated externally
    while not rospy.is_shutdown():
        # global yici
        
        if (inputkey== 1):
            u = dwa_control(x, u, config, obs.obst)
            x[0] = config.x
            x[1] = config.y
            x[2] = config.th
            x[3] = u[0]
            x[4] = u[1]
            speed.linear.x = x[3]
            speed.angular.z = x[4]
            # speed.linear.x = 0.04
            # speed.angular.z =0.6
            line(line_num)
            pub_line.publish(marker)
            h_line(line_num_human)
            pub_line_human.publish(h_marker)
        
        else:
            # if 0 then do directly
            speed.linear.x = human.linear.x
            speed.angular.z = human.angular.z
            # if speed.linear.x==0.0:
            #     speed.angular.z=0.0

        marker_pub.publish(markers)
        pub.publish(speed)
        x_value_pub.publish(obs.minx)

        if write==1:

            if yici>0:
                print("YOU have arrive the goal point")
                # save(get_time(start_time),config.distance,counter.send_count,inputkey,args.param,chizu)
                # print("distance in this time: %.2f m" % config.distance)
                # print("hit time: %d " % counter.send_count)
                with open(f'/home/frienkie/cood/obs.txt', 'w') as f:
                    json.dump(list(obs.obst), f)
                with open(f'/home/frienkie/cood/obs1.txt', 'w') as f:
                    json.dump(list(obs.obs1), f)
                # with open(f'/home/frienkie/cood/obs2.txt', 'w') as f:
                #     json.dump(list(obs.obs2), f)
                # stop_rosbag()
                with open('/home/frienkie/cood/test0.txt', 'w') as f:
                    json.dump(list(config.xy), f)
                change_goal(config,rand.get_next())
                goal_sphere(config)
                write=0
                yici=1
                
            
        config.r.sleep()


if __name__ == '__main__':
    rospy.init_node('shared_dwa')
    main()