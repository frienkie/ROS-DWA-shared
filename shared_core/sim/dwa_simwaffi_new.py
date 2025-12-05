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
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker
from cal_ctime import *
import numpy as np
import signal
import sys
from pynput import keyboard  # for keyboard input detect
import simpleaudio as sa
import threading
# import argparse
# parser = argparse.ArgumentParser(description="设置参数")

# 添加参数
# parser.add_argument("--param", type=int, required=True, help="参数值")

# # 解析参数
# args = parser.parse_args()

RECORDER = True

class Config():
    # simulation parameters

    def __init__(self):
        # robot parameter
        #NOTE good params:
        #NOTE 0.55,0.1,1.0,1.6,3.2,0.15,0.05,0.1,1.7,2.4,0.1,3.2,0.18
        self.max_speed = 0.25  # [m/s]
        self.min_speed = 0.0  # [m/s]
        # self.max_yawrate = 1.2  # [rad/s]
        self.max_yawrate = 1.0  # [rad/s]
        self.max_accel = 2.5  # [m/ss]
        self.max_dyawrate = 3.2  # [rad/ss]
        ##################################################33
        self.v_reso = 0.05  # [m/s]
        # self.yawrate_reso = 0.1  # [rad/s]
        self.yawrate_reso = 0.05  # [rad/s]
        #######################################################
        self.dt = 0.4  # [s]
        self.dtsys = 0.05
        self.predict_time = 15.0  # [s]
        self.showpredict_time = 3.0  # [s]
        self.showdt = 1.0
        self.goal_radius=0.4
        #########################################
        # self.speed_cost_gain = 1.5 
        # self.obs_cost_gain = 1.0
        # self.to_human_cost_gain =0.5
        # if args.param == 1:
        #     self.to_human_cost_gain = 1.0 #lower = detour
        #     self.speed_cost_gain = 2.0 #lower = faster
        #     self.obs_cost_gain = 1.0 #lower z= fearless

        self.to_human_cost_gain = 1.0 #lower = detour
        self.speed_cost_gain = 1.0 #lower = faster
        self.obs_cost_gain = 1.0 #lower z= fearless
        # if args.param == 3:
        #     self.to_human_cost_gain = 2.0 #lower = detour
        #     self.speed_cost_gain = 1.0 #lower = faster
        #     self.obs_cost_gain = 1.0 #lower z= fearless
        # #############################
        self.robot_radius = 0.22  # [m]
        self.obs_radius =0.2
        self.cube_half_size=1.0
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
        
        # 保留三位小数精度
        self.x = round(msg.pose.pose.position.x, 3)
        self.y = round(msg.pose.pose.position.y, 3)
        rot_q = msg.pose.pose.orientation
        (roll,pitch,theta) = \
            euler_from_quaternion ([rot_q.x,rot_q.y,rot_q.z,rot_q.w])
        # 保留三位小数精度
        self.th = round(theta, 3)
        self.xy.add((self.x,self.y))




    # Callback for attaining goal co-ordinates from Rviz Publish Point #rviz中设置目标点
    def goalCB(self,msg):
        self.goalX = msg.point.x
        self.goalY = msg.point.y

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
        # self.obstacle_list = []  # 先用列表存储，再一次性加入 set


    def assignObs(self, msg, config):

        deg = len(msg.ranges)   # Number of degrees - varies in Sim vs real world
        # print("Laser degree length {}".format(deg))
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
            # elif self.index == 1:
            #     self.obs2 = self.obst.copy()
            self.index = (self.index + 1) % 1
            config.prev_x0=config.x
            config.prev_y0=config.y
        if self.pattern==1:
            self.obst= self.obs1 | self.obst

        if len(self.obs_last) - len(self.obst)>50:
            self.obst=self.obs_last | self.obst
        else:
            self.obs_last=self.obst.copy()



        
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
def calc_final_input(x, u, dw, config, ob,cube):
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

            # traj = calc_trajectory(xinit, v, w, config)

            to_human_cost = (1-calc_to_human_cost(v,w,config,4)) * config.to_human_cost_gain

            speed_cost = config.speed_cost_gain *(1-abs(human.linear.x - v)/(config.max_speed-config.min_speed))

            # ob_cost = calc_obstacle_cost(traj, ob, config,readob) * config.obs_cost_gain

            ob_movecost=(cube,config.x, config.y, config.th, v, w, config.predict_time)

            if ob_movecost==0:
                continue

            final_cost = to_human_cost + speed_cost + ob_movecost/config.predict_time

            # search minimum trajectory     ##最大代价
            if max_cost <= final_cost:
                max_cost = final_cost
                max_u = [v, w]
    # if max_u[0]==0.0:
    #     max_u[1]=human.angular.z
    show_trajectory(xinit, max_u[0], max_u[1], config)
    show_trajectory_human(xinit, human.linear.x, human.angular.z, config)
    return max_u
#################################################################################



# Calculate obstacle cost inf: collision, 0:free
def calc_obstacle_cost(traj, ob, config,readob):
    skip_n = 1
    minr = config.maxdect
    moving_future = readob.moving_future
    obs_info=readob.obs_info
    # 安全检查：如果 moving_future 为 None 或为空，则跳过移动障碍物检查
    if moving_future is None or moving_future.shape[0] == 0:
        num_obs = 0
    else:
        num_obs = moving_future.shape[0]

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

    for obs_i in range(num_obs):
        mob_traj = moving_future[obs_i]    # shape = (num_steps, 2)
        mob_radius = obs_info[obs_i, 4]    # 半径

        for t_i in range(mob_traj.shape[0]):
            dx = traj[t_i, 0] - mob_traj[t_i, 0]
            dy = traj[t_i, 1] - mob_traj[t_i, 1]
            dist = np.hypot(dx, dy)-mob_radius

            if dist <= config.robot_radius:
                return float("-Inf")
                # print(f"Collision at obs {obs_i}, timestep {t_i}")
            if minr>dist:
                minr=dist

    return (minr-config.robot_radius)/(config.maxdect-config.robot_radius)

############################################################################
def calc_to_human_cost( v, w,config,n):
    global human_angle

    if n==4:# w minus directly
        robot_angle=w
        cost = abs(human_angle-robot_angle)/(config.max_yawrate*2)
    elif n==5:
        cost=abs(math.atan2(w,v)-human_r)/math.pi

    return cost
##################################################################################

# Begin DWA calculations
def dwa_control(x, u, config, ob,cube):
    # Dynamic Window control

    dw = calc_dynamic_window(x, config)

    u = calc_final_input(x, u, dw, config, ob,cube)

    return u

def atGoal(config):
    # check at goal
    if math.sqrt((config.x - config.goalX)**2 + (config.y - config.goalY)**2) \
        <= config.goal_radius:
        return 1
    return 0


human_angle=0
yici=1

def share1(vel_msg,config):# human command get 获取人类指令
    global human
    global human_r
    global inputkey
    global human_angle

    human.linear.x=vel_msg.linear.x
    human.angular.z=vel_msg.angular.z
    
    # if human.linear.x<-0.08:
    #     human.linear.x=-0.08

    if human.linear.x<0.0:
        human.linear.x=0.0
    human_angle=human.angular.z
    if human.linear.x<=0.001 and human.linear.x>=-0.001:
        human.linear.x = 0.0
    # human_angle=cal_angle(human.linear.x,human.angular.z)
    human_r=math.atan2(human.angular.z,human.linear.x)





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
    global yici
    global write
    print(__file__ + " start!!")
    print("human is 0,share is 1")
    #inputkey=1
    inputs=input()
    if inputs=="0":
            inputkey=0
    elif inputs=="1":
            inputkey=1
    else:
            rospy.loginfo("input error,run as human")
            inputkey = 0

    # robot specification
    # rand=RandomNumberGenerator()
    config = Config()
    # position of obstacles
    obs = Obstacles()
    cube=CubeReader(config)
    # if chizu=="4":
    #     config.robot_radius=0.106
    #     obs.pattern=1
    #     print("changed OK")

    threading.Thread(target=listen_key, daemon=True).start()

    subOdom = rospy.Subscriber("/odom", Odometry, config.assignOdomCoords)
    subLaser = rospy.Subscriber("/filtered_scan", LaserScan, obs.assignObs, config)

    sub_hum = rospy.Subscriber("/cmd_vel_human",Twist,share1,config,queue_size=1)

    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    x_value_pub = rospy.Publisher('/min_d', Float32, queue_size = 1)
    # sub_obs = rospy.Subscriber("/gazebo/base_collision",Contact,StringMessageCounter.callbackobs,queue_size=10)
    pub_line = rospy.Publisher('~line_list', Marker, queue_size=10)
    pub_line_human = rospy.Publisher('~line_list_human', Marker, queue_size=10)
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    
    speed = Twist()
    # change_goal(config,rand.get_next())
    # goal_sphere(config)
    # initial state [x(m), y(m), theta(rad), v(m/s), omega(rad/s)]
    x = np.array([config.x, config.y, config.th, 0.0, 0.0])
    # initial linear and angular velocities
    u = np.array([0.0, 0.0])

    # if RECORDER==True:
    #     file_value=start_rosbag()
    #     print("You can press y to stop and save rosbag when you need.")
    #     start_time = rospy.get_time()
    # runs until terminated externally
    while not rospy.is_shutdown():

        if (inputkey== 1):
            u = dwa_control(x, u, config, obs.obst,cube)
            x[0] = config.x
            x[1] = config.y
            x[2] = config.th
            x[3] = u[0]
            x[4] = u[1]
            speed.linear.x = x[3]
            speed.angular.z = x[4]
            line(line_num)
            pub_line.publish(marker)
            h_line(line_num_human)
            pub_line_human.publish(h_marker)
        
        else:
            # if 0 then do directly
            speed.linear.x = human.linear.x
            speed.angular.z = human.angular.z
        # if yici>0:
        #     marker_pub.publish(markers)
        pub.publish(speed)
        x_value_pub.publish(obs.minx)
        if write==1:
            if yici>0:
                print("YOU have arrive the goal point")

                # with open(f'/home/frienkie/cood/test{file_value}.txt', 'w') as f:
                #     json.dump(list(config.xy), f)

                # change_goal(config,rand.get_next())
                # goal_sphere(config)
                yici=0
            
        config.r.sleep()


if __name__ == '__main__':
    rospy.init_node('shared_dwa')
    main()