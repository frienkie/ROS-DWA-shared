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
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker
from distancetime import *
import json
import argparse
parser = argparse.ArgumentParser(description="设置参数")

# 添加参数
parser.add_argument("--param", type=int, required=True, help="参数值")

# 解析参数
args = parser.parse_args()

class Config():
    # simulation parameters

    def __init__(self):
        # robot parameter
        #NOTE good params:
        #NOTE 0.55,0.1,1.0,1.6,3.2,0.15,0.05,0.1,1.7,2.4,0.1,3.2,0.18
        self.max_speed = 0.20  # [m/s]
        self.min_speed = 0.0  # [m/s]
        self.max_yawrate = 0.6  # [rad/s]

        self.max_accel = 1.0  # [m/ss]
        self.max_dyawrate = 2.8  # [rad/ss]
        ##################################################33
        self.v_reso = 0.04  # [m/s]
        self.yawrate_reso = 0.04  # [rad/s]
        #######################################################
        self.dt = 0.5  # [s]
        self.predict_time = 3.0  # [s]
        self.showpredict_time = 3.0  # [s]
        self.showdt = 1.0
        #########################################
        # self.speed_cost_gain = 1.5 
        # self.obs_cost_gain = 1.0
        # self.to_human_cost_gain =0.5
        if args.param == 1:
            self.to_human_cost_gain = 1.0 #lower = detour
            self.speed_cost_gain = 2.0 #lower = faster
            self.obs_cost_gain = 1.0 #lower z= fearless
        if args.param == 2:
            self.to_human_cost_gain = 1.0 #lower = detour
            self.speed_cost_gain = 1.0 #lower = faster
            self.obs_cost_gain = 2.0 #lower z= fearless
        if args.param == 3:
            self.to_human_cost_gain = 2.0 #lower = detour
            self.speed_cost_gain = 1.0 #lower = faster
            self.obs_cost_gain = 1.0 #lower z= fearless
        #############################
        self.robot_radius = 0.106  # [m]
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.goalX = 0.0
        self.goalY = 0.0
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.distance = 0.0
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
        # self.xy.add((self.x,self.y))
        odom_callback(self)



    # Callback for attaining goal co-ordinates from Rviz Publish Point #rviz中设置目标点
    def goalCB(self,msg):
        self.goalX = msg.point.x
        self.goalY = msg.point.y

class Obstacles():
    def __init__(self):
        # Set of coordinates of obstacles in view
        self.obst = set()

    def assignObs1(self, msg, config):

        deg = len(msg.ranges)   # Number of degrees - varies in Sim vs real world
        # print("Laser degree length {}".format(deg))
        self.obst = set()   # reset the obstacle set to only keep visible objects
        scan_range = []
        for i in range(len(msg.ranges)):
            # if msg.ranges[i] == float('Inf'):
            if msg.ranges[i]>3.5:
                continue
                # scan_range.append(3.5)
            elif np.isnan(msg.ranges[i]):
                # scan_range.append(0)
                distance = 0.12

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

# Model to determine the expected position of the robot after moving along trajectory
def motion(x, u, dt):
    # motion model
    # x = [x(m), y(m), theta(rad), v(m/s), omega(rad/s)]
    x[0] += u[0] * math.cos(x[2]) * dt
    x[1] += u[0] * math.sin(x[2]) * dt
    x[2] += u[1] * dt
    x[3] = u[0]
    x[4] = u[1]

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

    x = np.array([0,0,0,v,y])
    time = 0
    angle =0.0

    while time <= 1.0:
        # store each motion model along a trajectory
        x = motion(x, [v, y], config.dt)
        time += config.showdt # next sample
    if abs(v)<=0.0001:
        angle=y*1.0
        # print(angle)
    else:
        angle=math.atan2(x[1], x[0])
    #print(angle)
    return angle


################################################################################



# Calculate trajectory, costings, and return velocities to apply to robot
def calc_final_input(x, u, dw, config, ob):
    # global list_x,list_y,line_num
    xinit = x[:]######
    max_cost = 0.0
    # trajs=[]
    max_u = u
    max_u[0] = 0.0 #全部为死路
    max_u[1] = human.angular.z
    # list_x.clear()
    # list_y.clear()
    global angle_robot
    # evaluate all trajectory with sampled input in dynamic window
    for v in np.arange(config.min_speed, config.max_speed+config.v_reso, config.v_reso):
        for w in np.arange(-config.max_yawrate, config.max_yawrate+config.yawrate_reso, config.yawrate_reso):
            traj = calc_trajectory(xinit, v, w, config)
            
            # calc costs with weighted gains
            to_human_cost = (1-calc_to_human_cost(v,w,config,4)) * config.to_human_cost_gain

            speed_cost = config.speed_cost_gain *(1-abs(human.linear.x - v)/(config.max_speed-config.min_speed))

            ob_cost = calc_obstacle_cost(traj, ob, config) * config.obs_cost_gain

            if np.isinf(ob_cost):
                continue

            final_cost = to_human_cost + speed_cost + ob_cost

            # search minimum trajectory     ##最大代价
            if max_cost <= final_cost:
                max_cost = final_cost
                max_u = [v, w]
                # trajs=traj
    # for element in trajs:
    #                 list_x.append(element[0])
    #                 list_y.append(element[1])
    #                 line_num=len(list_x)
    # print(max_u[0],max_u[1])
    show_trajectory(xinit, max_u[0], max_u[1], config)
    
    return max_u
#################################################################################



# Calculate obstacle cost inf: collision, 0:free
def calc_obstacle_cost(traj, ob, config):
    skip_n = 1
    minr = 3.5
    
    # Loop through every obstacle in set and calc Pythagorean distance
    # Use robot radius to determine if collision
    for ii in range(2, len(traj[:, 1]), skip_n):
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


    return (minr-0.12)/3.38

############################################################################333
def calc_to_human_cost( v, w,config,n):
    if n==1:
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
    elif n==3:
        angle_robot=calc_angle_fromtraj(v, w, config)
        cost=abs(angle_human-angle_robot)/math.pi
    elif n==4:
        robot_angle=w
        cost = abs(human_angle-robot_angle)/(config.max_yawrate*2)
    else:
        robot_angle=cal_angle(v,w)
        cost = abs(human_angle-robot_angle)/math.pi
    # if (np.isinf(robot_r)&np.isinf(human_r)):
    #     cost=0
    # else:
    #     cost =abs(robot_r-human_r)

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
        <= config.robot_radius*4:
        return 1
    return 0


def cal_angle(v,w):
    if v==0:
        angle=w+math.pi/2
    else:
        y=v*math.cos(w)
        x=v*math.sin(w)
        angle = math.atan2(y, x)
    return angle

human_angle=math.pi/2
angle_human=0
yici=1

def share1(vel_msg,config):# human command get 获取人类指令
    global human
    global human_r
    global inputkey
    global human_angle
    global angle_human

    

    human.linear.x=vel_msg.linear.x

    if inputkey==1:
        if human.linear.x<0:
            human.linear.x=0.0
    human.angular.z=vel_msg.angular.z

    human_angle=human.angular.z

    if human.linear.x<=0.001 and human.linear.x>=-0.001:
        human.linear.x = 0.0
    # human_angle=cal_angle(human.linear.x,human.angular.z)

    # angle_human=calc_angle_fromtraj(human.linear.x,human.angular.z,config)

    # print(human.angular.z,angle_human)

    # if human.angular.z == 0:
    #     human_r=float("inf")
    # else:
    #     human_r=human.linear.x/human.angular.z
    # if (human_r< config.v_reso/config.max_yawrate) and human.angular.z>0:
    #     human_r=config.v_reso/config.max_yawrate-0.01
    # if (human_r> -config.v_reso/config.max_yawrate) and human.angular.z<0:
    #     human_r=-config.v_reso/config.max_yawrate+0.01
    # if human.linear.x==0:
    #     if human.angular.z>0:
    #         human_r=config.v_reso/config.max_yawrate-0.01
    #     if human.angular.z<0:
    #         human_r=-config.v_reso/config.max_yawrate+0.01
    # 取正符号


global marker
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

def line(num):
    # marker line points
    marker.points = []

    for i in range(num):
        exec(f"line_point{i} = Point()") 
        exec(f"line_point{i}.x = list_x[{i}]")
        exec(f"line_point{i}.y = list_y[{i}]")
        exec(f"line_point{i}.z = 0.0")
        exec(f"marker.points.append(line_point{i})")

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


def main():
    print(__file__ + " start!!")
    print("human is 0,share is 1")
    inputs=input()
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
    obs = Obstacles()
    counter = StringMessageCounter()

    model_name = "turtlebot3"
    # 指定目标位置和方向 (四元数)
    target_position = [-3.6, -3.0, 0.0]  # x, y, z
    target_orientation = [0.0, 0.0, 0.707,  0.707]  # x, y, z, w
    set_robot_position(model_name, target_position, target_orientation)

    subOdom = rospy.Subscriber("/odom", Odometry, config.assignOdomCoords)
    subLaser = rospy.Subscriber("/scan", LaserScan, obs.assignObs1, config)


    sub_hum = rospy.Subscriber("/cmd_vel_human",Twist,share1,config,queue_size=1)

    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    # human_value_pub = rospy.Publisher('cost', Float64, queue_size = 1)
    # sub_obs = rospy.Subscriber("/gazebo/base_collision",Contact,StringMessageCounter.callbackobs,queue_size=10)
    pub_line = rospy.Publisher('~line_list', Marker, queue_size=10)
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    
    speed = Twist()
    change_goal(config,rand.get_next())
    goal_sphere(config)
    # initial state [x(m), y(m), theta(rad), v(m/s), omega(rad/s)]
    x = np.array([config.x, config.y, config.th, 0.0, 0.0])
    # initial linear and angular velocities
    u = np.array([0.0, 0.0])

    start_time = rospy.get_time()
    # runs until terminated externally
    while not rospy.is_shutdown():
        global yici
        if (inputkey== 1):
            u = dwa_control(x, u, config, obs.obst)
            x[0] = config.x
            x[1] = config.y
            x[2] = config.th
            x[3] = u[0]
            x[4] = u[1]
            speed.linear.x = x[3]
            speed.angular.z = x[4]
            line(line_num)
            pub_line.publish(marker)
        
        else:
            # if 0 then do directly
            speed.linear.x = human.linear.x
            speed.angular.z = human.angular.z
        if yici>0:
            marker_pub.publish(markers)
        pub.publish(speed)
        if atGoal(config)==1:

            if yici>0:
                print("YOU have arrive the goal point")
                save(get_time(start_time),config.distance,counter.send_count,inputkey,args.param)
                print("distance in this time: %.2f m" % config.distance)
                print("hit time: %d " % counter.send_count)
                # with open('/home/frienkie/cood/test1', 'w') as f:
                #     json.dump(list(config.xy), f)

                
            change_goal(config,rand.get_next())
            goal_sphere(config)
        config.r.sleep()


if __name__ == '__main__':
    rospy.init_node('shared_dwa')
    main()