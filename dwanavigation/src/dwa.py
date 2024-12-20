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
from geometry_msgs.msg import Twist, PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from distancetime import *
import json

import argparse

# 创建解析器
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
        self.max_speed = 0.2  # [m/s]
        self.min_speed = 0.0  # [m/s]
        self.max_yawrate = 0.6  # [rad/s]
        self.max_accel = 1.0  # [m/ss]
        self.max_dyawrate = 3.2  # [rad/ss]
        self.v_reso = 0.04  # [m/s]
        self.yawrate_reso = math.pi / 12 # [rad/s]
        self.dt = 0.2  # [s]
        self.predict_time = 4.0  # [s]
        if args.param == 1:
            self.to_goal_cost_gain = 1.0 #lower = detour
            self.speed_cost_gain = 2.0 #lower = faster
            self.obs_cost_gain = 1.0 #lower z= fearless
        if args.param == 2:
            self.to_goal_cost_gain = 1.0 #lower = detour
            self.speed_cost_gain = 1.0 #lower = faster
            self.obs_cost_gain = 2.0 #lower z= fearless
        if args.param == 3:
            self.to_goal_cost_gain = 2.0 #lower = detour
            self.speed_cost_gain = 1.0 #lower = faster
            self.obs_cost_gain = 1.0 #lower z= fearless
        self.robot_radius = 0.106  # [m]
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.goalX = 14.0
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
        # print(self.th)
        self.xy.add((self.x,self.y))
        odom_callback(self)

class Obstacles():
    def __init__(self):
        # Set of coordinates of obstacles in view
        self.obst = set()


    # Callback for LaserScan
    def assignObs(self, msg, config):

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

# Calculate trajectory, costings, and return velocities to apply to robot
def calc_final_input(x, u, dw, config, ob):

    xinit = x[:]
    max_cost = 0.0
    max_u = u
    max_u[0] = 0.0

    # evaluate all trajectory with sampled input in dynamic window
    for v in np.arange(dw[0], dw[1]+config.v_reso, config.v_reso):
        for w in np.arange(dw[2], dw[3]+config.yawrate_reso, config.yawrate_reso):
            traj = calc_trajectory(xinit, v, w, config)

            # calc costs with weighted gains
            # to_goal_cost = calc_to_goal_cost(traj, config) * config.to_goal_cost_gain

            to_goal_cost=config.to_goal_cost_gain * (1-calc_heading_angle_cost(traj, config.goalX, config.goalY)/math.pi)
            
            speed_cost = config.speed_cost_gain * abs(v/config.max_speed)

            ob_cost = calc_obstacle_cost(traj, ob, config) * config.obs_cost_gain

            if np.isinf(ob_cost):
                continue

            final_cost = to_goal_cost + speed_cost + ob_cost

            # search minimum trajectory
            if max_cost <= final_cost:
                max_cost = final_cost
                max_u = [v, w]

    return max_u

def angle_range_corrector(angle):

    if angle > math.pi:
        while angle > math.pi:
            angle -=  2 * math.pi
    elif angle < -math.pi:
        while angle < -math.pi:
            angle += 2 * math.pi

    return angle

def calc_heading_angle_cost(traj, goal_x, goal_y):
        last_x = traj[-1,0]
        last_y = traj[-1,1]
        last_th = traj[-1,2]

        angle_to_goal = math.atan2(goal_y - last_y, goal_x - last_x)

        score_angle = angle_to_goal - last_th

        score_angle = abs(angle_range_corrector(score_angle))

        return score_angle

# Calculate obstacle cost inf: collision, 0:free
def calc_obstacle_cost(traj, ob, config):
    skip_n = 1
    minr = 3.5
    
    # Loop through every obstacle in set and calc Pythagorean distance
    # Use robot radius to determine if collision
    for ii in range(0, len(traj[:, 1]), skip_n):
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

# Calculate goal cost via Pythagorean distance to robot
def calc_to_goal_cost(traj, config):
    # If-Statements to determine negative vs positive goal/trajectory position
    # traj[-1,0] is the last predicted X coord position on the trajectory
    if (config.goalX >= 0 and traj[-1,0] < 0):
        dx = config.goalX - traj[-1,0]
    elif (config.goalX < 0 and traj[-1,0] >= 0):
        dx = traj[-1,0] - config.goalX
    else:
        dx = abs(config.goalX - traj[-1,0])
    # traj[-1,1] is the last predicted Y coord position on the trajectory
    if (config.goalY >= 0 and traj[-1,1] < 0):
        dy = config.goalY - traj[-1,1]
    elif (config.goalY < 0 and traj[-1,1] >= 0):
        dy = traj[-1,1] - config.goalY
    else:
        dy = abs(config.goalY - traj[-1,1])

    cost = math.sqrt(dx**2 + dy**2)
    return cost

# Begin DWA calculations
def dwa_control(x, u, config, ob):
    # Dynamic Window control

    dw = calc_dynamic_window(x, config)

    u = calc_final_input(x, u, dw, config, ob)

    return u

# Determine whether the robot has reached its goal
def atGoal(config):
    # check at goal
    if math.sqrt((config.x - config.goalX)**2 + (config.y - config.goalY)**2) \
        <= config.robot_radius*4:
        return 1
    return 0

yici=1

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
                yici = 0  # 此时 yici 为 1
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

def main():
    print(__file__ + " start!!")
    # robot specification
    config = Config()
    # position of obstacles
    obs = Obstacles()
    rand=RandomNumberGenerator()
    counter = StringMessageCounter()
    print("which map is used now?")
    chizu=input()
    file_value=start_rosbag()
    model_name = "turtlebot3"
    # 指定目标位置和方向 (四元数)
    target_position = [-3.6, -3.0, 0.0]  # x, y, z
    target_orientation = [0.0, 0.0, 0.707,  0.707]  # x, y, z, w
    set_robot_position(model_name, target_position, target_orientation)

    subOdom = rospy.Subscriber("/odom", Odometry, config.assignOdomCoords)
    subLaser = rospy.Subscriber("/scan", LaserScan, obs.assignObs, config)
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    #subGoal = rospy.Subscriber("/clicked_point", PointStamped, config.goalCB)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    speed = Twist()
    # initial state [x(m), y(m), theta(rad), v(m/s), omega(rad/s)]
    x = np.array([config.x, config.y, config.th, 0.0, 0.0])
    # initial linear and angular velocities
    u = np.array([0.0, 0.0])
    change_goal(config,rand.get_next())
    goal_sphere(config)

    start_time = rospy.get_time()
    # runs until terminated externally
    while not rospy.is_shutdown():
        if (atGoal(config) == False):
            u = dwa_control(x, u, config, obs.obst)
            x[0] = config.x
            x[1] = config.y
            x[2] = config.th
            x[3] = u[0]
            x[4] = u[1]
            speed.linear.x = x[3]
            speed.angular.z = x[4]
            # print(x[0])
            
        else:
            # if at goal then stay there until new goal published
            global yici
            if yici>0:
                print("YOU have arrive the goal point")
                save(get_time(start_time),config.distance,counter.send_count,args.param+4,chizu)
                print("distance in this time: %.2f m" % config.distance)
                print("hit time: %d " % counter.send_count)
                with open(f'/home/frienkie/cood/test{file_value}.txt', 'w') as f:
                    json.dump(list(config.xy), f)
                stop_rosbag()
                
            change_goal(config,rand.get_next())
            goal_sphere(config)
        pub.publish(speed)
        marker_pub.publish(markers)
        config.r.sleep()


if __name__ == '__main__':
    rospy.init_node('dwa')
    main()
