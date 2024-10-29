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

class Config():
    # simulation parameters

    def __init__(self):
        # robot parameter
        #NOTE good params:
        #NOTE 0.55,0.1,1.0,1.6,3.2,0.15,0.05,0.1,1.7,2.4,0.1,3.2,0.18
        self.max_speed = 0.20  # [m/s]
        self.min_speed = 0.0  # [m/s]
        self.max_yawrate = 2.80  # [rad/s]

        self.max_accel = 1.0  # [m/ss]
        self.max_dyawrate = 2.8  # [rad/ss]
        ##################################################33
        self.v_reso = 0.05  # [m/s]
        self.yawrate_reso = 0.1  # [rad/s]
        #######################################################
        self.dt = 0.1  # [s]
        self.predict_time = 1.7  # [s]
        self.to_goal_cost_gain = 2.4       
        #########################################
        self.speed_cost_gain = 1 
        self.obs_cost_gain = 5 
        self.to_human_cost_gain =1

        #############################
        self.robot_radius = 0.15  # [m]
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.goalX = 0.0
        self.goalY = 0.0
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

    # Callback for attaining goal co-ordinates from Rviz Publish Point #rviz中设置目标点
    def goalCB(self,msg):
        self.goalX = msg.point.x
        self.goalY = msg.point.y

class Obstacles():
    def __init__(self):
        # Set of coordinates of obstacles in view
        self.obst = set()

    # Custom range implementation to loop over LaserScan degrees with
    # a step and include the final degree
    def myRange(self,start,end,step):
        i = start
        while i < end:
            yield i
            i += step
        yield end

    # Callback for LaserScan
    def assignObs(self, msg, config):
        deg = len(msg.ranges)   # Number of degrees - varies in Sim vs real world
        self.obst = set()   # reset the obstacle set to only keep visible objects
        for angle in self.myRange(0,deg-1,16):
            distance = msg.ranges[angle]
            # only record obstacles that are within 4 metres away
            if (distance < 4):
                # angle of obstacle wrt robot
                # angle/2.844 is to normalise the 512 degrees in real world
                # for simulation in Gazebo, use angle/4.0
                # laser from 0 to 180
                scanTheta = (angle/2.844 + deg*(-180.0/deg)+90.0) *math.pi/180.0
                # angle of obstacle wrt global frame
                objTheta = config.th - scanTheta
                # back quadrant negative X negative Y
                if (objTheta < -math.pi):
                    # e.g -405 degrees >> 135 degrees
                    objTheta = objTheta + 1.5*math.pi
                # back quadrant negative X positve Y
                elif (objTheta > math.pi):
                    objTheta = objTheta - 1.5*math.pi

                # round coords to nearest 0.125m
                obsX = round((config.x + (distance * math.cos(abs(objTheta))))*8)/8
                # determine direction of Y coord
                # if (objTheta < 0): # uncomment and comment line below for Gazebo simulation
                if (objTheta > 0):
                    obsY = round((config.y - (distance * math.sin(abs(objTheta))))*8)/8
                else:
                    obsY = round((config.y + (distance * math.sin(abs(objTheta))))*8)/8

                # add coords to set so as to only take unique obstacles   添加扫描的障碍物到记录
                self.obst.add((obsX,obsY))
                #print self.obst

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





################################################################################




# Calculate trajectory, costings, and return velocities to apply to robot
def calc_final_input(x, u, dw, config, ob):

    xinit = x[:]######默认为0和1
    max_cost = 0
    max_u = u
    max_u[0] = 0.0 #全部为死路，原地回转

    # evaluate all trajectory with sampled input in dynamic window
    for v in np.arange(0.0, 0.20+config.v_reso, config.v_reso):
        for w in np.arange(-2.8, 2.8+config.yawrate_reso, config.yawrate_reso):
            traj = calc_trajectory(xinit, v, w, config)

            # calc costs with weighted gains
            to_human_cost = (1-calc_to_human_cost(v,w,config)) * config.to_human_cost_gain

            speed_cost = config.speed_cost_gain *(1-abs(human.linear.x - v)/0.22)

            ob_cost = calc_obstacle_cost(traj, ob, config) * config.obs_cost_gain

            if np.isinf(ob_cost):
                continue

            final_cost = to_human_cost + speed_cost + ob_cost

            # search minimum trajectory     ##最大代价
            if max_cost <= final_cost:
                max_cost = final_cost
                max_u = [v, w]
    if human.linear.x==0:
        max_u[0] = 0.0
    return max_u
#################################################################################





# Calculate obstacle cost inf: collision, 0:free
def calc_obstacle_cost(traj, ob, config):
    skip_n = 2
    minr = 3

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
    if minr>3:
        minr=3
    return (minr-0.1)/2.9

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
############################################################################333
def calc_to_human_cost( v, w,config):

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

# Determine whether the robot has reached its goal
def atGoal(config, x):
    # check at goal
    if math.sqrt((x[0] - config.goalX)**2 + (x[1] - config.goalY)**2) \
        <= config.robot_radius:
        return True
    return False

def share1(vel_msg):# human command get 获取人类指令
    global human
    global human_r
    human.linear.x=vel_msg.linear.x
    human.angular.z=vel_msg.angular.z
    if human.angular.z == 0:
        human_r=float("inf")
    else:
        human_r=human.linear.x/human.angular.z
    if (human_r< 0.05/2.8) and human.angular.z>0:
        human_r=0.05/2.8-0.01
    if (human_r> -0.05/2.8) and human.angular.z<0:
        human_r=-0.05/2.8+0.01
    if human.linear.x==0:
        if human.angular.z>0:
            human_r=0.05/2.8-0.01
        if human.angular.z<0:
            human_r=-0.05/2.8+0.01
    # 取正符号


human=Twist()
human_r=float("inf")

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
    config = Config()
    # position of obstacles
    obs = Obstacles()
    subOdom = rospy.Subscriber("/odom", Odometry, config.assignOdomCoords)
    subLaser = rospy.Subscriber("/scan", LaserScan, obs.assignObs, config)
    # subGoal = rospy.Subscriber("/clicked_point", PointStamped, config.goalCB)

    sub_hum = rospy.Subscriber("/cmd_vel_human",Twist,share1,queue_size=1)

    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    speed = Twist()
    # initial state [x(m), y(m), theta(rad), v(m/s), omega(rad/s)]
    x = np.array([config.x, config.y, config.th, 0.0, 0.0])
    # initial linear and angular velocities
    u = np.array([0.0, 0.0])

    # runs until terminated externally
    while not rospy.is_shutdown():
        if (inputkey== 1):
            u = dwa_control(x, u, config, obs.obst)
            x[0] = config.x
            x[1] = config.y
            x[2] = config.th
            x[3] = u[0]
            x[4] = u[1]
            speed.linear.x = x[3]
            speed.angular.z = x[4]
        else:
            # if 0 then do directly
            speed.linear.x = human.linear.x
            speed.angular.z = human.angular.z

        pub.publish(speed)
        config.r.sleep()


if __name__ == '__main__':
    rospy.init_node('dwa')
    main()
