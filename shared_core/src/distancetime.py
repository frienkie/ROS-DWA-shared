#!/usr/bin/env python

import rospy
import math
from visualization_msgs.msg import Marker

markers = Marker()
def goal_sphere(config):
    
    global markers
    # 创建Marker消息

    markers.header.frame_id = "odom"  # 设置参考坐标系
    markers.header.stamp = rospy.Time.now()
    markers.ns = "sphere"  # 命名空间
    markers.id = 2  # ID
    markers.type = Marker.SPHERE  # 类型设置为球体
    markers.action = Marker.ADD  # 操作类型

    # 设置球体的中心坐标和大小
    markers.pose.position.x = config.goalX  # 圆心 x 坐标
    markers.pose.position.y = config.goalY  # 圆心 y 坐标
    markers.pose.position.z = 0.0  # 圆心 z 坐标
    markers.pose.orientation.x = 0.0
    markers.pose.orientation.y = 0.0
    markers.pose.orientation.z = 0.0
    markers.pose.orientation.w = 1.0

    markers.scale.x = config.robot_radius  # 球体在 x 方向的大小
    markers.scale.y = config.robot_radius  # 球体在 y 方向的大小
    markers.scale.z = config.robot_radius  # 球体在 z 方向的大小

    # 设置颜色
    markers.color.r = 0.0  # 红色分量
    markers.color.g = 1.0  # 绿色分量
    markers.color.b = 0.0  # 蓝色分量
    markers.color.a = 1.0  # 透明度

def get_time(start_time):

    end_time = rospy.get_time()  # 获取结束时间
    elapsed_time = end_time - start_time  # 计算运行时间
    print("程序运行时间: %.2f 秒" % elapsed_time)


def odom_callback(config):


    # 计算当前位置和之前位置的距离
    delta_distance = math.sqrt((config.x - config.prev_x)**2 + (config.y - config.prev_y)**2)
    config.distance += delta_distance

    # 更新之前的位置
    config.prev_x = config.x
    config.prev_y = config.y
    #print("当前总里程: %.2f 米", config.distance)

