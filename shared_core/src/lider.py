#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

def callback(scan_data):
    # 获取激光雷达数据范围（距离）
    ranges = scan_data.ranges
    
    # 打印各个方向的距离信息
    for i, distance in enumerate(ranges):
        angle = scan_data.angle_min + i * scan_data.angle_increment
        print(f"Angle: {angle:.2f} rad, Distance: {distance:.2f} m")

def main():
    rospy.init_node('turtlebot3_lidar_printer', anonymous=True)
    
    # 订阅 /scan 话题
    rospy.Subscriber('/scan', LaserScan, callback)
    
    # 保持节点运行
    rospy.spin()

if __name__ == '__main__':
    main()
