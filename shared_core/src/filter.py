#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from statistics import median
import numpy as np

# Range Filter
def range_filter(scan, min, max):
    index = []
    for i, ray in enumerate(scan) :
        if ray < min:
            scan[i] = float('Inf')
        elif ray > max:
            scan[i] = float('Inf')
        elif np.isnan(ray):
            scan[i] = min
            index.append(i)
        else :
            index.append(i)
    return scan, index

# Median Deviation
def median_deviation(scan, window):
    result = []
    scan_size = len(scan)
    windows_num = scan_size//window 
    mod = scan_size%window
    
    for i in range(windows_num):
        # Extract window from scan and find its median
        temp = scan[window*i : window*(i+1)]
        med = median(temp)
        
        # Check the deviation of every ray from median
        # If it's higher than 20% replace its value with an extreme
        # None value is not  acceptable
        for j, elem in enumerate(temp):
            if (med - elem)/med*100 > 10:
                temp[j] = float('Inf')
        
        result.extend(temp)
    
    # Check if part of the scan is left excluded
    if (mod != 0):
        temp = scan[-mod:]
        med = median(temp)
        for j, elem in enumerate(temp):
            if (med - elem)/med*100 > 10:
                temp[j] = float('Inf')
        result.extend(temp)
    
    return result

def callback(msg):
        rate = rospy.Rate(20)
        scan = list(msg.ranges)
        
        # Get a list of rays in specified range and their indexes 
        range_scan, index = range_filter(scan, min = msg.range_min, max =msg.range_max )
        not_none = []

        # Get not none values 
        not_none = [range_scan[i] for i in index if range_scan[i] !=float('Inf')]        
        not_none = median_deviation(not_none,10)

        # Replace the old values of the scan with the filtered ones 
        for i in index:
            scan[i] = not_none.pop(0) 
        
        msg.ranges = tuple(scan)
        pub.publish(msg)  

rospy.init_node('noise_filtering')
sub = rospy.Subscriber('/scan', LaserScan, callback)
pub = rospy.Publisher('/filtered_scan', LaserScan, queue_size=10)
print("filter node starts")
 
rospy.spin()