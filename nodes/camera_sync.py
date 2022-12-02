#! /usr/bin/env python
# coding=utf-8

import rospy
import message_filters
import numpy as np
from sensor_msgs.msg import LaserScan

rospy.init_node("camera_sync")

def callback(camera, back_camera):
    scan_data = []
    for i in range(len(camera.ranges)/2):
        if camera.ranges[i+6] == float("Inf"):
            scan_data.append(3.5)
        elif np.isnan(camera.ranges[i+6]):
            scan_data.append(0)
        else:
            scan_data.append(camera.ranges[i+6])

    for i in range(len(back_camera.ranges)):
        if back_camera.ranges[i] == float("Inf"):
            scan_data.append(3.5)
        elif np.isnan(back_camera.ranges[i]):
            scan_data.append(0)
        else:
            scan_data.append(back_camera.ranges[i])

    for i in range(len(camera.ranges)/2):
        if camera.ranges[i] == float("Inf"):
            scan_data.append(3.5)
        elif np.isnan(camera.ranges[i]):
            scan_data.append(0)
        else:
            scan_data.append(camera.ranges[i])
    #print(scan_data)
    pub.publish(header = camera.header, ranges = scan_data)

front_sub = message_filters.Subscriber("camera", LaserScan)
back_sub = message_filters.Subscriber("back_camera", LaserScan)

pub = rospy.Publisher('camera_sync', LaserScan, queue_size=1)

# ats = message_filters.ApproximateTimeSynchronizer([front_sub, back_sub], queue_size=5, slop=0.01)
ats = message_filters.TimeSynchronizer([front_sub, back_sub], queue_size=1)
ats.registerCallback(callback)

rospy.spin()
