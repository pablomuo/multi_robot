#! /usr/bin/env python
# coding=utf-8

import rospy
import message_filters
import numpy as np
from sensor_msgs.msg import LaserScan

rospy.init_node("camera_sync_1cam_laser")

def callback(camera, scan):
    val_max = 0
    sum_data = 0
    promedio = 0
    scan_max = 1
    scan_data_pre = []
    scan_data = 48*[0]
    
    #print(scan.header)
    for i in range(len(camera.ranges)*2):
        if i < 12:
            if camera.ranges[i] == float("Inf"):
                scan_data_pre.append(3.5)
                val_max = 3.5
            elif np.isnan(camera.ranges[i]):
                scan_data_pre.append(0)
                val_max = 0
            else:
                scan_data_pre.append(camera.ranges[i])
                val_max = camera.ranges[i]
            
            sum_data = sum_data + val_max
            if camera.ranges[i] > scan_max:
                if camera.ranges[i] <= 2:
                    scan_max = camera.ranges[i]
                else:
                    pass
            else: 
                pass

        if i == 12:
            promedio = sum_data/12
            if scan_max == 1:           #son todos o mayores de 2 o menor de 1, pillamos el promedio entonces
                scan_max = promedio
                if scan_max > 2:
                    scan_max = 2
        
        if i >= 12:
            scan_data_pre.append(scan_max)

    #hasta ahora se han analizado los valores de la camara
    #ahora se analizaran los valores del laser

    for i in range(len(scan.ranges)):             
        if scan.ranges[i] == float("Inf"):
                scan_data_pre.append(3.5)
        elif np.isnan(scan.ranges[i]):
            scan_data_pre.append(0)
        else:
            scan_data_pre.append(scan.ranges[i])
    
    # Los 24 primeros valores de scan_data se corresponden a los datos de la CAMERA
    scan_data[0:6] = scan_data_pre[6:12]
    scan_data[6:18] = scan_data_pre[12:24]
    scan_data[18:24] = scan_data_pre[0:6]
    # Los 24 últimos valores de scan_data se corresponden a los datos del SCAN
    scan_data[24:48] = scan_data_pre[24:48]

    pub.publish(header = camera.header, ranges = scan_data)

front_sub = message_filters.Subscriber("camera", LaserScan)
back_sub = message_filters.Subscriber("scan", LaserScan)

pub = rospy.Publisher('/camera_sync_1cam_laser', LaserScan, queue_size=1)
ats = message_filters.ApproximateTimeSynchronizer([front_sub, back_sub], queue_size=5, slop=0.2)
#ats = message_filters.TimeSynchronizer([front_sub, back_sub], queue_size=1)
ats.registerCallback(callback)

rospy.spin()


