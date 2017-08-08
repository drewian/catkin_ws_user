#!/usr/bin/env python
import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String, Int16, Float32
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
#import matplotlib
from time import sleep
from math import atan2, sqrt
#matplotlib.use('Agg')
#from matplotlib import pyplot as plt

bridge = CvBridge()
imageWindow = None
minDistance = 500
maxDistance = 700
lastOdom = None


def odomCallback(odom_msg):
    global lastOdom
    lastOdom = odom_msg

def imgCallback(img_msg):
    global bridge, imageWindow, lastOdom

    if lastOdom == None: return

    print("Callback received!")
    cv_img = bridge.imgmsg_to_cv2(img_msg, "passthrough")
    cv_img = cv2.resize(cv_img, None, fx=0.85, fy=0.85, interpolation=cv2.INTER_LINEAR)
    height = cv_img.shape[0]
    width = cv_img.shape[1]

    lowerBound = 100
    higherBound = 2000
    
#    middle = (height // 2, width // 2)
#    startx = width // 4
#    endx = width // 4 * 3
#    starty = height // 4
#    endy = height // 4 * 3
#    vals = []
#    scanWindowHeight = 40
#    windows = range(scanWindowHeight)

    distance = cv_img[middle[0], middle[1]]
    print("Distance: ", distance)
    neutral = 70

    speedToPub = 0
    if not (distance < lowerBound or higherBound < distance):
        if distance < minDistance:
            print("Driving backwards!")
            speedToPub = 100
    	    drivePub.publish(0)
    	    steerPub.publish(neutral)
        elif maxDistance < distance: 
            print("Driving forwards!")
            speedToPub = -100
    	    drivePub.publish(0)
    	    steerPub.publish(neutral)
        else:
            print("Not moving!")
    	odom = Odometry()
    	odom = lastOdom
    	odom.pose.pose.position.x = (lastOdom.pose.pose.position.x - distance) / 1000
    	adjOdomPub.publish(odom)
    else:
        print("No obstacle detected! Not moving...")

            
    #speedPub.publish(speedToPub)



rospy.init_node("obs_avoidance")
rospy.Subscriber("/app/camera/depth/image_raw", Image, imgCallback, queue_size=1)
rospy.Subscriber("/odom", Odometry, odomCallback, queue_size=1)
drivePub = rospy.Publisher("/manual_control/stop_start", Int16, queue_size=1)
speedPub = rospy.Publisher("/manual_control/speed", Int16, queue_size=1)
steerPub = rospy.Publisher("/manual_control/steering", Int16, queue_size=1)
adjOdomPub = rospy.Publisher("/obstacle/pose", Odometry, queue_size=1)

print("Waiting for callback")


rospy.spin()
