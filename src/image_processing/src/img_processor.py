#!/usr/bin/env python
import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import matplotlib
from time import sleep
from math import atan2, sqrt
#matplotlib.use('Agg')
from matplotlib import pyplot as plt
#from IPython import display


realworld = {
            "red": (3.57, 3.0),
            "blue": (3.57, 1.15),
            "purple": (2.33, 3.0),
            "green": (2.33, 1.15),
        }

distortion = np.array([-0.2848041238667301, 0.0659206596895867, 0.0001416462376699293, -0.002084876269696945, 0]).reshape((5, 1))

cameraMatrix = np.array([366.8488601935177, 0, 331.7407554090113, 0, 366.7888272707089, 240.8568678888034, 0, 0, 1]).reshape((3, 3))

bridge = CvBridge()
skip_cluster_detection = False
dims = (640, 480)
scale_factor = 4
scaled_dims = (dims[0] // scale_factor, dims[1] // scale_factor)

is_green = lambda pix: pix[1] 

def imgCallback():
    global bridge, skip_cluster_detection, scaled_dims, scale_factor, realworld, cameraMatrix
    #cv_img = bridge.compressed_imgmsg_to_cv2(img_msg)
    cv_img = cv2.imread("center.png",cv2.IMREAD_COLOR)
    copy = cv_img
    cb, cg, cr = cv2.split(copy)
    cv_img = cv2.resize(cv_img, scaled_dims)
    cv2.imshow("gps", copy)

    circles = cv2.HoughCircles(cg, cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=20, minRadius=4, maxRadius=0)

    #print("CIRCLES: ", circles)

    #cv2.imshow("gps", cg)
    #cv2.imshow("gps1", cr)
    #cv2.imshow("gps2", cb)
    cv2.moveWindow("gps", 100, 100)
    #cv2.moveWindow("gps1", 600, 100)
    #cv2.moveWindow("gps2", 100, 600)
    cv2.waitKey(50)

    if skip_cluster_detection:
        return
    colors = {"green": [], "red": [], "blue": [], "purple": []}

    b, g, r = cv2.split(cv_img)
#    green = cv2.inRange(g, 150, 255)
#    red = cv2.inRange(r, 150, 255)
#    blue = cv2.inRange(b, 150, 255)
#    white = cv2.inRange(cv_img, 190, 255)
#
#    colors["purple"] = [r for r in red if r not in white and r in red and r in blue] 
#    colors["red"] = [r for r in red if r not in white]
#    colors["blue"] = [r for r in red if r not in white]
#    colors["green"] = [r for r in red if r not in white]
    
    for y in range(cv_img.shape[0]):
        for x in range(cv_img.shape[1]):
            is_green = 150 <= g[y, x] and r[y, x] <= 120 and b[y, x] <= 120
            is_red = 180 <= r[y, x] and g[y, x] <= 150 and b[y, x] <= 150
            is_blue = 180 <= b[y, x] and r[y, x] <= 150 and g[y, x] <= 150
            #is_white = 180 <= g[y, x] and 180 <= b[y, x]
            is_purple = 180 <= b[y, x] and 180 <= r[y, x] and g[y, x] <= 150
#(not is_white) and

            """if is_white:
                continue"""
            if is_blue:
                colors["blue"].append((x, y))
            elif is_red:
                colors["red"].append((x, y))
            elif is_green:
                colors["green"].append((x, y))
            elif is_purple:
                colors["purple"].append((x, y))

    tuple_sum = lambda t1, t2: (t1[0] + t2[0], t1[1] + t2[1])
    color_pos_means = {} #{"green": (0, 0), "purple": (0, 0), "blue": (0, 0), "red": (0, 0)}

    scale = int(scale_factor)
    for color in colors:
        if len(colors[color]) == 0:
            continue
	color_pos_means[color] = (0, 0)
        for tup in colors[color]:
            color_pos_means[color] = tuple_sum(color_pos_means[color], tup)
        color_pos_means[color] = (scale*color_pos_means[color][0] / len(colors[color]), scale * color_pos_means[color][1] / len(colors[color]))


#    center = (0, 0)
#    for color in color_pos_means:
#        center = tuple_sum(center, color_pos_means[color])

    """color_pos_means = {
            "blue": (320, 98),
            "red": (466, 120),
            "green": (316, 215),
            "purple": (470, 218)}
"""
    print color_pos_means

    #color_pos_means["center"] = center

    # find pose.
    numPoints = len(color_pos_means)
    realpoints = np.zeros((numPoints, 3))
    imagepoints = np.zeros((numPoints, 2))
    colors = color_pos_means.keys()
    for i in range(numPoints):
        color =  colors[i]
        print("Color: ", color, " Values: ", color_pos_means[color], realworld[color])
        imagepoints[i, 0] = color_pos_means[color][0]
        imagepoints[i, 1] = color_pos_means[color][1]
        realpoints[i, 0] = realworld[color][0]
        realpoints[i, 1] = realworld[color][1]
        realpoints[i, 2] = 0

    try:
        retval, rvec, tvec = cv2.solvePnP(realpoints, imagepoints, cameraMatrix, distortion)
        print("Translation :", tvec)
        print("Rotation :", rvec)
        pass
    except:
        print("Not enough points detected in image!")


#rospy.init_node("visual_gps")
#rospy.Subscriber("/usb_cam/image_color/compressed", CompressedImage, imgCallback, queue_size=1)
#rospy.Subscriber("/image_processing/image_raw", CompressedImage, imgCallback, queue_size=1)

#rospy.spin()
while True:
    imgCallback()
    rospy.sleep(5)


