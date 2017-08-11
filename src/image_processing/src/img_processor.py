#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from geometry import findRealPos, rigid_transform_3D, findRealPos2
import matplotlib
from time import sleep
from utils import get2DRotationMatrix
from math import atan2, sqrt, pi
from matplotlib import pyplot as plt

realworld = {
            "red": (0.62, 0.925),
            "blue": (0.62, -0.925),
            "purple": (-0.62, 0.925),
            "green": (-0.62, -0.925)
        }

distortion = np.array([-0.287348224330035, 0.05364856147008246,
    0.00921819101079906, 0.003424786599010765, 0]).reshape((5, 1))
cameraMatrix = np.array([275.0386008874681, 0, 296.3819553629593, 0,
    273.0746410330246, 215.4805927869714, 0, 0, 1]).reshape((3, 3))

bridge = CvBridge()
skip_cluster_detection = False
dims = (640, 480)
scale_factor = 2
scaled_dims = (dims[0] // scale_factor, dims[1] // scale_factor)
lastKnownPos = (0, 0, 0)

def imgCallback(img_msg):
    global bridge, skip_cluster_detection, scaled_dims, scale_factor, realworld, cameraMatrix, lastKnownPos
    cv_img = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    #cv_img = bridge.compressed_imgmsg_to_cv2(img_msg, "bgr8")
    copy = cv_img
    cv_img = cv2.undistort(cv_img, cameraMatrix, distortion)
    cv_img = cv2.resize(cv_img, scaled_dims)
    #copy = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

#    cv2.imshow("gps", copy)
#    cv2.imshow("gps_undist", cv_img)
#    cv2.moveWindow("gps", 100, 100)
#    cv2.waitKey(50)

    colors = {"green": [], "red": [], "blue": [], "purple": []}

    b, g, r = cv2.split(cv_img)
    
    for y in range(cv_img.shape[0]):
        for x in range(cv_img.shape[1]):
            is_green = 60 <= g[y, x] and r[y, x] <= 45 and b[y, x] <= 50
            is_red = 70 <= r[y, x] and g[y, x] <= 45 and b[y, x] <= 55
            is_blue = 95 <= b[y, x] and r[y, x] <= 40 and g[y, x] <= 50
            is_purple = 185 <= b[y, x] and 55 <= r[y, x] and g[y, x] <= 55

            if is_blue:
                colors["blue"].append((x, y))
            elif is_red:
                colors["red"].append((x, y))
            elif is_green:
                colors["green"].append((x, y))
            elif is_purple:
                colors["purple"].append((x, y))

    tuple_sum = lambda t1, t2: (t1[0] + t2[0], t1[1] + t2[1])
    color_pos_means = {}

    scale = int(scale_factor)
    for color in colors:
        if len(colors[color]) == 0:
            continue
        color_pos_means[color] = (0, 0)
        for tup in colors[color]:
            color_pos_means[color] = tuple_sum(color_pos_means[color], tup)
        color_pos_means[color] = (scale*color_pos_means[color][0] / len(colors[color]), scale * color_pos_means[color][1] / len(colors[color]))


    colors = color_pos_means.keys()
    if len(colors) < 3:
        print("Not enough points found")
        return
    
    distInPixels = sqrt((color_pos_means[colors[0]][0]
            - color_pos_means[colors[1]][0]) ** 2 + (color_pos_means[colors[0]][1]
            - color_pos_means[colors[1]][1]) ** 2)

    distInM = sqrt((realworld[colors[0]][0]
            - realworld[colors[1]][0]) ** 2 + (realworld[colors[0]][1]
            - realworld[colors[1]][1]) ** 2)

    mmPerPixel = distInM / distInPixels

    numPoints = len(color_pos_means)
    realpoints = np.zeros((numPoints, 3))
    imagepoints = np.zeros((numPoints, 3))
    for i in range(numPoints):
        color = colors[i]
        imagepoints[i, 0] = mmPerPixel * color_pos_means[color][0]
        imagepoints[i, 1] = mmPerPixel * color_pos_means[color][1]
        imagepoints[i, 2] = 0
        realpoints[i, 0] = realworld[color][0]
        realpoints[i, 1] = realworld[color][1]
        realpoints[i, 2] = 0


    mapping = lambda x,y: ((320 - x) * mmPerPixel, (y - 240) * mmPerPixel)
    d = {}
    for color in color_pos_means:
        d[color] = mapping(color_pos_means[color][0], color_pos_means[color][1])
    yaw, coords = findRealPos2(d, realworld)

    center = np.array([2.96, 2.075]).reshape((2, 1))

    print("Translation in Coords: ", coords)
    print("Rotation in Coords: ", yaw)
    pos = np.array([coords[1], coords[0]]).reshape((2, 1)) + center

    x = pos[0]
    y = pos[1]

    print("Possible pos: ", pos)

    lastKnownPos = (x, y, yaw)


rospy.init_node("visual_gps")
#rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, imgCallback, queue_size=1)
rospy.Subscriber("/usb_cam/image_color", Image, imgCallback, queue_size=1)
#rospy.Subscriber("/image_processing/image_raw", CompressedImage, imgCallback, queue_size=1)
#rospy.Subscriber("/app/camera/rgb/image_color/compressed", CompressedImage, imgCallback, queue_size=1)

rospy.spin()

