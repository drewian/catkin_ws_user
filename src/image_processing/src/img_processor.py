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
from IPython import display

bridge = CvBridge()
skip_cluster_detection = False
dims = (640, 480)
scale_factor = 4
scaled_dims = (dims[0] // scale_factor, dims[1] // scale_factor)

is_green = lambda pix: pix[1] 

def imgCallback(img_msg):
    global bridge, skip_cluster_detection, scaled_dims, scale_factor
    cv_img = bridge.compressed_imgmsg_to_cv2(img_msg)
    copy = cv_img
    cv_img = cv2.resize(cv_img, scaled_dims)
    #cv2.imshow("gps", cv_img)
    cv2.imshow("gps", copy)
    cv2.moveWindow("gps", 100, 100)
    cv2.waitKey(50)

    if skip_cluster_detection:
        return
    colors = {"green": [], "red": [], "blue": [], "purple": []}

    b, g, r = cv2.split(cv_img)
    for y in range(cv_img.shape[0]):
        for x in range(cv_img.shape[1]):
            is_green = 220 <= g[y, x]
            is_red = 220 <= r[y, x] and b[y, x] <= 180
            is_blue = 220 <= b[y, x] and r[y, x] <= 180
            is_white = 170 <= g[y, x] and 170 <= b[y, x]
            is_purple = (not is_white) and 180 <= r[y, x] and 180 <= b[y, x]

            if is_white:
                continue
            if is_blue:
                colors["blue"].append((x, y))
            elif is_red:
                colors["red"].append((x, y))
            elif is_green:
                colors["green"].append((x, y))
            elif is_purple:
                colors["purple"].append((x, y))

#            if is_white:
#                continue
#            if is_green:
#                colors["green"].append((x, y))
#            elif is_red:
#                if is_blue:
#                    colors["purple"].append((x, y))
#                else:
#                    colors["red"].append((x, y))
#            elif is_blue:
#                colors["blue"].append((x, y))

    tuple_sum = lambda t1, t2: (t1[0] + t2[0], t1[1] + t2[1])
    color_pos_means = {"green": (0, 0), "purple": (0, 0), "blue": (0, 0), "red": (0, 0)}
    scale = int(scale_factor)
    for color in colors:
        if len(colors[color]) == 0:
            del colors[color]
            continue
        for tup in colors[color]:
            color_pos_means[color] = tuple_sum(color_pos_means[color], tup)
        color_pos_means[color] = (scale*color_pos_means[color][0] / len(colors[color]), scale * color_pos_means[color][1] / len(colors[color]))






    print color_pos_means


rospy.init_node("visual_gps")
rospy.Subscriber("/usb_cam/image_color/compressed", CompressedImage, imgCallback, queue_size=1)

rospy.spin()


