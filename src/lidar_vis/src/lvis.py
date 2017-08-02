
import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image, LaserScan
from sensor_msgs.msg import OccupancyGrid
from cv_bridge import CvBridge, CvBridgeError
import matplotlib
from time import sleep
from math import atan2, sqrt
#matplotlib.use('Agg')
from matplotlib import pyplot as plt
# from __future__ import print_function

class lidar_sub:

    def lidar_callback(self, msg):
        self.lidar_pub.publish(msg.data)

    def __init__(self):
        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback, queue_size=1)
        self.lidar_pub = rospy.Publisher("/lidar_pub/raw_scan", LaserScan, queue_size=1)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node("lidar_pub", anonymous=True)
    sb = lidar_sub()
    sb.run()








