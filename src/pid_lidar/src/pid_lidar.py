#!/usr/bin/env python

# --- imports ---
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Int16
from math import sin, cos, pi, atan
from sklearn import linear_model, datasets
import numpy as np

goal_distance = 0.9
eps = 0.05
last_odom = None
forward_dist = 1.2
angle_straight = 90
speed = -150
started = False
angle_sum = 0
last_angle = angle_straight
Kp = 0.65
Ki = 0.45
Kd = 0.65
last_timestamp = None
last_rects = []
num_rects = 15
last_yaw = Float32()
angle_eps = 5

# --- definitions ---

def start_car():
    global speed
    rospy.sleep(5)
    start_pub.publish(0)
    speed_pub.publish(speed)
    print("Published speed!")

def odomCallback(odom):
    global last_odom, last_timestamp
    if last_timestamp == None:
        last_timestamp = odom.header.stamp
    last_odom = odom

def yawCallback(yaw):
    global last_yaw
    last_yaw = yaw

def scanCallback(scan_msg):
    global goal_distance, eps, last_odom, forward_dist, angle_straight, speed, started
    global Ki, Kp, Kd, last_angle, angle_sum, last_timestamp
    global last_rects, num_rects, last_yaw, angle_eps

    while last_odom == None:
        print("Waiting for odometry.")
        rospy.sleep(1)

    # convert scan measurements into an occupancy grid    
    #<>
    ranges = scan_msg.ranges
    angle = scan_msg.angle_min
    incr = scan_msg.angle_increment
    Xs, ys = [], []
    distances = []
    for r in ranges:
        constraint = angle < -0.7 and angle > -2.5
        if constraint and r != float("inf"):
            distances.append(r)
        angle += incr

    #current_dist = sum(distances)/len(distances)
    current_dist = min(distances)
    
    diff = current_dist - goal_distance
    # nothing to do.
    if abs(diff) < eps:
        angle_pub.publish(angle_straight)
        return

    ct = scan_msg.header.stamp
    timeinsecs_current = ct.secs + 1e-9 * ct.nsecs
    timeinsecs_past = last_timestamp.secs + 1e-9 * last_timestamp.nsecs
    last_timestamp = ct
    time_diff = timeinsecs_current - timeinsecs_past
    
    drive_left = diff < 0

    current_pos = last_odom.pose.pose.position
    x = current_pos.x
    y = current_pos.y

    print("XY: ", x, y)
    print("Distance to wall: ", current_dist)

    y_far = y - forward_dist # subtract, since we want to move forward.

    angle = atan(abs(diff) / forward_dist)
    steering_angle = angle * 2 / pi * 90


    if not len(last_rects) <= num_rects:
        angle_sum -= last_rects[0]
        last_rects = last_rects[1:]

    last_rects.append(steering_angle * time_diff)

    angle_sum += last_rects[-1]
    angle_diff = abs(last_angle - steering_angle) / time_diff
    last_angle = steering_angle

    steering_angle = Kp * steering_angle + Ki * angle_sum + Kd * angle_diff

    if drive_left: steering_angle = max(steering_angle + angle_straight, 150)
    else: steering_angle = min(steering_angle, 30)

    #else: steering_angle = min(steering_angle, 90)
    if angle_eps <= abs(last_yaw.data - float(steering_angle)):
        print("Steering angle: ", steering_angle)
        angle_pub.publish(steering_angle)

    if not started:
        start_car()
        started = True


#    if len(Xs) > 0 and len(ys) > 0:
#        Xs = np.array(Xs).reshape(len(Xs), 1)
#        ys = np.array(ys).reshape(len(ys), 1)
#
#        model_ransac = linear_model.RANSACRegressor(linear_model.LinearRegression())
#        try:
#            #model_ransac.fit(Xs, ys)
#            model_ransac.fit(ys, Xs)
#            print("Coeffs: ", model_ransac.estimator_.coef_)
#        except:
#            print("NOT ENOUGH POINTS!")

#    for i in range(occupancy_grid.info.width):
#        setCell(i, 

    #pub_grid.publish(occupancy_grid)

rospy.Subscriber("scan", LaserScan, scanCallback, queue_size=5)
rospy.Subscriber("odom", Odometry, odomCallback, queue_size=5)
rospy.Subscriber("/model_car/yaw", Float32, yawCallback, queue_size=5)

start_pub = rospy.Publisher("manual_control/stop_start", Int16, queue_size=5)
angle_pub = rospy.Publisher("manual_control/steering", Int16, queue_size=5)
speed_pub = rospy.Publisher("manual_control/speed", Int16, queue_size=5)
#pub_grid = rospy.Publisher("lidar_pid", OccupancyGrid, queue_size=5)

# --- main ---
rospy.init_node("lidar_pid")
#angle_pub.publish(angle_straight)
rospy.spin()
