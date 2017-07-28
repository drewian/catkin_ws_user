#!/usr/bin/env python

# --- imports ---
import rospy
from math import sqrt
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from std_msgs.msg import String

# --- definitions ---
epsilon = 0.05   # allowed inaccuracy for distance calculation
speed_rpm = -400
angle_left = 0
angle_straight = 90
angle_right = 180
last_odom = None
last_yaw  = None
is_active = False
kp = 30


def callbackOdom(msg):
    global last_odom
    last_odom = msg


def callbackyaw(msg):
   global last_yaw 
   last_yaw = msg

def waitForFirstValues():
    while not rospy.is_shutdown() and (last_odom is None or last_yaw is None):
        rospy.loginfo(
            "%s: No initial odometry message received. Waiting for message...",
            rospy.get_caller_id())
        rospy.sleep(1.0)
rospy.init_node("simple_drive_control")
sub_odom = rospy.Subscriber("odom", Odometry, callbackOdom, queue_size=100)
sub_yaw  = rospy.Subscriber("/model_car/yaw", Float32, callbackyaw)

pub_stop_start = rospy.Publisher(
    "manual_control/stop_start",
    Int16,
    queue_size=100)
pub_speed = rospy.Publisher("manual_control/speed", Int16, queue_size=100)
pub_steering = rospy.Publisher(
    "manual_control/steering",
    Int16,
    queue_size=100)

def drive(distance=5, speed=speed_rpm, angle=angle_straight):

    # stop the car and set desired steering angle + speed
    pub_speed.publish(0)
    pub_stop_start.publish(1)
    rospy.sleep(1)
    pub_steering.publish(angle)
    pub_stop_start.publish(0)
    rospy.sleep(1)
    pub_speed.publish(speed)

    yaw_target = last_yaw

    start_pos = last_odom.pose.pose.position
    current_distance = 0

    while not rospy.is_shutdown() and current_distance < (distance - epsilon):
        current_pos = last_odom.pose.pose.position
        current_distance = sqrt(
            (current_pos.x - start_pos.x)**2 + (current_pos.y - start_pos.y)**2)
        # rospy.loginfo("current distance = %f", current_distance)
        err = ((last_yaw.data - yaw_target.data + 180) % 360) - 180
        control = max(0,min(180, err * kp + angle_straight))
        pub_steering.publish(control)
        rospy.sleep(0.1)

    pub_speed.publish(0)
    is_active = False
    current_pos = last_odom.pose.pose.position
    current_distance = sqrt((current_pos.x - start_pos.x)
                            ** 2 + (current_pos.y - start_pos.y)**2)
    pub_info.publish("FINISHED")

    rospy.loginfo(
        "%s: Finished %s(%f)\nActual travelled distance = %f",
        rospy.get_caller_id(),
        command,
        distance,
        current_distance)


if __name__ == "__main__":
    waitForFirstValues()
    drive()

