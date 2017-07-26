#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float32

pub = None

def callback(data):
    global pub

    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    pub.publish("Stringified data: " + str(data.data))

    
def listener():
    global pub

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/model_car/yaw", Float32, callback)
    pub = rospy.Publisher('/assignment1_publisher_subscriber', String, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    listener()
