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

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/model_car/yaw", Float32, callback)
    pub = rospy.Publisher('/assignment1_publisher_subscriber', String, queue_size=10)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
