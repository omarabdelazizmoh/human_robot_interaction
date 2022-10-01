#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty
from std_msgs.msg import Int16
# import turtlesim
# from geometry_msgs import Twist


last_data = ""
started = False
pub = rospy.Publisher('num_addition_publisher', Int16, queue_size=1000) 

def callback(data):
    print("New message received")
    global started, last_data
    last_data = data.data + 21
    print "last_data = ", last_data     #Python 2
    if (not started):
        started = True

def timer_callback(event):
    global started, pub, last_data
    if (started):
        
        pub.publish(last_data)
        print("Last message published")


def listener():

    rospy.init_node('control', anonymous=True)

    rospy.Subscriber('num_publisher', Int16, callback)
    timer = rospy.Timer(rospy.Duration(0.5), timer_callback)

    rospy.spin()    
    timer.shutdown()

if __name__ == '__main__':
    print("Running")
    listener()