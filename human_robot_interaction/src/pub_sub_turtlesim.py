#!/usr/bin/env python

import rospy
# from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class Turtlesim:

    def __init__(self):
        
        rospy.init_node('turtlesim_move', anonymous=True)
        self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1000) 
        self.sub = rospy.Subscriber('/turtle1/pose', Pose, self.callback)
        self.speed = Twist()

    def callback(self, data):

        if(data.x <= 1 or data.x >= 9):
            self.speed.linear.x = 0
            self.speed.angular.z = 0
            rospy.signal_shutdown("Reached wall.")

        else:
            self.speed.linear.x = 1
            self.speed.angular.z = 0
        
        self.pub.publish(self.speed)



if __name__ == '__main__':
    print("Running")
    Turtlesim()
    rospy.spin()
    