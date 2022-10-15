#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(data):
    
    string_var1 = String()
    string_var1.data = data

    string_var2 = String()
    string_var2 = data

    # print(string_var1," / ", data, " / ", string_var2)

    if(string_var1.data == "crossing"):
            print("CROSSING 1")

    if(string_var2.data == "crossing"):
            print("CROSSING 2")

def listener():

    rospy.init_node('control', anonymous=True)

    rospy.Subscriber('human_behavior', String, callback)
    rospy.spin()    

if __name__ == '__main__':
    print("Running")
    listener()