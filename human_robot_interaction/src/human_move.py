#!/usr/bin/env python
import rospy
from human_robot_interaction.msg import Act_msg
import sys

def main():

    rospy.init_node('move_human')
    
    pub = rospy.Publisher('actor1/vel_cmd', Act_msg, queue_size=10)

    rate = rospy.Rate(10)
    
    argv = sys.argv[1:] # Takes in every character after the 'space' in the command line as an argument. It also filters out any other spaces.
    print("Arg Value 0 = ", argv[0])
    print("Arg Value 1 = ", argv[1])

    while not rospy.is_shutdown():
        
            # Publisher 
            cmd_vel = Act_msg()
            cmd_vel.target_x = int(argv[0])
            cmd_vel.target_y = int(argv[1])
            cmd_vel.velocity = 0.5

            pub.publish(cmd_vel)
    

if __name__ =='__main__':
    try:
        main()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

