#!/usr/bin/env python
import rospy
from human_robot_interaction.msg import Act_msg

def main():

    rospy.init_node('move_human')
    
    pub = rospy.Publisher('actor1/vel_cmd', Act_msg, queue_size=10)

    rate = rospy.Rate(10) 
    while not rospy.is_shutdown():
        
            # Publisher 
            cmd_vel = Act_msg()
            cmd_vel.target_x = -25
            cmd_vel.target_y = 0
            cmd_vel.velocity = 0.5

            pub.publish(cmd_vel)
    

if __name__ =='__main__':
    try:
        main()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

