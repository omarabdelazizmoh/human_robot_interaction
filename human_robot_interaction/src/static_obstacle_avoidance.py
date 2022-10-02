#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates

class Static_Obstacle_Avoidance:

    def __init__(self):
        
        rospy.init_node('husky_move', anonymous=True)
        self.pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=1000) 
        self.sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)
        self.speed = Twist()

    def callback(self, data):

        # Extracting positon info from /gazebo/model_states
        human_posX = data.pose[2].position.x
        human_posY = data.pose[2].position.y
        # print("Human PosX: ", human_posX)
        # print("Human PosY: ", human_posY)

        robot_posX = data.pose[3].position.x
        robot_posY = data.pose[3].position.y
        # print("Robot PosX: ", robot_posX)
        # print("Robot PosY: ", robot_posY)
        
        # Distance between robot and human
        dist = abs(human_posX - robot_posX)
        
        # Defining safe position for robot to move to when coming close to human in the passing scenario
        self.safe_posX = human_posX
        self.safe_posY = human_posY - 3

        # Robot moving/stopping conditions
        if(dist < 3):
            
            # Setting robot speeds
            self.speed.linear.x = 0
            self.speed.angular.z = 0
            print("Distance is less than 3.")

        else:
            # Setting robot speeds
            self.speed.linear.x = 1
            self.speed.angular.z = 0
            print("Keep going.")
 
        # Publishing robot speeds to robot's /cmd_vel
        self.pub.publish(self.speed)


if __name__ == '__main__':
    Static_Obstacle_Avoidance()
    rospy.spin()