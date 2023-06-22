#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt
# from tf.transformations import euler_from_quaternion
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import OccupancyGrid
import math
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class Husky:
    def __init__(self):
        rospy.init_node('husky_controller_gridding', anonymous=True)

        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.robot_pose = Pose()
        self.human_pose = Pose()
        self.vel_msg = Twist()
        self.goals_X, self.goals_Y = [], []
        self.passing_state, self.i = 0, 0
        self.min_distance_human_robot = float('inf')


        self.sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)
        self.drive()

    def drive(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.go_straight() if self.passing_state == 0 else self.p_controller()
            self.cmd_vel_pub()
            rate.sleep()



    def cmd_vel_pub(self):
        self.velocity_publisher.publish(self.vel_msg)

    def go_straight(self):
        self.vel_msg.linear.x = 1.0
        self.vel_msg.angular.z = 0

    def callback(self, data):
        if len(data.pose) < 3:
            return

        self.update_human_pose(data.pose[1].position)
        self.update_robot_pose(data.pose[-1])

        self.distance_from_human = sqrt(pow(self.human_pose.x - self.robot_pose.x, 2) + pow(self.human_pose.y - self.robot_pose.y, 2))
        self.min_distance_human_robot = min(self.min_distance_human_robot, self.distance_from_human)


        if self.distance_from_human < 10.0 and self.passing_state == 0:
            self.update_passing_state()

    def update_human_pose(self, position):
        self.human_pose.x, self.human_pose.y = position.x, position.y

    def update_robot_pose(self, pose):
        self.robot_pose.x, self.robot_pose.y = pose.position.x, pose.position.y
        _, _, self.robot_pose.theta = euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])


    def update_passing_state(self):
        self.passing_state = 1
        self.goals_X.append(self.human_pose.x-3)
        self.goals_X.append(self.human_pose.x-3+25)
        self.goals_Y.append(self.human_pose.y-3)
        self.goals_Y.append(self.human_pose.y-3)
        print(f"Update the goal position to: {self.goals_X[0]} {self.goals_Y[0]}")


    def p_controller(self):
            ka = 2.5
            kb = -1.1
            kp = 1
            d_posX, d_posY = self.goals_X[self.i], self.goals_Y[self.i]
            d_theta = 0
            c_posX, c_posY, c_theta = self.robot_pose.x, self.robot_pose.y, self.robot_pose.theta
            deltax, deltay, deltaz = d_posX - c_posX, d_posY - c_posY, 1

            rotz = np.array([ [math.cos(-d_theta), -math.sin(-d_theta), 0], [math.sin(-d_theta), math.cos(-d_theta), 0], [0, 0, 1] ])
            deltas = [[deltax], [deltay], [deltaz]]
            deltaxg, deltayg, deltazg = np.matmul(rotz, deltas).flatten()

            rho = np.sqrt( deltaxg**2 + deltayg**2 )
            alpha = -c_theta + np.arctan2(deltay, deltax)
            beta = np.arctan2(deltay, deltax) - d_theta

            # Ensure alpha and beta are within the range [-pi, pi]
            alpha = (alpha + np.pi) % (2 * np.pi) - np.pi
            beta = (beta + np.pi) % (2 * np.pi) - np.pi

            c_v, c_w = kp * rho, ka * alpha + kb * beta

            # Limit the velocities to a maximum value
            if self.distance_from_human < 3:
                c_v = np.clip(c_v, 0.254, 0.381)
            else:
                c_v = min(c_v, 1.5)

            c_w = min(c_w, 1.0)

            self.vel_msg.linear.x = c_v
            self.vel_msg.angular.z = c_w

            if abs(c_posX - d_posX) < 1.8 and abs(c_posY - d_posY) < 1.8 and abs(c_theta - d_theta) < 0.4:
                self.i += 1
                print("Goal Reached")
                if self.i < len(self.goals_X):
                    self.p_controller()
                    print("Minimum distance from human:", self.min_distance_human_robot)
                    self.vel_msg.linear.x = 0
                    self.vel_msg.angular.z = 0

            return self.i




if __name__ == '__main__':
    try:
        Husky()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass