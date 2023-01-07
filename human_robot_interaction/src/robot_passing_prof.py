#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt
# from tf.transformations import euler_from_quaternion
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import String
import math
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class Husky:

    def __init__(self):
        # Creates a node with name 'husky_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('husky_controller', anonymous=True)

        # Publisher which will publish to the topic '/husky_velocity_controller/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.goals_X = []
        self.goals_Y = []

        self.sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)

        # We used Pose() message for the self.robot_pose because the Odometry msg is bigger than needed, so we saved the values of the variables we're interested in from the Odometry msg into this Pose msg
        self.robot_pose = Pose()
        self.human_pose = Pose()

        self.vel_msg = Twist()
        self.rate = rospy.Rate(10)

        self.passing_state = 0
        self.goal_pose = Pose()
        self.goal_k = 0
        self.i = 0


        while not rospy.is_shutdown():
            if(self.passing_state==0):
                self.go_straight()
            else:
                self.p_controller(self.i)

            self.cmd_vel_pub()

    def cmd_vel_pub(self):

        # if(self.passing_state == 0):

        #     # Robot moves forward
        #     self.vel_msg.linear.x = 0.5
        #     self.vel_msg.linear.y = 0
        #     self.vel_msg.linear.z = 0

        #     self.vel_msg.angular.x = 0
        #     self.vel_msg.angular.y = 0
        #     self.vel_msg.angular.z = 0

        # elif(self.passing_state == 1):

        #     if(self.goal_k < 2):

        #         self.goal_pose.x = self.goals_X[self.goal_k]
        #         self.goal_pose.y = self.goals_Y[self.goal_k]
        #         # print("goal pose: %f,%f"%(self.goal_pose.x,self.goal_pose.y))
        #         distance_tolerance = 0.5

        #         if(self.euclidean_distance(self.goal_pose) >= distance_tolerance):

        #                 # Calculating errors
        #                 self.dist_error_X = self.goal_pose.x - self.robot_pose.x
        #                 self.dist_error_Y = self.goal_pose.y - self.robot_pose.y
        #                 self.orientation_error = self.steering_angle(self.goal_pose) - self.robot_pose.theta

        #         else:
        #             self.goal_k += 1
        #             print("next goal" + str(self.goal_k))

        #         self.vel_msg.linear.x = 0.5
        #         self.vel_msg.linear.y = 0
        #         self.vel_msg.linear.z = 0

        #         # Angular velocity in the z-axis.
        #         self.vel_msg.angular.x = 0
        #         self.vel_msg.angular.y = 0
        #         self.vel_msg.angular.z = self.angular_vel(self.goal_pose)
        #         # self.vel_msg.angular.z = 0

        #     elif(self.goal_k == 2):

        #         # Target reached, stop the robot
        #         self.vel_msg.linear.x = 0
        #         self.vel_msg.linear.y = 0
        #         self.vel_msg.linear.z = 0

        #         self.vel_msg.angular.x = 0
        #         self.vel_msg.angular.y = 0
        #         self.vel_msg.angular.z = 0

        #     # Publish vel_msg
        self.velocity_publisher.publish(self.vel_msg)

    def go_straight(self):
        self.vel_msg.linear.x = 1.0
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0

        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
        self.vel_msg.angular.z = 0


    def callback(self, data):

        if(len(data.pose) < 3):
            return


        # Extracting positon info from /gazebo/model_states
        self.human_pose.x = data.pose[1].position.x
        self.human_pose.y = data.pose[1].position.y

        # print("Human Pose X: ", self.human_pose.x)
        # print("Human Pose Y: ", self.human_pose.y)

        self.robot_pose.x = data.pose[-1].position.x
        self.robot_pose.y = data.pose[-1].position.y
        #self.robot_pose.theta = data.pose[-1].orientation.w*2


        orientation_q = data.pose[-1].orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        self.robot_pose.theta = yaw
        # print("Robot Pose X: ", self.robot_pose.x)
        # print("Robot Pose Y: ", self.robot_pose.y)

        dist = sqrt( pow(self.human_pose.x - self.robot_pose.x, 2) + pow(self.human_pose.y - self.robot_pose.y, 2) )
        # print(dist)

        if ( dist < 10.0 and self.passing_state == 0):

            self.passing_state = 1
            self.goals_X.append(self.human_pose.x-3)
            self.goals_Y.append(self.human_pose.y-3)

            self.goals_X.append(self.human_pose.x-3+25)
            self.goals_Y.append(self.human_pose.y-3)

            print("Update the goal position to: " + str(self.goals_X[0]) + " " + str(self.goals_Y[0]))

        # print("Goals X: ", self.goals_X)
        # print("Goals Y: ", self.goals_Y)

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.robot_pose.x), 2) + pow((goal_pose.y - self.robot_pose.y), 2))

    def linear_vel(self, goal_pose, constant=0.3):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return atan2(goal_pose.y - self.robot_pose.y, goal_pose.x - self.robot_pose.x)

    def angular_vel(self, goal_pose, constant=1.2):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * (self.steering_angle(goal_pose) - self.robot_pose.theta)

    def p_controller(self, i):

        # K configuration
        ka = 2.5
        kb = -1.1
        kp = 1

        # Initializing Index
        # i = 0

        # Desired destination
        d_posX = self.goals_X[i]
        d_posY = self.goals_Y[i]
        d_theta = 0


        # Current location
        c_posX = self.robot_pose.x
        c_posY = self.robot_pose.y
        c_theta = self.robot_pose.theta

        # print("c_posX = ", c_posX, "c_posY = ", c_posY, "d_posX = ", d_posX, "d_posY = ", d_posY)


        # Calculating delta X and Y in the robot's global frame
        deltax = d_posX - c_posX
        deltay = d_posY - c_posY
        deltaz = 1

        # Rotation matrix about Z-axis from the goal frame to the global frame
        rotz = np.array([ [math.cos(-d_theta), -math.sin(-d_theta), 0], [math.sin(-d_theta), math.cos(-d_theta), 0], [0, 0, 1] ])

        # Finding the values of delta X, delta Y, and delta Z with respect to the global frame
        deltas = [[deltax], [deltay], [deltaz]]
        [deltaxg,deltayg,deltazg]=np.matmul(rotz, deltas)
        deltas2=[deltaxg,deltayg,deltazg]

        # Defining rho, alpha, and beta
        rho = math.sqrt( (deltaxg)**2 + (deltayg)**2 )
        alpha = (-c_theta + math.atan2( (deltay), (deltax)))
        beta = ( math.atan2( (deltay), (deltax)) - d_theta)

        if(alpha>math.pi):
            alpha-=math.pi*2
        if(alpha<-math.pi):
            alpha+=math.pi*2

        if(beta>math.pi):
            beta-=math.pi*2
        if(beta<-math.pi):
            beta+=math.pi*2

        # print("c_theta = ", round(c_theta,4), "rho=", round(rho,4), "alpha = ", round(alpha,4), "beta= ", round(beta,4))


        # Updating the linear and angular velocities
        c_v = kp * rho
        c_w = ka * alpha + kb * beta

        # Updating current local velocity configurations
        # if (alpha <= math.pi/2 and alpha >= (-(math.pi/2))):

        # # Forward motion control law

        #     if (alpha > math.pi):
        #         alpha = math.pi
        #     if (alpha < -math.pi):
        #         alpha = -math.pi
        #     if (beta > math.pi):
        #         beta = math.pi
        #     if (beta < -math.pi):
        #         beta = -math.pi

        #     # Updating the linear and angular velocities
        #     c_v = kp * rho
        #     c_w = ka * alpha + kb * beta
        #     # print(alpha)
        #     # print(beta)

        #     # Constraining the wheel speeds to less than 16 rad/sec
        #     if (c_v > 24):
        #         c_v = 24

        #     if (c_w > 2):
        #         c_w = 2

        # else:

        #     # Backward motion control law
        #     alpha = (-c_theta + math.atan2(-(d_posY - c_posY), -(d_posX - c_posX)))
        #     beta = -c_theta - alpha

        #     while (alpha > math.pi or alpha < -math.pi):
        #         if (alpha > 0):
        #             alpha = alpha - 2 * math.pi
        #             # print(alpha)
        #         if (alpha < 0):
        #             alpha = alpha + 2 * math.pi
        #             # print(alpha)

        #     if (alpha > math.pi):
        #         alpha = math.pi
        #     if (alpha < -math.pi):
        #         alpha = -math.pi
        #     if (beta > math.pi):
        #         beta = math.pi
        #     if (beta < -math.pi):
        #         beta = -math.pi

        #     # Updating the linear and angular velocities
        #     c_v = -kp * rho
        #     c_w = (ka * alpha + kb * beta)
        #     # print(alpha)
        #     # print(beta)

        #     # Constraining the wheel speeds to less than 16 rad/sec
        #     if (c_v < -24):
        #         c_v = -24

        #     if (c_w < -2):
        #         c_w = -2

        if( c_v > 1.5 ):
            c_v = 1.5

        if( c_w > 1.0 ):
            c_w = 1.0

        self.vel_msg.linear.x = c_v
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0

        # Angular velocity in the z-axis.
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
        self.vel_msg.angular.z = c_w


        # # Calculating robot wheel speeds -- phi_l and phi_r are the robot's wheel speeds
        # r = 16.5 			# robot wheel radius in cm
        # L = 60			# robot chassis radius in cm

        # robot_param_mat = np.array([ [r/2, r/2], [r/(2*L), -r/(2*L)] ])
        # inv_robot_param_mat = np.linalg.inv(robot_param_mat)
        # [phi_r, phi_l] = np.matmul( inv_robot_param_mat, [ [c_v], [c_w]] )		# unit rad/sec


        if abs(c_posX - d_posX) < 1.8 and abs(c_posY - d_posY) < 1.8 and abs(c_theta - d_theta) < 0.4:

            self.i = self.i + 1

            if(self.i < 2):
                self.p_controller(self.i)

            print("final goal reached")

            self.vel_msg.linear.x = 0
            self.vel_msg.linear.y = 0
            self.vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            self.vel_msg.angular.x = 0
            self.vel_msg.angular.y = 0
            self.vel_msg.angular.z = 0

        #else:
            # print("one goal point reached, continute to next goal point")

        # return False


if __name__ == '__main__':
    try:
        Husky()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass