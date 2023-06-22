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
        # Creates a node with name 'husky_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('husky_controller_gridding', anonymous=True)

        # Publisher which will publish to the topic '/husky_velocity_controller/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.goals_X = []
        self.goals_Y = []

        self.sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)

        # We used Pose() message for the self.robot_pose because the Odometry msg is bigger than needed,
        # so we saved the values of the variables we're interested in from the Odometry msg into this Pose msg
        self.robot_pose = Pose()
        self.human_pose = Pose()

        self.vel_msg = Twist()
        self.rate = rospy.Rate(10)

        self.passing_state = 0
        self.goal_pose = Pose()
        self.goal_k = 0
        self.i = 0


        self.grid = OccupancyGrid()
        self.grid.info.resolution = 1.0  # Each cell is 1m x 1m
        self.grid.info.width = 10  # The grid is 10 cells wide
        self.grid.info.height = 10  # The grid is 10 cells high
        self.grid.info.origin.position.x = -5  # The x position of the left edge of the grid
        self.grid.info.origin.position.y = -5  # The y position of the bottom edge of the grid
        self.grid.data = [0]*self.grid.info.width*self.grid.info.height  # All cells are free


        # Create the subscriber and publisher
        self.sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.update_grid)
        self.pub = rospy.Publisher('dynamic_map', OccupancyGrid, queue_size=10)



        while not rospy.is_shutdown():
            if(self.passing_state==0):
                self.go_straight()
            else:
                self.p_controller(self.i)

            self.cmd_vel_pub()

    def cmd_vel_pub(self):

        #     # Publish vel_msg
        self.velocity_publisher.publish(self.vel_msg)

    def go_straight(self):
        self.vel_msg.linear.x = 1.0
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0

        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
        self.vel_msg.angular.z = 0

    def update_grid(self, data):
        # Clear the grid
        self.grid.data = [0]*self.grid.info.width*self.grid.info.height

        # Mark the human's position as occupied
        human_x = int((data.pose[1].position.x - self.grid.info.origin.position.x) / self.grid.info.resolution)
        human_y = int((data.pose[1].position.y - self.grid.info.origin.position.y) / self.grid.info.resolution)
        self.grid.data[human_y*self.grid.info.width + human_x] = 100  # Occupied cells are marked with 100

        # Mark the robot's position as occupied
        robot_x = int((data.pose[-1].position.x - self.grid.info.origin.position.x) / self.grid.info.resolution)
        robot_y = int((data.pose[-1].position.y - self.grid.info.origin.position.y) / self.grid.info.resolution)
        self.grid.data[robot_y*self.grid.info.width + robot_x] = 100

        # Publish the grid
        self.pub.publish(self.grid)


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
        self.update_grid(data)

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
        # Updating the linear and angular velocities
        c_v = kp * rho
        c_w = ka * alpha + kb * beta



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




if __name__ == '__main__':
    try:
        Husky()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass