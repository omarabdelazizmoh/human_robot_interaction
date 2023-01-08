#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt
from tf.transformations import euler_from_quaternion
from gazebo_msgs.msg import ModelStates


class Husky:

    def __init__(self):
        # Creates a node with name 'husky_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('husky_controller', anonymous=True)

        # Publisher which will publish to the topic '/husky_velocity_controller/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=10)

        # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
        # when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber('/husky_velocity_controller/odom', Odometry, self.update_pose)

        

        self.sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)                                                

        # We used Pose() message for the self.robot_pose because the Odometry msg is bigger than needed, so we saved the values of the variables we're interested in from the Odometry msg into this Pose msg
        self.robot_pose = Pose()
        self.rate = rospy.Rate(10)

    
    def callback(self, data):

        # Extracting positon info from /gazebo/model_states
        human_pose = Pose()
        human_pose.x = data.pose[2].position.x
        human_pose.y = data.pose[2].position.y

        # robot_posX = data.pose[3].position.x
        # robot_posY = data.pose[3].position.y
        # print("Robot PosX: ", robot_posX)
        # print("Robot PosY: ", robot_posY)
        
        # Distance between robot and human
        # dist = abs(human_pose.x - robot_posX)
        dist = self.euclidean_distance(human_pose)
        print("Eucl dist = ", dist)
        
        # Defining safe position for robot to move to when coming close to human in the passing scenario
        # self.safe_posX = human_pose.x
        # self.safe_posY = human_pose.y - 3

        # Robot moving/stopping conditions
        if(dist < 5):

            self.goals_X = [20,0,0]
            self.goals_Y = [0,0,0]
            
            # Setting robot speeds
            self.goals_X = [human_pose.x - 3, human_pose.x - 1,     human_pose.x,     human_pose.x + 1,     human_pose.x + 3]
            self.goals_Y = [human_pose.y,     human_pose.y - 1.5,   human_pose.y - 3, human_pose.y -1.5,    human_pose.y]
            print("Distance is less than 5.")

        else:
            # Setting robot speeds
            pass
            print("Keep going.")

        # print("human_pose.x: ", human_pose.x)
        # print("human_pose.y: ", human_pose.y)

        # # Target points
        # self.goals_X = [human_pose.x - 3, human_pose.x, human_pose.x + 3]
        # self.goals_Y = [human_pose.y, human_pose.y - 3, human_pose.y]


    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        
        self.robot_pose.x = data.pose.pose.position.x
        self.robot_pose.y = data.pose.pose.position.y

        q = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]

        (self.theta_x, self.theta_y, self.robot_pose.theta) = euler_from_quaternion(q)
        # print("Pose X Update: ", self.robot_pose.x)


    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.robot_pose.x), 2) + pow((goal_pose.y - self.robot_pose.y), 2))

    def linear_vel(self, goal_pose, constant=0.5):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return atan2(goal_pose.y - self.robot_pose.y, goal_pose.x - self.robot_pose.x)

    def angular_vel(self, goal_pose, constant=1):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * (self.steering_angle(goal_pose) - self.robot_pose.theta)

    def move2goal(self):
        """Moves the turtle to the goal."""
        goal_pose = Pose()

        for i in range( len(self.goals_X) ):

            # Get the input from the user.
            goal_pose.x = self.goals_X[i]
            goal_pose.y = self.goals_Y[i]
            
            # Please, insert a number slightly greater than 0 (e.g. 0.01).
            distance_tolerance = 0.5

            vel_msg = Twist()
            # print("For loop")

            if(self.euclidean_distance(goal_pose)) > 5:
                 
                 # Linear velocity in the x-axis.
                    vel_msg.linear.x = 1
                    vel_msg.linear.y = 0
                    vel_msg.linear.z = 0

            else:

                while self.euclidean_distance(goal_pose) >= distance_tolerance:

                    # Porportional controller.
                    # https://en.wikipedia.org/wiki/Proportional_control

                    # Calculating errors
                    self.dist_error_X = goal_pose.x - self.robot_pose.x
                    self.dist_error_Y = goal_pose.y - self.robot_pose.y
                    self.orientation_error = self.steering_angle(goal_pose) - self.robot_pose.theta
                    
                    # Print errors    
                    # print("Error in X: ", self.dist_error_X)
                    # print("Error in Y: ", self.dist_error_Y)
                    # print("Error in Orientation: ", self.orientation_error)

                    # Linear velocity in the x-axis.
                    vel_msg.linear.x = self.linear_vel(goal_pose)
                    vel_msg.linear.y = 0
                    vel_msg.linear.z = 0

                    # Angular velocity in the z-axis.
                    vel_msg.angular.x = 0
                    vel_msg.angular.y = 0
                    vel_msg.angular.z = self.angular_vel(goal_pose)

                    # Publishing our vel_msg
                    self.velocity_publisher.publish(vel_msg)

                    # Publish at the desired rate.
                    # self.rate.sleep()

                        
                # Stopping our robot after the movement is over.
                vel_msg.linear.x = 0
                vel_msg.angular.z = 0
                self.velocity_publisher.publish(vel_msg)
                print("Reached goal ", i)

                # Get the input from the user.
                goal_pose.x = self.goals_X[i]
                goal_pose.y = self.goals_Y[i]

                if(i == 2):
                    rospy.signal_shutdown("All goals reached.")

                # If we press control + C, the node will stop.
                # rospy.spin()

if __name__ == '__main__':
    try:
        Husky()
        # x.move2goal()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass