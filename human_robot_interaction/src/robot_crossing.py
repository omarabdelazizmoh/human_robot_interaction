#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt
from tf.transformations import euler_from_quaternion
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import String


class Husky:

    def __init__(self):
        # Creates a node with name 'husky_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('husky_controller', anonymous=True)

        # Publisher which will publish to the topic '/husky_velocity_controller/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=10)

        self.sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)

        # We used Pose() message for the self.robot_pose because the Odometry msg is bigger than needed, so we saved the values of the variables we're interested in from the Odometry msg into this Pose msg
        self.robot_pose = Pose()
        self.store_robot_pose = Pose()
        self.human_pose = Pose()
        self.vel_msg = Twist()
        self.rate = rospy.Rate(10)

    
    def callback(self, data):

        # Extracting positon info from /gazebo/model_states
        self.human_pose.x = data.pose[1].position.x
        self.human_pose.y = data.pose[1].position.y
        
        # self.robot_pose.x = data.pose[2].position.x
        # self.robot_pose.y = data.pose[2].position.y

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

    def crossing_scenario(self):
        
        threshold = 5.25
        eucl_dist = sqrt(pow((self.human_pose.x - self.robot_pose.x), 2) + pow((self.human_pose.y - self.robot_pose.y), 2))
        print("human_pose x = ", self.human_pose.x)
        print("robot_pose x = ", self.robot_pose.x)
        print("human_pose y = ", self.human_pose.y)
        print("robot_pose y = ", self.robot_pose.y)
        print("eucl dist = ", eucl_dist)
        print("-------------------------------------")

        if(eucl_dist > threshold): # Robot moving when safe to do so

            # Linear velocity in the x-axis.
            self.vel_msg.linear.x = 0.5
            print("move forward")

        else: # Robot waiting for human to safely pass

            # Linear velocity in the x-axis.
            self.vel_msg.linear.x = 0
            print("stop till human passes")

        # Publishing our vel_msg
        self.velocity_publisher.publish(self.vel_msg)


if __name__ == '__main__':
    try:
        x= Husky()
        x.crossing_scenario()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass