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
        self.human_pose = Pose()
        self.robot_orientation = 0
        self.vel_msg = Twist()
        self.rate = rospy.Rate(10)
        self.if_value = 0

        # Initializing flags
        self.human_path_avoided_flag = False
        self.human_avoided_flag = False




    def callback(self, data):

        # Extracting positon info from /gazebo/model_states
        self.human_pose.x = data.pose[1].position.x
        self.human_pose.y = data.pose[1].position.y

        # print("Human Pose X: ", self.human_pose.x)
        # print("Human Pose Y: ", self.human_pose.y)
        
        self.robot_pose.x = data.pose[-1].position.x
        self.robot_pose.y = data.pose[-1].position.y

        # print("Robot Pose X: ", self.robot_pose.x)
        # print("Robot Pose Y: ", self.robot_pose.y)

        self.robot_orientation = data.pose[-1].orientation.z       

        self.overtaking_scenario()
        

    def euclidean_distance(self):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((self.human_pose.x - self.robot_pose.x), 2) + pow((self.human_pose.y - self.robot_pose.y), 2))

    def overtaking_scenario(self):

        eucl_dist = self.euclidean_distance()
        dist_x = abs(self.human_pose.x - self.robot_pose.x)
        
        print("ED ", round(eucl_dist, 3), " Dx ", round(dist_x, 3)," LV ", round(self.vel_msg.linear.x, 3),
        " AV ", round(self.vel_msg.angular.z, 3), "RO ", round(self.robot_orientation, 3), "HPAF ", self.human_path_avoided_flag,
        "HAF ", self.human_avoided_flag, "IFV ", self.if_value)

        if( eucl_dist <= 8 and self.human_avoided_flag == False ):

            if( self.robot_orientation > -0.35 and self.human_path_avoided_flag == False ):
                self.vel_msg.linear.x = 0.7
                self.vel_msg.angular.z = -0.3

                self.if_value = 1

                if( -0.3 > self.robot_orientation >= -0.35 ):
                    self.human_path_avoided_flag = True
                    self.if_value = 2
            
            else:
                self.vel_msg.linear.x = 0.7
                self.vel_msg.angular.z = 0.3
                self.if_value = 3

                if( self.robot_orientation >= 0 ):
                    self.human_avoided_flag = True
                    self.if_value = 4

        else:
            self.vel_msg.linear.x = 0.7
            self.vel_msg.angular.z = 0
            self.if_value = 5
        
        self.velocity_publisher.publish(self.vel_msg)

if __name__ == '__main__':
    try:
        Husky()
        # x.passing_scenario()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass