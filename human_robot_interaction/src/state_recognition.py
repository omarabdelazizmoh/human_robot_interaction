#!/usr/bin/env python
import rospy
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from math import pow, sqrt
from tf.transformations import euler_from_quaternion
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import String

class Human_Behavior:

    def __init__(self):
        # Creates a node with name 'husky_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('state_recognition', anonymous=True)

        # Publisher which will publish to the topic '/husky_velocity_controller/cmd_vel'.
        self.state_publisher = rospy.Publisher('/human_behavior', String, queue_size=10)

        # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
        # when a message of type Pose is received.
        # self.pose_subscriber = rospy.Subscriber('/husky_velocity_controller/odom', Odometry, self.update_pose)

        self.sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)                                                

        # Initializing
        self.slope_prev = 0
        self.eucl_dist_prev = 0

        # Storing previous values for human and robot poses
        self.human_pose_x_prev = 0
        self.human_pose_y_prev = 0

        self.robot_pose_x_prev = 0
        self.robot_pose_y_prev = 0
        
        # We used Pose() message for the self.robot_pose because the Odometry msg is bigger than needed, so we saved the values of the variables we're interested in from the Odometry msg into this Pose msg
        self.robot_pose = Pose()
        self.human_pose = Pose()
        self.state = String()
        self.rate = rospy.Rate(10)

    
    def callback(self, data):

        # Extracting positon info from /gazebo/model_states
        self.human_pose.x = data.pose[2].position.x
        self.human_pose.y = data.pose[2].position.y
        
        self.robot_pose.x = data.pose[3].position.x
        self.robot_pose.y = data.pose[3].position.y

        # Calculate the instantaneous slopes of the human and robot during their motion
        # slope_new =  (self.human_pose.y - self.robot_pose.y)/(self.human_pose.x - self.robot_pose.x)
        # print("Slope prev = ", self.slope_prev, " Slope new = ", slope_new)

        # Calculating Euclidean distances at two different time instances
        eucl_dist_new = self.euclidean_distance()
        # print("Eucl dist prev = ", self.eucl_dist_prev, "Eucl dist new = ", eucl_dist_new)

        # Delta position
        delta_human_x = abs(self.human_pose.x - self.human_pose_x_prev)
        delta_human_y = abs(self.human_pose.y - self.human_pose_y_prev)

        delta_robot_x = abs(self.robot_pose.x - self.robot_pose_x_prev)
        delta_robot_y = abs(self.robot_pose.y - self.robot_pose_y_prev)

        print("delta human x = ", delta_human_x)
        print("delta human y = ", delta_human_y)

        # Test if the human is moving along the X or Y axis (if any)
        if( (eucl_dist_new - self.eucl_dist_prev) < 0 ):

            if( (delta_human_x > 0 and (self.human_pose.y == self.human_pose_y_prev)) ):
                print("Human moving along X axis")

                if( (delta_robot_x > 0 and (self.robot_pose.y == self.robot_pose_y_prev)) ):
                    print("passing")
                    self.state = "passing"

                elif( (delta_robot_y > 0 and (self.robot_pose.x == self.robot_pose_x_prev)) ):
                    print("crossing")
                    self.state = "crossing"

            elif( delta_human_y > 0 and (self.human_pose.x == self.human_pose_x_prev) ):
                print("Human moving along Y axis")

                if( (delta_robot_x > 0 and (self.robot_pose.y == self.robot_pose_y_prev)) ):
                    print("crossing")
                    self.state = "crossing"

                elif( (delta_robot_y > 0 and (self.robot_pose.x == self.robot_pose_x_prev)) ):
                    print("passing")
                    self.state = "passing"

        else:
            print("Moving away")

        # Update values
        self.human_pose_x_prev = self.human_pose.x
        self.human_pose_y_prev = self.human_pose.y

        # Comparing Euclidean distances
        # Passing scenario
        # if( (eucl_dist_new - self.eucl_dist_prev) < 0 ):
        #     # print("passing")
        #     self.state = "passing"
        #     # self.slope_prev = slope_new
        #     self.eucl_dist_prev = eucl_dist_new
        
        # # Crossing scenario
        # elif( (eucl_dist_new - self.eucl_dist_prev) > 0 ):
        #     # print("crossing")
        #     self.state = "crossing"
        #     # self.slope_prev = slope_new
        #     self.eucl_dist_prev = eucl_dist_new

        # else:
        #     # print("error")
        #     self.state = "error"

        self.state_publisher.publish(self.state)


    # def update_pose(self, data):
    #     """Callback function which is called when a new message of type Pose is
    #     received by the subscriber."""
        
    #     self.robot_pose.x = data.pose.pose.position.x
    #     self.robot_pose.y = data.pose.pose.position.y

    #     q = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]

    #     (self.theta_x, self.theta_y, self.robot_pose.theta) = euler_from_quaternion(q)


    def euclidean_distance(self):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((self.human_pose.x - self.robot_pose.x), 2) + pow((self.human_pose.y - self.robot_pose.y), 2))


if __name__ == '__main__':
    try:
        Human_Behavior()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass