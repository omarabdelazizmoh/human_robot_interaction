#!/usr/bin/env python3
import rospy
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from math import pow, sqrt
from tf.transformations import euler_from_quaternion
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import String
from std_msgs.msg import Int8

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
        self.human_side = String()
        self.human_nearby = Int8()
        self.rate = rospy.Rate(10)


    def callback(self, data):

        # Extracting positon info from /gazebo/model_states
        self.human_pose.x = data.pose[1].position.x
        self.human_pose.y = data.pose[1].position.y

        self.robot_pose.x = data.pose[2].position.x
        self.robot_pose.y = data.pose[2].position.y

        # Calculate the instantaneous slopes of the human and robot during their motion
        # slope_new =  (self.human_pose.y - self.robot_pose.y)/(self.human_pose.x - self.robot_pose.x)
        # print("Slope prev = ", self.slope_prev, " Slope new = ", slope_new)

        # Calculating Euclidean distances at two different time instances
        eucl_dist_new = self.euclidean_distance()
        # print("Eucl dist prev = ", self.eucl_dist_prev, "Eucl dist new = ", eucl_dist_new)

        # Initializing threshold
        dist_threshold = 10

         # Testing whether human is near the robot
        if( eucl_dist_new <= dist_threshold ): # Human is near the robot
            self.human_nearby = 1

        else:   #Human is far away from robot
            self.human_nearby = 0

        # Delta position
        delta_human_x = abs(self.human_pose.x - self.human_pose_x_prev)
        delta_human_y = abs(self.human_pose.y - self.human_pose_y_prev)

        delta_robot_x = abs(self.robot_pose.x - self.robot_pose_x_prev)
        delta_robot_y = abs(self.robot_pose.y - self.robot_pose_y_prev)

        # print("delta human x = ", delta_human_x)
        # print("delta human y = ", delta_human_y)
        # print("delta robot x = ", delta_robot_x)
        # print("delta robot y = ", delta_robot_y)
        # print("--------------------------------")

        if(self.human_nearby): # If the human is near the robot

            # print("Human is near the robot")

            # Test if the human is moving along the X or Y axis (if any)
            if( (eucl_dist_new - self.eucl_dist_prev) < 0 ):

                if( (delta_human_x > 0 and ( round(self.human_pose.y, 3) == round(self.human_pose_y_prev, 3) )) ):
                    # print("Human moving along X axis")

                    if( (delta_robot_x > 0 and ( round(self.robot_pose.y, 3) == round(self.robot_pose_y_prev, 3))) ):
                        # print("passing")
                        self.state = "passing"

                    elif( (delta_robot_y > 0 and ( round(self.robot_pose.x, 3) == round(self.robot_pose_x_prev, 3) )) ):
                        print("crossing")
                        self.state = "crossing"

                elif( delta_human_y > 0 and ( round(self.human_pose.x, 3) == round(self.human_pose_x_prev, 3) ) ):
                    # print("Human moving along Y axis")

                    if( (delta_robot_x > 0 and ( round(self.robot_pose.y, 3) == round(self.robot_pose_y_prev, 3) )) ):
                        print("crossing")
                        self.state = "crossing"

                    elif( (delta_robot_y > 0 and ( round(self.robot_pose.x, 3) == round(self.robot_pose_x_prev, 3) )) ):
                        print("passing")
                        self.state = "passing"

            else:
                # print("Human is moving away")
                pass

        else:
            # print("Human is far away")
            pass



        # Checking which side human exists with respect to robot: "left" or "right"
        # if(self.state == "passing"):

        #     if( (self.robot_pose.y - self.human_pose.y) > 0 and round(self.robot_pose.x - self.robot_pose_x_prev, 3) > 0 ):
        #         print("1. Human is on the right side of the robot.")
        #         self.human_side = "right"

        #     elif( (self.robot_pose.y - self.human_pose.y) > 0 and round(self.robot_pose.x - self.robot_pose_x_prev, 3) < 0 ):
        #         print("2. Human is on the left side of the robot.")
        #         self.human_side = "left"

        #     elif( (self.robot_pose.y - self.human_pose.y) < 0 and round(self.robot_pose.x - self.robot_pose_x_prev, 3) > 0 ):
        #         print("3. Human is on the left side of the robot.")
        #         self.human_side = "left"

        #     elif( (self.robot_pose.y - self.human_pose.y) < 0 and round(self.robot_pose.x - self.robot_pose_x_prev, 3) < 0 ):
        #         print("4. Human is on the right side of the robot.")
        #         self.human_side = "right"

        #     else:
        #         print("The human is moving along the same line as the robot")

        # print("The human is on the ", self.human_side, " side of the robot.")


        # Update values
        self.eucl_dist_prev = eucl_dist_new
        self.human_pose_x_prev = self.human_pose.x
        self.human_pose_y_prev = self.human_pose.y
        self.robot_pose_x_prev = self.robot_pose.x
        self.robot_pose_y_prev = self.robot_pose.y

        # Publish the human behavior
        self.state_publisher.publish(self.state)


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