#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelStates
from turtlesim.msg import Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class live_env_map_updater:
     
    def __init__(self):
     # We used Pose() message for the self.robot_pose because the Odometry msg is bigger than needed, so we saved the values of the variables we're interested in from the Odometry msg into this Pose msg
        self.robot_pose = Pose()
        self.human_pose = Pose()

    def callback(data):
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
        
    def listener():

        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('live_env_map', anonymous=True)

        rospy.Subscriber("live_env_map_updater", self.callback)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

if __name__ == '__main__':
    live_env_map()