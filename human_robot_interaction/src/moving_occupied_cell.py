#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from gazebo_msgs.msg import ModelStates
from turtlesim.msg import Pose

class OccupancyGridPublisher:
    def __init__(self):
        self.map_pub = rospy.Publisher("map", OccupancyGrid, queue_size=10)
        self.sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)
        self.x = 0 #initial x position of the occupied cell
        self.y = 0 #initial y position of the occupied cell
        self.speed = 0.2 #speed of the occupied cell
        self.width = 100
        self.height = 100
        self.human_pose = Pose()

    def callback(self, data):

        # Extracting positon info from /gazebo/model_states
        self.human_pose.x = data.pose[1].position.x
        self.human_pose.y = data.pose[1].position.y
    
    
    def publish_grid(self):
        map_msg = OccupancyGrid()
        map_msg.header = Header()
        map_msg.header.stamp = rospy.Time.now()
        map_msg.header.frame_id = "map_tf"
        map_msg.info.resolution = 0.4
        map_msg.info.width = self.width
        map_msg.info.height = self.height
        map_msg.info.origin.position.x = -15.0
        map_msg.info.origin.position.y = -15.0
        map_msg.info.origin.position.z = 0.0
        map_msg.data = [0]*(self.width*self.height) #Initialize all cells as free
        map_msg.data[int(self.y*self.width+self.x )] = 100 #Mark the current position of the occupied cell
        self.map_pub.publish(map_msg)
        self.x = self.human_pose.x
        self.y = self.human_pose.y
        # self.x += self.speed #update the x position of the occupied cell
        # if the occupied cell reach the end of the grid, move it back to the initial position
        # if self.x>=30:
        #     self.x=5
        #     self.y=5


if __name__ == "__main__":
    rospy.init_node("occupancy_grid_publisher")
    pub = OccupancyGridPublisher()
    rate = rospy.Rate(10) #10Hz
    while not rospy.is_shutdown():
        pub.publish_grid()
        rate.sleep()
