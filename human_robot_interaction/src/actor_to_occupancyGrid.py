#!/usr/bin/env python3
import rospy
from nav_msgs.msg import OccupancyGrid
from gazebo_msgs.msg import ModelStates
from turtlesim.msg import Pose


class actor_occupancy_grid():
    def __init__(self):
        rospy.init_node('og_from_actor', anonymous=True)
        # Create the publisher
        self.map_publisher= rospy.Publisher('map', OccupancyGrid, queue_size=10)
        self.model_states_subscriber = rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)
        # Set the rate at which messages will be published
        self.grid = OccupancyGrid()
        self.human_pose = Pose()

    def callback(self, data):
        self.human_pose.x = data.pose[1].position.x
        self.human_pose.y = data.pose[1].position.y

            # Fill the grid message with data
        self.grid.header.stamp = rospy.Time.now()
        self.grid.header.frame_id = "map_tf"
        self.grid.info.resolution = 0.05
        self.grid.info.width = 100
        self.grid.info.height = 100
        self.grid.info.origin.position.x = 0
        self.grid.info.origin.position.y = 0
        self.grid.data = [0] * (self.grid.info.width * self.grid.info.height)

        # Fill the data around a point as occupied
        center_x = 50
        center_y = 50
        radius = 5
        for x in range(self.grid.info.width):
            for y in range(self.grid.info.height):
                if ((x - center_x) ** 2 + (y - center_y) ** 2) <= radius ** 2:
                    self.grid.data[y * self.grid.info.width + x] = 100

        # Working on Dynamic HUman Location updating in Occupancy grid

        # x = int((self.human_pose.x - self.grid.info.origin.position.x) / self.grid.info.resolution)
        # y = int((self.human_pose.y - self.grid.info.origin.position.y) / self.grid.info.resolution)
        # index = y * self.grid.info.width + x
        # data[index] = 100  # Mark the cell where the robot is located as occupied
        # self.grid.data = data.tolist()


        # Publish the message
        self.map_publisher.publish(self.grid)
        print("H_X | ", self.human_pose.x , "//  H_Y | " , self.human_pose.y)


if __name__ == '__main__':
    try:
        actor_occupancy_grid()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass