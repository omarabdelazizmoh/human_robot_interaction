#This code creates a custom ROS node called "dynamic_object_occupancy" that subscribes to the topics "static_map" for the occupancy grid map and "objects" for the dynamic objects location. The set_map function is where the map is stored. The object_callback function is where the occupancy grid is updated and republished with the location of the dynamic objects as occupied. The function world_to_grid converts the location of the object from world coordinates to grid coordinates.
#Note that this is just an example and the object detection and localization should be implemented somewhere else and the location of the object should be published in the "objects" topic.


import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from std_msgs.msg import Header

class DynamicObjectOccupancy:
    def __init__(self):
        self.map_pub = rospy.Publisher("map", OccupancyGrid, queue_size=10)
        self.object_sub = rospy.Subscriber("objects", Point, self.object_callback)
        self.map = None

    def object_callback(self, msg):
        if self.map is None:
            return

        # Update the occupancy grid map to mark the position of the object as occupied
        grid_x, grid_y = self.world_to_grid(msg.x, msg.y)
        self.map.data[grid_y * self.map.info.width + grid_x] = 100
        self.map_pub.publish(self.map)

    def world_to_grid(self, x, y):
        # Convert from world coordinates to grid coordinates
        grid_x = int((x - self.map.info.origin.position.x) / self.map.info.resolution)
        grid_y = int((y - self.map.info.origin.position.y) / self.map.info.resolution)
        return grid_x, grid_y

    def set_map(self, map):
        self.map = map

if __name__ == "__main__":
    rospy.init_node("dynamic_object_occupancy")
    pub = DynamicObjectOccupancy()
    map_sub = rospy.Subscriber("static_map", OccupancyGrid, pub.set_map)
    rospy.spin()
