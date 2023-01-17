#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
from sklearn import preprocessing

def trajectory_pub():
    
    pub = rospy.Publisher('path_topic', Path, queue_size=10)
    rospy.init_node('rviz_trajectory_pub_node', anonymous=True)
    rate = rospy.Rate(1) # 1 hz

    # create a path message to be published
    path_msg = PoseStamped()
    path_msg.header.seq = 0
    path_msg.header.stamp = rospy.Time.now()
    path_msg.header.frame_id = "map"

    # create a pose message
    pose_msg = PoseStamped()
    pose_msg.header.seq = 0
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.header.frame_id = "map"

    # Initializing pose
    pose_msg.pose.position.x = 0.0
    pose_msg.pose.position.y = 0.0
    pose_msg.pose.position.z = 0.0

    # Initializing orientation
    yaw_angle = 0.0
    q = quaternion_from_euler(0,0,0)
    pose_msg.pose.orientation.x = q[0]
    pose_msg.pose.orientation.y = q[1]
    pose_msg.pose.orientation.z = q[2]
    pose_msg.pose.orientation.w = q[3]

    # add the pose to the path
    path_msg.pose.append(pose_msg)


    while not rospy.is_shutdown():

        # Update the header information
        path_msg.header.seq += 1
        path_msg.header.stamp = rospy.Time.now()

        pose_msg.header = path_msg.header
        pose_msg.pose.position.x += 1.0

        yaw_angle += 0.1
        q = quaternion_from_euler(0,0,yaw_angle)
        q = preprocessing.normalize(q)

        pose_msg.pose.orientation.x = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]

        path_msg.pose.append(pose_msg)
        
        # add the pose to the path
        path_msg.append(pose_msg.pose)

        # publish the message
        pub.publish(path_msg)

        # sleep for the time remaining to let us hit our 1Hz publish rate
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass