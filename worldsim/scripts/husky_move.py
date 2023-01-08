#!/usr/bin/env python
import rospy
from move_base_msgs.msg import MoveBaseActionGoal
# from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

def send_goal_position():
    
    pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)
    rospy.init_node('husky_goal_position')
    rate = rospy.Rate(10) # 10hz
    var = MoveBaseActionGoal()

    # Initializing the Euler angles of the robot
    roll = 0.0
    pitch = 0.0
    yaw = math.pi/2

    # Calculating the Quaternion from the Euler angles
    [qx, qy, qz, qw] = quaternion_from_euler(roll, pitch, yaw)

    # Defining the goal pose frame

    # /move_base_simple/goal -- PoseStamped msg
    # var.header.frame_id = "odom"

    # /move_base/goal -- MoveBaseActionGoal
    var.goal.target_pose.header.frame_id = "odom"

    # Debugging
    print("Frame ID: ", var.goal.target_pose.header.frame_id)

    # Defining the velocities and orientations of the robot
        
    # /move_base_simple/goal -- PoseStamped msg
    # var.pose.position.x = 2.0
    # var.pose.position.y = 4.0
    # var.pose.orientation.x = qx
    # var.pose.orientation.y = qy
    # var.pose.orientation.z = qz
    # var.pose.orientation.w = qw

    # /move_base/goal -- MoveBaseActionGoal
    var.goal.target_pose.pose.position.x = 2.0
    var.goal.target_pose.pose.position.y = 4.0
    var.goal.target_pose.pose.orientation.x = qx
    var.goal.target_pose.pose.orientation.y = qy
    var.goal.target_pose.pose.orientation.z = qz
    var.goal.target_pose.pose.orientation.w = qw

    while not rospy.is_shutdown():

        pub.publish(var)
        rate.sleep()


if __name__ == '__main__':
    try:
        send_goal_position()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass