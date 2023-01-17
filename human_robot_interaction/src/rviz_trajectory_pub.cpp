#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rviz_trajectory_pub_node");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<nav_msgs::Path>("path_topic", 10);
    ros::Rate loop_rate(1);

    // create a path message to be published
    nav_msgs::Path path_msg;
    path_msg.header.seq = 0;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "map";

    // create a pose message
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header = path_msg.header;
    // pose_msg.header.seq = 0;
    // pose_msg.header.stamp = ros::Time::now();
    // pose_msg.header.frame_id = "map";
    pose_msg.pose.position.x = 0.0;
    pose_msg.pose.position.y = 0.0;
    pose_msg.pose.position.z = 0.0;

    tf2::Quaternion q;
    double yaw_angle = 0.0;
    q.setRPY(0, 0, 0); // roll, pitch, yaw
    pose_msg.pose.orientation.x = q.x(); //or q[0]
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    // add the pose to the path
    path_msg.poses.push_back(pose_msg);
    while (ros::ok()){
        //Update the header information
        path_msg.header.seq++;
        path_msg.header.stamp = ros::Time::now();
        
        pose_msg.header = path_msg.header;
        pose_msg.pose.position.x += 1.0;

        yaw_angle += 0.1;
        q.setRPY(0, 0, yaw_angle);
        q.normalize();
        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();
        pose_msg.pose.orientation.w = q.w();

        path_msg.poses.push_back(pose_msg);

        // publish the message
        pub.publish(path_msg);

        // sleep for the time remaining to let us hit our 1Hz publish rate
        loop_rate.sleep();
    }

}

// nav_msgs/Path
// path:
// 1. header
// 2. poses (geometry_msgs/PoseStamped)-> vector or/ list
//    2.1 header
//    2.2 pose
//         2.2.1 position
//         2.2.2 orientation (x,y,z,w)    