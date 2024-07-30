#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

ros::Publisher real_path_pub;
nav_msgs::Path path;

// Callback function for PoseStamped messages
void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg) {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = pose_msg->header; // Copy timestamp and frame_id
    pose_stamped.pose = pose_msg->pose; // Copy pose

    path.poses.push_back(pose_stamped);
    real_path_pub.publish(path);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "mocap_path_publisher");
    ros::NodeHandle nh;

    // Publisher for the path
    real_path_pub = nh.advertise<nav_msgs::Path>("mocap_state", 10, true);

    // Subscriber to the pose
    ros::Subscriber pose_sub = nh.subscribe("/vrpn_client_node/JACKAL/pose", 10, poseCallback);

    // Initialize path message
    path.header.frame_id = "world";  // Make sure this is the correct frame

    ros::spin();

    return 0;
}

