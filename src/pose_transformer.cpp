#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

ros::Publisher transformed_pose_pub;

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    geometry_msgs::PoseStamped transformed_msg;
    transformed_msg.header = msg->header;

    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.orientation, quat);

    tf::Quaternion rotation;
    rotation.setRPY(0, 0, -115.0 * M_PI / 180.0); 

    tf::Quaternion transformed_quat = rotation * quat;
    transformed_quat.normalize();

    tf::quaternionTFToMsg(transformed_quat, transformed_msg.pose.orientation);

    transformed_msg.pose.position = msg->pose.position;

    transformed_pose_pub.publish(transformed_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pose_transformer");
    ros::NodeHandle nh;

    ros::Subscriber pose_sub = nh.subscribe("/vrpn_client_node/JACKAL/pose", 10, poseCallback);
    
    transformed_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("transformed_pose", 10);

    ros::spin();
    return 0;
}

