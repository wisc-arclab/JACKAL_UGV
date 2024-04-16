#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "drive_robot");
    ros::NodeHandle nh;

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Rate loop_rate(10); // 10 Hz

    while (ros::ok()) {
        geometry_msgs::Twist msg;
        msg.linear.x = 0.5;  //m/s 
        msg.angular.z = 0.5; //rad/s

        vel_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
