#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <eigen3/Eigen/Dense>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, nav_msgs::Odometry, sensor_msgs::Image> MySyncPolicy;

std::ofstream imu_file("imu_data.csv", std::ofstream::out);
std::ofstream odom_file("odom_data.csv", std::ofstream::out);
int sync_message_count = 0;
int imu_message_count = 0;
float sub_imu_count = 1.0;

void imu_callback(const sensor_msgs::ImuConstPtr& imu)
{
    // IMU数据保存，10倍频率输出
    imu_file << sync_message_count + sub_imu_count / 10 << "," << imu->angular_velocity.x << "," << imu->angular_velocity.y << "," << imu->angular_velocity.z << ","
             << imu->linear_acceleration.x << "," << imu->linear_acceleration.y << "," << imu->linear_acceleration.z << "\n";
    sub_imu_count += 1.0;
    if (sub_imu_count > 10.0) sub_imu_count = 1.0;  // 重置计数器
}

void sync_callback(const sensor_msgs::ImuConstPtr& imu, const nav_msgs::OdometryConstPtr& odometry, const sensor_msgs::ImageConstPtr& image)
{
    // 图片保存
    try {
        cv::Mat cv_image = cv_bridge::toCvCopy(image, "bgr8")->image;
        std::string filename = "image_" + std::to_string(sync_message_count) + ".jpg";
        cv::imwrite(filename, cv_image);
    } catch (const cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    // 里程计数据保存
    Eigen::Quaterniond q(odometry->pose.pose.orientation.w, odometry->pose.pose.orientation.x, odometry->pose.pose.orientation.y, odometry->pose.pose.orientation.z);
    Eigen::Matrix3d rot_matrix = q.toRotationMatrix();
    odom_file << sync_message_count;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            odom_file << "," << rot_matrix(i, j);
        }
    }
    odom_file << "," << odometry->pose.pose.position.x << "," << odometry->pose.pose.position.y << "," << odometry->pose.pose.position.z << "\n";

    sync_message_count++;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sync_node");
    ros::NodeHandle nh;

    // 设置同步消息订阅者
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub_sync(nh, "/imu/data", 1);
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "/odometry/filtered", 1);
    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/right_zed2i/color/image_raw", 1);
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), imu_sub_sync, odom_sub, image_sub);
    sync.registerCallback(boost::bind(&sync_callback, _1, _2, _3));

    // 单独的IMU数据订阅者
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/imu/data", 50, imu_callback);

    // 初始化CSV文件头
    imu_file << "ID,Ang_Vel_X,Ang_Vel_Y,Ang_Vel_Z,Lin_Acc_X,Lin_Acc_Y,Lin_Acc_Z\n";
    odom_file << "ID,Rot_11,Rot_12,Rot_13,Rot_21,Rot_22,Rot_23,Rot_31,Rot_32,Rot_33,Pos_X,Pos_Y,Pos_Z\n";

    ros::Rate rate(4); // 设置为4 Hz
    while (ros::ok())
    {
        ros::spinOnce(); // 处理一次回调
        rate.sleep();    // 等待直到达到4 Hz的周期
    }

    imu_file.close();
    odom_file.close();

    return 0;
}

