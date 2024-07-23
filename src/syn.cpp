#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <sensor_msgs/Imu.h>
#include <data_for_deepvio/StampedModelStates.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <eigen3/Eigen/Dense>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, data_for_deepvio::StampedModelStates, sensor_msgs::Image> MySyncPolicy;

std::ofstream imu_file("imu_data.csv", std::ofstream::out);
std::ofstream odom_file("odom_data.csv", std::ofstream::out);
int sync_message_count = 0;
float sub_imu_index = 0.0; // follow imu msg from the last syn point

void imu_callback(const sensor_msgs::ImuConstPtr& imu)
{
    // save IMU data
    float imu_index = sync_message_count + sub_imu_index / 10.0;
    if(sub_imu_index < 1.0)
    {
        imu_file << imu_index << "," << imu->angular_velocity.x << "," << imu->angular_velocity.y << "," << imu->angular_velocity.z << ","
                 << imu->linear_acceleration.x << "," << imu->linear_acceleration.y << "," << imu->linear_acceleration.z << "\n";
        sub_imu_index += 0.1; // renew index
    } 
}

void sync_callback(const sensor_msgs::ImuConstPtr& imu, const data_for_deepvio::StampedModelStates::ConstPtr& model_states, const sensor_msgs::ImageConstPtr& image)
{
    // Extract the state of the "jackal" model
    int jackal_index = -1;
    for (size_t i = 0; i < model_states->name.size(); ++i)
    {
        if (model_states->name[i] == "jackal")
        {
            jackal_index = i;
            break;
        }
    }

    if (jackal_index == -1)
    {
        ROS_ERROR("Jackal model not found in /gazebo/model_states");
        return;
    }

    // save image data
    try {
        cv::Mat cv_image = cv_bridge::toCvCopy(image, "bgr8")->image;
        std::string filename = "image_" + std::to_string(sync_message_count) + ".jpg";
        cv::imwrite(filename, cv_image);
    } catch (const cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    // save odom data
    const geometry_msgs::Pose& pose = model_states->pose[jackal_index];
    Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    Eigen::Matrix3d rot_matrix = q.toRotationMatrix();
    odom_file << sync_message_count;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            odom_file << "," << rot_matrix(i, j);
        }
    }
    odom_file << "," << pose.position.x << "," << pose.position.y << "," << pose.position.z << "\n";

    sync_message_count++;
    sub_imu_index = 0.0; // reset IMU index
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sync_node");
    ros::NodeHandle nh;

    // syn subscribers
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub_sync(nh, "/imu/data", 1);
    message_filters::Subscriber<data_for_deepvio::StampedModelStates> model_states_sub(nh, "/stamped_model_states", 1);
    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/right_zed2i/color/image_raw", 1);
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), imu_sub_sync, model_states_sub, image_sub);
    sync.registerCallback(boost::bind(&sync_callback, _1, _2, _3));

    // imu subscriber
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/imu/data", 50, imu_callback);

    // CSV file header
    imu_file << "ID,Ang_Vel_X,Ang_Vel_Y,Ang_Vel_Z,Lin_Acc_X,Lin_Acc_Y,Lin_Acc_Z\n";
    odom_file << "ID,Rot_11,Rot_12,Rot_13,Rot_21,Rot_22,Rot_23,Rot_31,Rot_32,Rot_33,Pos_X,Pos_Y,Pos_Z\n";

    ros::Rate rate(4); // total rate: 5 Hz
    while (ros::ok())
    {
        ros::spinOnce(); // one loop
        rate.sleep();    
    }

    imu_file.close();
    odom_file.close();

    return 0;
}

