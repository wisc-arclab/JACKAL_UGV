#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "mpc_tracking/mpc.h"
#include "std_msgs/Empty.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>
#include <Eigen/Dense>
#include <memory>

using std::vector;
using std::unique_ptr; //Smart Pointers
using std::cout;
using std::endl;

unique_ptr<Mpc> mpc_ptr;

ros::Publisher cmd_vel_pub, local_path_pub, predict_path_pub;//publisher

nav_msgs::Odometry odom; //odom
Eigen::Vector3d current_state;

vector<Eigen::Vector3d> traj; //total trajectory
vector<Eigen::Vector3d> local_traj; //local trajectory
bool receive_traj = false; //flag for local trajectory
int current_index = 0; // Index in the global trajectory

nav_msgs::Path local_path, predict_path; //predict path

const int N = 10;

void odomCallback(const nav_msgs::Odometry &msg) {
    //update current state
    odom = msg;
    current_state(0) = msg.pose.pose.position.x; //x
    current_state(1) = msg.pose.pose.position.y; //y
    current_state(2) = tf2::getYaw(msg.pose.pose.orientation); //theta
}

void receiveTrajectory(const nav_msgs::Path& path) {
    //receive total trajectory
    if (!receive_traj) {
        for (const auto& pose_stamped : path.poses) {
            double x = pose_stamped.pose.position.x;
            double y = pose_stamped.pose.position.y;
            double theta = tf2::getYaw(pose_stamped.pose.orientation);
            traj.push_back(Eigen::Vector3d(x, y, theta));
        }

        receive_traj = true;
    }
}

void updateLocalTrajectory() {
    //update local trajectory
    local_traj.clear();

	local_path.poses.clear();

    int traj_size = static_cast<int>(traj.size());

    for (int i = 0; i <= N; ++i) {
        int index = (current_index + 5 * i) % traj_size;
        local_traj.push_back(traj[index]);
        
        // create PoseStamped for local path
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "odom";
        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.pose.position.x = traj[index][0];
        pose_stamped.pose.position.y = traj[index][1];
        pose_stamped.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), traj[index][2]));
        local_path.poses.push_back(pose_stamped);
    }

	//publish local_path
    local_path.header.frame_id = "odom";
    local_path.header.stamp = ros::Time::now();
    local_path_pub.publish(local_path);
    
    current_index = (current_index + 3) % traj_size;
    ROS_INFO("Received new trajectory with %lu points", traj.size());
    ROS_INFO("Updated local trajectory starting from index %d", current_index);
}


void run_mpc(const ros::TimerEvent &e) {

    ROS_INFO("run_mpc called");

    // Check if receive global traj
    if (!receive_traj) {
        ROS_INFO("No new trajectory received.");
        return;
    }
    
    //update local traj
    updateLocalTrajectory();
    
    Eigen::MatrixXd desired_state = Eigen::MatrixXd::Zero(N+1, 3);
    for (int i = 0; i < local_traj.size(); ++i) {
        desired_state(i, 0) = local_traj[i][0];
        desired_state(i, 1) = local_traj[i][1];
        desired_state(i, 2) = local_traj[i][2];
    }

    Eigen::MatrixXd desired_state1 = desired_state.transpose();
    
    if (mpc_ptr->solve(current_state, desired_state1)) {
        geometry_msgs::Twist cmd;
        auto control_cmd = mpc_ptr->getFirstU();
        cmd.linear.x = control_cmd[0];
        cmd.angular.z = control_cmd[1];
        cmd_vel_pub.publish(cmd);
        //publish control cmd
        ROS_INFO("Updated current state: x=%f, y=%f, theta=%f", current_state(0), current_state(1), current_state(2));
        ROS_INFO("Published control command: linear.x=%f, angular.z=%f", cmd.linear.x, cmd.angular.z);
        
        predict_path.header.frame_id = "odom";
		    predict_path.header.stamp = ros::Time::now();
		    geometry_msgs::PoseStamped pose_msg;
		    auto predict_states = mpc_ptr->getPredictX();
		    //cout << "got predict x" << endl;
		    for (int i = 0; i < predict_states.size(); i += 2) {
		        pose_msg.pose.position.x = predict_states[i];
		        pose_msg.pose.position.y = predict_states[i + 1];
		        predict_path.poses.push_back(pose_msg);
				    }
		    predict_path_pub.publish(predict_path);
		    //publish predict path
		    ROS_INFO("Published predicted path with %lu points", predict_path.poses.size());
		    predict_path.poses.clear();
		}
		ROS_INFO("MPC path computed and update_traj flag reset");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mpc_tracking_node");
    ros::NodeHandle nh;
    mpc_ptr.reset(new Mpc());

	ros::Subscriber odom_sub = nh.subscribe("/odometry/filtered", 1, &odomCallback);
	ros::Subscriber traj_sub = nh.subscribe("/trajectory", 10, &receiveTrajectory);
	
	local_path_pub = nh.advertise<nav_msgs::Path>("/local_path", 1);
	predict_path_pub = nh.advertise<nav_msgs::Path>("/predict_path", 1);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    
    ros::Timer control_cmd_pub = nh.createTimer(ros::Duration(0.1), run_mpc);
    
    ros::spin();
    return 0;
}

