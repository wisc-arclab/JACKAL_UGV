#include "acado_common.h"
#include "acado_auxiliary_functions.h"
#include "ros/ros.h"
#include <time.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include <stdio.h>
#include <math.h>
#include <chrono>
#include <ctime>

std::vector<geometry_msgs::PoseStamped::ConstPtr> pose;

/* Some convenient definitions. */
#define NX          ACADO_NX  /* Number of differential state variables.  */
#define NXA         ACADO_NXA /* Number of algebraic variables. */
#define NU          ACADO_NU  /* Number of control inputs. */
#define NOD         ACADO_NOD  /* Number of online data values. */

#define NY          ACADO_NY  /* Number of measurements/references on nodes 0..N - 1. */
#define NYN         ACADO_NYN /* Number of measurements/references on node N. */

#define N           ACADO_N   /* Number of intervals in the horizon. */

#define NUM_STEPS   1000        /* Number of real-time iterations. */
#define VERBOSE     1         /* Show iterations: 1, silent: 0.  */

/* Global variables used by the solver. */
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

//concatted struct for input and output
struct controlIO{
	double x;
	double y;
	double theta;

	double v;
	double omega;
};
extern controlIO controlio = {};

//predicted trajectory by MPC
extern double pathO [NX*(N+1)] = {};

//reference trajectory by MPC
extern double traj_ref [NY*N] = {};

//TODO for trajectory modification
//pre-defined parameters for reference trajectory
extern double pi = 2*acos(0.0);
extern double radius = 1;
extern double speed = 0.2;
extern double pred_time = 1;
extern double inter_time = pred_time/N;

/*
-- MPC solver function, invoked below in main
-- interact with controlio struct to read in current states and write out first optimal inputs
-- following time-dependent reference trajectory traj_ref, write out model-predicted trajectory to pathO
*/
int mpc_solver( )
{
	/* Some temporary variables. */
	int    i, iter;
	acado_timer t;

	/* Initialize the solver. */
	acado_initializeSolver();

	/* Initialize the states and controls. */
	for (i = 0; i < NX * (N + 1); ++i)  acadoVariables.x[ i ] = 0.0;
	for (i = 0; i < NU * N; ++i)  acadoVariables.u[ i ] = 0.0;
	acadoVariables.u[0] = 0.3;
	acadoVariables.u[1] = 0.0;

	/* Initialize the measurements/reference, terminal reference is commented*/
	for (i = 0; i < NY * N; ++i)  acadoVariables.y[ i ] = traj_ref[i];
	//for (i = 0; i < NYN; ++i)  acadoVariables.yN[ i ] = 10.0;
	//acadoVariables.yN[2] = 0;

	/* MPC: initialize the current state feedback. */

#if ACADO_INITIAL_STATE_FIXED
	for (i = 0; i < NX; ++i) acadoVariables.x0[ i ] = 0;
	acadoVariables.x0[ 0 ] = controlio.x;
	acadoVariables.x0[ 1 ] = controlio.y;
	acadoVariables.x0[ 2 ] = controlio.theta;
#endif

	/* Get the time before start of the loop. */
	acado_tic( &t );

	/* The "real-time iterations" loop. */
	for(iter = 0; iter < NUM_STEPS; ++iter)
	{
        /* Perform the feedback step. */
		acado_feedbackStep( );

		/* Prepare for the next step. */
		acado_preparationStep();
	}

	/* Read the elapsed time. */
	real_t te = acado_toc( &t );

	//read first inputs into IO struct
	controlio.v = acadoVariables.u[ 0 ];
	controlio.omega = acadoVariables.u[ 1 ];

	//read model-predicted trajectory
	for (i = 0; i < NX * (N + 1); ++i)  pathO[ i ] = acadoVariables.x[ i ];	
	
	//print status info
	printf("x: %f\ny: %f\ntheta: %f\nv: %f\nomega: %f\n", controlio.x, controlio.y, controlio.theta, controlio.v, controlio.omega);
	acado_printDifferentialVariables();
	acado_printControlVariables();

    return 0;
}

/*
-- Callback function for real state subscription
-- Read in msgs from mocap, and feed to IO struct
-- make conversion from quaternion to radian
-- approximate small angle (near 0 rad) to 0. (avoid high cost value if orientation slightly oscillated from 0 to 2pi)
*/

/*
void pos_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
	double w = 0.0;
	double z = 0.0;
	controlio.x = msg->pose.position.x;
	controlio.y = msg->pose.position.y;

	//approximate small angle (z)
	if(fabs(msg->pose.orientation.z) < 0.0001){
		z = 0.0;
	} else {
		z = msg->pose.orientation.z;
	}
	if(z < 0){
		w = -msg->pose.orientation.w;
	} else {
		w = msg->pose.orientation.w;
	}
	controlio.theta = 2*acos(w);
}
*/


void pos_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {

    double x = -msg->pose.orientation.x;
    double y = msg->pose.orientation.y;
    double z = 0.0;
    double w = 0.0;
      
    	//approximate small angle (z)
	if(fabs(msg->pose.orientation.z) < 0.0001){
		z = 0.0;
	} else {
		z = msg->pose.orientation.z;
	}
	if(z < 0){
		w = -msg->pose.orientation.w;
	} else {
		w = msg->pose.orientation.w;
	}
    
    // 控制点的位置信息
    controlio.x = msg->pose.position.x;
    controlio.y = msg->pose.position.y;

    // 计算偏航角
    double yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));

    controlio.theta = yaw;
    // 偏航角转换为度
    //controlio.theta = yaw * (180.0 / M_PI);
}


/*
-- Main function
-- subs: subscriberpos
-- pubs: input_pub, path_pub, traj_pub
*/
int main(int argc, char **argv)
{

  //-- get initial real time as time_1, in seconds
  struct timeval time_now{};
  gettimeofday(&time_now, NULL);
  time_t time_1 = time_now.tv_sec + (time_now.tv_usec / 1000000);
  long time_d;  // time difference from initial real time for ref traj
  double dis;

  //---- ROS subs and pubs ----
  ros::init(argc, argv,"subscriberPos");
  ros::NodeHandle nh;
  // sub for orientation from mocap, change to /car/car_pose for HITL simulation
  ros::Subscriber subscriberpos = nh.subscribe("/vrpn_client_node/JACKAL/pose",1000,pos_callback); 
  // pub for Mushr input, will drive the hardware drivetrain
  //ros::Publisher input_pub = nh.advertise <ackermann_msgs::AckermannDriveStamped> ("/car/mux/ackermann_cmd_mux/input/navigation", 1000);
  ros::Publisher input_pub = nh.advertise <geometry_msgs::Twist> ("/cmd_vel", 1000);
  // pub for predicted path for visualization
  ros::Publisher path_pub = nh.advertise <geometry_msgs::PoseArray> ("/JACKAL_MPC_traj", 1000);
  // pub for reference traj for visualization
  ros::Publisher traj_pub = nh.advertise <geometry_msgs::PoseArray> ("/JACKAL_ref_traj", 1000);

  //ROS iteration
  ros::Rate loop_rate(1000); //adjust loop rate and computation time for better performance
  int count = 0;
  while (ros::ok())
  {
    //---- reference trajectory ----
    geometry_msgs::PoseArray traj; //pose array for publishing
    geometry_msgs::Pose trajpose; // poses in pose array

    //get current real time and calculated time difference
    gettimeofday(&time_now, NULL);
    time_t time_2 = time_now.tv_sec + (time_now.tv_usec / 1000000);
    time_d = time_2 - time_1;
    
    //distance already passed
    dis = time_d*speed;

    //trajectory setpoints over horizon
    for (int i = 0; i < N; ++i){
	
	//TODO for trajectory modification
	/*
	The code below will generate time-dependent figure-8 trajectory.
	Predefined: [speed]:  targetted speed of the trajectory
	Predefined: [radius]: size of the trajectory
	Predefined: [N]:      horizon of the trajectory
	Given:      [dis]:    distance passed
	*/

	//--------------- starts ---------------
	
	//distance incrementation over horizon
	dis += inter_time*speed;

	//calculation for orientation
	double x0 = radius*sin(dis-inter_time*speed);
	double y0 = radius*sin(dis-inter_time*speed)*cos(dis-inter_time*speed);
	double x1 = radius*sin(dis);
    	double y1 = radius*sin(dis)*cos(dis);

	//ACADO compatible array (traj_ref)
	traj_ref[5*i] = radius*sin(dis); //x
    	traj_ref[5*i+1] = radius*sin(dis)*cos(dis); //y
    	traj_ref[5*i+2] = -atan2((x1-x0),(y1 - y0))+pi/2; //orientation
	traj_ref[5*i+3] = speed; //speed
	traj_ref[5*i+4] = 0; //steering

	//--------------- ends ---------------

	//add new traj setpoint to ROS compatible pose array (traj, trajpose)
	printf("\n traj_ref_theta = %.5f",traj_ref[5*i+2]);
	trajpose.position.x = traj_ref[5*i];
	trajpose.position.y = traj_ref[5*i+1];
	trajpose.orientation.z = sin(traj_ref[5*i+2]/2);
	trajpose.orientation.w = cos(traj_ref[5*i+2]/2);
	traj.poses.push_back(trajpose);
    }

    //---- solve MPC ----
    mpc_solver();

    //print solved results
    printf("v: %f, omega: %f\n", controlio.v, controlio.omega);

    
    //---- PUBLISHERS ----
    
    //publish inputs for drivetrain
    //ackermann_msgs::AckermannDriveStamped msg;
    geometry_msgs::Twist msg;
    msg.angular.z = controlio.omega;
    if (controlio.v < 0) { 
        controlio.v = 0; //avoid negative v
    }
    msg.linear.x = controlio.v;
    input_pub.publish(msg); //publish

    //publish reference trajectory for visualization
    traj.header.frame_id = "/map"; //add header and stamp for visualization
    traj.header.stamp = ros::Time::now();
    traj_pub.publish(traj); //publish

    //publish predicted trajectory for visualization
    geometry_msgs::PoseArray path;
    path.header.frame_id = "/map"; //add header and stamp for visualization
    path.header.stamp = ros::Time::now();
    geometry_msgs::Pose pathpose; 
    for (int i = 0; i<N+1; ++i){ //convert from radian to quaternion
	pathpose.position.x = pathO[ i*3+0 ];
	pathpose.position.y = pathO[ i*3+1 ];
	pathpose.orientation.z = sin(pathO[ i*3+2 ]/2);
	pathpose.orientation.w = cos(pathO[ i*3+2 ]/2);
    	path.poses.push_back(pathpose);
    }
    path_pub.publish(path); //publish
    
    //ROS misc
    ros::spinOnce();
    loop_rate.sleep();
    count++;
  }
  
  ros::spin();
  return 0;
}
