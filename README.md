# JACKAL_UGV 
1. For 2D lidar navigation based on the move_base algorithm, please look directly at module [`3.Navigation`](https://github.com/wisc-arclab/JACKAL_UGV/blob/main/README.md#3navigation) in this tutorial.  
2. For VIO using OpenVINS, please see module [`5.VIO`](https://github.com/wisc-arclab/JACKAL_UGV/blob/main/README.md#5vio) in this tutorial.
3. For NMPC control using ACADO, please see [`ACADO_NMPC_ROS`](https://github.com/wisc-arclab/JACKAL_UGV/tree/ACADO_NMPC_ROS) branch.
4. For NMPC control using ACADOS, please see [`ACADO_NMPC_ROS`](https://github.com/wisc-arclab/JACKAL_UGV/tree/ACADO_NMPC_ROS) branch.
5. For NMPC control using CppAD/Ipopt, please see [`CppAD/Ipopt-NMPC`](https://github.com/wisc-arclab/JACKAL_UGV/tree/CppAD/Ipopt-NMPC) branch.
6. For NMPC control using CasADi, please see[`CasADi-NMPC`](https://github.com/wisc-arclab/JACKAL_UGV/tree/CasADi-NMPC) branch.
7. For Data synchronization and collection from jackal gazebo simulator, please see[`Data-synchronization-and-collection`](https://github.com/wisc-arclab/JACKAL_UGV/tree/Data-synchronization-and-collection) branch.
8. If this is your first time using JACKAL, it is recommended to start from the beginning of this tutorial.

Created by [TianxiaoYe](https://github.com/fuwafuwaboom). If you have any questions, please contact via email: **tye46@wisc.edu**
***
![711034013600912247](https://github.com/fuwafuwaboom/JACKAL_UGV/assets/80655645/7897fb24-0650-414a-af3f-b28be7ebe328)

**Welcome to the JACKAL_UGV  Tutorial!**

JACKAL is a rugged, lightweight, fast and easy-to-use unmanned ground robot for rapid prototyping and research applications produced by Canada's clearpath company.

The original JACKAL has some inconveniences (such as network card configuration issues, no graphical interface, no GPU on the onboard computer, etc.). 

Arclab's JACKAL has made corresponding improvements to address these issues, so I specially wrote this wiki that is different from the [official tutorial](https://docs.clearpathrobotics.com/docs/ros1noetic/robots/outdoor_robots/jackal/tutorials_jackal/#driving-jackal).
***
# 0.Overview
## JACKAL's Password
root account: `1`

administrator account: `1`

## JACKAL's Battery
JACKAL itself has a large battery, and it takes about 5-6 hours to fully charge. Open the JACKAL buckle and lift the upper cover to see the battery slot and battery. The first thing you need to do is to charge the battery and put it into the battery slot and plug in the JACKAL.

Charge the battery |  Open the JACKAL buckle step1 | Open the JACKAL buckle step2
:-------------------------:|:-------------------------:|:-------------------------:
![image](https://github.com/fuwafuwaboom/JACKAL_UGV/assets/80655645/3ea715cc-01e9-48ed-87d9-0453ad015277) |  ![image1](https://github.com/fuwafuwaboom/JACKAL_UGV/assets/80655645/a294f2f4-d0af-49f5-a3e2-010ad52fb319) | ![image2](https://github.com/fuwafuwaboom/JACKAL_UGV/assets/80655645/f26aa260-1516-4157-b86e-ab2a2da6f0e9) 

lift the upper cover             |  Plug battery in Jackal
:-------------------------:|:-------------------------:
![image](https://github.com/fuwafuwaboom/JACKAL_UGV/assets/80655645/bac3268a-dfc6-4ad1-a218-12153b244487)  |  ![image](https://github.com/fuwafuwaboom/JACKAL_UGV/assets/80655645/24d040ab-09f8-412a-8da5-f5a151431943)


## JACKAL's Onboard Computer
JACKAL in our lab has an onboard computer, the system is ubuntu 20.04 with x86 architecture.

![image](https://github.com/fuwafuwaboom/JACKAL_UGV/assets/80655645/b786782d-64e6-48dc-b41a-5adaccd7ee2a)

JACKAL's onboard computer

There are two ways to use JACKAL's onboard computer, one is direct control and the other is remote desktop control. For remote desktop control, please see the 'Remote Desktop Control' module. Direct control is very simple: Open the JACKAL buckle and lift the upper cover, connect the keyboard and mouse to the onboard computer through the USB interface, connect to the external display with an HDMI cable, and then press the JACKAL power-on button.
![image](https://github.com/fuwafuwaboom/JACKAL_UGV/assets/80655645/3601e4b9-c2db-441b-9f8f-0d587e61e72b)

The original JACKAL's onboard computer does not have a graphical interface, only a command line. For ease of use, I installed a graphical interface then you will be able to use any graphical ubuntu functionality that should be available.

 After booting, enter the password of administrator account (here is `1`) and you will see the the graphical interface desktop. The system automatically runs all ROS packages required for JACKAL to run (you can directly view it by entering `rostopic` through the terminal), that is to say, there is no need to manually roslaunch. If you're wondering where these packages are located, they are located in `/opt/ros/noetic/share/jackal*` (* stands for all folders named after jackal).



## JACKAL's Network
The original JACKAL requires netplan to modify the network, which is very troublesome. This is because it lacks key network components and drivers. Fortunately, JACKAL can currently easily connect to wired and wireless networks through the network settings in the upper right corner. You can use it to connect to uwnet, arclab's wifi, etc.
***
# 1.Manual Control
There are four ways to manually control JACKAL, namely using PS4 handle control, keyboard control, rviz interactive button control and using ROS command.
## PS4 Handle Control
At present, JACKAL has been configured to communicate with the px4 controller (If you want to know how to pair, click [here](https://docs.clearpathrobotics.com/docs/ros1noetic/robots/outdoor_robots/jackal/tutorials_jackal/#pairing-the-controller)). The only thing you need to do is to press the switch button(button with PS logo in the middle) in the center of the controller, and then wait for the flashing light of the controller to turn solid blue to control JACKAL. 

Slow mode: Press and hold the L1 button of the handle while using the left joystick to control the movement of jackal

Quick mode: Hold down the R1 button of the handle and use the left joystick to control the movement of Jackal.

![846959510563857308](https://github.com/fuwafuwaboom/JACKAL_UGV/assets/80655645/d5e2ad0b-df1d-4a5e-8922-c16730a05474)
This status proves that the connection has been successful

## Keyboard Control
We do have our own nodes, but the official ones are better：

`sudo apt-get install ros-noetic-teleop-twist-keyboard`

`rosrun teleop_twist_keyboard teleop_twist_keyboard.py`

## Rviz Interactive Button Control
Open the terminal and enter: `roslaunch jackal_viz view_robot.launch`

You can control JACKAL in the pop-up interface. Drag the red arrow to move forward and backward, and drag the blue circle to turn left and right.
## Using ROS command
Use the rostopic pub command to publish messages to the topic`/cmd_vel`. 

This method is actually a method of directly communicating with ROS topics, while the keyboard control method continues topic communication by writing node files.

Open a terminal and enter: 

`rostopic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}' -r 10`

You should see Jackal spinning in place

***
It should be noted that manually published linear and angular velocities are often inaccurate due to the presence of friction and other reasons (this is why we need control algorithms to combat these noises)
***
# 2.Lidar Configuring
The radar used by JACKAL is velodyne's VLP-16 lidar called Puck. Since I have already installed Puck's ros driver package, it can be used directly.
## Using Puck
Using Puck is divided into the following parts:

1.Puck has a network card inside, so it has an independent wired IP address. 

Turn on the power of Puck and connect Puck to the computer using a network cable. 

Create a new ipv4 configuration in the computer's wired network configuration and enter: 

`address: 192.168.1.77`

`netmask: 255.255.255.0`

`gateway: 192.168.1.1`.

Open the browser and enter `192.168.1.201` (Puck’s IP address) as the URL. You should be able to see the radar interface. **Please do not change any of the parameters without special circumstances**

2.Open Puck’s ros driver:

c `roslaunch velodyne_pointcloud VLP16_points.launch`

At this time, Radar has started to publish topics. The topic of Puck radar data is /scan and

3.Open Puck’s rviz interface:

Open another terminal and type `rosrun rviz rviz -f velodyne`

Add Puck's topic and you should see the data visualized in rviz
## Installing Puck driver 

If you haven't installed the driver package yet, here's how to install it:

Open a terminal and type

`cd`

`mkdir -p catkin_velodyne/src`

`cd catkin_velodyne/src`

`git clone https://github.com/ros-drivers/velodyne.git`

`cd..`

`rosdep install --from-paths src --ignore-src --rosdistro noetic -y`

`catkin_make`

`source devel/setup.bash`
***
# 3.Navigation
There are many ways to navigate. The first is to first build a map through SLAM, and then use the created map as a global map for path planning. The second method is to only build a local map (assuming that the global map is infinite, that is, there are no obstacles), and then refresh the local map frequently to re-plan the path and finally achieve obstacle avoidance. This tutorial uses and explains the second method.

Both methods can build maps from the following sensors:
1. Using lidar alone
2. Use the camera alone
3. Mix lidar and cameras

This tutorial only uses lidar (method 1). If you are interested in the second method, one solution is to perform mapping through orb-slam3 and then eliminate moving objects through yolov8. If you are interested in the third option, please search for related papers (of course slam is another huge topic)

Since Jackal's VLP-16 radar can provide both 3D laser data and 2D laser data, in theory we can use very powerful 3D lidar mapping and navigation algorithms. This tutorial only uses 2D lidar data for navigation.

## Navigating Jackal
_(This assune you have completed sections 0 and 2)_

Here's how to navigate jackal using built algorithm package (have built it on jackal's onboard computer):
1. First, in order to be able to view the process in rviz in real time, please complete `6.remote desktop control` part first.
2. Manually publish static transformation and connect puck's tf tree with jackal's tf tree: open a new terminal and type`rosrun tf2_ros static_transform_publisher 0.05 0 0.15 0 0 0 1 /mid_mount /velodyne`
3. open velodyne:open a new terminal and type`roslaunch velodyne_pointcloud VLP16_points.launch`
4. launch navigation demo:open a new terminal and type`roslaunch jackal_navigation odom_navigation_demo.launch`
5. open Rviz:open a new terminal and type`roslaunch jackal_viz view_robot.launch config:=navigation`

Use the `2D Nav Goal` pink arrow above to set the end point in the diagram shown and Jackal will reach the destination while avoiding all obstacles.

## Understanding Navigation
To understand the navigation function, it is necessary to first understand the relationships between the hardware, and then to clearly grasp how the software algorithms operate. 
### Hardware Structure
![image](https://github.com/fuwafuwaboom/JACKAL_UGV/assets/80655645/1f06da83-04b3-48e1-a740-ad919b72a999)
Since only lidar is used for navigation, the hardware structure is very simple: the onboard computer is connected to the radar through a network cable. After receiving and processing the radar information, it sends a control signal to the motor of the chassis to control the movement of the car.

### Algorithm Structure
![image](https://github.com/fuwafuwaboom/JACKAL_UGV/assets/80655645/027d230c-5a46-47e0-a62d-d52876fefe4e)
This is a flowchart of the algorithm. We explain each part:

1. rviz / Move_base_goal: This is the user interface where the operator can set a goal position for the robot. The goal is set as a 'PoseStamped' message, which includes position and orientation.

2. Move_base: This is a major component that coordinates the navigation process. It takes the 'PoseStamped' message as input and uses it to manage the robot's path planning and execution.
* global_planner: This creates a plan for the robot to reach its destination from its current global position. It uses the 'global_costmap' for planning a collision-free path.

* local_planner: This works in a smaller scope, planning and replanning the immediate movements of the robot based on 'local_costmap'. It responds to dynamic obstacles and changes in the robot's immediate environment.

* global_costmap and local_costmap: These components build a cost map of the environment around the robot. 'global_costmap' is for the entire known map, while 'local_costmap' is concerned with the immediate surroundings. They use data from 'Map_server' and 'Puck Lidar' to detect obstacles and assign costs to different areas on the map, which influence the path planning.

* Recovery_behaviors: If the robot encounters a situation where it can't proceed (e.g., blocked path), recovery behaviors are actions it can take to try to resolve the issue, such as rotating in place or backing up a bit before retrying.

3. Map_server: This node provides map data to the rest of the system. It responds to requests from 'global_costmap' with 'nav_msgs/GetMap' messages.

4. Puck Lidar: This is the Puck lidar sensor that provides 'LaserScan' messages. It helps the robot to understand its surroundings by detecting objects and their distances.

5. Filtered Odometry: This process takes raw odometry data from the robot's sensors (e.g., wheel encoders) and filters it to produce more accurate position and movement information.

6. static_transforms: This component provides fixed (static) transformations between Lidar frames in the jackal frame system, enabling different parts of the system to understand where sensors and other parts of the robot are located relative to each other.

7. Base Controller: This receives velocity commands ('Twist' messages) from the 'local_planner' and translates them into movement commands for the robot's motors to execute the desired path.
***
# 4.Multi-machine Communication
Different machines will only publish topics and run nodes in their own ROS systems, but we often encounter situations where we want to integrate the control of the entire system. For example, one computer is responsible for receiving sensor signals, and another computer is responsible for running the control algorithm. At this time, we need to centralize these devices in a ROS system for control through multi-machine communication (that is, all nodes and topics can be controlled from the terminal of only one machine)
## Set up the Master system
The host is the computer we directly control, here is Jackal's onboard computer.

Add a sentence at the end of the `~/.bashrc` file:`alias sshcar0="ssh arclab-jetson-orin-nano@10.140.78.161"`

Add a sentence at the middle of the `/etc/hosts` file:`10.140.78.161   arclab-jetson-orin-nano`

**Remember to replace "10.140.78.161" with Jetson's current wireless network IP address**

## Set up the Slave system
The Slave is the computer we are not directly control, here is the Jetson orin nano (it is used to process the camera data).

Add sentences at the end of the `~/.bashrc` file:

`export ROS_MASTER_URI=http://jackal:11311`

`alias sshmaster="ssh jackal@10.140.69.172"`

`alias controllermaster="export ROS_MASTER_URI=https://10.140.69.172:11311;export ROS_IP=10.140.78.161"`

Add a sentence at the middle of the `/etc/hosts` file:`10.140.69.172   jackal`

**Remember to replace "10.140.69.172" with Jackal's current wireless network IP address**
***
# 5.VIO
Since Jackal's own onboard computer does not have a GPU, and most VIO algorithms rely on GPU acceleration. Therefore, Jetson orin nano 8G is used to run the VIO algorithm and communicate with the Jackal onboard computer for master-slave communication. Since now all the settings have been configured, you can go directly to the "Using OpenVINS" section.

_**Assume that your system has successfully installed cuda and cuda-supported OpenCV**_

## Install ZED2i SDK
Download [ZED2i SDK 4.1](https://www.stereolabs.com/developers/release)

Run the installation file: `./ZED_SDK_Ubuntuxxxx.zstd.run`

## Install ZED2i Ros Package and Communicate with Jackal
Create a workspace:

`cd`

`mkdir -p ZED_WS/src/`

`cd ZED_WS/src/`

`catkin_init_workspace`

`cd ~/ZED_WS`

`catkin_make`

`echo "source ~/Jackal_ws/devel/setup.bash" >> ~/.bashrc`

Open a new terminal and type:

`cd ~/ZED_WS/src`

`git clone https://github.com/stereolabs/zed-ros-wrapper.git`

`git clone https://github.com/stereolabs/zed-ros-interfaces`

`cd ../`

`rosdep install --from-paths src --ignore-src -r -y`

`catkin_make -DCMAKE_BUILD_TYPE=Release`

`source ./devel/setup.bash`

The part about communicating with Jackal is in the previous section "4.Multi‐machine Communication"

## Install OpenVINS
see [here](https://docs.openvins.com/gs-installing.html)

## Perform Calibration
see [here](https://www.youtube.com/watch?v=BtzmsuJemgI)

## Using OpenVINS
1. republish topic: 

`rosrun topic_tools throttle messages /zed2i/zed_node/imu/data_raw 200 /zed2/zed_node/imu/data_raw200`

`rosrun topic_tools throttle messages /zed2i/zed_node/right_raw/image_raw_gray 20 /zed2/zed_node/right_raw/image_raw_gray20 `

`rosrun topic_tools throttle messages /zed2i/zed_node/left_raw/image_raw_gray 20 /zed2/zed_node/left_raw/image_raw_gray20`

2. Open Rviz

`rviz -d /home/arclab/workspace/catkin_ws_ov/src/open_vins/ov_msckf/launch/zed.rviz`

3. Using OpenVINS

`roslaunch ov_msckf zed2i.launch `
***
# 6.Remote Desktop Control
The easiest way is to use nomachine.

Download [Nomachine](https://www.nomachine.com/) at both the main device and the remotely connected device, plug in the HDMI faker to the remotely connected device, and ensure that both devices are connected to the same network.

Open Nomachine, click "Add", enter any name in "Name" (the name you gave the device to be connected), enter the IP address of the connected device in "Host", and select the corresponding Port model in "Protocol" . Then click the "Add" button with a green round white cross.

At this point you should be able to remotely control another device
