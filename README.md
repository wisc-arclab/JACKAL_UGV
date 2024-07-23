# Data-synchronization-and-collection
This tutorial will teach you how to use ARClab's desktop-03 to collect data which for neural network training. 

Created by [TianxiaoYe](https://github.com/fuwafuwaboom). If you have any questions, please contact via email: **tye46@wisc.edu**

## Open the JACKAL simulator with camera and city map
**[for every rosrun or roslaunch, open a new terminal]**

The effect is as follows. The following text will explain how to operate it.

https://github.com/user-attachments/assets/3447570c-e99e-4801-86a8-ed14f003ce95

Since training VIO's neural network requires a car camera and rich scenes, I wrote a camera plug-in based on the simulator that comes with JACKAL. First, load the camera's xacro plug-in file:
```
export JACKAL_URDF_EXTRAS=~/Downloads/zed2i.urdf.xacro
```
Then open the JACKAL simulator with the city scene (I have embedded the city scene into the simulator scene):
```
roslaunch jackal_gazebo jackal_world.launch
```
Open the camera image to observe the front of the vehicle in real time: 
```
rosrun image_view image_view image:=/right_zed2i/color/image_raw
```
Then use the USB cable to directly connect the PS4 controller, and then open the controller control node:
```
roslaunch jackal_control teleop.launch joy_dev:=/dev/input/js0
```
Now you can use the controller to control JACKAL and observe the camera image changes.

## Install this ROS function package
Build your own workspace and in your `/src` folder, run:

```
git clone --single-branch --branch Data-synchronization-and-collection https://github.com/wisc-arclab/JACKAL_UGV.git
```
create an `include` folder in the `JACKAL_UGV` folder:
```
mkdir include
```
then in your workspace root directory, complie the ROS package by running:
```
catkin_make
```

## Collect data
After you finished [Open the JACKAL simulator with camera and city map](https://github.com/wisc-arclab/JACKAL_UGV/blob/Data-synchronization-and-collection/README.md#open-the-jackal-simulator-with-camera-and-city-map) part, now we can collect data.

Open the model_state_stamper node, which will publish ground truth messages with timestamps:
```
rosrun data_for_deepvio model_state_stamper
```

Then in the `/data` folder (or any folder should be created by yourself anywhere), run:

```
rosrun data_for_deepvio sync_node
```
Then move jackal in the gazebo, and type `ctrl+c` to finish the collection
It will give you synchronized image data, odometry data and imu data (now at 4hz), and will be about 10 imu data between two synchronized data(40 hz).
