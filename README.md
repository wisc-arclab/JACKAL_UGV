# Data-synchronization-and-collection
This tutorial will teach you how to use ARClab's desktop-03 to collect data which for neural network training. 

Created by [TianxiaoYe](https://github.com/fuwafuwaboom). If you have any questions, please contact via email: **tye46@wisc.edu**

## Open the JACKAL simulator with camera and city map
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

After you install this ros package, in the `/data` folder (should be created by yourself anywhere), run:

```
rosrun data_for_deepvio sync_subscriber_node
```
Then move jackal in the gazebo, and type `ctrl+c` to finish the collection
It will give you synchronized image data, odometry data and imu data (now at 4hz), and will be 10 imu data between two synchronized data(40 hz).
