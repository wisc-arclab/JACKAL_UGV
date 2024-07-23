# Data-synchronization-and-collection
This tutorial will teach you how to use ARClab's desktop-03 to collect data and use it for neural network training. 

Created by [TianxiaoYe](https://github.com/fuwafuwaboom). If you have any questions, please contact via email: **tye46@wisc.edu**

## Open the JACKAL simulator with camera and city map
Since training VIO's neural network requires a car camera and rich scenes, I wrote a camera plug-in based on the simulator that comes with JACKAL. First, load the camera's xacro plug-in file:
```
export JACKAL_URDF_EXTRAS=~/Downloads/zed2i.urdf.xacro
```
After you install this ros package, in the `/data` folder (should be created by yourself anywhere), run:

```
rosrun data_for_deepvio sync_subscriber_node
```
Then move jackal in the gazebo, and type `ctrl+c` to finish the collection
It will give you synchronized image data, odometry data and imu data (now at 4hz), and will be 10 imu data between two synchronized data(40 hz).
