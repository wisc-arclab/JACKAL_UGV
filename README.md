# CasADi-NMPC
This code repository uses CasADi to automatically solve the optimal control problem of MPC. The following is a tutorial. The code runs well on Ubuntu 20.04 + ROS Noetic.

Created by [TianxiaoYe](https://github.com/fuwafuwaboom). If you have any questions, please contact via email: **tye46@wisc.edu**
***
Before following the steps below, make sure you have installed casadi and ipopt
1.Create your own ros workspace and download this code to the src folder of your workspace. To clone this branch code, you can use the following command:

```
git clone --single-branch --branch CasADi-NMPC https://github.com/wisc-arclab/JACKAL_UGV.git
```

2.Modify the `/odometry/filtered` topic in the code to your own odometer topic, and modify the `/cmd_vel` topic in the code to your own control command publishing topic

3.Modify the parameters of your own mpc controller in the `mpc.cpp` file

4.Return to the workspace root directory `cd /path/to/your/workspace` and use `catkin_make` to compile all files.

5.First run your own robot (such as the car in the simulator)

6.Then open the Track Tracking Environment panel. Then open the Track Tracking Environment panel. This should give you a rivz file with a full green circular track:

```
roslaunch mpc_tracking tracking_env.launch
```

7.Then open the control input real-time monitor:

```
rosrun mpc_tracking plot_control_input.py
```

8. Finally, run the mpc controller node and start mpc control. You should see your robot automatically following a circular trajectory.

```
rosrun mpc_tracking mpc_tracking_node
```
