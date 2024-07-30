This repo is using ACADO-Toolkits to deploy Model Predictive Control on JACKAL(or JACKAL Gazebo simulator).

Created by [TianxiaoYe](https://github.com/fuwafuwaboom). If you have any questions, please contact via email: **tye46@wisc.edu**
***
The following is the effect based on the JACKAL with MOCAP system:

https://github.com/user-attachments/assets/e2ae594c-e129-48eb-9734-08d5a29e63e0

# Build ACADO NMPC ROS package from scratch
## 1. Ubuntu and ROS Versions

This ROS package is built on Ubuntu 20.04 and ROS Noetic full desktop version. It is recommended to follow this version, as the JACKAL simulator is also built on it.

## 2. Creating a ROS Workspace

Copy the following code into the terminal to create a workspace:

```
mkdir -p ~/NMPC_ACADO_ws/src/
cd ~/NMPC_ACADO_ws/src/
catkin_init_workspace
cd ~/NMPC_ACADO_ws
catkin_make
echo "source ~/NMPC_ACADO_ws/devel/setup.bash" >> ~/.bashrc
```

You can rename `NMPC_ACADO_ws` to your preferred workspace name.

## 3. Downloading ROS Packages

Navigate to the `/src` directory in your workspace and download the package:

```
cd ~/NMPC_ACADO_ws/src
git clone --single-branch --branch ACADO_NMPC_ROS https://github.com/wisc-arclab/JACKAL_UGV.git
```

## 4. Installing ACADO

Install dependencies:

```
sudo apt-get install gcc g++ cmake git gnuplot doxygen graphviz
```

Choose any working directory (I use `~/`) to download the ACADO source code:

```
cd
git clone https://github.com/acado/acado.git -b stable ACADOtoolkit
```

Install:

```
cd ACADOtoolkit
mkdir build
cd build
cmake ..
make
sudo make install
```

Configure the environment variable:

```
echo "source ~/ACADOtoolkit/build/acado_env.sh" >> ~/.bashrc
```

## 5. Compiling ROS Packages

ACADO's advantage is that it can generate efficient C code through a symbolic language. First, modify your own MPC model in the file `symbolic_mpc.cpp` inside `ACADO_NMPC_ROS/acado_export_code` (it's suggested not to modify it on the first run).

Then generate the C code package in `acado_export_code` directory:

```
cd path/to/acado_export_code
mkdir build && cd build
cmake ..
make
./mpc
```

Move the generated code and build the static library:

```
mv symbolic_mpc_export/* ../../acado_mpc_export/
cd ../../acado_mpc_export
make
```

Compile the entire ROS package:

```
cd ~/NMPC_ACADO_ws
catkin_make
```

If everything is successful, your package is now fully set up. If you do not have any testing environment, the JACKAL simulator is a good choice next.

## 6. Setting Up JACKAL Simulator

Install the JACKAL simulator with one command:

```
sudo apt-get install ros-noetic-jackal-simulator ros-noetic-jackal-desktop ros-noetic-jackal-navigation
```

Modify the simulator environment, we need a spacious area:

```
cd /opt/ros/noetic/share/jackal_gazebo/launch/
sudo vi jackal_world.launch
```

Press `i` to enter edit mode.

Change the line `<arg name="world_name..."` to `<arg name="world_name" default="$(find gazebo_ros)/launch/empty_world.launch" />`

Then modify the spawn pose of Jackal to match the trajectory in `jackal_world.launch`:

Change the `x` `y` `z` `yaw` value to `0 0 1.0 0.78`

Then press `ESC`, type `:wq` to save and exit.

The JACKAL simulator is now fully set up.
## 7. Start Running!

(For each of the following `rosrun` and `roslaunch`, you need to open a new terminal)

First, open the JACKAL simulator:

```
roslaunch jackal_gazebo jackal_world.launch
```

You should see the JACKAL vehicle parked in Gazebo.

Then open the tracking test environment (including publishing trajectory, configuring rviz):

```
roslaunch acado_mpc tracking_env.launch
```

You should see a green circular trajectory in rviz (you can customize your trajectory in the `trajectory_publisher.cpp` file).

Then open the control input monitor:

```
rosrun acado_mpc plot_control_input.py
```

You should see the monitor displaying messages on two control input topics, which are static since no messages are published yet.

Configure the MPC weight parameters and run mpc node:

```
roslaunch acado_mpc set_weight.launch
```

At this point, you should see the JACKAL vehicle start moving and following the trajectory.
