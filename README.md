The following is the effect based on the JACKAL gazebo simulator:

https://github.com/wisc-arclab/JACKAL_UGV/assets/80655645/be046752-33a6-4dac-829a-18542c17d076

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
git clone https://github.com/TianxiaoYe-Shawn/ACADO_NMPC_ROS.git
```


Move the package to the `/src` directory and remove other files:

```
mv ACADO_NMPC_ROS/acado_mpc .
rm -r ACADO_NMPC_ROS
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

ACADO's advantage is that it can generate efficient C code through a symbolic language. First, modify your own MPC model in the file `symbolic_mpc.cpp` inside `acado_mpc/acado_export_code` (it's suggested not to modify it on the first run).

Then generate the C code package:

```
cd ~/NMPC_ACADO_ws/src/acado_mpc/acado_export_code
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

add `include` folder in your ROS package:
```
cd ~/NMPC_ACADO_ws/src/acado_mpc/
mkdir include
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

Configure the MPC weight parameters:

```
roslaunch acado_mpc set_weight.launch
```

Then directly `ctrl+c` to cancel, at this point the custom weight parameters have been passed.

Finally, in this terminal, open the MPC control node:

```
rosrun acado_mpc mpc_node
```

At this point, you should see the JACKAL vehicle start moving and following the trajectory.
