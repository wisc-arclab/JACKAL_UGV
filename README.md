This repo is using ACADO-Toolkits to deploy Model Predictive Control on [JACKAL](https://clearpathrobotics.com/jackal-small-unmanned-ground-vehicle/).

Created by [TianxiaoYe](https://github.com/fuwafuwaboom). If you have any questions, please contact via email: **tye46@wisc.edu**
***
The following is the effect based on the JACKAL with MOCAP system:

https://github.com/user-attachments/assets/57a63b16-b198-497a-86ee-d900d4d4dafb

https://github.com/user-attachments/assets/5a6a158c-1f5f-4c19-b4e9-896cc26bc623

# ACADO MOCAP NMPC ROS package
## 1. Ubuntu and ROS Versions

This ROS package is built on Ubuntu 20.04 and ROS Noetic full desktop version. It is recommended to follow this version.

## 2. Creating a ROS Workspace

In the base station (arclab desktop-03)

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

In the base station (arclab desktop-03)

Navigate to the `/src` directory in your workspace and download the package:

```
cd ~/NMPC_ACADO_ws/src
git clone --single-branch --branch ACADO_MOCAP_NMPC https://github.com/wisc-arclab/JACKAL_UGV.git
```

## 4. Installing ACADO

In the base station (arclab desktop-03)

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

ACADO's advantage is that it can generate efficient C code through a symbolic language. 

In the base station (arclab desktop-03)

First, modify your own MPC model in the file `symbolic_mpc.cpp` inside `ACADO_NMPC_ROS/acado_export_code` (it's suggested not to modify it on the first run).

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

## 6. Open MOCAP system and JACKAL VRPN node

Open the mocap system and calibrate.

Paste the mocap reflective ball on JACKAL.

In the motive interface, select the ball combination and create a rigid body.

Publish the rigid body information to the 192.168.1.223 network segment (this is the network segment when jackal connects to arclab wifi)

Make sure JACKAL is connected to arclab wifi. 

Open jackal and enter in the terminal:
```
roslaunch vrpn_client_ros sample.launch  server:=192.168.1.202
```
You will see jackal's pose message published in the topic `/vrpn_client_node/JACKAL/pose`

## 7. Start Running!

(For each of the following `rosrun` and `roslaunch`, you need to open a new terminal)

In the base station (arclab desktop-03)

First, open the tracking test environment (including publishing trajectory, configuring rviz):

```
roslaunch acado_mpc tracking_env.launch
```

You should see a green circular trajectory in rviz (you can customize your trajectory in the `trajectory_publisher.cpp` file).

Then configure the MPC weight parameters and run mpc node:

```
roslaunch acado_mpc set_weight.launch
```

At this point, you should see the JACKAL vehicle start moving and following the trajectory.

**Tips:**
**You need to use ros multi-machine communication to set the base station as the `ros slave` and jackal as the `ros master`.**
