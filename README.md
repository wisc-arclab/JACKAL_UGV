# JACKAL_ACADO_MPC 
This work refers to the work of [MuSHR MPC](https://github.com/wisc-arclab/arclab_vehicles/tree/ACADO-MPC?tab=readme-ov-file)
## ACADO generated MPC code with qpOASES interface
1. Follow the tutorial below to install [ACADO from MATLAB](https://acado.github.io/matlab_overview.html)
2. Download optimization formulation for ACADO, [JACKAL_mpc_continuous.cpp](https://github.com/wisc-arclab/JACKAL_UGV/blob/ACADO_MPC/JACKAL_mpc_continuous.cpp)., and move it to `ACADOtoolkit/examples/code-generation/mpc_mhe/` inside the ACADO package. Please check [tutorials](https://acado.sourceforge.net/doc/html/db/d4e/tutorial.html) for ACADOtoolkit to see how to modify the formulation.
3. Compile this file:
```
cd ACADOtoolkit/build/
cmake ..
make
```
4. Now you should see a execuatable program named "code_generation_JACKAL_mpc_continuous" in `ACADOtoolkit/examples/code-generation/mpc_mhe/`, run it:
```
./code_generation_JACKAL_mpc_continuous
```
5. Now you should have the qpOASES interface package in the same folder, named as "JACKAL_mpc_continuous". Lastly, please move a copy of qpoases folder from ACADOtoolkit/external_packages into the "JACKAL_mpc_continuous" directory.
```
cp -r /ACADOtoolkit/external_packages/qpoases JACKAL_mpc_continuous
```
6. Compile and run
```
make clean
make
./test
```
If your formulation file is written correctly, you should see reasonable control output at this step.

7. Move `JACKAL_mpc_continous` folder in a new folder as our workspace (I chose home directory)
```
mv JACKAL_mpc_continous /workspace/you/choose
```
8. Remove `qpoases` `test.c` `Makefile` and add new version of them:
```
rm -r test.c Makefile qpoases/
```
New version of these files are [here](https://github.com/wisc-arclab/JACKAL_UGV/tree/ACADO_MPC/replacement_files)  and you can clone this branch by:
```
git clone --single-branch --branch ACADO_MPC https://github.com/wisc-arclab/JACKAL_UGV.git
```
And then move them in the same directory as before.

9. Compile it
```
make clean
make
```
## JACKAL MPC Simulation in Gazebo

1. On your linux pc with ubuntu20.04 and ros-noetic, run this command to install JACKAL Gazebo simulator: 
```
sudo apt-get install ros-noetic-jackal-simulator ros-noetic-jackal-desktop ros-noetic-jackal-navigation
```
2. open jackal in gazebo
```
roslaunch jackal_gazebo jackal_world.launch
```
3. Download the rviz configuration file [jackal_mpc_traj.rviz](https://github.com/wisc-arclab/JACKAL_UGV/blob/ACADO_MPC/jackal_mpc_traj.rviz) and run the following command in this directory to open rviz
```
rviz -d jackal_mpc_traj.rviz
```

4. In your generated qpOASES interface package directory, run the executable file. 
```
./test
```
This will subscribe to JACKAL's pose information (`/odometry/filtered` topic), execute the controller designed by MPC and issue movement instructions to the car's drive system (`/cmd_vel` topic). You should be able to see two arrow flows of different colors in rviz (the green one represents the scheduled trajectory, and the red one represents the mpc predicted trajectory) and your car moving in the gazebo.

## JACKAL MPC with MoCap

_**Make sure your controller runs well in Gazebo before experimenting on JACKAL**_

1. Open the laboratory's Mocap system, open the motive software, and ensure that the rigid body information has been published to the relevant network ip (192.168.1.202). Then on Jackal, in the /home/vrpn_ws directory:
```
source devel/setup.bash
roslaunch vrpn_client_ros sample.launch  server:=192.168.1.202
```
This will create a publisher to receive Jackal pose messages from the Mocap system and publish them.

2. Modify the pose subscriber in the [test.c](https://github.com/wisc-arclab/JACKAL_UGV/blob/ACADO_MPC/replacement_files/test.c) file to `/vrpn_client_node/JACKAL/pose`. Modify the reference coordinate system of the published trajectory to `/world` and then recompile it:
```
make clean
make
```

If there is a discrepancy between the pose released by motive and the simulation in gazebo, please adjust it in the `pos_callback` function of [test.c](https://github.com/wisc-arclab/JACKAL_UGV/blob/ACADO_MPC/replacement_files/test.c)

3. run the executable file:
```
./test
```
If you want to see the visual interface, please open rviz yourself and open related topics.
