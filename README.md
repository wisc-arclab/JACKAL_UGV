# CasADi-NMPC
This code repository uses CasADi to automatically solve the optimal control problem of MPC. The following is a tutorial. The code runs well on Ubuntu 20.04 + ROS Noetic.

Created by [TianxiaoYe](https://github.com/fuwafuwaboom). If you have any questions, please contact via email: **tye46@wisc.edu**
***

The following is the effect based on the JACKAL gazebo simulator: 

https://github.com/wisc-arclab/JACKAL_UGV/assets/80655645/0d3f67fa-11c7-4bb6-806f-5d0a5fc2a871


***
# Install CasADi
Before following the steps below, make sure you have installed casadi and ipopt.

Create the following two scripts for installation:

scripts 1:

create new script file

```
gedit install_ipopt.sh
```

copy the code below in this file

```
#!/usr/bin/env bash
 
# Fail on first error.
set -e
 
cd "$(dirname "${BASH_SOURCE[0]}")"
 
sudo apt-get install cppad gfortran 
 
wget https://www.coin-or.org/download/source/Ipopt/Ipopt-3.12.8.zip -O Ipopt-3.12.8.zip
unzip Ipopt-3.12.8.zip
 
# Step by step   
pushd Ipopt-3.12.8/ThirdParty/Blas
./get.Blas    
cd ../Lapack
./get.Lapack  
cd ../Mumps  
./get.Mumps  
cd ../Metis  
./get.Metis
cd ../ASL
./get.ASL
 
cd ..
cd ..
 
mkdir build  
cd build  
../configure  
make -j4  
make install  
 
cp -a include/* /usr/include/.  
cp -a lib/* /usr/lib/. 
 
popd
 
# Clean up.
cd ..
cd ..
apt-get clean && rm -rf /var/lib/apt/lists/*
rm -rf Ipopt-3.12.8.zip Ipopt-3.12.8
```

run the script

```
chmod +x install_ipopt.sh
```

```
./install_ipopt.sh
```

When executing ./get.Mumps, it will prompt that it cannot connect and you need to change line 31: `$wgetcmd http://mumps.enseeiht.fr/MUMPS_${mumps_ver}.tar.gz` into `$wgetcmd http://graal.ens-lyon.fr/MUMPS/MUMPS_${mumps_ver}.tar.gz`


scripts 2:

create new script file

```
gedit install_casadi.sh
```

copy the code below in this file

```
#!/usr/bin/env bash
 
# Fail on first error.
set -e
 
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
echo -e "\033[40;32m${DIR} \033[0m"
 
# download
wget https://github.com/casadi/casadi/releases/download/3.5.5/casadi-3.5.5-1.tar.gz
tar -zxvf casadi-3.5.5-1.tar.gz
echo -e "\033[40;32mdownload finish \033[0m"
 
cd casadi-3.5.5.1
mkdir build && cd build
cmake .. -DWITH_IPOPT=ON -DWITH_EXAMPLES=OFF
make -j4
sudo make install
sudo ldconfig
 
# Clean up.
sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*
sudo rm -fr casadi-3.5.5-1.tar.gz casadi-3.5.5.1
```

run the script

```
chmod +x install_casadi.sh
```

```
./install_casadi.sh
```

# Use this ROS Package
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
