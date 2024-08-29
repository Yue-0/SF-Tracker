# SF-Tracker

__English__ | [简体中文](README_cn.md)

## Paper

__Safety-First Tracker: A Trajectory Planning Framework for Omnidirectional Robot Tracking__. 
Accepted by __IROS 2024__.
Authors: [Yue Lin](https://github.com/Yue-0), Yang Liu, Pingping Zhang, [Xin Chen](https://github.com/chenxin-dlut), Dong Wang and Huchuan Lu.

## Video

https://github.com/Yue-0/SF-Tracker/assets/98451547/b0d80213-9b5e-4fec-a20e-c4929c28dfb8

## Instruction for Use

The following code has been tested under Ubuntu20.04 & ROS-noetic.

### 1. Clone

```shell
git clone https://github.com/Yue-0/SF-Tracker.git
cd SF-Tracker
```

### 2. Install Requirements

```shell
sudo apt update
sudo apt install lib-eigen3-dev
sudo apt install ros-noetic-nav-msgs
sudo apt install ros-noetic-sensor-msgs
sudo apt install ros-noetic-geometry-msgs
sudo apt install ros-noetic-vision-opencv
```

We use NLopt to solve numerical optimization problems. If you do not have NLopt installed, please follow the steps below to compile NLopt from source:

```shell
cd src/sf_tracker/include
git clone https://github.com/stevengj/nlopt.git
cd nlopt
mkdir build
cd build
cmake ..
make
sudo make install
cd ../../../../..
```

### 3. Build SF-Tracker

```shell
catkin_make
source devel/setup.bash
```

### 4. Using SF-Tracker in Simulation Environment

We use Gazebo for simulation and need to install simulation requirements first.

```shell
sudo apt install gazobo11
sudo apt install ros-noetic-xacro
sudo apt install ros-noetic-controller-manager
sudo apt install ros-noetic-robot-state-publisher
sudo apt install ros-noetic-joint-state-publisher
```

To start the launch file to run the simulation, you need to publish the position of the target via `2D Nav Goal` in RVIZ.

```shell
roslaunch simulator tracking.launch
```

### 5. Using SF-Tracker on Real-world Robot

See [real_world](src/real_world/README.md) package.

### 6. Modify Hyperparameters

All hyperparameters of SF-Tracker are defined in the file [planning.launch](src/sf_tracker/launch/planning.launch).
