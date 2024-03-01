# SF-Tracker

__English__ | [简体中文](README_cn.md)

<!-- Paper：[arXiv]() -->

<!-- Video：[Bilibili]() -->

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

To start the launch file to run the simulation, you need to publish the position of the target via `2D Pose Estimate` in RVIZ.

```shell
roslaunch simulator tracking.launch
```

### 5. Using SF-Tracker on Real-world Robot

See [real_world](src/real_world/README.md) package.

### 6. Modify Hyperparameters

All hyperparameters of SF-Tracker are defined in the file [planning.launch](src/sf_tracker/launch/planning.launch). It should be noted that if you modify the size parameter, you need to change the first two parameters of args on line 14 to match size.

### 7. PnP-Tracker

Run a complete navigation framework, and then run the following launch file to add tracking functionality to the navigation:

```shell
roslaunch sf_tracker pnp_tracker.launch
```

The launch file will run a ROS node named pnp_tracker, which subscribes to the following topics:

| Topic    | Message                | Note                    |
|:--------:|:----------------------:|:------------------------|
| /path    | nav_msgs/Path          | Trajectory output by trajectory planner. The last point is the target position |
| /costmap | nav_msgs/OccupancyGrid | Map used for navigation |

This node publishes the following topics:

| Topic       | Message             | Note       |
|:-----------:|:-------------------:|:-----------|
| /cmd_vel    | geometry_msgs/Twist | Velocity   |
| /trajectory | nav_msgs/Path       | Trajectory |
