# SF-Tracker

[English](README.md) | __简体中文__

<!-- ## 论文

即将发布 -->

## 视频

![视频](https://github.com/Yue-0/SF-Tracker/assets/98451547/478b51a2-debc-4079-9fda-35872534d592)

## 使用说明

以下代码在 Ubuntu20.04 & ROS-noetic 下测试通过。

### 1. 克隆项目

```shell
git clone https://github.com/Yue-0/SF-Tracker.git
cd SF-Tracker
```

### 2. 安装依赖

```shell
sudo apt update
sudo apt install lib-eigen3-dev
sudo apt install ros-noetic-nav-msgs
sudo apt install ros-noetic-sensor-msgs
sudo apt install ros-noetic-geometry-msgs
sudo apt install ros-noetic-vision-opencv
```

我们使用 NLopt 求解数值优化问题，如果你没有安装 NLopt，请按照以下步骤从源码编译 NLopt：

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

### 3. 编译 SF-Tracker

```shell
catkin_make
source devel/setup.bash
```

### 4. 在模拟环境中使用 SF-Tracker

我们使用 Gazebo 进行仿真，需要先安装仿真依赖。

```shell
sudo apt install gazobo11
sudo apt install ros-noetic-xacro
sudo apt install ros-noetic-controller-manager
sudo apt install ros-noetic-robot-state-publisher
sudo apt install ros-noetic-joint-state-publisher
```

启动 launch 文件运行仿真，你需要通过 RVIZ 中的 `2D Nav Goal` 发布目标的位置。

```shell
roslaunch simulator tracking.launch
```

### 5. 在真实机器人系统上使用 SF-Tracker

参见 [real_world 功能包](src/real_world/README_cn.md)。

### 6. 修改超参数

SF-Tracker 的所有超参数都定义在文件 [planning.launch](src/sf_tracker/launch/planning.launch) 中。

## 鸣谢

我们参考了 [Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner) 的开源代码。
