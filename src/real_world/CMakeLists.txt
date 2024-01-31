cmake_minimum_required(VERSION 3.0.2)
project(real_world)

find_package(catkin REQUIRED COMPONENTS
    rospy
    roscpp
    sensor_msgs
    geometry_msgs
)

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_executable(lidar src/lidar.cpp)

target_link_libraries(lidar
    ${catkin_LIBRARIES}
)

catkin_install_python(PROGRAMS
    scripts/camera.py
    scripts/tracker.py
    scripts/detector.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
