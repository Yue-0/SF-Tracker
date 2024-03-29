cmake_minimum_required(VERSION 3.0.2)
project(sf_tracker)

find_package(NLopt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    tf
    roscpp
    nav_msgs
    std_msgs
    sensor_msgs
    geometry_msgs
)

catkin_package()

include_directories(
    include
    ${NLopt_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(pp src/trajectory.cpp)
add_executable(op src/orientation.cpp)
add_executable(costmap src/costmap.cpp)

target_link_libraries(op
    ${NLOPT_LIBRARIES}
    ${catkin_LIBRARIES}
)
target_link_libraries(pp
    ${OpenCV_LIBS}
    ${NLOPT_LIBRARIES}
    ${catkin_LIBRARIES}
)
target_link_libraries(costmap
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES}
)
