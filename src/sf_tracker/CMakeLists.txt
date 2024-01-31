cmake_minimum_required(VERSION 3.0.2)
project(sf_tracker)

find_package(NLopt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    tf
    roscpp
    nav_msgs
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

add_executable(costmap src/costmap.cpp)
add_executable(planner src/planner.cpp)
add_executable(controller src/controller.cpp)

target_link_libraries(costmap
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES}
)
target_link_libraries(planner
    ${OpenCV_LIBS}
    ${NLOPT_LIBRARIES}
    ${catkin_LIBRARIES}
)
target_link_libraries(controller
    ${NLOPT_LIBRARIES}
    ${catkin_LIBRARIES}
)
