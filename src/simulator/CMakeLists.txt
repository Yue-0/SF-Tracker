cmake_minimum_required(VERSION 3.0.2)
project(simulator)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
)

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_executable(target src/target.cpp)

target_link_libraries(target
    ${catkin_LIBRARIES}
)
