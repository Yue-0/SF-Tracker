cmake_minimum_required(VERSION 3.0.2)
project(sf_tracker)

# Compile as C++14, enable O3 optimization
add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

# Find catkin macros and libraries
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

# Catkin specific configuration
catkin_package()

# Specify additional locations of header files
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

# Declare C++ executables
add_executable(costmap src/costmap.cpp)
add_executable(planner src/planner.cpp)
add_executable(control src/control.cpp)

# Specify libraries to link a library or executable target against
target_link_libraries(costmap
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES}
)
target_link_libraries(planner
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES}
)
target_link_libraries(control
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES}
)
