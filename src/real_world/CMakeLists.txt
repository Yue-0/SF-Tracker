cmake_minimum_required(VERSION 3.0.2)
project(real_world)

# Compile as C++14, enable O3 optimization
add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    rospy
    roscpp
    sensor_msgs
    geometry_msgs
)

# Catkin specific configuration
catkin_package()

# Specify additional locations of header files
include_directories(
    ${catkin_INCLUDE_DIRS}
)

# Declare a C++ executable
add_executable(lidar src/lidar.cpp)

# Specify libraries to link a library or executable target against
target_link_libraries(lidar
    ${catkin_LIBRARIES}
)

# Mark executable scripts for installation
catkin_install_python(PROGRAMS
    scripts/camera.py
    scripts/tracker.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
