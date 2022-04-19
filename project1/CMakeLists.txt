cmake_minimum_required(VERSION 2.8.3)
project(project1)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs geometry_msgs nav_msgs tf2 tf2_ros cmath)

## Add custom message and services and their dependencies


## Declare a catkin package
catkin_package()

## Build talker and listener

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(velocities src/velocities.cpp)

target_link_libraries(velocities ${catkin_LIBRARIES})

add_executable(odometry src/odometry.cpp)

target_link_libraries(odometry ${catkin_LIBRARIES})

##target_link_libraries(project1 PRIVATE m)
