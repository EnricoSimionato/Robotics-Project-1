cmake_minimum_required(VERSION 2.8.3)
project(project1)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs geometry_msgs nav_msgs tf2 tf2_ros message_generation dynamic_reconfigure)

## Add custom message and services and their dependencies
add_message_files(
  FILES
  wheels_rpm.msg
 )

add_service_files(
  FILES
  Reset.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
 )

generate_dynamic_reconfigure_options(
  cfg/parameters.cfg 
  cfgCalibration/parametersCalibration.cfg
)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime)

## Build talker and listener

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(velocities src/velocities.cpp)
add_dependencies(velocities ${catkin_EXPORTED_TARGETS})
target_link_libraries(velocities ${catkin_LIBRARIES})

add_executable(odometry src/odometry.cpp)
add_dependencies(odometry ${catkin_EXPORTED_TARGETS})
target_link_libraries(odometry ${catkin_LIBRARIES})

add_executable(control src/control.cpp)
add_dependencies(control ${catkin_EXPORTED_TARGETS})
target_link_libraries(control ${catkin_LIBRARIES})

add_executable(reset_client src/client.cpp)
add_dependencies(reset_client ${catkin_EXPORTED_TARGETS})
target_link_libraries(reset_client ${catkin_LIBRARIES})

add_executable(calibration src/calibration.cpp)
target_link_libraries(calibration ${catkin_LIBRARIES})

##target_link_libraries(project1 PRIVATE m)
