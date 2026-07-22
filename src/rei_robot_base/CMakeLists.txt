cmake_minimum_required(VERSION 3.0.2)
project(rei_robot_base)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -fPIC)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  nav_msgs
  message_generation
  tf2_ros
)

add_message_files(
  FILES
  CarData.msg
  MotorCmd.msg
  BumperCliff.msg
)

add_service_files(
  FILES
  SetIO.srv
  Int8.srv
  CtrlMode.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs#   sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES rei_kinematics rei_base_communication
  CATKIN_DEPENDS nav_msgs roscpp sensor_msgs geometry_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(rei_kinematics
  src/kinematics/diff_kinematics.cpp
  src/kinematics/three_omni_kinematics.cpp
  src/kinematics/mecanum_box_kinematics.cpp
)

add_library(rei_base_communication
  src/communication/rei_base_communication.cpp
)
target_link_libraries(rei_base_communication
  modbus
)
add_executable(robot_base_node
  src/rei_robot_base.cpp
  src/ros_node.cpp
)

add_dependencies(robot_base_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(robot_base_node
  ${catkin_LIBRARIES}
  rei_kinematics
  rei_base_communication
)

install(TARGETS robot_base_node
 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS rei_base_communication rei_kinematics
 ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
 FILES_MATCHING PATTERN "*.h"
 PATTERN ".svn" EXCLUDE
)

foreach(dir launch srv msg)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

