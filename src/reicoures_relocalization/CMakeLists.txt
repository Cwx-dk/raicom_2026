cmake_minimum_required(VERSION 3.0.2)

project(reicoures_relocalization)

# ============================================================
# Catkin dependencies
# ============================================================

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  actionlib
  move_base_msgs
  relative_move
  ar_pose
)


# ============================================================
# Catkin package
# ============================================================

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    actionlib
    move_base_msgs
    relative_move
    ar_pose
)


# ============================================================
# Include directories
# ============================================================

include_directories(
  ${catkin_INCLUDE_DIRS}
)


# ============================================================
# auto_charge_node
# ============================================================

add_executable(
  auto_charge_node
  src/auto_charge.cpp
)


# ============================================================
# IMPORTANT:
#
# auto_charge.cpp 使用：
#
#   relative_move/SetRelativeMove.h
#   ar_pose/Track.h
#
# 这些头文件由其他 ROS package 的 .srv 自动生成。
#
# 所以必须确保 message/service generation 完成之后，
# 才开始编译 auto_charge_node。
# ============================================================

add_dependencies(
  auto_charge_node

  ${catkin_EXPORTED_TARGETS}
)


# ============================================================
# Link libraries
# ============================================================

target_link_libraries(
  auto_charge_node
  ${catkin_LIBRARIES}
)
