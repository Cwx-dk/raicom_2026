cmake_minimum_required(VERSION 2.8.3)
project(auto_charging)

find_package(catkin REQUIRED COMPONENTS
  relative_move
  ar_pose
  move_base_msgs
  actionlib
  geometry_msgs
  roscpp
  message_generation
)

add_service_files(
  FILES
  SetCharge.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)
catkin_package(
 CATKIN_DEPENDS message_runtime
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(auto_charge
  src/auto_charging.cpp
  src/ros_node.cpp
)
add_dependencies(auto_charge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(auto_charge
  ${catkin_LIBRARIES}
)

install(TARGETS auto_charge
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir launch srv)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
install(FILES
  readme.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)