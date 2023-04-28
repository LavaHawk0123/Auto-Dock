cmake_minimum_required(VERSION 3.0.2)
project(mybot_control)


find_package(catkin REQUIRED
	nav_msgs 
	roscpp 
	tf
	message_generation
)
## Generate messages in the 'msg' folder
add_message_files(
   FILES
   ArucoMsg.msg
 )

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
  arucoDetection
)

catkin_package()
include_directories(${catkin_INCLUDE_DIRS})

add_executable(velocity_publisher src/velocity_publisher.cpp)
add_dependencies(velocity_publisher ${nav_msgs_EXPORTED_TARGETS})
target_link_libraries(velocity_publisher ${catkin_LIBRARIES})

add_executable(parabolic_controller src/parabolic_controller.cpp)
add_dependencies(parabolic_controller ${nav_msgs_EXPORTED_TARGETS})
target_link_libraries(parabolic_controller ${catkin_LIBRARIES})

add_executable(path_follower src/path_follower.cpp)
add_dependencies(path_follower ${nav_msgs_EXPORTED_TARGETS})
target_link_libraries(path_follower ${catkin_LIBRARIES})

add_executable(cubic_bezier src/cubic_bezier.cpp)
add_dependencies(cubic_bezier ${nav_msgs_EXPORTED_TARGETS})
target_link_libraries(cubic_bezier ${catkin_LIBRARIES})
