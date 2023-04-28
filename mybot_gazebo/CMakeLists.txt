cmake_minimum_required(VERSION 3.0.2)
project(mybot_gazebo)


find_package(catkin REQUIRED
	  nav_msgs 
	  roscpp 
	  tf
	  rospy
	  sensor_msgs
	  std_msgs
	  cv_bridge
	  image_transport
	  message_generation
)

FIND_PACKAGE( OpenCV REQUIRED )                              
INCLUDE_DIRECTORIES( ${OpenCV_INCLUDE_DIRS} )
include_directories(${catkin_INCLUDE_DIRS})

catkin_package()

add_executable(edge_detect src/edge_detect_laser.cpp)
add_dependencies(edge_detect ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(edge_detect ${catkin_LIBRARIES})
