cmake_minimum_required(VERSION 3.0.2)
project(arucoDetection)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  sensor_msgs
  nav_msgs
  std_msgs
  cv_bridge
  image_transport
  message_generation
  tf
  tf2_ros
)

include_directories(include ${catkin_INCLUDE_DIRS})
FIND_PACKAGE( OpenCV REQUIRED )                              
INCLUDE_DIRECTORIES( ${OpenCV_INCLUDE_DIRS} )

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   ArucoMsg.msg
 )

generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
  geometry_msgs
  arucoDetection
)

catkin_package()

###########
## Build ##
###########

## Your package locations should be listed before other locations
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)


add_executable(aruco_detector src/ArucoDetect.cpp src/ArucoDetect.h)
add_dependencies(aruco_detector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(aruco_detector
    ${catkin_LIBRARIES}
  )

add_executable(kf_pose src/kalmanFilter.cpp)
add_dependencies(kf_pose ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(kf_pose ${catkin_LIBRARIES} )


add_executable(aruco_detect_cam src/aruco_detect_cam.cpp)
add_dependencies(aruco_detect_cam ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(aruco_detect_cam ${catkin_LIBRARIES} )
