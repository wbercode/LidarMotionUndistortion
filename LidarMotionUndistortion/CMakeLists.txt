cmake_minimum_required(VERSION 2.8.3)
project(LidarMotionUndistortion)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  sensor_msgs
  nav_msgs
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL 1.7 REQUIRED)

catkin_package()

include_directories(
  include/LidarUndistortion
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

message(status ${PCL_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node src/LidarMotionUndistortion.cpp src/LidarMotionUndistortion_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
 ${catkin_LIBRARIES}
 ${PCL_LIBRARIES}
)
