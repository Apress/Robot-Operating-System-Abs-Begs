cmake_minimum_required(VERSION 2.8.3)
project(mobile_robot_pkg)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  move_base_msgs
  nav_msgs
  robot_state_publisher
  roscpp
  rospy
  std_msgs
  tf
)


catkin_package()


include_directories(
  ${catkin_INCLUDE_DIRS}
)


