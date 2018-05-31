cmake_minimum_required(VERSION 2.8.3)
project(hello_world)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)



add_executable(talker src/talker.cpp)
 target_link_libraries(talker
   ${catkin_LIBRARIES}
 )

add_executable(listener src/listener.cpp)
 target_link_libraries(listener
   ${catkin_LIBRARIES}
 )

