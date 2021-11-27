cmake_minimum_required(VERSION 3.0.2)
project(vaccum_walker)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vaccum_walker
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
add_executable(walker src/walker.cpp)
target_link_libraries(walker ${catkin_LIBRARIES})
add_dependencies(walker vaccum_walker_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_executable(walker_node src/walker_node.cpp)
target_link_libraries(walker_node ${catkin_LIBRARIES})
add_dependencies(walker_node vaccum_walker_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
