cmake_minimum_required(VERSION 2.8.3)

project(denmpc)
add_definitions(-std=c++11)

find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS ${catkin_INCLUDE_DIRS}
#  LIBRARIES symbolic
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS Boost
)

###########
## Build ##
###########

###################################################################
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_library(controller
  src/controller/Controller.cpp
  src/controller/Cmscgmres.cpp
)

add_library(mpc_lib
  src/Agent.cpp
  src/Constraint.cpp
  src/MathLib.cpp
  src/rbcar.cpp
)

add_executable(mpc_node src/mpc_node.cpp )
add_dependencies(mpc_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(mpc_node  ${catkin_LIBRARIES} controller mpc_lib)
