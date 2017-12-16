cmake_minimum_required(VERSION 2.8.3)
project(sabertooth_simple)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  message_generation
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  SabertoothEstop.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sabertooth_simple
#  CATKIN_DEPENDS roscpp serial
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


add_executable(sabertooth_simple src/sabertooth_simple_node.cpp
    src/sabertooth_simple.cpp)
target_link_libraries(sabertooth_simple ${catkin_LIBRARIES})

