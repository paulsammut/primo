cmake_minimum_required(VERSION 2.8.3)
project(primo_stereo)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
    roscpp
    std_msgs
    pcl_ros
    pcl_msgs
    pcl_conversions
    tf2
    )

find_package(Boost REQUIRED COMPONENTS chrono thread system timer)

find_package(PCL 1.2 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES primo_stereo
#  CATKIN_DEPENDS other_catkin_pkg
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

# This adds a dependency on the message generation
add_executable(pcl_merge src/pcl_merge.cpp)

target_link_libraries(pcl_merge 
    ${Boost_LIBRARIES} 
    ${catkin_LIBRARIES} 
)
