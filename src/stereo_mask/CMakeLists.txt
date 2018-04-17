cmake_minimum_required(VERSION 2.8.3)
project(stereo_mask)

find_package(catkin REQUIRED COMPONENTS
  image_transport
  nodelet
  roscpp
  sensor_msgs
  cv_bridge
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES stereo_mask
#  CATKIN_DEPENDS image_transport nodelet roscpp sensor_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Stuff for the nodelet
add_library(stereo_mask_nodelet src/stereo_mask_nodelet.cpp)
target_link_libraries(stereo_mask_nodelet ${catkin_LIBRARIES})

# Stuff for the node version of the nodelet
add_executable(stereo_mask_node src/stereo_mask_node.cpp)
target_link_libraries(stereo_mask_node ${catkin_LIBRARIES})
