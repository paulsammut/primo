cmake_minimum_required(VERSION 2.8.3)
project(stereo_mask)

find_package(catkin REQUIRED COMPONENTS
  image_transport
  nodelet
  roscpp
  sensor_msgs
  cv_bridge
)


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

add_library(stereo_mask
  src/stereo_mask_nodelet.cpp
)

# add_executable(stereo_mask_node src/stereo_mask_node.cpp)

target_link_libraries(stereo_mask
   ${catkin_LIBRARIES}
)
