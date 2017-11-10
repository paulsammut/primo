cmake_minimum_required(VERSION 2.8.3)
project(differential_drive)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  nav_msgs
  rospy
  std_msgs
  tf
  tf2
  tf2_ros
  roscpp
)

generate_dynamic_reconfigure_options(
    cfg/DiffTfConfig.cfg
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(primo_diff_tf src/primo_diff_tf.cpp)

target_link_libraries(primo_diff_tf
   ${catkin_LIBRARIES}
)


include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(pid_velocity src/pid_velocity.cpp)
target_link_libraries(pid_velocity ${catkin_LIBRARIES})
