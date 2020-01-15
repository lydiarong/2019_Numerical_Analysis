cmake_minimum_required(VERSION 2.8.3)
project(jaco_trajectory)
find_package(catkin REQUIRED COMPONENTS
  controller_manager
  effort_controllers
  joint_state_controller
  robot_state_publisher
  roscpp
  rospy
  rqt_gui
  std_msgs
  tf
  geometry_msgs
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(calc_broken_joint_angle_ src/calc_broken_joint_angle.cpp)
target_link_libraries(calc_broken_joint_angle_ ${catkin_LIBRARIES})
