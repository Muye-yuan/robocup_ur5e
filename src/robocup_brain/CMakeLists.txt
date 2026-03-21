cmake_minimum_required(VERSION 3.0.2)
project(robocup_brain)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  moveit_msgs
  actionlib
  actionlib_msgs
  common_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    rospy 
    std_msgs 
    geometry_msgs 
    moveit_msgs 
    actionlib
    actionlib_msgs
    common_msgs
)

catkin_install_python(PROGRAMS
  nodes/brain_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
