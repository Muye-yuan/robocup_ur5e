cmake_minimum_required(VERSION 3.0.2)
project(motion_control)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  rospy
  std_msgs
  std_srvs
  geometry_msgs
  trajectory_msgs
  sensor_msgs
  control_msgs
  common_msgs
  tf2_ros
  tf2_geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    actionlib
    actionlib_msgs
    rospy
    std_msgs
    std_srvs
    geometry_msgs
    trajectory_msgs
    sensor_msgs
    control_msgs
    common_msgs
    tf2_ros
    tf2_geometry_msgs
)

catkin_install_python(PROGRAMS
  nodes/motion_control_node.py
  nodes/gripper_monitor_node.py
  nodes/gripper_control_node.py
  scripts/check_moveit_ik.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
