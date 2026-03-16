cmake_minimum_required(VERSION 3.0.2)
project(perception_grasp)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
  common_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    rospy 
    std_msgs 
    sensor_msgs 
    geometry_msgs
    tf2_ros
    common_msgs
)

catkin_install_python(PROGRAMS
  nodes/grasp_estimator_node.py
  tools/publish_labeled_pcd.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/ config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
