cmake_minimum_required(VERSION 3.0.2)
project(perception_yolo)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  message_filters
  actionlib
  actionlib_msgs
  message_generation
  common_msgs
)

add_action_files(
  FILES
  SegmentXYZL.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    rospy 
    std_msgs 
    sensor_msgs 
    cv_bridge
    message_filters
    actionlib
    actionlib_msgs
    message_runtime
    common_msgs
)

catkin_install_python(PROGRAMS
  nodes/yolo_detector_node.py
  nodes/yolo26_info_node.py
  nodes/yolo26_seg_json_node.py
  nodes/yolo26_seg_pointcloud_node.py
  nodes/yolo26_seg_xyzl_cloud_node.py
  nodes/yolo26_seg_xyzl_action_server_node.py
  nodes/best_seg_gazebo_to_pcd_gpu.py
  nodes/camera_view_capture_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/ config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
