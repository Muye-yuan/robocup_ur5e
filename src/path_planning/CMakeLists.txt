cmake_minimum_required(VERSION 3.0.2)
project(path_planning)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  moveit_msgs
  trajectory_msgs
  common_msgs
  message_generation
)

## 运动学服务接口 (供 motion_control 实现)
add_service_files(
  FILES
  ComputeIK.srv
  ComputeFK.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
    actionlib
    actionlib_msgs
    rospy
    std_msgs
    geometry_msgs
    sensor_msgs
    visualization_msgs
    moveit_msgs
    trajectory_msgs
    common_msgs
    message_runtime
)

catkin_install_python(PROGRAMS
  nodes/aco_rrtstar_planner_node.py
  nodes/PRM_Planner.py
  nodes/Planner_Scene.py
  nodes/test_aco_rrtstar.py
  nodes/demo_one_shot_planning.py
  nodes/pointcloud_target_removal.py
  nodes/demo_one_shot_with_target_removal.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/ config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
