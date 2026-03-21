#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
One-shot 规划轨迹 RViz 显示：仿照 PRM_Planner 发布末端路径 LINE_STRIP、目标点球体、状态文字。

供 demo_one_shot_planning 或其它节点调用：
  from one_shot_trajectory_display import publish_plan_markers
  publish_plan_markers(path_marker_pub, target_marker_pub, status_marker_pub, vis_data, success=True, frame_id="base_link")

vis_data 需包含：rrt_path_xyz (list of (x,y,z))、goal_xyz、start_xyz（后两者可选，用于目标球与状态文字位置）。
"""

import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker


def publish_plan_markers(path_marker_pub, target_marker_pub, status_marker_pub, vis_data, success=True, frame_id="base_link"):
    """
    仿照 PRM_Planner：发布末端路径 LINE_STRIP、目标点球体、状态文字，供 RViz 实时显示。

    Args:
        path_marker_pub: rospy.Publisher for ~ee_path_marker (Marker)
        target_marker_pub: rospy.Publisher for ~target_marker (Marker)
        status_marker_pub: rospy.Publisher for ~status_marker (Marker)
        vis_data: dict with rrt_path_xyz (list of (x,y,z)), goal_xyz, start_xyz
        success: True 为绿/蓝，False 为红
        frame_id: 坐标系，默认 base_link
    """
    if vis_data is None:
        return
    now = rospy.Time.now()
    rrt_path_xyz = vis_data.get("rrt_path_xyz", [])
    goal_xyz = vis_data.get("goal_xyz")

    # 目标点球体（与 PRM target_marker 一致）
    if target_marker_pub is not None and goal_xyz is not None and len(goal_xyz) >= 3:
        target_marker = Marker()
        target_marker.header.frame_id = frame_id
        target_marker.header.stamp = now
        target_marker.ns = "one_shot_target"
        target_marker.id = 1
        target_marker.type = Marker.SPHERE
        target_marker.action = Marker.ADD
        target_marker.pose.position.x = float(goal_xyz[0])
        target_marker.pose.position.y = float(goal_xyz[1])
        target_marker.pose.position.z = float(goal_xyz[2])
        target_marker.pose.orientation.w = 1.0
        target_marker.scale.x = 0.035
        target_marker.scale.y = 0.035
        target_marker.scale.z = 0.035
        target_marker.color.a = 1.0
        target_marker.color.r = 0.1 if success else 1.0
        target_marker.color.g = 1.0 if success else 0.2
        target_marker.color.b = 0.1
        target_marker_pub.publish(target_marker)

    # 末端路径 LINE_STRIP（与 PRM path_marker 一致）
    if path_marker_pub is not None and rrt_path_xyz:
        path_marker = Marker()
        path_marker.header.frame_id = frame_id
        path_marker.header.stamp = now
        path_marker.ns = "one_shot_ee_path"
        path_marker.id = 2
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.scale.x = 0.01
        path_marker.color.a = 1.0
        path_marker.color.r = 0.1 if success else 1.0
        path_marker.color.g = 0.8 if success else 0.2
        path_marker.color.b = 1.0 if success else 0.2
        path_marker.points = []
        for x, y, z in rrt_path_xyz:
            pt = Point()
            pt.x = float(x)
            pt.y = float(y)
            pt.z = float(z)
            path_marker.points.append(pt)
        path_marker_pub.publish(path_marker)

    # 状态文字（与 PRM status_marker 一致）
    if status_marker_pub is not None and goal_xyz is not None and len(goal_xyz) >= 3:
        status_marker = Marker()
        status_marker.header.frame_id = frame_id
        status_marker.header.stamp = now
        status_marker.ns = "one_shot_status"
        status_marker.id = 3
        status_marker.type = Marker.TEXT_VIEW_FACING
        status_marker.action = Marker.ADD
        status_marker.pose.position.x = float(goal_xyz[0])
        status_marker.pose.position.y = float(goal_xyz[1])
        status_marker.pose.position.z = float(goal_xyz[2]) + 0.08
        status_marker.pose.orientation.w = 1.0
        status_marker.scale.z = 0.04
        status_marker.color.a = 1.0
        status_marker.color.r = 0.1 if success else 1.0
        status_marker.color.g = 1.0 if success else 0.2
        status_marker.color.b = 0.1
        status_marker.text = "OneShot: OK" if success else "OneShot: Fail"
        status_marker_pub.publish(status_marker)
