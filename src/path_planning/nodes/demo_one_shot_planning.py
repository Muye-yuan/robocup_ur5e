#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
一次性规划 Demo：与 ur5e_teach_pendant_ui 一致，不直接使用 IK/FK，相信 motion_control。
- 当前末端位姿从 TF (base_link -> tcp_link) 获取；
- 当前关节从 /joint_states 获取；
- 目标关节从参数 ~goal_joints 获取（由配置或 motion_control 侧约定）；
- 仅通过 /motion/command 发布 MotionCommand(EXECUTE_TRAJECTORY) 供 motion_control 执行。

运行前请确保：
  - roscore 已启动
  - motion_control_node 已运行（订阅 /motion/command）
  - TF 中有 base_link -> gripper_tip_link（或 ~tcp_link）
  - 参数 ~goal_joints 已设置（或使用默认）

运行方式：
  rosrun path_planning demo_one_shot_planning.py
  rosrun path_planning demo_one_shot_planning.py _goal_joints:="[0, -1.57, 0, -1.57, 0, 0]"
"""

import sys
import os

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

import rospy
from sensor_msgs.msg import PointCloud2, JointState
from common_msgs.msg import MotionCommand
import tf2_ros

from one_shot_planner import (
    plan_one_shot,
    get_default_virtual_grasp_point,
    build_motion_command_execute_trajectory,
    plot_planning_result_3d,
)
from aco_rrtstar_planner_node import (
    pointcloud_to_obstacles,
    GAZEBO_DEFAULT_OBSTACLES,
)


def _parse_virtual_grasp_point(param_val):
    """解析 ~virtual_grasp_point：支持 [x,y,z] 或 "x,y,z" 或列表。"""
    if param_val is None:
        return get_default_virtual_grasp_point()
    if isinstance(param_val, (list, tuple)) and len(param_val) >= 3:
        return [float(param_val[0]), float(param_val[1]), float(param_val[2])]
    if isinstance(param_val, str):
        import ast
        try:
            v = ast.literal_eval(param_val)
            if isinstance(v, (list, tuple)) and len(v) >= 3:
                return [float(v[0]), float(v[1]), float(v[2])]
        except Exception:
            pass
    return get_default_virtual_grasp_point()


def _parse_goal_joints(param_val):
    """解析 ~goal_joints：支持列表或字符串形式的列表。未设置时返回 None，由调用方用默认值。"""
    if param_val is None:
        return None
    if isinstance(param_val, (list, tuple)) and len(param_val) >= 6:
        return [float(param_val[i]) for i in range(6)]
    if isinstance(param_val, str):
        import ast
        try:
            v = ast.literal_eval(param_val)
            if isinstance(v, (list, tuple)) and len(v) >= 6:
                return [float(v[i]) for i in range(6)]
        except Exception:
            pass
    return None


def _get_current_pose_from_tf(tf_buffer, tcp_link="gripper_tip_link", timeout=0.5):
    """
    与 ur5e_teach_pendant_ui 一致：从 TF 获取当前末端位姿 (base_link -> tcp_link)。
    返回 (start_xyz, pose_stamped) 或 (None, None)。start_xyz 为 [x, y, z]。
    """
    try:
        trans = tf_buffer.lookup_transform(
            "base_link", tcp_link, rospy.Time(0), rospy.Duration(timeout)
        )
        x = trans.transform.translation.x
        y = trans.transform.translation.y
        z = trans.transform.translation.z
        start_xyz = [x, y, z]
        return start_xyz, trans
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logdebug("[Demo] TF lookup failed: %s", str(e))
        return None, None


def _get_start_joints_from_topic(timeout=5.0):
    """从 /joint_states 取一次当前关节角；超时返回 None。"""
    joints = [None]

    def cb(msg):
        if joints[0] is None and msg.name and msg.position:
            # 使用 UR5e 关节顺序
            order = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                     'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
            pos = list(msg.position)
            names = list(msg.name)
            try:
                j = [pos[names.index(n)] for n in order]
                joints[0] = j
            except (ValueError, IndexError):
                if len(pos) >= 6:
                    joints[0] = list(pos)[:6]
            if joints[0] is not None:
                rospy.loginfo("[Demo] 从 /joint_states 获取起点: %s", [round(x, 3) for x in joints[0]])

    sub = rospy.Subscriber('/joint_states', JointState, cb, queue_size=1)
    rate = rospy.Rate(20)
    t0 = rospy.Time.now()
    while not rospy.is_shutdown() and (rospy.Time.now() - t0).to_sec() < timeout:
        if joints[0] is not None:
            break
        rate.sleep()
    sub.unregister()
    return joints[0]


def _get_obstacles_from_pointcloud_topic(timeout=3.0):
    """从 ~pointcloud_topic 取一帧点云并转为障碍物；超时返回 None。"""
    topic = rospy.get_param('~pointcloud_topic', '/camera/depth/points')
    voxel_res = rospy.get_param('~voxel_resolution', 0.05)
    frame_id = rospy.get_param('~frame_id', 'base_link')
    bounds = (
        rospy.get_param('~workspace_x', [-0.5, 1.0]),
        rospy.get_param('~workspace_y', [-0.5, 0.5]),
        rospy.get_param('~workspace_z', [0.0, 0.8]),
    )
    cloud = [None]

    def cb(msg):
        if cloud[0] is None:
            cloud[0] = msg

    sub = rospy.Subscriber(topic, PointCloud2, cb, queue_size=1)
    rate = rospy.Rate(20)
    t0 = rospy.Time.now()
    while not rospy.is_shutdown() and (rospy.Time.now() - t0).to_sec() < timeout:
        if cloud[0] is not None:
            break
        rate.sleep()
    sub.unregister()
    if cloud[0] is None:
        return None
    obs = pointcloud_to_obstacles(cloud[0], voxel_res=voxel_res, frame_id=frame_id, bounds=bounds)
    rospy.loginfo("[Demo] 从点云得到 %d 个障碍物", len(obs))
    return obs if obs else None


def main():
    rospy.init_node('demo_one_shot_planning', anonymous=False)

    # 与示教器一致：TF 用于当前末端位姿
    tcp_link = rospy.get_param('~tcp_link', 'gripper_tip_link')
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.sleep(0.3)  # 让 TF 缓冲有数据

    # 起点位姿：从 TF 获取（同 ur5e_teach_pendant_ui._get_current_pose_from_tf）
    start_xyz, _ = _get_current_pose_from_tf(tf_buffer, tcp_link=tcp_link)
    if start_xyz is None:
        start_xyz = rospy.get_param('~start_xyz', [0.2, 0.0, 0.5])
        rospy.logwarn("[Demo] TF 无 base_link->%s，使用 ~start_xyz: %s", tcp_link, start_xyz)
    else:
        rospy.loginfo("[Demo] 从 TF 获取起点位姿 (base_link->%s): %s", tcp_link, [round(x, 3) for x in start_xyz])

    # 起点关节：从 /joint_states 获取（同示教器）
    start_joints = _get_start_joints_from_topic(timeout=5.0)
    if start_joints is None:
        start_joints = rospy.get_param('~home_joints', [0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0])
        rospy.loginfo("[Demo] 使用 ~home_joints 作为起点关节: %s", [round(x, 3) for x in start_joints])
    else:
        rospy.loginfo("[Demo] 从 /joint_states 获取起点关节: %s", [round(x, 3) for x in start_joints])

    # 目标：笛卡尔点用于 ACO；关节角由配置提供，不在此做 IK（相信 motion_control）
    goal_xyz = _parse_virtual_grasp_point(rospy.get_param('~virtual_grasp_point', None))
    goal_joints = _parse_goal_joints(rospy.get_param('~goal_joints', None))
    if goal_joints is None:
        # 默认目标关节：与桌面中心 (0.35, 0, 0.2) 适配，臂前伸下探、肘弯曲，不与桌面冲突
        goal_joints = rospy.get_param('~goal_joints_default', [0.0, -2.0, 1.2, -1.5708, -1.5708, 0.0])
        rospy.loginfo("[Demo] 使用 ~goal_joints_default 作为目标关节: %s", [round(x, 3) for x in goal_joints])
    else:
        rospy.loginfo("[Demo] 目标关节 ~goal_joints: %s", [round(x, 3) for x in goal_joints])
    rospy.loginfo("[Demo] 目标位姿 (ACO 用) virtual_grasp_point: %s", goal_xyz)

    # 障碍物：优先点云，否则 Gazebo 默认
    obstacles = _get_obstacles_from_pointcloud_topic(timeout=3.0)
    if obstacles is None:
        obstacles = GAZEBO_DEFAULT_OBSTACLES
        rospy.loginfo("[Demo] 使用 Gazebo 默认障碍物，数量: %d", len(obstacles))
    else:
        rospy.loginfo("[Demo] 从点云得到 %d 个障碍物", len(obstacles))

    # 一次性规划（不调用 IK/FK，start_xyz 来自 TF，goal_joints 来自配置），并返回可视化数据用于 3D 图
    result = plan_one_shot(
        start_xyz=start_xyz,
        start_joints=start_joints,
        goal_xyz=goal_xyz,
        goal_joints=goal_joints,
        obstacles=obstacles,
        return_vis_data=True,
    )
    if result is None or len(result) < 2:
        rospy.logerr("[Demo] 规划失败，退出")
        return
    path_joints, trajectory = result[0], result[1]
    vis_data = result[2] if len(result) > 2 else None

    if path_joints is None or trajectory is None:
        rospy.logerr("[Demo] 规划失败，退出")
        return

    # 可选：发布轨迹 Marker 供 RViz 显示（由 one_shot_trajectory_display 模块实现）
    try:
        from one_shot_trajectory_display import publish_plan_markers
        frame_id = rospy.get_param('~frame_id', 'base_link')
        if rospy.get_param('~publish_plan_markers', True):
            from visualization_msgs.msg import Marker
            path_marker_pub = rospy.Publisher('~ee_path_marker', Marker, queue_size=1, latch=True)
            target_marker_pub = rospy.Publisher('~target_marker', Marker, queue_size=1, latch=True)
            status_marker_pub = rospy.Publisher('~status_marker', Marker, queue_size=1, latch=True)
            rospy.sleep(0.2)
            publish_plan_markers(path_marker_pub, target_marker_pub, status_marker_pub, vis_data, success=True, frame_id=frame_id)
            rospy.loginfo("[Demo] 已发布轨迹 Marker：~ee_path_marker, ~target_marker, ~status_marker")
    except Exception as e:
        rospy.logdebug("[Demo] 轨迹显示未启用或模块不可用: %s", e)

    # 根据 ACO 路径、RRT* 路径、最终路径与点云障碍物绘制 3D 图并保存（默认 /tmp，避免 Docker/只读环境下写包目录失败）
    if vis_data:
        viz_path = rospy.get_param("~planning_viz_3d_output", "/tmp/planning_result_3d.png")
        plot_planning_result_3d(vis_data, output_path=viz_path)

    # 与示教器一致：仅通过 /motion/command 发布 MotionCommand，相信 motion_control 执行
    cmd_pub = rospy.Publisher('/motion/command', MotionCommand, queue_size=10)
    rospy.sleep(0.5)  # 等待订阅者
    cmd = build_motion_command_execute_trajectory(trajectory)
    cmd_pub.publish(cmd)
    rospy.loginfo("[Demo] 已发布 EXECUTE_TRAJECTORY 到 /motion/command，共 %d 个点（与 teach pendant 同方式）", len(trajectory.points))

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
