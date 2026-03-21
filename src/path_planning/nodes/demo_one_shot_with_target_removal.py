#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
挖掉指定点云部分后的 One-Shot 路径规划 Demo（方式 B：基于 YOLO 分割点云）。

流程：
  1. 订阅原始点云 /camera/depth/points 和分割点云 /perception/yolo26_seg_cloud
  2. 将两者变换到 base_link，用 pointcloud_target_removal 挖空“目标类别”区域
  3. 剩余点云体素化为障碍物，调用 ACO+RRT* 做一次性规划
  4. 发布轨迹到 /motion/command，可选发布 Marker、保存 3D 图

参数：
  ~target_class_id: int，要在点云中挖掉的 YOLO 类别 id（如 0=can）
  ~pointcloud_topic: 原始点云话题，默认 /camera/depth/points
  ~seg_cloud_topic: 带 label 的分割点云，默认 /perception/yolo26_seg_cloud
  ~frame_id: 规划坐标系，默认 base_link
  ~goal_pose_4x4: 可选。若传入 4x4 齐次变换（base 系下目标末端位姿），则使用 plan_one_shot_from_goal_pose，
                  忽略 virtual_grasp_point/goal_joints；默认不设则仍用 virtual_grasp_point + IK。
  其余与 demo_one_shot_planning 一致（~goal_joints_default, ~virtual_grasp_point 等）

运行前请确保：
  - roscore、motion_control 已运行
  - /camera/depth/points 与 /perception/yolo26_seg_cloud 有数据（YOLO 分割节点已跑）
  - TF 中有 base_link -> 点云 frame（如 camera_depth_optical_frame）

运行：
  rosrun path_planning demo_one_shot_with_target_removal.py
  rosrun path_planning demo_one_shot_with_target_removal.py _target_class_id:=0
"""

import sys
import os

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

import rospy
from sensor_msgs.msg import PointCloud2, JointState
from geometry_msgs.msg import Point as PointMsg, TransformStamped
from common_msgs.msg import MotionCommand
import tf2_ros
import tf2_geometry_msgs
import sensor_msgs.point_cloud2 as pc2

from one_shot_planner import (
    plan_one_shot,
    plan_one_shot_from_goal_pose,
    get_default_virtual_grasp_point,
    build_motion_command_execute_trajectory,
    plot_planning_result_3d,
    build_obstacles_from_yolo_instance_cloud,
)
from aco_rrtstar_planner_node import (
    pointcloud_to_obstacles,
    GAZEBO_DEFAULT_OBSTACLES,
    KinematicsClient,
)
from pointcloud_target_removal import remove_target_region_from_pointcloud


def _clear_all_plan_markers(frame_id="base_link"):
    """启动时清除本 demo 使用的所有 Marker（ee_path、status），以及历史上用 target_marker 发布过的目标点球残留。目标位姿已改为仅用 TF 显示，不再发布目标点。"""
    try:
        from visualization_msgs.msg import Marker
        now = rospy.Time.now()
        pubs = [
            ("~ee_path_marker", "one_shot_ee_path", 2),
            ("~target_marker", "one_shot_target", 1),  # 仅 DELETE 旧球体残留，目标现用 TF one_shot_target_pose
            ("~status_marker", "one_shot_status", 3),
        ]
        for topic, ns, mid in pubs:
            pub = rospy.Publisher(topic, Marker, queue_size=1, latch=True)
            m = Marker()
            m.header.stamp = now
            m.header.frame_id = frame_id
            m.ns = ns
            m.id = mid
            m.action = Marker.DELETE
            pub.publish(m)
        rospy.sleep(0.15)
        rospy.loginfo("[DemoTargetRemoval] Cleared previous plan markers (ee_path, target_marker残留, status)")
    except Exception as e:
        rospy.logdebug("[DemoTargetRemoval] Clear markers: %s", e)


def _transform_pointcloud_to_frame(cloud_msg, target_frame, tf_buffer, timeout=0.5):
    """
    将 PointCloud2 变换到 target_frame（仅 x,y,z）。失败返回 None。
    """
    if cloud_msg.header.frame_id == target_frame:
        return cloud_msg
    try:
        trans = tf_buffer.lookup_transform(
            target_frame,
            cloud_msg.header.frame_id,
            cloud_msg.header.stamp if cloud_msg.header.stamp.to_sec() > 0 else rospy.Time(0),
            rospy.Duration(timeout),
        )
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logwarn("[DemoTargetRemoval] TF %s -> %s failed: %s",
                      cloud_msg.header.frame_id, target_frame, str(e))
        return None
    points = []
    for p in pc2.read_points(cloud_msg, skip_nans=True, field_names=("x", "y", "z")):
        pt = PointMsg()
        pt.x = float(p[0])
        pt.y = float(p[1])
        pt.z = float(p[2])
        pt_base = tf2_geometry_msgs.do_transform_point(pt, trans)
        points.append((pt_base.point.x, pt_base.point.y, pt_base.point.z))
    if not points:
        return None
    from std_msgs.msg import Header
    header = Header(stamp=trans.header.stamp, frame_id=target_frame)
    return pc2.create_cloud_xyz32(header, points)


def _transform_seg_cloud_to_frame(seg_cloud_msg, target_frame, tf_buffer, timeout=0.5):
    """
    将带 label 的分割点云变换到 target_frame，保留 label 字段。失败返回 None。
    """
    if seg_cloud_msg.header.frame_id == target_frame:
        return seg_cloud_msg
    has_label = any(f.name in ("label", "l") for f in seg_cloud_msg.fields)
    label_name = "label" if any(f.name == "label" for f in seg_cloud_msg.fields) else "l"
    field_names = ["x", "y", "z"]
    if has_label:
        field_names.append(label_name)
    try:
        trans = tf_buffer.lookup_transform(
            target_frame,
            seg_cloud_msg.header.frame_id,
            seg_cloud_msg.header.stamp if seg_cloud_msg.header.stamp.to_sec() > 0 else rospy.Time(0),
            rospy.Duration(timeout),
        )
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logwarn("[DemoTargetRemoval] TF seg %s -> %s failed: %s",
                      seg_cloud_msg.header.frame_id, target_frame, str(e))
        return None
    points_out = []
    for p in pc2.read_points(seg_cloud_msg, skip_nans=True, field_names=field_names):
        pt = PointMsg()
        pt.x = float(p[0])
        pt.y = float(p[1])
        pt.z = float(p[2])
        pt_base = tf2_geometry_msgs.do_transform_point(pt, trans)
        if has_label and len(p) >= 4:
            points_out.append((pt_base.point.x, pt_base.point.y, pt_base.point.z, int(p[3])))
        else:
            points_out.append((pt_base.point.x, pt_base.point.y, pt_base.point.z, 0))
    if not points_out:
        return None
    from std_msgs.msg import Header
    from sensor_msgs.msg import PointField
    header = Header(stamp=trans.header.stamp, frame_id=target_frame)
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="label", offset=12, datatype=PointField.UINT32, count=1),
    ]
    return pc2.create_cloud(header, fields, points_out)


def _parse_float_list(param_val, default, min_len=3):
    if param_val is None:
        return default
    if isinstance(param_val, (list, tuple)) and len(param_val) >= min_len:
        return [float(param_val[i]) for i in range(min_len)]
    if isinstance(param_val, str):
        import ast
        try:
            v = ast.literal_eval(param_val)
            if isinstance(v, (list, tuple)) and len(v) >= min_len:
                return [float(v[i]) for i in range(min_len)]
        except Exception:
            pass
    return default


def _parse_goal_joints(param_val):
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


def _parse_goal_pose_4x4(param_val):
    """解析 ~goal_pose_4x4：4x4 齐次变换。支持 [[row0],[row1],[row2],[row3]] 或 16 个数行优先。未设置或空字符串返回 None。"""
    if param_val is None or (isinstance(param_val, str) and param_val.strip() == ""):
        return None
    import ast
    try:
        if isinstance(param_val, str):
            param_val = ast.literal_eval(param_val)
        if isinstance(param_val, (list, tuple)):
            if len(param_val) == 4 and all(isinstance(r, (list, tuple)) and len(r) == 4 for r in param_val):
                return [[float(param_val[i][j]) for j in range(4)] for i in range(4)]
            if len(param_val) == 16:
                return [[float(param_val[i * 4 + j]) for j in range(4)] for i in range(4)]
    except Exception:
        pass
    return None


def _get_current_pose_from_tf(tf_buffer, tcp_link="gripper_tip_link", timeout=0.5):
    try:
        trans = tf_buffer.lookup_transform(
            "base_link", tcp_link, rospy.Time(0), rospy.Duration(timeout)
        )
        x = trans.transform.translation.x
        y = trans.transform.translation.y
        z = trans.transform.translation.z
        return [x, y, z], trans
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        return None, None


def _get_start_joints_from_topic(timeout=5.0):
    joints = [None]

    def cb(msg):
        if joints[0] is None and msg.name and msg.position:
            order = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                     'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
            names = list(msg.name)
            pos = list(msg.position)
            try:
                joints[0] = [pos[names.index(n)] for n in order]
            except (ValueError, IndexError):
                if len(pos) >= 6:
                    joints[0] = list(pos)[:6]

    sub = rospy.Subscriber('/joint_states', JointState, cb, queue_size=1)
    rate = rospy.Rate(20)
    t0 = rospy.Time.now()
    while not rospy.is_shutdown() and (rospy.Time.now() - t0).to_sec() < timeout:
        if joints[0] is not None:
            break
        rate.sleep()
    sub.unregister()
    return joints[0]


def _get_raw_and_seg_clouds(pointcloud_topic, seg_cloud_topic, tf_buffer, frame_id, timeout=5.0):
    """等待一帧原始点云和一帧分割点云，并都变换到 frame_id。返回 (raw_in_frame, seg_in_frame) 或 (None, None)。"""
    raw = [None]
    seg = [None]

    def on_raw(msg):
        if raw[0] is None:
            raw[0] = msg

    def on_seg(msg):
        if seg[0] is None:
            seg[0] = msg

    sub_raw = rospy.Subscriber(pointcloud_topic, PointCloud2, on_raw, queue_size=1)
    sub_seg = rospy.Subscriber(seg_cloud_topic, PointCloud2, on_seg, queue_size=1)
    rate = rospy.Rate(10)
    t0 = rospy.Time.now()
    while not rospy.is_shutdown() and (rospy.Time.now() - t0).to_sec() < timeout:
        if raw[0] is not None and seg[0] is not None:
            break
        rate.sleep()
    sub_raw.unregister()
    sub_seg.unregister()

    if raw[0] is None:
        rospy.logerr("[DemoTargetRemoval] No raw point cloud from %s", pointcloud_topic)
        return None, None
    if seg[0] is None:
        rospy.logerr("[DemoTargetRemoval] No seg point cloud from %s", seg_cloud_topic)
        return None, None

    raw_in_frame = _transform_pointcloud_to_frame(raw[0], frame_id, tf_buffer, timeout=1.0)
    seg_in_frame = _transform_seg_cloud_to_frame(seg[0], frame_id, tf_buffer, timeout=1.0)
    if raw_in_frame is None:
        rospy.logerr("[DemoTargetRemoval] Failed to transform raw cloud to %s", frame_id)
        return None, None
    if seg_in_frame is None:
        rospy.logerr("[DemoTargetRemoval] Failed to transform seg cloud to %s", frame_id)
        return None, None
    return raw_in_frame, seg_in_frame


def main():
    rospy.init_node("demo_one_shot_with_target_removal", anonymous=False)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.sleep(0.5)

    frame_id = rospy.get_param("~frame_id", "base_link")
    _clear_all_plan_markers(frame_id)
    target_class_id = int(rospy.get_param("~target_class_id", 0))
    pointcloud_topic = rospy.get_param("~pointcloud_topic", "/camera/depth/points")
    seg_cloud_topic = rospy.get_param("~seg_cloud_topic", "/perception/yolo26_seg_cloud")
    instance_cloud_topic = rospy.get_param("~instance_cloud_topic", None)
    removal_padding = float(rospy.get_param("~removal_padding", 0.02))
    voxel_res = float(rospy.get_param("~voxel_resolution", 0.05))
    bounds = (
        tuple(_parse_float_list(rospy.get_param("~workspace_x", None), [-0.5, 1.0], min_len=2)),
        tuple(_parse_float_list(rospy.get_param("~workspace_y", None), [-0.5, 0.5], min_len=2)),
        tuple(_parse_float_list(rospy.get_param("~workspace_z", None), [0.0, 0.8], min_len=2)),
    )

    rospy.loginfo("[DemoTargetRemoval] target_class_id=%s, removal_padding=%.3f, frame=%s",
                  target_class_id, removal_padding, frame_id)

    # 起点
    start_xyz, _ = _get_current_pose_from_tf(tf_buffer)
    if start_xyz is None:
        start_xyz = _parse_float_list(
            rospy.get_param("~start_xyz", None),
            [0.2, 0.0, 0.5],
        )
        rospy.logwarn("[DemoTargetRemoval] Using ~start_xyz: %s", start_xyz)
    rospy.loginfo("[DemoTargetRemoval] Start from TF: %s", [round(x, 3) for x in start_xyz])

    start_joints = _get_start_joints_from_topic(timeout=5.0)
    if start_joints is None:
        start_joints = _parse_goal_joints(rospy.get_param("~home_joints", None))
    if start_joints is None:
        start_joints = [0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0]
    rospy.loginfo("[DemoTargetRemoval] Start joints: %s", [round(x, 3) for x in start_joints])

    # 目标：若传入 ~goal_pose_4x4 则使用笛卡尔矩阵模式（plan_one_shot_from_goal_pose）；否则用 virtual_grasp_point + goal_joints/IK
    goal_pose_4x4 = _parse_goal_pose_4x4(rospy.get_param("~goal_pose_4x4", None))
    use_goal_pose_4x4 = goal_pose_4x4 is not None

    if not use_goal_pose_4x4:
        goal_xyz = _parse_float_list(
            rospy.get_param("~virtual_grasp_point", None),
            get_default_virtual_grasp_point(),
        )
        goal_joints = _parse_goal_joints(rospy.get_param("~goal_joints", None))
        if goal_joints is None:
            kc = KinematicsClient(use_motion_control=True)
            ik_joints = kc.ik(goal_xyz)
            if ik_joints is not None:
                goal_joints = list(ik_joints)
                rospy.loginfo("[DemoTargetRemoval] 由 goal_xyz 经 IK 得到 goal_joints: %s", [round(x, 3) for x in goal_joints])
            else:
                goal_joints = _parse_goal_joints(rospy.get_param("~goal_joints_default", None))
                if goal_joints is None:
                    goal_joints = [0.0, -2.0, 1.2, -1.5708, -1.5708, 0.0]
                rospy.logwarn("[DemoTargetRemoval] IK 失败，使用 goal_joints_default: %s", [round(x, 3) for x in goal_joints])
        rospy.loginfo("[DemoTargetRemoval] Goal xyz: %s, goal_joints: %s",
                      [round(x, 3) for x in goal_xyz], [round(x, 3) for x in goal_joints])
    else:
        rospy.loginfo("[DemoTargetRemoval] 使用 goal_pose_4x4 笛卡尔目标位姿")

    # 获取点云并挖空目标（方式 B：若提供 YOLO 实例点云与目标中心，则优先使用 new API）
    obstacles = None

    # 方式 B：基于 yolo26_seg_xyzl_instance_cloud_node 的实例点云 + 目标中心
    target_center = rospy.get_param("~target_center", None)
    if instance_cloud_topic and target_center is not None:
        try:
            if isinstance(target_center, (list, tuple)) and len(target_center) >= 3:
                target_center_xyz = [float(target_center[0]), float(target_center[1]), float(target_center[2])]
            else:
                import ast

                parsed = ast.literal_eval(str(target_center))
                target_center_xyz = [float(parsed[0]), float(parsed[1]), float(parsed[2])]
        except Exception:
            target_center_xyz = None

        if target_center_xyz is not None:
            seg_instance_cloud = [None]

            def _on_instance_cloud(msg):
                if seg_instance_cloud[0] is None:
                    seg_instance_cloud[0] = msg

            sub_inst = rospy.Subscriber(instance_cloud_topic, PointCloud2, _on_instance_cloud, queue_size=1)
            rate = rospy.Rate(10)
            t0 = rospy.Time.now()
            while not rospy.is_shutdown() and (rospy.Time.now() - t0).to_sec() < 8.0:
                if seg_instance_cloud[0] is not None:
                    break
                rate.sleep()
            sub_inst.unregister()

            if seg_instance_cloud[0] is not None:
                seg_instance_in_frame = _transform_pointcloud_to_frame(
                    seg_instance_cloud[0], frame_id, tf_buffer, timeout=1.0
                )
                if seg_instance_in_frame is not None:
                    target_and_env = build_obstacles_from_yolo_instance_cloud(
                        seg_instance_in_frame,
                        target_center_xyz,
                        items_list_path=None,
                        voxel_res=voxel_res,
                        bounds=bounds,
                        include_target_obstacle=False,
                        current_joints=start_joints,
                    )
                    if target_and_env is not None:
                        obstacles = target_and_env.get("obstacles") or []
                        cls_id = target_and_env.get("class_id")
                        center = target_and_env.get("center")
                        rospy.loginfo(
                            "[DemoTargetRemoval] YOLO 目标 class_id=%s, center=%s, 环境障碍物数量=%d",
                            str(cls_id),
                            ["{:.3f}".format(c) for c in center] if center is not None else "None",
                            len(obstacles),
                        )
                        if not obstacles:
                            obstacles = None

    # 方式 A：若未提供实例点云或方式 B 失败，回退到原有：基于类别 ID 从分割点云“挖空”目标
    if obstacles is None:
        raw_in_frame, seg_in_frame = _get_raw_and_seg_clouds(
            pointcloud_topic, seg_cloud_topic, tf_buffer, frame_id, timeout=8.0
        )
        if raw_in_frame is None or seg_in_frame is None:
            rospy.logerr("[DemoTargetRemoval] Cannot get point clouds, using default obstacles")
            obstacles = GAZEBO_DEFAULT_OBSTACLES
        else:
            from aco_rrtstar_planner_node import filter_pointcloud_robot_arm
            filtered_cloud = remove_target_region_from_pointcloud(
                raw_in_frame,
                seg_in_frame,
                target_class_id,
                padding=removal_padding,
                output_frame_id=frame_id,
            )
            cloud_no_robot = filter_pointcloud_robot_arm(
                filtered_cloud, start_joints, frame_id=frame_id
            )
            obstacles = pointcloud_to_obstacles(
                cloud_no_robot,
                voxel_res=voxel_res,
                frame_id=frame_id,
                bounds=bounds,
            )
            if not obstacles:
                obstacles = GAZEBO_DEFAULT_OBSTACLES
                rospy.logwarn("[DemoTargetRemoval] No obstacles after removal, using default")
            else:
                rospy.loginfo("[DemoTargetRemoval] Obstacles after target removal: %d", len(obstacles))

    # 一次性规划：若已设 goal_pose_4x4 则用 plan_one_shot_from_goal_pose，否则 plan_one_shot
    if use_goal_pose_4x4:
        result = plan_one_shot_from_goal_pose(
            goal_pose_4x4=goal_pose_4x4,
            start_xyz=start_xyz,
            start_joints=start_joints,
            obstacles=obstacles,
            bounds=bounds,
            frame_id=frame_id,
            return_vis_data=True,
            seed_joints_for_ik=start_joints,
        )
    else:
        result = plan_one_shot(
            start_xyz=start_xyz,
            start_joints=start_joints,
            goal_xyz=goal_xyz,
            goal_joints=goal_joints,
            obstacles=obstacles,
            return_vis_data=True,
        )
    if result is None or len(result) < 2:
        rospy.logerr("[DemoTargetRemoval] Planning failed")
        return
    path_joints, trajectory = result[0], result[1]
    vis_data = result[2] if len(result) > 2 else None

    if path_joints is None or trajectory is None:
        rospy.logerr("[DemoTargetRemoval] Planning failed")
        return

    # 目标位姿用 TF 发布（位置来自 vis_data.goal_xyz，与规划一致）
    target_pose_frame_id = rospy.get_param("~target_pose_frame_id", "one_shot_target_pose")
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    target_pose_xyz = list(vis_data["goal_xyz"]) if vis_data and "goal_xyz" in vis_data else [0.0, 0.0, 0.0]

    def _publish_target_pose_tf(_event=None):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = frame_id
        t.child_frame_id = target_pose_frame_id
        t.transform.translation.x = float(target_pose_xyz[0])
        t.transform.translation.y = float(target_pose_xyz[1])
        t.transform.translation.z = float(target_pose_xyz[2])
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        tf_broadcaster.sendTransform(t)

    _publish_target_pose_tf()
    rospy.Timer(rospy.Duration(0.1), _publish_target_pose_tf)

    # 可选：发布轨迹与状态 Marker；目标位姿仅用 TF 显示，不发 target 点
    try:
        from one_shot_trajectory_display import publish_plan_markers
        from visualization_msgs.msg import Marker
        if rospy.get_param("~publish_plan_markers", True):
            path_marker_pub = rospy.Publisher("~ee_path_marker", Marker, queue_size=1, latch=True)
            status_marker_pub = rospy.Publisher("~status_marker", Marker, queue_size=1, latch=True)
            rospy.sleep(0.2)
            publish_plan_markers(
                path_marker_pub,
                None,  # 目标用 TF 显示，不发布 target 点
                status_marker_pub,
                vis_data,
                success=True,
                frame_id=frame_id,
            )
            rospy.loginfo("[DemoTargetRemoval] Published trajectory markers and TF %s -> %s (target pose only in TF)", frame_id, target_pose_frame_id)
    except Exception as e:
        rospy.logdebug("[DemoTargetRemoval] Markers: %s", e)

    # 保存 3D 图
    if vis_data:
        viz_path = rospy.get_param("~planning_viz_3d_output", "/tmp/planning_result_3d_target_removal.png")
        plot_planning_result_3d(vis_data, output_path=viz_path)

    # 发布执行命令
    cmd_pub = rospy.Publisher("/motion/command", MotionCommand, queue_size=10)
    rospy.sleep(0.5)
    cmd = build_motion_command_execute_trajectory(trajectory)
    cmd_pub.publish(cmd)
    rospy.loginfo("[DemoTargetRemoval] Published EXECUTE_TRAJECTORY, %d points", len(trajectory.points))

    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
