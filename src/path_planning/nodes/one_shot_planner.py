#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
一次性规划模块：从当前起点规划到目标，避障，输出 JointTrajectory 供 motion_control 执行。
与 ur5e_teach_pendant_ui 一致：不直接调用 IK/FK，相信 motion_control；起点位姿由 TF 获取，
目标关节由配置/上层给出，仅通过 /motion/command 下发 MotionCommand。

用法:
  from one_shot_planner import plan_one_shot, joints_to_trajectory, get_default_virtual_grasp_point
  path_joints, traj = plan_one_shot(
      start_xyz=..., start_joints=...,   # 由 TF + /joint_states 获取
      goal_xyz=..., goal_joints=...,     # goal_joints 由配置提供，不在此做 IK
      ...
  )
"""

from __future__ import division

import sys
import os
import json
from math import sqrt

# 保证可导入同目录下的 aco_rrtstar_planner_node
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

import numpy as np
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from common_msgs.msg import MotionCommand

try:
    from path_planning.srv import ComputeIK
    _HAS_COMPUTE_IK = True
except ImportError:
    _HAS_COMPUTE_IK = False

# 从 aco_rrtstar_planner_node 复用（仅 ACO/RRT* 与障碍物，不依赖 KinematicsClient/IK/FK）
from aco_rrtstar_planner_node import (
    pointcloud_to_obstacles,
    ACOPhase,
    RRTStarPhase,
    GAZEBO_DEFAULT_OBSTACLES,
    DEFAULT_VIRTUAL_GRASP_POINT,
    DEFAULT_EE_HALF_EXTENTS,
    UR5eFK,
)

# 不传 ee_to_object_4x4 时使用的默认夹持物体包围盒半长（EE 系，米）
DEFAULT_GRASPED_OBJECT_HALF_EXTENTS = (0.02, 0.02, 0.03)

# items_list.json 默认路径（config 相对于 path_planning 包根目录）
_PKG_DIR = os.path.dirname(_SCRIPT_DIR)
DEFAULT_ITEMS_LIST_JSON = os.path.join(_PKG_DIR, "config", "items_list.json")


def _load_items_list_json(items_list_path=None):
    """
    读取 items_list.json，成功返回 dict，否则返回 None。
    单独封装，供按 class_name / class_id 等多种方式索引。
    """
    path = items_list_path or DEFAULT_ITEMS_LIST_JSON
    if not os.path.isfile(path):
        if rospy.get_logger().getEffectiveLevel() <= rospy.logging.DEBUG:
            rospy.logdebug("[OneShot] items_list 不存在: %s", path)
        return None
    try:
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception as e:
        rospy.logwarn_throttle(5, "[OneShot] 读取 items_list.json 失败: %s", str(e))
        return None


def _get_half_extents_by_class_name(class_name, items_list_path=None):
    """
    从 items_list.json 中按 class_name 查找模型，返回 half_extents (hx, hy, hz)。
    若未找到或文件不存在，返回 None。
    """
    data = _load_items_list_json(items_list_path)
    if data is None:
        return None
    models = data.get("models") if isinstance(data, dict) else None
    if not models:
        return None
    for m in models:
        if not isinstance(m, dict):
            continue
        if m.get("class_name") == class_name and "half_extents" in m:
            he = m["half_extents"]
            if isinstance(he, (list, tuple)) and len(he) >= 3:
                return tuple(float(x) for x in he[:3])
            break
    return None


def _get_half_extents_by_class_id(class_id, items_list_path=None):
    """
    从 items_list.json 中按 class_id 查找模型，返回 half_extents (hx, hy, hz)。
    若未找到或文件不存在，返回 None。
    """
    data = _load_items_list_json(items_list_path)
    if data is None:
        return None
    models = data.get("models") if isinstance(data, dict) else None
    if not models:
        return None
    for m in models:
        if not isinstance(m, dict):
            continue
        if m.get("class_id") == class_id and "half_extents" in m:
            he = m["half_extents"]
            if isinstance(he, (list, tuple)) and len(he) >= 3:
                return tuple(float(x) for x in he[:3])
            break
    return None


def build_obstacles_from_yolo_instance_cloud(
    instance_cloud,
    target_center_xyz,
    items_list_path=None,
    voxel_res=0.05,
    bounds=None,
    instance_label_field="label",
    class_id_field="class_id",
    center_match_radius=0.10,
    include_target_obstacle=True,
    current_joints=None,
):
    """
    从 YOLO 分割实例点云构造一次性规划可用的障碍物列表。

    假定点云来自 yolo26_seg_xyzl_instance_cloud_node，后续可能扩展字段：
      - x, y, z: 点在规划坐标系（如 base_link）下的位置
      - label:   uint32，实例 ID（同一物体所有点一致），字段名默认 "label"
      - class_id: int32，可选，物体类别 id（同一物体所有点一致），字段名默认 "class_id"

    逻辑：
      1) 根据输入的目标中心点 target_center_xyz，在所有 label 中找到“最近”的实例 ID；
      2) 读取该实例的 class_id（若有），在 items_list.json 中查找 half_extents；
         - 命中：使用配置的 half_extents 作为该物体的包围盒半轴；
         - 未命中或无 class_id：使用 DEFAULT_GRASPED_OBJECT_HALF_EXTENTS 兜底；
      3) 将该物体构造成一个 AABB 障碍物 ((cx, cy, cz), (hx, hy, hz))，按需加入返回列表；
      4) 从全局点云中过滤掉该 label 对应的所有点（若 include_target_obstacle=True 则保留其几何信息），其余点构成“环境点云”，
         使用 pointcloud_to_obstacles 体素化为普通障碍物并追加到返回列表。

    Args:
        instance_cloud: sensor_msgs/PointCloud2，需包含至少 x,y,z,label。
        target_center_xyz: [x,y,z]，目标物体中心在同一坐标系下的位置。
        items_list_path: 可选，items_list.json 的路径；None 时使用默认路径。
        voxel_res: 体素大小（米），用于环境点云 -> 障碍物体素化。
        bounds: 工作空间边界 ((xmin,xmax),(ymin,ymax),(zmin,zmax))；None 时使用 ACO/RRT* 默认值。
        instance_label_field: 实例 ID 字段名，默认 "label"。
        class_id_field: 类别 ID 字段名，默认 "class_id"；不存在时自动忽略。
        center_match_radius: 若所有实例到目标中心的最近距离都大于该值（米），视为匹配失败。
        include_target_obstacle: 若为 True，则把目标物体自身 AABB 也视作障碍物加入列表；
                                 若为 False，则仅返回环境障碍物（适合“目标是吸引点、而非障碍”的抓取场景）。
        current_joints: 可选，长度 6 的当前关节角（弧度）；若提供，体素化前会从环境点云中排除落在机械臂 AABB 内的点。

    Returns:
        dict，字段包括：
            - 'class_id':  目标物体的类别 id（若存在，否则为 None）
            - 'center':    目标物体中心点 (cx, cy, cz)（即 target_center_xyz）
            - 'half_extents':  (hx, hy, hz) 目标物体的半轴（items_list 查表或默认值）
            - 'obstacles': list[((cx,cy,cz),(hx,hy,hz))]，仅包含环境障碍物（不含目标物体自身）
            - 'env_cloud': sensor_msgs/PointCloud2，环境点云（仅 x,y,z），若无则为 None

        匹配失败或输入异常时返回 None。
    """
    if instance_cloud is None or not isinstance(instance_cloud, PointCloud2):
        rospy.logerr("[OneShot] build_obstacles_from_yolo_instance_cloud: instance_cloud 无效")
        return None

    try:
        tc = [float(target_center_xyz[0]), float(target_center_xyz[1]), float(target_center_xyz[2])]
    except Exception:
        rospy.logerr("[OneShot] build_obstacles_from_yolo_instance_cloud: target_center_xyz 无效: %r", target_center_xyz)
        return None

    field_names = [f.name for f in instance_cloud.fields]
    has_label = instance_label_field in field_names
    has_class_id = class_id_field in field_names
    if not has_label:
        rospy.logerr("[OneShot] YOLO 实例点云中找不到 label 字段(%s)，无法按实例聚合", instance_label_field)
        return None

    # 1) 按 label 聚合点，并统计每个 label 到目标中心的最小距离 & class_id
    label_to_points = {}
    label_to_min_dist2 = {}
    label_to_class_id = {}

    read_fields = ["x", "y", "z", instance_label_field]
    if has_class_id and class_id_field not in read_fields:
        read_fields.append(class_id_field)

    for p in pc2.read_points(instance_cloud, skip_nans=True, field_names=read_fields):
        x, y, z = float(p[0]), float(p[1]), float(p[2])
        label_val = int(p[3])
        label_to_points.setdefault(label_val, []).append((x, y, z))
        dx, dy, dz = x - tc[0], y - tc[1], z - tc[2]
        d2 = dx * dx + dy * dy + dz * dz
        prev = label_to_min_dist2.get(label_val)
        label_to_min_dist2[label_val] = d2 if prev is None or d2 < prev else prev
        if has_class_id and len(p) >= 5:
            try:
                label_to_class_id[label_val] = int(p[4])
            except Exception:
                pass

    if not label_to_points:
        rospy.logwarn("[OneShot] YOLO 实例点云为空，无法构建障碍物")
        return []

    # 选择到目标中心最近的实例 ID
    best_label = None
    best_dist2 = None
    for lbl, d2 in label_to_min_dist2.items():
        if best_dist2 is None or d2 < best_dist2:
            best_label = lbl
            best_dist2 = d2

    if best_label is None:
        rospy.logwarn("[OneShot] 未能从实例点云中匹配目标物体")
        return []

    if best_dist2 is None or best_dist2 > center_match_radius * center_match_radius:
        rospy.logwarn(
            "[OneShot] 最近实例与目标中心距离为 %.3f m，大于阈值 %.3f，放弃匹配",
            sqrt(best_dist2) if best_dist2 is not None else -1.0,
            center_match_radius,
        )
        return []

    # 2) 目标物体包围盒：中心使用传入的 target_center_xyz，半轴根据 class_id / items_list 确定
    class_id = label_to_class_id.get(best_label) if has_class_id else None
    half_extents = None
    if class_id is not None:
        half_extents = _get_half_extents_by_class_id(class_id, items_list_path)
        if half_extents is None:
            class_name = f"class_{class_id}"
            half_extents = _get_half_extents_by_class_name(class_name, items_list_path)
    if half_extents is None:
        half_extents = DEFAULT_GRASPED_OBJECT_HALF_EXTENTS
        if class_id is not None:
            rospy.logwarn_throttle(
                5,
                "[OneShot] class_id=%s 在 items_list 中未找到 half_extents，使用默认 %s",
                class_id,
                half_extents,
            )

    obstacles = []

    # 3) 环境障碍物：将除目标实例以外的所有点体素化；若提供 current_joints 则先排除机械臂内的点
    other_points = []
    for lbl, pts in label_to_points.items():
        if lbl == best_label:
            continue
        other_points.extend(pts)

    if other_points and current_joints is not None and len(current_joints) >= 6:
        from aco_rrtstar_planner_node import filter_points_by_robot_aabbs
        other_points = filter_points_by_robot_aabbs(other_points, current_joints)

    env_cloud = None
    if other_points:
        header = instance_cloud.header
        # pointcloud_to_obstacles 只需要 x,y,z，因此这里仅创建 XYZ 点云
        env_cloud = pc2.create_cloud_xyz32(header, other_points)
        from aco_rrtstar_planner_node import pointcloud_to_obstacles

        env_obstacles = pointcloud_to_obstacles(
            env_cloud,
            voxel_res=voxel_res,
            frame_id=header.frame_id or "base_link",
            bounds=bounds,
        )
        obstacles.extend(env_obstacles)

    result = {
        "class_id": class_id,
        "center": (tc[0], tc[1], tc[2]),
        "half_extents": tuple(float(x) for x in half_extents[:3]),
        "obstacles": obstacles,
        "env_cloud": env_cloud,
    }
    return result

# 默认虚拟抓取点：与 planning_config.yaml 一致，Gazebo 箱体后方
def get_default_virtual_grasp_point():
    """返回默认虚拟抓取点 [x, y, z]（base_link 系），与既有配置一致。"""
    return list(DEFAULT_VIRTUAL_GRASP_POINT)


def _to_bounds(workspace_x, workspace_y, workspace_z):
    """(wx, wy, wz) -> bounds 三元组"""
    def to_pair(val, default):
        if isinstance(val, (list, tuple)) and len(val) >= 2:
            return (float(val[0]), float(val[1]))
        return default
    wx = to_pair(workspace_x, (-0.5, 1.0))
    wy = to_pair(workspace_y, (-0.5, 0.5))
    wz = to_pair(workspace_z, (0.0, 0.8))
    return (wx, wy, wz)


def plan_one_shot(
    start_xyz,
    start_joints,
    goal_xyz,
    goal_joints,
    obstacles=None,
    bounds=None,
    frame_id="base_link",
    aco_grid_res=0.08,
    aco_n_ants=40,
    aco_n_iters=30,
    aco_greedy_prob=0.3,
    aco_elite_deposit_ratio=2.0,
    rrt_step_size=0.12,
    rrt_max_iter=4000,
    return_vis_data=False,
    ee_half_extents=None,
):
    """
    一次性规划：从 start 到 goal，避障，返回关节路径与轨迹消息。
    与示教器一致：不在此调用 IK/FK，相信 motion_control；start_xyz 由 TF 提供，goal_joints 由配置/上层提供。

    Args:
        start_xyz: [x, y, z] 起点笛卡尔位置（base_link），由 TF base_link->tcp 获取，同 ur5e_teach_pendant_ui。
        start_joints: 起点关节角 (6,)；由 /joint_states 获取。
        goal_xyz: [x, y, z] 目标笛卡尔位置（用于 ACO 路径），一般即 virtual_grasp_point。
        goal_joints: 目标关节角 (6,)；由配置或上层提供，不在此做 IK。
        obstacles: 障碍物列表；None 时使用 GAZEBO_DEFAULT_OBSTACLES。
        bounds: 工作空间 (wx, wy, wz)；None 时从 rospy 参数或默认值取。
        ee_half_extents: 末端执行器在 EE 系下包围盒半长 (hx, hy, hz)，用于路径上姿态/几何碰撞检测；None 时从 ~end_effector_collision_box 读取，默认 (0.02, 0.02, 0.05)；设为 (0,0,0) 可禁用末端盒检测。

    Returns:
        (path_joints, trajectory_msg): path_joints 为 list of 6-tuple 或 None；trajectory_msg 为 trajectory_msgs/JointTrajectory 或 None。
    """
    try:
        start_xyz = list(start_xyz)[:3]
        start_joints = list(start_joints)[:6]
        goal_xyz = list(goal_xyz)[:3]
        goal_joints = list(goal_joints)[:6]
        if len(start_joints) != 6 or len(goal_joints) != 6:
            rospy.logerr("[OneShot] start_joints / goal_joints 需各 6 个关节角")
            return None, None

        if obstacles is None:
            obstacles = list(GAZEBO_DEFAULT_OBSTACLES)
        else:
            obstacles = list(obstacles)
        ground = ((0.0, 0.0, -0.06), (1.5, 1.5, 0.01))
        if ground not in obstacles:
            obstacles.append(ground)

        if bounds is None:
            wx = rospy.get_param("~workspace_x", [-0.5, 1.0])
            wy = rospy.get_param("~workspace_y", [-0.5, 0.5])
            wz = rospy.get_param("~workspace_z", [0.0, 0.8])
            bounds = _to_bounds(wx, wy, wz)

        if ee_half_extents is None:
            raw = rospy.get_param("~end_effector_collision_box", list(DEFAULT_EE_HALF_EXTENTS))
            ee_half_extents = tuple(float(x) for x in raw[:3]) if isinstance(raw, (list, tuple)) and len(raw) >= 3 else DEFAULT_EE_HALF_EXTENTS

        start = tuple(start_joints)
        goal = tuple(goal_joints)
        start_xyz = tuple(float(x) for x in start_xyz)
        goal_xyz = tuple(float(x) for x in goal_xyz)

        aco = ACOPhase(grid_res=aco_grid_res, bounds=bounds)
        aco.mark_obstacles(obstacles)
        aco.run(start_xyz, goal_xyz, n_ants=aco_n_ants, n_iters=aco_n_iters,
                greedy_prob=aco_greedy_prob, elite_deposit_ratio=aco_elite_deposit_ratio)

        pheromone_fn = lambda x, y, z: aco.get_pheromone(x, y, z)
        rrt = RRTStarPhase(obstacles, pheromone_fn, step_size=rrt_step_size, max_iter=rrt_max_iter,
                           ee_half_extents=ee_half_extents)
        if return_vis_data:
            path, rrt_nodes = rrt.plan(start, goal, return_tree=True)
        else:
            path = rrt.plan(start, goal)

        if path is None or len(path) == 0:
            rospy.logwarn("[OneShot] RRT* 未找到路径")
            if return_vis_data:
                aco_path_xyz = aco.get_path_xyz()
                aco_all_paths_xyz = aco.get_all_paths_xyz()
                tree_edges = [(rrt_nodes[q]['parent'], q) for q in rrt_nodes if rrt_nodes[q].get('parent') is not None]
                rrt_tree_edges_xyz = [(tuple(UR5eFK.fk(p)), tuple(UR5eFK.fk(q))) for (p, q) in tree_edges]
                vis_data = {
                    "aco_path_xyz": aco_path_xyz,
                    "aco_all_paths_xyz": aco_all_paths_xyz,
                    "rrt_path_xyz": [],
                    "rrt_tree_edges_xyz": rrt_tree_edges_xyz,
                    "obstacles": obstacles,
                    "start_xyz": start_xyz,
                    "goal_xyz": goal_xyz,
                }
                return None, None, vis_data
            return None, None

        rospy.loginfo("[OneShot] 规划完成，路径点数: %d", len(path))
        joint_names = list(RRTStarPhase.JOINT_NAMES)
        traj = joints_to_trajectory(path, joint_names, time_step=0.5)

        if return_vis_data:
            aco_path_xyz = aco.get_path_xyz()
            aco_all_paths_xyz = aco.get_all_paths_xyz()
            rrt_path_xyz = [tuple(UR5eFK.fk(q)) for q in path]
            tree_edges = [(rrt_nodes[q]['parent'], q) for q in rrt_nodes if rrt_nodes[q].get('parent') is not None]
            rrt_tree_edges_xyz = [(tuple(UR5eFK.fk(p)), tuple(UR5eFK.fk(q))) for (p, q) in tree_edges]
            vis_data = {
                "aco_path_xyz": aco_path_xyz,
                "aco_all_paths_xyz": aco_all_paths_xyz,
                "rrt_path_xyz": rrt_path_xyz,
                "rrt_tree_edges_xyz": rrt_tree_edges_xyz,
                "obstacles": obstacles,
                "start_xyz": start_xyz,
                "goal_xyz": goal_xyz,
            }
            return path, traj, vis_data
        return path, traj

    except Exception as e:
        rospy.logerr("[OneShot] 规划异常: %s", str(e))
        import traceback
        traceback.print_exc()
        if return_vis_data:
            return None, None, None
        return None, None


def _rotation_matrix_to_quaternion_xyzw(R):
    """3x3 旋转矩阵 -> ROS 四元数 (x, y, z, w)。"""
    R = np.asarray(R, dtype=float).reshape(3, 3)
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    n = sqrt(x*x + y*y + z*z + w*w)
    if n > 1e-10:
        x, y, z, w = x/n, y/n, z/n, w/n
    return (float(x), float(y), float(z), float(w))


def _matrix_4x4_to_pose_stamped(T, frame_id="base_link"):
    """4x4 齐次变换矩阵 -> geometry_msgs/PoseStamped（base 系下目标末端位姿）。"""
    T = np.asarray(T, dtype=float)
    if T.shape != (4, 4):
        T = np.asarray(T, dtype=float).reshape(4, 4)
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = float(T[0, 3])
    pose.pose.position.y = float(T[1, 3])
    pose.pose.position.z = float(T[2, 3])
    qx, qy, qz, qw = _rotation_matrix_to_quaternion_xyzw(T[0:3, 0:3])
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


def _inv_4x4(T):
    """4x4 齐次变换矩阵的逆（刚体变换）。"""
    T = np.asarray(T, dtype=float).reshape(4, 4)
    R = T[0:3, 0:3]
    t = T[0:3, 3]
    inv_R = R.T
    inv_t = -inv_R.dot(t)
    out = np.eye(4)
    out[0:3, 0:3] = inv_R
    out[0:3, 3] = inv_t
    return out


def _merge_ee_object_half_extents(ee_half_extents, ee_to_object_4x4, grasped_object_half_extents):
    """
    合并末端包围盒与夹持物体包围盒（均在 EE 系下），得到仍以 EE 原点为中心、能包住两者的半长 (hx, hy, hz)。
    ee_half_extents: (hx, hy, hz) 末端盒半长；
    ee_to_object_4x4: 末端系到物体系的变换，物体中心在 EE 系下为 T[0:3, 3]；
    grasped_object_half_extents: (ox, oy, oz) 物体在 EE 系下的包围盒半长（沿 EE 轴）。
    """
    ee_hx, ee_hy, ee_hz = ee_half_extents[0], ee_half_extents[1], ee_half_extents[2]
    T = np.asarray(ee_to_object_4x4, dtype=float).reshape(4, 4)
    tx, ty, tz = float(T[0, 3]), float(T[1, 3]), float(T[2, 3])
    ox, oy, oz = grasped_object_half_extents[0], grasped_object_half_extents[1], grasped_object_half_extents[2]
    chx = max(ee_hx, abs(tx) + ox)
    chy = max(ee_hy, abs(ty) + oy)
    chz = max(ee_hz, abs(tz) + oz)
    return (chx, chy, chz)


def _compute_ik_for_pose(pose_stamped, seed_joints=None, timeout=2.0):
    """调用 /motion/compute_ik 由目标位姿求关节角，失败返回 None。"""
    if not _HAS_COMPUTE_IK:
        rospy.logwarn("[OneShot] 未找到 ComputeIK 服务定义，无法从 4x4 目标位姿求 IK")
        return None
    try:
        rospy.wait_for_service("/motion/compute_ik", timeout=timeout)
        ik = rospy.ServiceProxy("/motion/compute_ik", ComputeIK)
        req = ComputeIK.Request()
        req.target_pose = pose_stamped
        if seed_joints is not None and len(seed_joints) >= 6:
            from sensor_msgs.msg import JointState
            req.seed_state = JointState()
            req.seed_state.name = list(RRTStarPhase.JOINT_NAMES)
            req.seed_state.position = list(seed_joints)[:6]
        resp = ik(req)
        if resp.success and getattr(resp.solution, "position", None) and len(resp.solution.position) >= 6:
            return list(resp.solution.position[:6])
    except (rospy.ROSException, rospy.ROSInterruptException, Exception) as e:
        rospy.logwarn_throttle(5, "[OneShot] IK 服务调用失败: %s", str(e))
    return None


def plan_one_shot_from_goal_pose(
    goal_pose_4x4,
    start_xyz,
    start_joints,
    obstacles=None,
    bounds=None,
    frame_id="base_link",
    aco_grid_res=0.08,
    aco_n_ants=40,
    aco_n_iters=30,
    aco_greedy_prob=0.3,
    aco_elite_deposit_ratio=2.0,
    rrt_step_size=0.12,
    rrt_max_iter=4000,
    return_vis_data=False,
    ee_half_extents=None,
    seed_joints_for_ik=None,
    ik_timeout=2.0,
):
    """
    以「目标 4x4 笛卡尔矩阵（含目标点 + 旋转矩阵）」为目标的封装：先由 IK 求得 goal_joints，再调用 plan_one_shot。

    Args:
        goal_pose_4x4: 4x4 齐次变换矩阵（numpy 或 list），base 系下目标末端位姿。
        start_xyz: [x, y, z] 起点笛卡尔位置（base_link）。
        start_joints: 起点关节角 (6,)；由 /joint_states 获取。
        obstacles: 障碍物列表；None 时使用 GAZEBO_DEFAULT_OBSTACLES。
        bounds / frame_id / aco_* / rrt_* / return_vis_data / ee_half_extents: 同 plan_one_shot。
        seed_joints_for_ik: 可选，IK 求解的种子关节角；None 时用 start_joints。
        ik_timeout: 等待 /motion/compute_ik 的超时（秒）。

    Returns:
        与 plan_one_shot 相同：(path_joints, trajectory) 或 (path_joints, trajectory, vis_data)。
    """
    T = np.asarray(goal_pose_4x4, dtype=float)
    if T.shape != (4, 4):
        rospy.logerr("[OneShot] goal_pose_4x4 须为 4x4 矩阵")
        if return_vis_data:
            return None, None, None
        return None, None
    goal_xyz = [float(T[0, 3]), float(T[1, 3]), float(T[2, 3])]
    pose_stamped = _matrix_4x4_to_pose_stamped(T, frame_id=frame_id)
    seed = seed_joints_for_ik if seed_joints_for_ik is not None else start_joints
    goal_joints = _compute_ik_for_pose(pose_stamped, seed_joints=seed, timeout=ik_timeout)
    if goal_joints is None:
        rospy.logerr("[OneShot] 无法从目标 4x4 位姿求得 IK，规划取消")
        if return_vis_data:
            return None, None, None
        return None, None
    return plan_one_shot(
        start_xyz=start_xyz,
        start_joints=start_joints,
        goal_xyz=goal_xyz,
        goal_joints=goal_joints,
        obstacles=obstacles,
        bounds=bounds,
        frame_id=frame_id,
        aco_grid_res=aco_grid_res,
        aco_n_ants=aco_n_ants,
        aco_n_iters=aco_n_iters,
        aco_greedy_prob=aco_greedy_prob,
        aco_elite_deposit_ratio=aco_elite_deposit_ratio,
        rrt_step_size=rrt_step_size,
        rrt_max_iter=rrt_max_iter,
        return_vis_data=return_vis_data,
        ee_half_extents=ee_half_extents,
    )


def plan_one_shot_grasped_object_to_goal(
    object_goal_4x4,
    start_xyz,
    start_joints,
    ee_to_object_4x4=None,
    class_name=None,
    obstacles=None,
    bounds=None,
    frame_id="base_link",
    aco_grid_res=0.08,
    aco_n_ants=40,
    aco_n_iters=30,
    aco_greedy_prob=0.3,
    aco_elite_deposit_ratio=2.0,
    rrt_step_size=0.12,
    rrt_max_iter=4000,
    return_vis_data=False,
    ee_half_extents=None,
    grasped_object_half_extents=None,
    items_list_path=None,
    seed_joints_for_ik=None,
    ik_timeout=2.0,
):
    """
    夹住物品后规划路径，使夹持的物体到达指定位姿。可选：由「物体目标位姿」与「EE 到物体的变换」反推末端目标；
    不传 ee_to_object_4x4 时直接将 object_goal_4x4 当作末端目标，不做换算。

    Args:
        object_goal_4x4: 4x4 齐次变换（base 系）。若 ee_to_object_4x4 为 None，则直接作为末端目标位姿；否则为夹持物体要到达的目标位姿。
        start_xyz: [x, y, z] 起点笛卡尔位置（base_link）。
        start_joints: 起点关节角 (6,)。
        ee_to_object_4x4: 可选。4x4 齐次变换（末端系→物体系），夹持时物体在末端系下的位姿；物体中心在 EE 系下为 T[0:3, 3]。为 None 时不换算，object_goal_4x4 即末端目标。
        class_name: 可选。物品类别名（与 items_list.json 中 class_name 一致，如 "class_0"）。传入时从 config/items_list.json 索引该物体的 half_extents，不再依赖 grasped_object_half_extents。
        obstacles / bounds / frame_id / aco_* / rrt_* / return_vis_data: 同 plan_one_shot_from_goal_pose。
        ee_half_extents: 末端包围盒半长 (hx, hy, hz)；None 时从 ~end_effector_collision_box 读取。
        grasped_object_half_extents: 夹持物体在 EE 系下的包围盒半长 (ox, oy, oz)。当未传 class_name 时使用；传了 class_name 时由 items_list 覆盖，此参数可省略。
        items_list_path: 可选。items_list.json 路径；None 时使用 path_planning/config/items_list.json。
        seed_joints_for_ik: IK 种子关节角；None 时用 start_joints。
        ik_timeout: 等待 /motion/compute_ik 的超时（秒）。

    Returns:
        与 plan_one_shot 相同：(path_joints, trajectory) 或 (path_joints, trajectory, vis_data)。
    """
    object_goal_4x4 = np.asarray(object_goal_4x4, dtype=float)
    if object_goal_4x4.shape != (4, 4):
        object_goal_4x4 = object_goal_4x4.reshape(4, 4)

    if ee_to_object_4x4 is None:
        ee_goal_4x4 = object_goal_4x4
    else:
        ee_to_object_4x4 = np.asarray(ee_to_object_4x4, dtype=float)
        if ee_to_object_4x4.shape != (4, 4):
            ee_to_object_4x4 = ee_to_object_4x4.reshape(4, 4)
        T_ee_object_inv = _inv_4x4(ee_to_object_4x4)
        ee_goal_4x4 = object_goal_4x4.dot(T_ee_object_inv)

    if ee_half_extents is None:
        raw = rospy.get_param("~end_effector_collision_box", list(DEFAULT_EE_HALF_EXTENTS))
        ee_half_extents = tuple(float(x) for x in raw[:3]) if isinstance(raw, (list, tuple)) and len(raw) >= 3 else DEFAULT_EE_HALF_EXTENTS
    else:
        ee_half_extents = tuple(float(x) for x in ee_half_extents[:3])

    # 夹持物体半轴：优先用 class_name 从 items_list.json 索引，否则用传入的 grasped_object_half_extents 或默认值
    if class_name is not None:
        looked_up = _get_half_extents_by_class_name(class_name, items_list_path)
        if looked_up is not None:
            grasped_object_half_extents = looked_up
        elif grasped_object_half_extents is None:
            grasped_object_half_extents = DEFAULT_GRASPED_OBJECT_HALF_EXTENTS
            rospy.logwarn_throttle(5, "[OneShot] class_name=%s 在 items_list 中未找到 half_extents，使用默认", class_name)
        # else: class_name 未命中但调用方传了 grasped_object_half_extents，保留 grasped_object_half_extents
    if grasped_object_half_extents is not None and not any(x > 0 for x in grasped_object_half_extents):
        grasped_object_half_extents = None

    if ee_to_object_4x4 is not None and grasped_object_half_extents is not None:
        grasped_object_half_extents = tuple(float(x) for x in grasped_object_half_extents[:3])
        ee_half_extents = _merge_ee_object_half_extents(ee_half_extents, ee_to_object_4x4, grasped_object_half_extents)
    elif ee_to_object_4x4 is None:
        obj_half = DEFAULT_GRASPED_OBJECT_HALF_EXTENTS
        if grasped_object_half_extents is not None:
            obj_half = tuple(float(x) for x in grasped_object_half_extents[:3])
        ee_half_extents = _merge_ee_object_half_extents(ee_half_extents, np.eye(4), obj_half)

    return plan_one_shot_from_goal_pose(
        goal_pose_4x4=ee_goal_4x4,
        start_xyz=start_xyz,
        start_joints=start_joints,
        obstacles=obstacles,
        bounds=bounds,
        frame_id=frame_id,
        aco_grid_res=aco_grid_res,
        aco_n_ants=aco_n_ants,
        aco_n_iters=aco_n_iters,
        aco_greedy_prob=aco_greedy_prob,
        aco_elite_deposit_ratio=aco_elite_deposit_ratio,
        rrt_step_size=rrt_step_size,
        rrt_max_iter=rrt_max_iter,
        return_vis_data=return_vis_data,
        ee_half_extents=ee_half_extents,
        seed_joints_for_ik=seed_joints_for_ik,
        ik_timeout=ik_timeout,
    )


def joints_to_trajectory(path_joints, joint_names, time_step=0.5):
    """
    将关节路径转为 trajectory_msgs/JointTrajectory，各点间隔 time_step 秒。

    Args:
        path_joints: list of 6-tuple 或 list of list。
        joint_names: 关节名列表，与 motion_control 一致。
        time_step: 相邻点时间间隔（秒）。

    Returns:
        trajectory_msgs/JointTrajectory
    """
    traj = JointTrajectory()
    traj.joint_names = list(joint_names)
    traj.points = []
    t = 0.0
    from rospy import Duration
    for q in path_joints:
        pt = JointTrajectoryPoint()
        pt.positions = list(q)[:6]
        pt.time_from_start = Duration.from_sec(t)
        traj.points.append(pt)
        t += time_step
    return traj


def build_motion_command_execute_trajectory(trajectory):
    """构造 MotionCommand(EXECUTE_TRAJECTORY) 消息。"""
    cmd = MotionCommand()
    cmd.command_type = MotionCommand.EXECUTE_TRAJECTORY
    cmd.trajectory = trajectory
    cmd.max_velocity = 1.0
    cmd.max_acceleration = 1.0
    return cmd


def plot_planning_result_3d(vis_data, output_path=None):
    """
    根据 ACO 路径、RRT* 路径、最终路径与点云障碍物绘制 3D 坐标图并保存。

    vis_data: dict，需包含 aco_path_xyz, rrt_path_xyz, final_path_xyz, obstacles, start_xyz, goal_xyz。
    output_path: 保存路径，默认 /tmp/planning_result_3d.png
    """
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection
        # 使用英文标签，避免 Docker/无中文字体环境下的 Glyph missing 警告；若需中文可安装中文字体并在此指定
        plt.rcParams["axes.unicode_minus"] = False
    except ImportError:
        rospy.logwarn("[OneShot] 3D 绘图需要 matplotlib，跳过")
        return False

    if output_path is None:
        output_path = "/tmp/planning_result_3d.png"

    obstacles = vis_data.get("obstacles", [])
    aco_path_xyz = vis_data.get("aco_path_xyz", [])
    aco_all_paths_xyz = vis_data.get("aco_all_paths_xyz", [])
    rrt_path_xyz = vis_data.get("rrt_path_xyz", [])
    rrt_tree_edges_xyz = vis_data.get("rrt_tree_edges_xyz", [])
    start_xyz = vis_data.get("start_xyz")
    goal_xyz = vis_data.get("goal_xyz")

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    # RRT* 树：所有节点/边转笛卡尔后画枝杈（细线、半透明）
    tree_drawn = False
    for (p1, p2) in rrt_tree_edges_xyz:
        ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], "c-", linewidth=0.4, alpha=0.35)
        tree_drawn = True
    if tree_drawn:
        ax.plot([], [], [], "c-", linewidth=0.8, alpha=0.5, label="RRT* tree")

    # ACO 多条候选路径（细绿线、半透明）
    for path_xyz in aco_all_paths_xyz:
        if path_xyz and len(path_xyz) >= 2:
            xs, ys, zs = zip(*path_xyz)
            ax.plot(xs, ys, zs, "g-", linewidth=0.5, alpha=0.25)
    if aco_all_paths_xyz:
        ax.plot([], [], [], "g-", linewidth=1, alpha=0.5, label="ACO candidates")

    # 障碍物：每个 (center, half_extents) 画一个半透明立方体框
    for (cx, cy, cz), (hx, hy, hz) in obstacles:
        x = [cx - hx, cx + hx]
        y = [cy - hy, cy + hy]
        z = [cz - hz, cz + hz]
        verts = [
            [x[0], y[0], z[0]], [x[1], y[0], z[0]], [x[1], y[1], z[0]], [x[0], y[1], z[0]],
            [x[0], y[0], z[1]], [x[1], y[0], z[1]], [x[1], y[1], z[1]], [x[0], y[1], z[1]],
        ]
        faces = [
            [verts[0], verts[1], verts[2], verts[3]],
            [verts[4], verts[5], verts[6], verts[7]],
            [verts[0], verts[1], verts[5], verts[4]],
            [verts[2], verts[3], verts[7], verts[6]],
            [verts[0], verts[3], verts[7], verts[4]],
            [verts[1], verts[2], verts[6], verts[5]],
        ]
        ax.add_collection3d(Poly3DCollection(faces, facecolor="red", alpha=0.15, edgecolor="darkred", linewidths=0.5))

    # ACO 最优路径（粗绿线）
    if aco_path_xyz and len(aco_path_xyz) >= 2:
        xs, ys, zs = zip(*aco_path_xyz)
        ax.plot(xs, ys, zs, "g-", linewidth=2, alpha=0.95, label="ACO best")

    # RRT* 路径（即最终执行的关节路径在笛卡尔空间的投影）
    if rrt_path_xyz and len(rrt_path_xyz) >= 2:
        xs, ys, zs = zip(*rrt_path_xyz)
        ax.plot(xs, ys, zs, "b-", linewidth=2.5, alpha=0.9, label="RRT* / final path")

    # 起点、终点
    if start_xyz and len(start_xyz) >= 3:
        ax.scatter([start_xyz[0]], [start_xyz[1]], [start_xyz[2]], c="k", s=80, marker="^", label="Start")
    if goal_xyz and len(goal_xyz) >= 3:
        ax.scatter([goal_xyz[0]], [goal_xyz[1]], [goal_xyz[2]], c="orange", s=80, marker="o", label="Goal")

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.legend(loc="upper left", fontsize=8)
    ax.set_title("Planning 3D: obstacles / ACO & RRT* / final path")

    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
    fig.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close()
    rospy.loginfo("[OneShot] 3D 规划图已保存: %s", output_path)
    return True
