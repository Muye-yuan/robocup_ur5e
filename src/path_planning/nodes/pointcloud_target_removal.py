#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
方式 B 点云分割：基于带标签的 YOLO 分割点云，在原始点云中挖空“目标物体”区域。

输入：
  - 原始无标签点云 (sensor_msgs/PointCloud2)，如 /camera/depth/points
  - 带 label 的分割点云 (sensor_msgs/PointCloud2)，如 /perception/yolo26_seg_cloud
    格式：fields 含 x, y, z, label；label 为 uint32 的类别 id（与 YOLO cls_id 一致）
  - 目标类别 id (int)，即要在点云中挖掉的那一类

处理：
  - 从 seg_cloud 中取 label == target_label_id 的所有点，求 3D AABB（可加 padding）
  - 在 raw_cloud 中剔除落在该 AABB 内的点，得到“挖空目标后的点云”

本模块为库，不单独运行；供 demo_one_shot_with_target_removal 等节点调用。
"""

from __future__ import division

import copy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2


def get_target_aabb_from_seg_cloud(seg_cloud_msg, target_label_id, padding=0.02):
    """
    从带 label 的分割点云中，取目标类别所有点的 3D AABB（带 padding）。

    Args:
        seg_cloud_msg: sensor_msgs/PointCloud2，需含 field "label"（或 "l"）
        target_label_id: int，YOLO 类别 id
        padding: float，AABB 各方向扩展量（米）

    Returns:
        ((cx, cy, cz), (hx, hy, hz)) 即 (center, half_extents)，或 None（无该类别点）
    """
    try:
        field_names = ["x", "y", "z"]
        has_label = any(
            f.name in ("label", "l") for f in seg_cloud_msg.fields
        )
        if has_label:
            field_names.append("label" if any(f.name == "label" for f in seg_cloud_msg.fields) else "l")
    except Exception:
        return None

    pts = []
    try:
        for p in pc2.read_points(seg_cloud_msg, skip_nans=True, field_names=field_names):
            if len(p) >= 4:
                if int(p[3]) != target_label_id:
                    continue
            x, y, z = float(p[0]), float(p[1]), float(p[2])
            pts.append((x, y, z))
    except Exception:
        return None

    if not pts:
        return None

    pts = np.array(pts)
    mn = pts.min(axis=0)
    mx = pts.max(axis=0)
    center = ((mn + mx) * 0.5).tolist()
    half = ((mx - mn) * 0.5 + padding).tolist()
    half = [max(h, 0.01) for h in half]
    return (tuple(center), tuple(half))


def point_in_aabb(px, py, pz, center, half_extents):
    cx, cy, cz = center
    hx, hy, hz = half_extents
    return (
        (cx - hx <= px <= cx + hx) and
        (cy - hy <= py <= cy + hy) and
        (cz - hz <= pz <= cz + hz)
    )


def remove_points_in_aabb(cloud_msg, center, half_extents, output_frame_id=None):
    """
    从点云中移除落在 AABB 内的点，返回新的 PointCloud2（仅含 x,y,z）。

    Args:
        cloud_msg: sensor_msgs/PointCloud2
        center: (cx, cy, cz)
        half_extents: (hx, hy, hz)
        output_frame_id: 若指定，则输出 header.frame_id 用此；否则与输入相同

    Returns:
        sensor_msgs/PointCloud2
    """
    kept = []
    for p in pc2.read_points(cloud_msg, skip_nans=True, field_names=("x", "y", "z")):
        x, y, z = float(p[0]), float(p[1]), float(p[2])
        if not point_in_aabb(x, y, z, center, half_extents):
            kept.append((x, y, z))

    header = copy.deepcopy(cloud_msg.header)
    if output_frame_id is not None:
        header.frame_id = output_frame_id
    return pc2.create_cloud_xyz32(header, kept)


def remove_target_region_from_pointcloud(raw_cloud_msg, seg_cloud_msg, target_label_id, padding=0.02,
                                        output_frame_id=None):
    """
    方式 B：用分割点云中目标类别的 3D 区域，在原始点云中挖空该区域。

    要求：raw_cloud 与 seg_cloud 已在同一坐标系（调用方负责 TF 对齐）。

    Args:
        raw_cloud_msg: sensor_msgs/PointCloud2，整幅场景
        seg_cloud_msg: sensor_msgs/PointCloud2，含 x,y,z,label
        target_label_id: int，要挖掉的类别 id
        padding: float，AABB 扩展（米）
        output_frame_id: 可选，输出点云 frame_id

    Returns:
        sensor_msgs/PointCloud2，挖空目标后的点云；若 seg 中无该类别则返回 raw 的拷贝（仅 xyz）。
    """
    aabb = get_target_aabb_from_seg_cloud(seg_cloud_msg, target_label_id, padding=padding)
    if aabb is None:
        # 无目标点，返回原始点云（转为仅 xyz 的 cloud 以便格式统一）
        kept = []
        for p in pc2.read_points(raw_cloud_msg, skip_nans=True, field_names=("x", "y", "z")):
            kept.append((float(p[0]), float(p[1]), float(p[2])))
        header = copy.deepcopy(raw_cloud_msg.header)
        if output_frame_id is not None:
            header.frame_id = output_frame_id
        return pc2.create_cloud_xyz32(header, kept)
    center, half = aabb
    return remove_points_in_aabb(raw_cloud_msg, center, half, output_frame_id=output_frame_id)
