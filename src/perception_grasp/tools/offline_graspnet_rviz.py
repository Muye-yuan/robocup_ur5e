#!/usr/bin/env python3
"""Offline RViz publisher for GraspNet predictions on a local point cloud."""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

import numpy as np
import rospy
from geometry_msgs.msg import Pose, PoseArray, Point, Quaternion
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray
import sensor_msgs.point_cloud2 as pc2
from scipy.spatial.transform import Rotation

SCRIPT_DIR = Path(__file__).resolve().parent
PKG_DIR = SCRIPT_DIR.parent
NODES_DIR = PKG_DIR / "nodes"
if str(NODES_DIR) not in sys.path:
    sys.path.insert(0, str(NODES_DIR))

from grasp_inference_core import (  # noqa: E402
    GraspNetInferenceCore,
    filter_grasp_candidates_by_approach,
    load_point_cloud,
)


def _repo_root() -> Path:
    return PKG_DIR.parent.parent


def _resolve_path(path_text: str) -> str:
    raw = os.path.expanduser(path_text)
    path = Path(raw)
    if path.is_absolute():
        return str(path)
    candidate = _repo_root() / path
    if candidate.exists():
        return str(candidate)
    return str((Path.cwd() / path).resolve())


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Publish offline GraspNet results to RViz")
    parser.add_argument(
        "--cloud_path",
        default="/home/muye/robocup_ur5e/weights/graspnet/test_clouds/t1_label1.pcd",
        help="Path to local .ply/.pcd point cloud",
    )
    parser.add_argument(
        "--checkpoint_path",
        default="/home/muye/robocup_ur5e/weights/graspnet/checkpoint.tar",
        help="Path to GraspNet checkpoint",
    )
    parser.add_argument(
        "--graspnet_repo_path",
        default="/home/muye/robocup_ur5e/weights/graspnet/graspnet-baseline",
        help="Path to graspnet-baseline repo",
    )
    parser.add_argument("--top_k", type=int, default=5, help="Number of grasps to visualize")
    parser.add_argument(
        "--raw_top_k",
        type=int,
        default=None,
        help="Number of raw grasp candidates to fetch before filtering. Defaults to top_k.",
    )
    parser.add_argument("--frame_id", default="camera_depth_optical_frame", help="RViz frame_id")
    parser.add_argument("--rate", type=float, default=1.0, help="Republish rate in Hz")
    parser.add_argument("--cloud_topic", default="/grasp/offline_cloud", help="Point cloud topic")
    parser.add_argument("--pose_topic", default="/grasp/offline_poses", help="PoseArray topic")
    parser.add_argument("--marker_topic", default="/grasp/offline_markers", help="MarkerArray topic")
    parser.add_argument("--num_point", type=int, default=20000, help="GraspNet num_point")
    parser.add_argument("--voxel_size", type=float, default=0.0, help="Preprocess voxel size")
    parser.add_argument(
        "--approach_filter",
        default="top_side",
        choices=["off", "top", "top_side", "side_top"],
        help="Filter grasps by approach direction for table-top scenes.",
    )
    parser.add_argument(
        "--min_down_dot",
        type=float,
        default=0.25,
        help="Minimum dot product with world down direction to keep a grasp.",
    )
    parser.add_argument(
        "--max_up_dot",
        type=float,
        default=0.2,
        help="Maximum dot product with world up direction to keep a grasp.",
    )
    return parser


def build_point_cloud_msg(frame_id: str, points: np.ndarray) -> PointCloud2:
    header = Header(stamp=rospy.Time.now(), frame_id=frame_id)
    return pc2.create_cloud_xyz32(header, points.astype(np.float32).tolist())


def pose_from_candidate(candidate: dict) -> Pose:
    translation = np.asarray(candidate["translation"], dtype=np.float32).reshape(3)
    rotation_matrix = np.asarray(candidate["rotation"], dtype=np.float32).reshape(3, 3)
    quat = Rotation.from_matrix(rotation_matrix).as_quat()

    pose = Pose()
    pose.position = Point(x=float(translation[0]), y=float(translation[1]), z=float(translation[2]))
    pose.orientation = Quaternion(
        x=float(quat[0]),
        y=float(quat[1]),
        z=float(quat[2]),
        w=float(quat[3]),
    )
    return pose


def build_pose_array(frame_id: str, candidates: list[dict]) -> PoseArray:
    msg = PoseArray()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id
    msg.poses = [pose_from_candidate(candidate) for candidate in candidates]
    return msg


def _offset_pose(candidate: dict, local_offset: np.ndarray) -> Pose:
    translation = np.asarray(candidate["translation"], dtype=np.float32).reshape(3)
    rotation_matrix = np.asarray(candidate["rotation"], dtype=np.float32).reshape(3, 3)
    quat = Rotation.from_matrix(rotation_matrix).as_quat()
    world_position = translation + rotation_matrix @ local_offset.astype(np.float32)

    pose = Pose()
    pose.position = Point(
        x=float(world_position[0]),
        y=float(world_position[1]),
        z=float(world_position[2]),
    )
    pose.orientation = Quaternion(
        x=float(quat[0]),
        y=float(quat[1]),
        z=float(quat[2]),
        w=float(quat[3]),
    )
    return pose


def _make_gripper_part(
    frame_id: str,
    stamp: rospy.Time,
    marker_id: int,
    candidate: dict,
    local_offset: np.ndarray,
    scale_xyz: tuple[float, float, float],
    color: ColorRGBA,
) -> Marker:
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = "offline_gripper"
    marker.id = marker_id
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.pose = _offset_pose(candidate, local_offset)
    marker.scale.x = float(scale_xyz[0])
    marker.scale.y = float(scale_xyz[1])
    marker.scale.z = float(scale_xyz[2])
    marker.color = color
    return marker


def build_marker_array(frame_id: str, candidates: list[dict]) -> MarkerArray:
    markers = MarkerArray()
    max_score = max((float(candidate["score"]) for candidate in candidates), default=1.0)
    stamp = rospy.Time.now()

    for index, candidate in enumerate(candidates):
        score = float(candidate["score"])
        width = max(float(candidate.get("width", 0.02)), 0.005)
        color_ratio = 0.0 if max_score <= 0 else min(max(score / max_score, 0.0), 1.0)
        color = ColorRGBA(
            r=float(1.0 - color_ratio),
            g=float(color_ratio),
            b=0.2,
            a=0.9,
        )
        base_id = index * 3

        palm_length = 0.014
        palm_width = max(width + 0.01, 0.03)
        finger_length = 0.06
        finger_width = 0.008
        finger_thickness = 0.008
        palm_thickness = 0.012
        finger_gap = width * 0.5

        # Local grasp frame convention for visualization:
        # x: approach direction, y: gripper opening direction, z: finger thickness axis.
        palm_offset = np.array([-0.02, 0.0, 0.0], dtype=np.float32)
        left_finger_offset = np.array([0.01, finger_gap, 0.0], dtype=np.float32)
        right_finger_offset = np.array([0.01, -finger_gap, 0.0], dtype=np.float32)

        markers.markers.append(
            _make_gripper_part(
                frame_id,
                stamp,
                base_id,
                candidate,
                palm_offset,
                (palm_length, palm_width, palm_thickness),
                color,
            )
        )
        markers.markers.append(
            _make_gripper_part(
                frame_id,
                stamp,
                base_id + 1,
                candidate,
                left_finger_offset,
                (finger_length, finger_width, finger_thickness),
                color,
            )
        )
        markers.markers.append(
            _make_gripper_part(
                frame_id,
                stamp,
                base_id + 2,
                candidate,
                right_finger_offset,
                (finger_length, finger_width, finger_thickness),
                color,
            )
        )

    return markers


def main() -> int:
    args = build_parser().parse_args()

    cloud_path = _resolve_path(args.cloud_path)
    checkpoint_path = _resolve_path(args.checkpoint_path)
    repo_path = _resolve_path(args.graspnet_repo_path)
    raw_top_k = int(args.raw_top_k if args.raw_top_k is not None else args.top_k)

    rospy.init_node("offline_graspnet_rviz", anonymous=False)

    points, colors = load_point_cloud(cloud_path)
    core = GraspNetInferenceCore(
        checkpoint_path=checkpoint_path,
        device="auto",
        num_point=args.num_point,
        voxel_size=args.voxel_size,
        graspnet_repo_path=repo_path,
    )
    raw_candidates = core.predict(points, colors, top_k=raw_top_k)
    filtered_candidates = filter_grasp_candidates_by_approach(
        raw_candidates,
        mode=args.approach_filter,
        min_down_dot=args.min_down_dot,
        max_up_dot=args.max_up_dot,
    )
    candidates = filtered_candidates[: max(int(args.top_k), 0)]

    rospy.loginfo(f"[Offline RViz] Cloud: {cloud_path}")
    rospy.loginfo(f"[Offline RViz] Raw points: {points.shape[0]}")
    rospy.loginfo(f"[Offline RViz] Raw candidate count: {len(raw_candidates)}")
    rospy.loginfo(
        f"[Offline RViz] Filtered candidate count: {len(filtered_candidates)} "
        f"(visualizing {len(candidates)}) "
        f"(mode={args.approach_filter}, min_down_dot={args.min_down_dot}, max_up_dot={args.max_up_dot})"
    )

    cloud_pub = rospy.Publisher(args.cloud_topic, PointCloud2, queue_size=1, latch=True)
    pose_pub = rospy.Publisher(args.pose_topic, PoseArray, queue_size=1, latch=True)
    marker_pub = rospy.Publisher(args.marker_topic, MarkerArray, queue_size=1, latch=True)

    rate = rospy.Rate(args.rate)
    while not rospy.is_shutdown():
        cloud_msg = build_point_cloud_msg(args.frame_id, points)
        pose_msg = build_pose_array(args.frame_id, candidates)
        marker_msg = build_marker_array(args.frame_id, candidates)

        cloud_pub.publish(cloud_msg)
        pose_pub.publish(pose_msg)
        marker_pub.publish(marker_msg)
        rate.sleep()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
