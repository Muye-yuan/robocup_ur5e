#!/usr/bin/env python3
"""Publish labeled ASCII PCD files as PointCloud2 for perception_grasp tests."""

from __future__ import annotations

import argparse
from pathlib import Path

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Publish a labeled ASCII PCD file to a ROS PointCloud2 topic."
    )
    parser.add_argument(
        "--cloud_path",
        default="/home/muye/robocup_ur5e/weights/graspnet/test_clouds/t1234_labeled_merged.pcd",
        help="Path to an ASCII PCD file with fields x y z label.",
    )
    parser.add_argument(
        "--topic",
        default="/perception/yolo26_seg_cloud",
        help="PointCloud2 topic to publish.",
    )
    parser.add_argument(
        "--frame_id",
        default="camera_depth_optical_frame",
        help="frame_id written into the PointCloud2 header.",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=1.0,
        help="Publish rate in Hz when not using --once.",
    )
    parser.add_argument(
        "--once",
        action="store_true",
        help="Publish one message and exit after a short delay.",
    )
    return parser.parse_args()


def parse_ascii_pcd(cloud_path: Path) -> tuple[list[str], list[list[float | int]]]:
    lines = cloud_path.read_text(encoding="utf-8").splitlines()
    header = []
    data_start = None
    for index, line in enumerate(lines):
        header.append(line)
        if line.strip().lower().startswith("data "):
            data_start = index + 1
            break

    if data_start is None:
        raise ValueError(f"Invalid PCD file without DATA header: {cloud_path}")

    fields_line = next((line for line in header if line.startswith("FIELDS ")), None)
    if fields_line is None:
        raise ValueError(f"PCD file missing FIELDS header: {cloud_path}")

    field_names = fields_line.split()[1:]
    if not {"x", "y", "z"}.issubset(field_names):
        raise ValueError(f"PCD fields must include x y z: {field_names}")

    label_field = "label" if "label" in field_names else ("l" if "l" in field_names else None)
    if label_field is None:
        raise ValueError(f"PCD file must include label or l field: {cloud_path}")

    x_index = field_names.index("x")
    y_index = field_names.index("y")
    z_index = field_names.index("z")
    label_index = field_names.index(label_field)

    points = []
    for raw_line in lines[data_start:]:
        stripped = raw_line.strip()
        if not stripped:
            continue
        values = stripped.split()
        points.append(
            [
                float(values[x_index]),
                float(values[y_index]),
                float(values[z_index]),
                int(float(values[label_index])),
            ]
        )
    return field_names, points


def build_cloud(frame_id: str, points: list[list[float | int]]) -> PointCloud2:
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id
    fields = [
        PointField("x", 0, PointField.FLOAT32, 1),
        PointField("y", 4, PointField.FLOAT32, 1),
        PointField("z", 8, PointField.FLOAT32, 1),
        PointField("label", 12, PointField.UINT32, 1),
    ]
    return pc2.create_cloud(header, fields, points)


def main() -> int:
    args = parse_args()
    cloud_path = Path(args.cloud_path).expanduser().resolve()
    if not cloud_path.exists():
        raise FileNotFoundError(f"Point cloud file not found: {cloud_path}")

    _, points = parse_ascii_pcd(cloud_path)
    if not points:
        raise ValueError(f"No points found in PCD file: {cloud_path}")

    rospy.init_node("labeled_pcd_publisher", anonymous=False)
    publisher = rospy.Publisher(args.topic, PointCloud2, queue_size=1, latch=args.once)
    cloud_msg = build_cloud(args.frame_id, points)

    rospy.loginfo(f"[PCD Publisher] Loaded {len(points)} points from {cloud_path}")
    rospy.loginfo(f"[PCD Publisher] Publishing to {args.topic} with frame_id={args.frame_id}")

    if args.once:
        rospy.sleep(0.5)
        publisher.publish(cloud_msg)
        rospy.loginfo("[PCD Publisher] Published one message")
        rospy.sleep(0.5)
        return 0

    rate = rospy.Rate(args.rate)
    while not rospy.is_shutdown():
        cloud_msg.header.stamp = rospy.Time.now()
        publisher.publish(cloud_msg)
        rate.sleep()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
