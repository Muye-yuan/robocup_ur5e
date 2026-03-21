#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLO26-Seg -> XYZL PointCloud2 发布节点（按实例ID——每个物体都有一个id）

输入(同步):
  - /camera/rgb/image_raw
  - /camera/depth/image_raw
  - /camera/rgb/camera_info

处理:
  1) YOLO26-Seg 推理得到 mask + class id
  2) mask 像素结合深度和内参反投影为 3D 点
  3) 每个目标点数超过 max_points_per_obj 时随机下采样
  4) label 字段写入实例ID（同一帧内每个检测目标一个ID）

输出:
  - /perception/yolo26_seg_instance_cloud (sensor_msgs/PointCloud2, XYZL)

说明:
  - 这里的实例ID是“单帧内”编号，不做跨帧跟踪，下一帧会重新编号。
"""

import os
import random
import struct

import cv2
import message_filters
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Header
from ultralytics import YOLO


class Yolo26SegXYZLInstanceCloudNode:
    def __init__(self):
        rospy.init_node("yolo26_seg_xyzl_instance_cloud", anonymous=False)

        self.bridge = CvBridge()
        self.model_path = rospy.get_param("~model_path", "/workspace/weights/yolo/best.pt")
        self.conf_threshold = float(rospy.get_param("~confidence_threshold", 0.5))
        self.max_points_per_obj = int(rospy.get_param("~max_points_per_obj", 3000))
        self.sync_queue_size = int(rospy.get_param("~sync_queue_size", 10))
        self.sync_slop = float(rospy.get_param("~sync_slop", 0.08))

        self.rgb_topic = rospy.get_param("~rgb_topic", "/camera/rgb/image_raw")
        self.depth_topic = rospy.get_param("~depth_topic", "/camera/depth/image_raw")
        self.camera_info_topic = rospy.get_param("~camera_info_topic", "/camera/rgb/camera_info")
        self.cloud_topic = rospy.get_param("~cloud_topic", "/perception/yolo26_seg_instance_cloud")

        if not os.path.exists(self.model_path):
            raise FileNotFoundError(self.model_path)
        self.model = YOLO(self.model_path)

        self.cloud_pub = rospy.Publisher(self.cloud_topic, PointCloud2, queue_size=1)

        rgb_sub = message_filters.Subscriber(self.rgb_topic, Image)
        depth_sub = message_filters.Subscriber(self.depth_topic, Image)
        info_sub = message_filters.Subscriber(self.camera_info_topic, CameraInfo)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub, info_sub],
            queue_size=self.sync_queue_size,
            slop=self.sync_slop,
        )
        self.sync.registerCallback(self._callback)

        rospy.loginfo("YOLO model: %s", self.model_path)
        rospy.loginfo("Sync topics: %s | %s | %s", self.rgb_topic, self.depth_topic, self.camera_info_topic)
        rospy.loginfo("Cloud topic: %s", self.cloud_topic)
        rospy.loginfo("conf_threshold=%.2f, max_points_per_obj=%d", self.conf_threshold, self.max_points_per_obj)

    @staticmethod
    def _depth_to_meters(depth_img):
        if np.issubdtype(depth_img.dtype, np.integer):
            return depth_img.astype(np.float32) / 1000.0
        return depth_img.astype(np.float32)

    @staticmethod
    def _mask_to_xyz(mask, depth_m, fx, fy, cx, cy):
        v, u = np.where(mask)
        if v.size == 0:
            return np.empty((0, 3), dtype=np.float32)

        z = depth_m[v, u]
        valid = np.isfinite(z) & (z > 0.05) & (z < 10.0)
        if not np.any(valid):
            return np.empty((0, 3), dtype=np.float32)

        u = u[valid].astype(np.float32)
        v = v[valid].astype(np.float32)
        z = z[valid].astype(np.float32)

        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        return np.stack((x, y, z), axis=1)

    @staticmethod
    def _build_xyzl_cloud(points_xyzl, stamp, frame_id):
        msg = PointCloud2()
        msg.header = Header(stamp=stamp, frame_id=frame_id)
        msg.height = 1
        msg.width = int(points_xyzl.shape[0])
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="label", offset=12, datatype=PointField.UINT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = False

        buf = bytearray(msg.row_step)
        for i, p in enumerate(points_xyzl):
            struct.pack_into("fffI", buf, i * 16, float(p[0]), float(p[1]), float(p[2]), int(p[3]))
        msg.data = bytes(buf)
        return msg

    def _callback(self, rgb_msg, depth_msg, camera_info_msg):
        try:
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
            depth_raw = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except CvBridgeError as exc:
            rospy.logerr_throttle(2.0, "CvBridge error: %s", str(exc))
            return

        depth_m = self._depth_to_meters(depth_raw)
        h, w = rgb.shape[:2]
        if depth_m.shape[:2] != (h, w):
            depth_m = cv2.resize(depth_m, (w, h), interpolation=cv2.INTER_NEAREST)

        k = camera_info_msg.K
        fx, fy, cx, cy = k[0], k[4], k[2], k[5]
        if fx == 0.0 or fy == 0.0:
            return

        results = self.model(rgb, verbose=False)
        if not results:
            return

        all_points_xyzl = []
        instance_id = 1

        for result in results:
            if result.boxes is None or result.masks is None:
                continue

            masks = result.masks.data.cpu().numpy()
            boxes = result.boxes

            for i, box in enumerate(boxes):
                conf = float(box.conf[0])
                if conf < self.conf_threshold or i >= len(masks):
                    continue

                mask = masks[i]
                if mask.shape[:2] != (h, w):
                    mask = cv2.resize(mask, (w, h), interpolation=cv2.INTER_NEAREST)
                mask_bin = mask > 0.5

                points_xyz = self._mask_to_xyz(mask_bin, depth_m, fx, fy, cx, cy)
                if points_xyz.shape[0] == 0:
                    continue

                if points_xyz.shape[0] > self.max_points_per_obj:
                    idx = random.sample(range(points_xyz.shape[0]), self.max_points_per_obj)
                    points_xyz = points_xyz[idx]

                label_col = np.full((points_xyz.shape[0], 1), instance_id, dtype=np.float32)
                points_xyzl = np.concatenate([points_xyz, label_col], axis=1)
                all_points_xyzl.append(points_xyzl)
                instance_id += 1

        if not all_points_xyzl:
            return

        merged = np.vstack(all_points_xyzl)
        cloud_msg = self._build_xyzl_cloud(merged, rgb_msg.header.stamp, rgb_msg.header.frame_id)
        self.cloud_pub.publish(cloud_msg)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        node = Yolo26SegXYZLInstanceCloudNode()
        node.run()
    except rospy.ROSInterruptException:
        pass



"""

cd /home/liufazhan/robocup_ur5e
docker compose run --rm perception_yolo_gpu_native bash -lc "
source /opt/ros/noetic/setup.bash &&
export ROS_MASTER_URI=http://127.0.0.1:11311 &&
python3 /workspace/src/perception_yolo/nodes/yolo26_seg_xyzl_instance_cloud_node.py \
  _model_path:=/workspace/weights/yolo/yolo26m-seg.pt \
  _rgb_topic:=/camera/rgb/image_raw \
  _depth_topic:=/camera/depth/image_raw \
  _camera_info_topic:=/camera/rgb/camera_info \
  _cloud_topic:=/perception/yolo26_seg_instance_cloud \
  _confidence_threshold:=0.5 \
  _max_points_per_obj:=3000
"

"""




"""
可以在另一个终端执行以下命令，查看点云
docker compose run --rm perception_yolo_gpu_native bash -lc "
source /opt/ros/noetic/setup.bash &&
export ROS_MASTER_URI=http://127.0.0.1:11311 &&
rostopic echo /perception/yolo26_seg_instance_cloud -n1
"
"""