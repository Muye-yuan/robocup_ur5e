#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLO26-Seg 轻量 JSON 输出节点

功能:
  - 同步订阅 RGB / Depth / CameraInfo
  - 使用 yolo26m-seg.pt 推理
  - 发布轻量 JSON 到 ROS 话题:
      /perception/yolo26_seg_detections (std_msgs/String)
  - 同时在终端打印同样的 JSON
"""

import json
import os
import time

import cv2
import message_filters
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String
from ultralytics import YOLO


#HARDCODED_MODEL_PATH = "/workspace/weights/yolo/yolo26m-seg.pt"
HARDCODED_MODEL_PATH = "/workspace/weights/yolo/best.pt"

class Yolo26SegJsonNode:
    def __init__(self):
        rospy.init_node("yolo26_seg_json", anonymous=False)

        self.bridge = CvBridge()
        self.model_path = rospy.get_param("~model_path", "/workspace/weights/yolo/yolo26m-seg.pt")
        self.conf_threshold = float(rospy.get_param("~confidence_threshold", 0.5))
        self.print_interval = float(rospy.get_param("~print_interval", 0.2))
        self.sync_slop = float(rospy.get_param("~sync_slop", 0.08))
        self.sync_queue_size = int(rospy.get_param("~sync_queue_size", 10))

        self.rgb_topic = rospy.get_param("~rgb_topic", "/camera/rgb/image_raw")
        self.depth_topic = rospy.get_param("~depth_topic", "/camera/depth/image_raw")
        self.camera_info_topic = rospy.get_param("~camera_info_topic", "/camera/rgb/camera_info")
        self.output_topic = rospy.get_param("~output_topic", "/perception/yolo26_seg_detections")
        self.enable_topic = rospy.get_param(
            "~enable_topic",
            "/perception/yolo26_seg_enabled",
        )
        self.enabled = bool(rospy.get_param("~start_enabled", True))

        if not os.path.exists(self.model_path):
            raise FileNotFoundError(self.model_path)
        self.model = YOLO(self.model_path)

        self._last_print = 0.0
        self.pub_json = rospy.Publisher(self.output_topic, String, queue_size=10)
        self.enable_sub = rospy.Subscriber(
            self.enable_topic,
            Bool,
            self._enable_callback,
            queue_size=1,
        )

        rgb_sub = message_filters.Subscriber(self.rgb_topic, Image)
        depth_sub = message_filters.Subscriber(self.depth_topic, Image)
        info_sub = message_filters.Subscriber(self.camera_info_topic, CameraInfo)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub, info_sub],
            queue_size=self.sync_queue_size,
            slop=self.sync_slop,
        )
        self.sync.registerCallback(self._callback)

        rospy.loginfo("YOLO26-Seg model: %s", self.model_path)
        rospy.loginfo("Sync topics: %s | %s | %s", self.rgb_topic, self.depth_topic, self.camera_info_topic)
        rospy.loginfo("Output topic: %s", self.output_topic)
        rospy.loginfo("Enable topic: %s (start_enabled=%s)", self.enable_topic, str(self.enabled))
        rospy.loginfo("conf_threshold=%.2f, print_interval=%.2fs", self.conf_threshold, self.print_interval)

    def _enable_callback(self, msg):
        new_state = bool(msg.data)
        if new_state == self.enabled:
            return
        self.enabled = new_state
        rospy.loginfo("YOLO26-Seg processing %s", "enabled" if self.enabled else "disabled")

    @staticmethod
    def _depth_to_meters(depth_img):
        if np.issubdtype(depth_img.dtype, np.integer):
            return depth_img.astype(np.float32) / 1000.0
        return depth_img.astype(np.float32)

    @staticmethod
    def _pixel_to_xyz(u, v, z, fx, fy, cx, cy):
        if not np.isfinite(z) or z <= 0.05 or z >= 10.0:
            return None
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        return float(x), float(y), float(z)

    @staticmethod
    def _mask_center_3d(mask, depth_m, fx, fy, cx, cy):
        v, u = np.where(mask)
        if v.size == 0:
            return None

        z = depth_m[v, u]
        valid = np.isfinite(z) & (z > 0.05) & (z < 10.0)
        if not np.any(valid):
            return None

        u = u[valid].astype(np.float32)
        v = v[valid].astype(np.float32)
        z = z[valid].astype(np.float32)

        uc = float(np.mean(u))
        vc = float(np.mean(v))
        zc = float(np.median(z))
        xyz = Yolo26SegJsonNode._pixel_to_xyz(uc, vc, zc, fx, fy, cx, cy)
        return xyz

    def _callback(self, rgb_msg, depth_msg, cam_info_msg):
        if not self.enabled:
            return

        now = time.time()
        if now - self._last_print < self.print_interval:
            return
        gazebo_stamp = {
            "secs": int(rgb_msg.header.stamp.secs),
            "nsecs": int(rgb_msg.header.stamp.nsecs),
        }

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

        k = cam_info_msg.K
        fx, fy, cx, cy = k[0], k[4], k[2], k[5]
        if fx == 0.0 or fy == 0.0:
            return

        results = self.model(rgb, verbose=False)
        if not results:
            return

        output = []
        for result in results:
            if result.boxes is None:
                continue

            masks = None
            if result.masks is not None:
                masks = result.masks.data.cpu().numpy()

            boxes = result.boxes
            names = result.names

            for i, box in enumerate(boxes):
                conf = float(box.conf[0])
                if conf < self.conf_threshold:
                    continue

                cls_id = int(box.cls[0])
                name = str(names.get(cls_id, str(cls_id)))

                center_xyz = None
                if masks is not None and i < len(masks):
                    mask = masks[i]
                    if mask.shape[:2] != (h, w):
                        mask = cv2.resize(mask, (w, h), interpolation=cv2.INTER_NEAREST)
                    center_xyz = self._mask_center_3d(mask > 0.5, depth_m, fx, fy, cx, cy)

                # 分割中心不可用时，回退 bbox 中心
                if center_xyz is None:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                    u = int((x1 + x2) / 2)
                    v = int((y1 + y2) / 2)
                    u = max(0, min(u, w - 1))
                    v = max(0, min(v, h - 1))
                    z = float(depth_m[v, u])
                    center_xyz = self._pixel_to_xyz(float(u), float(v), z, fx, fy, cx, cy)

                if center_xyz is None:
                    continue

                output.append({
                    "name": name,
                    "confidence": round(conf, 3),
                    "frame_id": rgb_msg.header.frame_id,
                    "gazebo_stamp": gazebo_stamp,
                    "center_3d": {
                        "x": round(center_xyz[0], 4),
                        "y": round(center_xyz[1], 4),
                        "z": round(center_xyz[2], 4),
                    },
                })

        if output:
            payload = json.dumps(output, ensure_ascii=False, separators=(",", ":"))
            self.pub_json.publish(String(data=payload))
            rospy.loginfo(payload)
            self._last_print = now

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        node = Yolo26SegJsonNode()
        node.run()
    except rospy.ROSInterruptException:
        pass




# # 运行节点（native 容器内）
# docker compose run --rm perception_yolo_gpu_native bash -lc "source /opt/ros/noetic/setup.bash && export ROS_MASTER_URI=http://127.0.0.1:11311 && python3 /workspace/src/perception_yolo/nodes/yolo26_seg_json_node.py _model_path:=/workspace/weights/yolo/yolo26m-seg.pt _confidence_threshold:=0.5"