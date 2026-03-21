#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLO26 检测节点（终端输出）
订阅:
  - /camera/rgb/image_raw
  - /camera/depth/image_raw
"""

import json
import os
import time

import message_filters
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


class Yolo26InfoNode:
    def __init__(self):
        rospy.init_node("yolo26_info", anonymous=True)

        self.bridge = CvBridge()
        self.model_path = rospy.get_param("~model_path", "/workspace/weights/yolo/yolo26m.pt")
        self.conf_threshold = float(rospy.get_param("~confidence_threshold", 0.5))
        self.image_topic = rospy.get_param("~image_topic", "/camera/rgb/image_raw")
        self.depth_topic = rospy.get_param("~depth_topic", "/camera/depth/image_raw")
        self.print_interval = float(rospy.get_param("~print_interval", 0.2))
        self._last_print = 0.0

        try:
            from ultralytics import YOLO
        except Exception as exc:
            raise RuntimeError(f"ultralytics not installed: {exc}")

        if not os.path.exists(self.model_path):
            raise FileNotFoundError(self.model_path)

        self.model = YOLO(self.model_path)
        rospy.loginfo("YOLO model: %s", self.model_path)
        rospy.loginfo("Subscribing RGB: %s", self.image_topic)
        rospy.loginfo("Subscribing Depth: %s", self.depth_topic)
        rospy.loginfo("Confidence threshold: %.2f", self.conf_threshold)

        self.camera_info_topic = rospy.get_param("~camera_info_topic", "/camera/rgb/camera_info")
        self.fx = None
        self.fy = None
        self.cx_cam = None
        self.cy_cam = None
        rospy.Subscriber(self.camera_info_topic, CameraInfo, self._camera_info_cb)
        rospy.loginfo("Subscribing CameraInfo: %s", self.camera_info_topic)

        self.detection_pub = rospy.Publisher("/perception/yolo26_detections", String, queue_size=10)

        rgb_sub = message_filters.Subscriber(self.image_topic, Image)
        depth_sub = message_filters.Subscriber(self.depth_topic, Image)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=10, slop=0.1
        )
        self.sync.registerCallback(self.image_callback)

    def _camera_info_cb(self, msg):
        if self.fx is None:
            K = msg.K
            self.fx = K[0]
            self.fy = K[4]
            self.cx_cam = K[2]
            self.cy_cam = K[5]
            rospy.loginfo("Camera intrinsics: fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
                          self.fx, self.fy, self.cx_cam, self.cy_cam)

    def _pixel_to_3d(self, u, v, z):
        if self.fx is None or z is None or z <= 0:
            return None
        x = (u - self.cx_cam) * z / self.fx
        y = (v - self.cy_cam) * z / self.fy
        return [round(x, 3), round(y, 3), round(z, 3)]

    def image_callback(self, rgb_msg, depth_msg):
        now = time.time()
        if now - self._last_print < self.print_interval:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except CvBridgeError as exc:
            rospy.logerr("CvBridge error: %s", str(exc))
            return

        results = self.model(frame, verbose=False)
        if not results:
            return

        h, w = frame.shape[:2]
        detections = []
        for result in results:
            if result.boxes is None:
                continue
            for box in result.boxes:
                conf = float(box.conf[0])
                if conf < self.conf_threshold:
                    continue
                cls_id = int(box.cls[0])
                label = result.names[cls_id]
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)

                x1 = max(0, min(x1, w - 1))
                x2 = max(0, min(x2, w - 1))
                y1 = max(0, min(y1, h - 1))
                y2 = max(0, min(y2, h - 1))

                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)

                dist = None
                try:
                    d = depth[cy, cx]
                    if np.issubdtype(depth.dtype, np.integer):
                        dist = float(d) / 1000.0
                    else:
                        dist = float(d)
                except Exception:
                    dist = None

                roi = frame[y1:y2 + 1, x1:x2 + 1]
                if roi.size > 0:
                    b, g, r = roi.mean(axis=(0, 1))
                    avg_bgr = [int(b), int(g), int(r)]
                else:
                    avg_bgr = None

                pos_3d = self._pixel_to_3d(cx, cy, dist)

                det = {
                    "label": label,
                    "confidence": round(conf, 2),
                    "bbox": [int(x1), int(y1), int(x2), int(y2)],
                    "center": [cx, cy],
                    "distance_m": round(dist, 2) if dist is not None else None,
                    "position_camera": pos_3d,
                    "avg_bgr": avg_bgr,
                }
                detections.append(det)

                rospy.loginfo(
                    "Detected %s conf=%.2f bbox=(%d,%d,%d,%d) center=(%d,%d) avg_bgr=%s distance=%s pos_cam=%s",
                    label, conf, x1, y1, x2, y2, cx, cy,
                    str(avg_bgr) if avg_bgr else "n/a",
                    f"{dist:.2f}m" if dist is not None else "n/a",
                    str(pos_3d) if pos_3d else "n/a"
                )

        if detections:
            msg = String()
            msg.data = json.dumps(detections, ensure_ascii=False)
            self.detection_pub.publish(msg)

        self._last_print = now

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        node = Yolo26InfoNode()
        node.run()
    except rospy.ROSInterruptException:
        pass


# 执行命令
# 1.启动gpu的docker容器
# docker compose up -d perception_yolo_gpu
# 2. 通过rosbridge连接到gpu的docker容器
# docker exec -d perception_yolo bash -c "source /opt/ros/noetic/setup.bash && export ROS_MASTER_URI=http://127.0.0.1:11311 && roslaunch rosbridge_server rosbridge_websocket.launch"
# 3. 运行yolo26_info_node
# docker exec -it perception_yolo_gpu bash -c "python3.10 /workspace/scripts/yolo26_gpu_rosbridge.py --host 127.0.0.1 --port 9090 --model /workspace/weights/yolo/yolo26m.pt --rgb /camera/rgb/image_raw --depth /camera/depth/image_raw"