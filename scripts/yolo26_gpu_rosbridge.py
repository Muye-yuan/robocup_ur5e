#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GPU YOLO26 推理（rosbridge 订阅图像与深度）
需要 rosbridge_server 已在 ROS 端启动
"""

import argparse
import base64
import json
import time

import cv2
import numpy as np
import roslibpy
from ultralytics import YOLO


def decode_image(msg):
    data = msg["data"]
    if isinstance(data, str):
        data = base64.b64decode(data)
    else:
        data = bytes(data)

    h = msg["height"]
    w = msg["width"]
    encoding = msg["encoding"]

    if encoding == "rgb8":
        img = np.frombuffer(data, dtype=np.uint8).reshape(h, w, 3)
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    if encoding == "bgr8":
        return np.frombuffer(data, dtype=np.uint8).reshape(h, w, 3)
    if encoding == "mono8":
        return np.frombuffer(data, dtype=np.uint8).reshape(h, w)

    raise ValueError(f"Unsupported encoding: {encoding}")


def decode_depth(msg):
    data = msg["data"]
    if isinstance(data, str):
        data = base64.b64decode(data)
    else:
        data = bytes(data)

    h = msg["height"]
    w = msg["width"]
    encoding = msg["encoding"]

    if encoding == "16UC1":
        depth = np.frombuffer(data, dtype=np.uint16).reshape(h, w)
        return depth.astype(np.float32) / 1000.0
    if encoding == "32FC1":
        return np.frombuffer(data, dtype=np.float32).reshape(h, w)

    raise ValueError(f"Unsupported depth encoding: {encoding}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9090)
    parser.add_argument("--model", default="/workspace/weights/yolo/yolo26m.pt")
    parser.add_argument("--rgb", default="/camera/rgb/image_raw")
    parser.add_argument("--depth", default="/camera/depth/image_raw")
    parser.add_argument("--camera-info", default="/camera/rgb/camera_info")
    parser.add_argument("--conf", type=float, default=0.5)
    parser.add_argument("--print-interval", type=float, default=0.2)
    args = parser.parse_args()

    model = YOLO(args.model)
    last_print = 0.0
    latest_depth = {"msg": None, "t": 0.0}
    cam_intrinsics = {"fx": None, "fy": None, "cx": None, "cy": None}

    ros = roslibpy.Ros(host=args.host, port=args.port)
    ros.run()

    depth_topic = roslibpy.Topic(ros, args.depth, "sensor_msgs/Image")
    rgb_topic = roslibpy.Topic(ros, args.rgb, "sensor_msgs/Image")
    info_topic = roslibpy.Topic(ros, args.camera_info, "sensor_msgs/CameraInfo")
    result_topic = roslibpy.Topic(ros, "/perception/yolo26_detections", "std_msgs/String")

    def on_camera_info(msg):
        if cam_intrinsics["fx"] is None:
            K = msg["K"]
            cam_intrinsics["fx"] = K[0]
            cam_intrinsics["fy"] = K[4]
            cam_intrinsics["cx"] = K[2]
            cam_intrinsics["cy"] = K[5]
            print(f"Camera intrinsics: fx={K[0]:.1f} fy={K[4]:.1f} cx={K[2]:.1f} cy={K[5]:.1f}")

    def pixel_to_3d(u, v, z):
        fx = cam_intrinsics["fx"]
        if fx is None or z is None or z <= 0:
            return None
        x = (u - cam_intrinsics["cx"]) * z / fx
        y = (v - cam_intrinsics["cy"]) * z / cam_intrinsics["fy"]
        return [round(x, 3), round(y, 3), round(z, 3)]

    def on_depth(msg):
        latest_depth["msg"] = msg
        latest_depth["t"] = time.time()

    def on_rgb(msg):
        nonlocal last_print
        now = time.time()
        if now - last_print < args.print_interval:
            return

        frame = decode_image(msg)
        depth_msg = latest_depth["msg"]
        depth = None
        if depth_msg is not None:
            try:
                depth = decode_depth(depth_msg)
            except Exception:
                depth = None

        results = model(frame, verbose=False)
        if not results:
            return

        h, w = frame.shape[:2]
        detections = []
        for result in results:
            if result.boxes is None:
                continue
            for box in result.boxes:
                conf = float(box.conf[0])
                if conf < args.conf:
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

                roi = frame[y1:y2 + 1, x1:x2 + 1]
                if roi.size > 0:
                    b, g, r = roi.mean(axis=(0, 1))
                    color_text = f"avg_bgr=({int(b)},{int(g)},{int(r)})"
                else:
                    color_text = "avg_bgr=(n/a)"

                dist = None
                if depth is not None:
                    try:
                        dist = float(depth[cy, cx])
                    except Exception:
                        dist = None

                pos_3d = pixel_to_3d(cx, cy, dist)

                print(
                    f"Detected {label} conf={conf:.2f} "
                    f"bbox=({x1},{y1},{x2},{y2}) center=({cx},{cy}) "
                    f"{color_text} distance={'n/a' if dist is None else f'{dist:.2f}m'} "
                    f"pos_cam={pos_3d if pos_3d else 'n/a'}"
                )

                detections.append({
                    "label": label,
                    "confidence": round(conf, 2),
                    "bbox": [int(x1), int(y1), int(x2), int(y2)],
                    "center": [cx, cy],
                    "distance_m": round(dist, 2) if dist is not None else None,
                    "position_camera": pos_3d,
                    "avg_bgr": [int(b), int(g), int(r)] if roi.size > 0 else None,
                })

        if detections:
            result_topic.publish(roslibpy.Message({"data": json.dumps(detections, ensure_ascii=False)}))

        last_print = now

    info_topic.subscribe(on_camera_info)
    depth_topic.subscribe(on_depth)
    rgb_topic.subscribe(on_rgb)

    try:
        while ros.is_connected:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        info_topic.unsubscribe()
        depth_topic.unsubscribe()
        rgb_topic.unsubscribe()
        result_topic.unadvertise()
        ros.terminate()


if __name__ == "__main__":
    main()
