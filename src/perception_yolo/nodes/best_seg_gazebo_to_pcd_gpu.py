#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Use YOLO segmentation model on Gazebo camera streams and save each object as PCD.

Data source (via rosbridge):
  - RGB image topic (sensor_msgs/Image)
  - Depth image topic (sensor_msgs/Image)
  - CameraInfo topic (sensor_msgs/CameraInfo)

Output:
  - One PCD file per detected object under --save-dir
"""

import argparse
import base64
import os
import time

import cv2
import numpy as np
import roslibpy
import torch
from ultralytics import YOLO


def decode_image(msg):
    data = msg["data"]
    data = base64.b64decode(data) if isinstance(data, str) else bytes(data)
    h, w = msg["height"], msg["width"]
    enc = msg["encoding"]

    if enc == "rgb8":
        img = np.frombuffer(data, dtype=np.uint8).reshape(h, w, 3)
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    if enc == "bgr8":
        return np.frombuffer(data, dtype=np.uint8).reshape(h, w, 3)
    if enc == "mono8":
        return np.frombuffer(data, dtype=np.uint8).reshape(h, w)

    raise ValueError(f"Unsupported RGB encoding: {enc}")


def decode_depth(msg):
    data = msg["data"]
    data = base64.b64decode(data) if isinstance(data, str) else bytes(data)
    h, w = msg["height"], msg["width"]
    enc = msg["encoding"]

    if enc == "16UC1":
        return np.frombuffer(data, dtype=np.uint16).reshape(h, w).astype(np.float32) / 1000.0
    if enc == "32FC1":
        return np.frombuffer(data, dtype=np.float32).reshape(h, w)

    raise ValueError(f"Unsupported depth encoding: {enc}")


def mask_to_3d(mask, depth, fx, fy, cx, cy):
    vs, us = np.where(mask > 0)
    if len(vs) == 0:
        return np.empty((0, 3), dtype=np.float32)

    zs = depth[vs, us].astype(np.float32)
    valid = (zs > 0.05) & (zs < 10.0)
    vs, us, zs = vs[valid], us[valid], zs[valid]
    if len(zs) == 0:
        return np.empty((0, 3), dtype=np.float32)

    xs = (us.astype(np.float32) - cx) * zs / fx
    ys = (vs.astype(np.float32) - cy) * zs / fy
    return np.stack([xs, ys, zs], axis=1)


def voxel_downsample(points, voxel_size):
    if len(points) == 0 or voxel_size <= 0:
        return points

    grid = np.floor(points / voxel_size).astype(np.int32)
    keys = grid[:, 0] * 1_000_000 + grid[:, 1] * 1_000 + grid[:, 2]
    order = np.argsort(keys)
    keys_sorted = keys[order]
    pts_sorted = points[order]

    unique_mask = np.empty(len(keys_sorted), dtype=bool)
    unique_mask[0] = True
    unique_mask[1:] = keys_sorted[1:] != keys_sorted[:-1]
    idx = np.where(unique_mask)[0]

    out = np.empty((len(idx), 3), dtype=np.float32)
    for k, (start, end) in enumerate(zip(idx, np.append(idx[1:], len(pts_sorted)))):
        out[k] = pts_sorted[start:end].mean(axis=0)
    return out


def save_pcd_xyz(points, path):
    if len(points) == 0:
        return False
    n = len(points)
    with open(path, "w", encoding="utf-8") as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z\n")
        f.write("SIZE 4 4 4\n")
        f.write("TYPE F F F\n")
        f.write("COUNT 1 1 1\n")
        f.write(f"WIDTH {n}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {n}\n")
        f.write("DATA ascii\n")
        for i in range(n):
            f.write(f"{points[i][0]:.6f} {points[i][1]:.6f} {points[i][2]:.6f}\n")
    return True


def main():
    parser = argparse.ArgumentParser(description="Gazebo segmentation to per-object PCD (GPU)")
    parser.add_argument("--host", default="127.0.0.1", help="rosbridge host")
    parser.add_argument("--port", type=int, default=9090, help="rosbridge port")
    parser.add_argument("--model", default="/workspace/weights/yolo/best.pt", help="YOLO seg model path")
    parser.add_argument("--rgb", default="/camera/rgb/image_raw", help="RGB topic")
    parser.add_argument("--depth", default="/camera/depth/image_raw", help="Depth topic")
    parser.add_argument("--camera-info", default="/camera/rgb/camera_info", help="CameraInfo topic")
    parser.add_argument("--save-dir", default="/workspace/pointclouds", help="Output directory")
    parser.add_argument("--conf", type=float, default=0.4, help="Confidence threshold")
    parser.add_argument("--infer-interval", type=float, default=0.25, help="Inference interval seconds")
    parser.add_argument("--save-interval", type=float, default=1.0, help="Save interval seconds")
    parser.add_argument("--voxel-size", type=float, default=0.01, help="Voxel size in meters")
    parser.add_argument("--max-pts", type=int, default=20000, help="Max points per object cloud")
    parser.add_argument("--min-pts", type=int, default=200, help="Min points to save one object cloud")
    args = parser.parse_args()

    os.makedirs(args.save_dir, exist_ok=True)

    if not torch.cuda.is_available():
        print("[WARN] CUDA is not available, inference will run on CPU.")
        device = "cpu"
    else:
        device = "cuda:0"
        print(f"[INFO] CUDA device: {torch.cuda.get_device_name(0)}")

    print(f"[INFO] Loading model: {args.model}")
    model = YOLO(args.model)

    ros = roslibpy.Ros(host=args.host, port=args.port)
    ros.run()
    if not ros.is_connected:
        raise RuntimeError(f"Failed to connect rosbridge: {args.host}:{args.port}")
    print(f"[INFO] Connected rosbridge: {args.host}:{args.port}")

    sub_rgb = roslibpy.Topic(ros, args.rgb, "sensor_msgs/Image")
    sub_depth = roslibpy.Topic(ros, args.depth, "sensor_msgs/Image")
    sub_info = roslibpy.Topic(ros, args.camera_info, "sensor_msgs/CameraInfo")

    latest_depth = {"msg": None}
    cam = {"fx": None, "fy": None, "cx": None, "cy": None}
    last_infer = 0.0
    last_save = 0.0

    def on_info(msg):
        if cam["fx"] is None:
            k = msg["K"]
            cam["fx"], cam["fy"], cam["cx"], cam["cy"] = k[0], k[4], k[2], k[5]
            print(
                f"[INFO] Camera intrinsics fx={cam['fx']:.2f} fy={cam['fy']:.2f} "
                f"cx={cam['cx']:.2f} cy={cam['cy']:.2f}"
            )

    def on_depth(msg):
        latest_depth["msg"] = msg

    def on_rgb(msg):
        nonlocal last_infer, last_save

        now = time.time()
        if now - last_infer < args.infer_interval:
            return
        if cam["fx"] is None or latest_depth["msg"] is None:
            return
        last_infer = now

        try:
            frame = decode_image(msg)
            depth = decode_depth(latest_depth["msg"])
        except Exception as e:
            print(f"[WARN] Decode failed: {e}")
            return

        results = model(frame, conf=args.conf, device=device, verbose=False)
        if not results:
            return

        result = results[0]
        if result.boxes is None or result.masks is None:
            return

        # reduce disk pressure
        if now - last_save < args.save_interval:
            return

        h, w = frame.shape[:2]
        fx, fy, cx, cy = cam["fx"], cam["fy"], cam["cx"], cam["cy"]

        ts = time.strftime("%Y%m%d_%H%M%S")
        save_count = 0

        for i, box in enumerate(result.boxes):
            conf = float(box.conf[0])
            if conf < args.conf or i >= len(result.masks.data):
                continue

            cls_id = int(box.cls[0])
            label = str(result.names.get(cls_id, f"class{cls_id}")).replace(" ", "_")

            mask_raw = result.masks.data[i].cpu().numpy()
            mask_full = cv2.resize(mask_raw, (w, h), interpolation=cv2.INTER_NEAREST)
            binary_mask = (mask_full > 0.5).astype(np.uint8)

            pts = mask_to_3d(binary_mask, depth, fx, fy, cx, cy)
            pts = voxel_downsample(pts, args.voxel_size)

            if len(pts) > args.max_pts:
                sel = np.random.choice(len(pts), args.max_pts, replace=False)
                pts = pts[sel]
            if len(pts) < args.min_pts:
                continue

            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
            filename = (
                f"{ts}_{label}_idx{i}_conf{conf:.2f}_"
                f"bbox{x1}_{y1}_{x2}_{y2}_pts{len(pts)}.pcd"
            )
            path = os.path.join(args.save_dir, filename)

            if save_pcd_xyz(pts, path):
                save_count += 1
                print(f"[SAVE] {path}")

        if save_count > 0:
            last_save = now
            print(f"[INFO] Saved {save_count} object clouds at {ts}")

    sub_info.subscribe(on_info)
    sub_depth.subscribe(on_depth)
    sub_rgb.subscribe(on_rgb)

    print("[INFO] Waiting for Gazebo camera data, Ctrl+C to stop...")
    try:
        while ros.is_connected:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        sub_info.unsubscribe()
        sub_depth.unsubscribe()
        sub_rgb.unsubscribe()
        ros.terminate()
        print("[INFO] Exit.")


if __name__ == "__main__":
    main()






# docker exec -d perception_yolo bash -c "source /opt/ros/noetic/setup.bash && export ROS_MASTER_URI=http://127.0.0.1:11311 && roslaunch rosbridge_server rosbridge_websocket.launch"
# docker exec -it perception_yolo_gpu bash -c "python3.10 /workspace/src/perception_yolo/nodes/best_seg_gazebo_to_pcd_gpu.py --host 127.0.0.1 --port 9090 --model /workspace/weights/yolo/yolo26m-seg.pt --save-dir /workspace/pointclouds"