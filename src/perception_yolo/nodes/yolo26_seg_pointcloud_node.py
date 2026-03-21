#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLO26-Seg 语义分割 + 物体点云发布节点
通过 rosbridge 连接 ROS，订阅 RGB/Depth/CameraInfo，
用 yolo26m-seg.pt 做语码与点云结合，
输出分割后的义分割，将掩物体点云。

发布话题:
  /perception/seg_objects        std_msgs/String  (JSON 检测摘要，轻量)
  /perception/seg_center_points  std_msgs/String  (JSON 3D 中心点列表，极轻量)
  /perception/seg_cloud          sensor_msgs/PointCloud2 (体素下采样后的完整点云)
"""

import argparse
import base64
import json
import os
import struct
import time

import cv2
import numpy as np
import roslibpy
from ultralytics import YOLO


# ─────────────────────────── 工具函数 ──────────────────────────────


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
    raise ValueError(f"Unsupported encoding: {enc}")


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
    """2D 掩码 + 深度图 → Nx3 float32 点云（米）"""
    vs, us = np.where(mask > 0)
    if len(vs) == 0:
        return np.empty((0, 3), dtype=np.float32)
    zs = depth[vs, us].astype(np.float32)
    valid = (zs > 0.05) & (zs < 10.0)
    vs, us, zs = vs[valid], us[valid], zs[valid]
    xs = (us.astype(np.float32) - cx) * zs / fx
    ys = (vs.astype(np.float32) - cy) * zs / fy
    return np.stack([xs, ys, zs], axis=1)


def voxel_downsample(points, voxel_size=0.02):
    """体素下采样，每个体素格取重心，减少点数"""
    if len(points) == 0:
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
    result = np.empty((len(idx), 3), dtype=np.float32)
    for k, (start, end) in enumerate(zip(idx, np.append(idx[1:], len(pts_sorted)))):
        result[k] = pts_sorted[start:end].mean(axis=0)
    return result


def save_pcd(points, filepath):
    """将 Nx3 点云保存为 PCD 格式"""
    if len(points) == 0:
        return False
    n = len(points)
    with open(filepath, "w") as f:
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
    print(f"[SEG] 已保存点云: {filepath} ({n} 点)")
    return True


def build_pc2_msg(points, labels, frame_id="camera_rgb_optical_frame"):
    """构建 PointCloud2 消息 (XYZL: 3×float32 + 1×uint32)"""
    n = len(points)
    point_step = 16
    buf = bytearray(n * point_step)
    for i in range(n):
        struct.pack_into("fffI", buf, i * point_step,
                         float(points[i][0]), float(points[i][1]), float(points[i][2]),
                         int(labels[i]))
    t = time.time()
    return {
        "header": {
            "seq": 0,
            "stamp": {"secs": int(t), "nsecs": int((t - int(t)) * 1e9)},
            "frame_id": frame_id,
        },
        "height": 1,
        "width": n,
        "fields": [
            {"name": "x",     "offset": 0,  "datatype": 7, "count": 1},
            {"name": "y",     "offset": 4,  "datatype": 7, "count": 1},
            {"name": "z",     "offset": 8,  "datatype": 7, "count": 1},
            {"name": "label", "offset": 12, "datatype": 6, "count": 1},
        ],
        "is_bigendian": False,
        "point_step": point_step,
        "row_step": n * point_step,
        "data": base64.b64encode(bytes(buf)).decode("ascii"),
        "is_dense": True,
    }


# ─────────────────────────── 主函数 ──────────────────────────────


def main():
    parser = argparse.ArgumentParser(description="YOLO26-Seg + 物体点云发布")
    parser.add_argument("--host",         default="127.0.0.1")
    parser.add_argument("--port",         type=int,   default=9090)
    parser.add_argument("--model",        default="/workspace/weights/yolo/yolo26m-seg.pt")
    parser.add_argument("--rgb",          default="/camera/rgb/image_raw")
    parser.add_argument("--depth",        default="/camera/depth/image_raw")
    parser.add_argument("--camera-info",  default="/camera/rgb/camera_info")
    parser.add_argument("--conf",         type=float, default=0.5)
    parser.add_argument("--interval",     type=float, default=0.3,
                        help="推理帧间隔（秒）")
    parser.add_argument("--voxel-size",   type=float, default=0.02,
                        help="体素下采样尺寸（米），0 表示不下采样")
    parser.add_argument("--max-pts",      type=int,   default=5000,
                        help="每帧最大点云点数（下采样后）")
    parser.add_argument("--save-dir",     default="/workspace/pointclouds",
                        help="分割点云保存目录")
    parser.add_argument("--save-interval", type=float, default=3.0,
                        help="点云保存间隔（秒）")
    args = parser.parse_args()

    model = YOLO(args.model)
    print(f"[SEG] 模型加载完成: {args.model}")

    last_infer = 0.0
    last_save = 0.0
    latest_depth = {"msg": None}
    cam = {"fx": None, "fy": None, "cx": None, "cy": None}
    os.makedirs(args.save_dir, exist_ok=True)
    print(f"[SEG] 点云保存目录: {args.save_dir} (间隔 {args.save_interval}s)")

    ros = roslibpy.Ros(host=args.host, port=args.port)
    ros.run()
    print(f"[SEG] 已连接 rosbridge {args.host}:{args.port}")

    # 订阅话题
    sub_rgb   = roslibpy.Topic(ros, args.rgb,         "sensor_msgs/Image")
    sub_depth = roslibpy.Topic(ros, args.depth,       "sensor_msgs/Image")
    sub_info  = roslibpy.Topic(ros, args.camera_info, "sensor_msgs/CameraInfo")

    # 发布话题
    pub_objects = roslibpy.Topic(ros, "/perception/seg_objects",       "std_msgs/String")
    pub_centers = roslibpy.Topic(ros, "/perception/seg_center_points", "std_msgs/String")
    pub_cloud   = roslibpy.Topic(ros, "/perception/seg_cloud",         "sensor_msgs/PointCloud2")

    def on_camera_info(msg):
        if cam["fx"] is None:
            K = msg["K"]
            cam["fx"], cam["fy"] = K[0], K[4]
            cam["cx"], cam["cy"] = K[2], K[5]
            print(f"[SEG] 相机内参: fx={cam['fx']:.1f} fy={cam['fy']:.1f} "
                  f"cx={cam['cx']:.1f} cy={cam['cy']:.1f}")

    def on_depth(msg):
        latest_depth["msg"] = msg

    def on_rgb(msg):
        nonlocal last_infer, last_save
        now = time.time()
        if now - last_infer < args.interval or cam["fx"] is None:
            return
        last_infer = now

        # 解码图像
        try:
            frame = decode_image(msg)
        except Exception as e:
            print(f"[SEG] 图像解码失败: {e}")
            return

        depth = None
        if latest_depth["msg"] is not None:
            try:
                depth = decode_depth(latest_depth["msg"])
            except Exception:
                pass

        # YOLO 推理
        results = model(frame, verbose=False)
        if not results:
            return

        h, w = frame.shape[:2]
        fx, fy, cx, cy = cam["fx"], cam["fy"], cam["cx"], cam["cy"]

        objects_json = []   # 轻量 JSON 摘要
        centers_json = []   # 极轻量：只发 3D 中心点
        all_pts = []        # 完整点云
        all_lbls = []       # 对应 label id

        for result in results:
            if result.boxes is None:
                continue
            masks_data = result.masks

            for i, box in enumerate(result.boxes):
                conf = float(box.conf[0])
                if conf < args.conf:
                    continue

                cls_id = int(box.cls[0])
                label  = result.names[cls_id]
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                x1 = max(0, min(x1, w - 1))
                x2 = max(0, min(x2, w - 1))
                y1 = max(0, min(y1, h - 1))
                y2 = max(0, min(y2, h - 1))
                px_cx = int((x1 + x2) / 2)
                px_cy = int((y1 + y2) / 2)

                # ── 3D 中心点 ──
                dist = None
                if depth is not None:
                    try:
                        d = float(depth[px_cy, px_cx])
                        if d > 0.05:
                            dist = d
                    except Exception:
                        pass

                center_3d = None
                if dist is not None:
                    cx3 = (px_cx - cx) * dist / fx
                    cy3 = (px_cy - cy) * dist / fy
                    center_3d = [round(cx3, 3), round(cy3, 3), round(dist, 3)]

                # ── 掩码 → 点云 ──
                pts_3d = np.empty((0, 3), dtype=np.float32)
                if masks_data is not None and i < len(masks_data.data) and depth is not None:
                    mask_raw = masks_data.data[i].cpu().numpy()
                    mask_full = cv2.resize(mask_raw, (w, h), interpolation=cv2.INTER_NEAREST)
                    binary_mask = (mask_full > 0.5).astype(np.uint8)
                    pts_3d = mask_to_3d(binary_mask, depth, fx, fy, cx, cy)

                    # 体素下采样
                    if args.voxel_size > 0 and len(pts_3d) > 0:
                        pts_3d = voxel_downsample(pts_3d, voxel_size=args.voxel_size)

                    # 最大点数截断
                    if len(pts_3d) > args.max_pts:
                        idx = np.random.choice(len(pts_3d), args.max_pts, replace=False)
                        pts_3d = pts_3d[idx]

                    for pt in pts_3d:
                        all_pts.append(pt)
                        all_lbls.append(cls_id)

                # ── 记录结果 ──
                objects_json.append({
                    "label":           label,
                    "class_id":        cls_id,
                    "confidence":      round(conf, 2),
                    "bbox":            [int(x1), int(y1), int(x2), int(y2)],
                    "center_px":       [px_cx, px_cy],
                    "distance_m":      round(dist, 3) if dist is not None else None,
                    "center_3d":       center_3d,
                    "cloud_pts":       len(pts_3d),
                })

                if center_3d is not None:
                    centers_json.append({
                        "label":    label,
                        "class_id": cls_id,
                        "xyz":      center_3d,
                    })

                print(
                    f"[SEG] {label}({cls_id}) conf={conf:.2f} "
                    f"dist={'n/a' if dist is None else f'{dist:.2f}m'} "
                    f"center_3d={center_3d} cloud_pts={len(pts_3d)}"
                )

        # ── 发布 ──
        if objects_json:
            pub_objects.publish(roslibpy.Message({
                "data": json.dumps(objects_json, ensure_ascii=False)
            }))

        if centers_json:
            pub_centers.publish(roslibpy.Message({
                "data": json.dumps(centers_json, ensure_ascii=False)
            }))

        if all_pts:
            pc2_msg = build_pc2_msg(all_pts, all_lbls)
            pub_cloud.publish(roslibpy.Message(pc2_msg))

            # 保存到本地 pointclouds 目录
            if now - last_save >= args.save_interval:
                pts_arr = np.array(all_pts, dtype=np.float32)
                ts = time.strftime("%Y%m%d_%H%M%S")
                path = os.path.join(args.save_dir, f"seg_object_{ts}.pcd")
                save_pcd(pts_arr, path)
                last_save = now

    sub_info.subscribe(on_camera_info)
    sub_depth.subscribe(on_depth)
    sub_rgb.subscribe(on_rgb)

    print("[SEG] 等待数据，按 Ctrl+C 退出...")
    try:
        while ros.is_connected:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        sub_info.unsubscribe()
        sub_depth.unsubscribe()
        sub_rgb.unsubscribe()
        pub_objects.unadvertise()
        pub_centers.unadvertise()
        pub_cloud.unadvertise()
        ros.terminate()
        print("[SEG] 已退出。")


if __name__ == "__main__":
    main()

# 执行命令
# 1.启动gpu的docker容器
# docker compose up -d perception_yolo_gpu
# 2.docker exec -d perception_yolo bash 连接rosbridge
# d -c "source /opt/ros/noetic/setup.bash && export ROS_MASTER_URI=http://127.0.0.1:11311 && roslaunch rosbridge_server rosbridge_websocket.launch"
# 3. 运行yolo26_seg_pointcloud_node
# docker exec -it perception_yolo_gpu bash -c "python3.10 /workspace/src/perception_yolo/nodes/yolo26_seg_pointcloud_node.py --host 127.0.0.1 --port 9090 --model /workspace/weights/yolo/yolo26m-seg.pt --voxel-size 0.02 --max-pts 3000"