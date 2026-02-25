#!/usr/bin/env python3
"""
GraspNet Estimator Node
Author: Muye Yuan
Environment: CUDA 11.3 + ROS Noetic (GraspNet-1Billion legacy requirements)
"""

import rospy
import numpy as np
import torch
from cv_bridge import CvBridge
from collections import deque

from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from common_msgs.msg import GraspCandidate, DetectedObject
import sensor_msgs.point_cloud2 as pc2
import message_filters

try:
    import open3d as o3d
except ImportError:
    o3d = None
    rospy.logwarn("[Grasp] open3d not installed. Install with: pip install open3d")


class GraspNetAdapter:
    """GraspNet 推理适配层（当前为接口骨架）。"""

    def __init__(self, checkpoint_path, device):
        self.checkpoint_path = checkpoint_path
        self.device = device
        self.model = None

    def load(self):
        """加载 GraspNet 模型。

        预期实现（建议）：
        1. 从 graspnet-baseline 导入模型定义与推理器。
        2. 加载 checkpoint 并切换 eval 模式。
        3. 统一输出为 [(position_xyz, quaternion_xyzw, score), ...]。
        """
        rospy.logwarn("[Grasp] GraspNetAdapter.load() is a placeholder. Please integrate graspnet-baseline here.")

    def infer(self, points_xyz, colors_rgb=None):
        """执行推理并返回抓取候选。

        Args:
            points_xyz: np.ndarray, shape [N,3], 单位米（相机坐标系）
            colors_rgb: np.ndarray, shape [N,3], 范围[0,1]，可选
        """
        # 占位输出，后续应替换为真实模型输出
        if points_xyz is None or len(points_xyz) == 0:
            return []

        center = np.mean(points_xyz, axis=0)
        grasps = []
        for _ in range(10):
            pos = center + np.random.randn(3) * 0.03
            quat = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
            score = float(np.random.uniform(0.5, 0.95))
            grasps.append((pos, quat, score))
        return grasps


class GraspEstimatorNode:
    def __init__(self):
        rospy.init_node('grasp_estimator', anonymous=False)
        rospy.loginfo("=" * 60)
        rospy.loginfo("GraspNet Estimator Node Initializing")
        rospy.loginfo("=" * 60)

        # 设备检查（CUDA 11.3）
        self.device = self._setup_device()

        # 参数配置
        self.checkpoint_path = rospy.get_param('~checkpoint_path', 'graspnet_checkpoints/checkpoint.tar')
        self.num_grasp_candidates = rospy.get_param('~num_grasp_candidates', 5)
        self.pointcloud_topic = rospy.get_param('~pointcloud_topic', '/camera/depth/points')
        self.rgb_topic = rospy.get_param('~rgb_topic', '/camera/color/image_raw')
        self.camera_info_topic = rospy.get_param('~camera_info_topic', '/camera/color/camera_info')
        self.sync_slop = rospy.get_param('~sync_slop', 0.08)
        self.use_detection_roi = rospy.get_param('~use_detection_roi', True)

        # 加载 GraspNet 模型
        self.model = self._load_graspnet_model()

        # ROS 接口
        self.bridge = CvBridge()

        # 同步订阅 RGB + PointCloud + CameraInfo
        self.pc_sub = message_filters.Subscriber(self.pointcloud_topic, PointCloud2)
        self.rgb_sub = message_filters.Subscriber(self.rgb_topic, Image)
        self.cam_info_sub = message_filters.Subscriber(self.camera_info_topic, CameraInfo)

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.pc_sub, self.rgb_sub, self.cam_info_sub],
            queue_size=5,
            slop=self.sync_slop,
            allow_headerless=False,
        )
        self.sync.registerCallback(self.synced_callback)

        # 订阅检测结果（用 ROI 裁剪点云）
        self.detection_sub = rospy.Subscriber(
            '/perception/detected_objects',
            DetectedObject,
            self.detection_callback,
            queue_size=20
        )

        # 发布抓取候选
        self.grasp_pub = rospy.Publisher(
            '/perception/grasp_candidates',
            GraspCandidate,
            queue_size=10
        )

        self.detected_objects = deque(maxlen=50)

        rospy.loginfo(f"[Grasp] Device: {self.device}")
        rospy.loginfo(f"[Grasp] Checkpoint: {self.checkpoint_path}")
        rospy.loginfo(f"[Grasp] Subscribing pointcloud: {self.pointcloud_topic}")
        rospy.loginfo(f"[Grasp] Subscribing rgb: {self.rgb_topic}")
        rospy.loginfo(f"[Grasp] Subscribing camera_info: {self.camera_info_topic}")
        rospy.loginfo("[Grasp] Initialization complete. Ready to estimate grasps!")

    def _setup_device(self):
        """设置 CUDA 11.3 环境"""
        if torch.cuda.is_available():
            device = torch.device('cuda')
            cuda_version = torch.version.cuda
            rospy.loginfo(f"[Grasp] CUDA available: {torch.cuda.get_device_name(0)}")
            rospy.loginfo(f"[Grasp] CUDA version: {cuda_version}")

            if cuda_version != "11.3":
                rospy.logwarn(f"[Grasp] Expected CUDA 11.3, but got {cuda_version}. GraspNet may have compatibility issues.")
        else:
            device = torch.device('cpu')
            rospy.logwarn("[Grasp] CUDA not available. Running on CPU (NOT RECOMMENDED for GraspNet)")

        return device

    def _load_graspnet_model(self):
        """加载 GraspNet-1Billion 模型"""
        adapter = GraspNetAdapter(self.checkpoint_path, self.device)
        adapter.load()
        return adapter

    def detection_callback(self, msg):
        """接收检测结果，用于裁剪点云"""
        self.detected_objects.append((rospy.Time.now(), msg))
        rospy.logdebug(f"[Grasp] Received detection: {msg.label} score={msg.score:.3f}")

    def synced_callback(self, pointcloud_msg, rgb_msg, camera_info_msg):
        """处理同步的点云+RGB+内参数据并估计抓取姿态。"""
        rospy.loginfo_throttle(2.0, "[Grasp] Processing synchronized point cloud + RGB...")

        try:
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            points_xyz, points_rgb = self._ros_pointcloud_to_numpy(pointcloud_msg, rgb_image)

            if points_xyz is None or len(points_xyz) == 0:
                rospy.logwarn("[Grasp] Empty point cloud received")
                return

            if self.use_detection_roi and len(self.detected_objects) > 0:
                points_xyz, points_rgb = self._filter_points_by_latest_roi(
                    points_xyz,
                    points_rgb,
                    pointcloud_msg,
                    camera_info_msg,
                )

            processed_points = self._preprocess_pointcloud(points_xyz)
            if processed_points is None or len(processed_points) == 0:
                rospy.logwarn("[Grasp] No valid points after preprocessing")
                return

            grasp_outputs = self.model.infer(processed_points, points_rgb)

            for i, (position, orientation, quality) in enumerate(grasp_outputs[:self.num_grasp_candidates]):
                pose = Pose()
                pose.position = Point(x=float(position[0]), y=float(position[1]), z=float(position[2]))
                pose.orientation = Quaternion(
                    x=float(orientation[0]),
                    y=float(orientation[1]),
                    z=float(orientation[2]),
                    w=float(orientation[3]),
                )
                self._publish_grasp_candidate(pose, quality, pointcloud_msg.header.frame_id)
                rospy.loginfo(f"[Grasp] Candidate {i + 1}: quality={quality:.3f}")

        except Exception as e:
            rospy.logerr(f"[Grasp] Error processing synchronized data: {e}")

    def _ros_pointcloud_to_numpy(self, msg, rgb_image=None):
        """将 ROS PointCloud2 转换为 NumPy 数组，可选关联 RGB 颜色。"""
        points_xyz = []
        points_rgb = []

        for point in pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z")):
            x, y, z = float(point[0]), float(point[1]), float(point[2])
            points_xyz.append([x, y, z])

            if rgb_image is not None:
                points_rgb.append([0.5, 0.5, 0.5])

        xyz = np.asarray(points_xyz, dtype=np.float32)
        rgb = np.asarray(points_rgb, dtype=np.float32) if points_rgb else None
        return xyz, rgb

    def _filter_points_by_latest_roi(self, points_xyz, points_rgb, pointcloud_msg, camera_info_msg):
        """根据最近检测到的 ROI 进行点云筛选。

        当前实现采用深度范围裁剪作为保守 fallback。
        如果点云是有组织点云，可进一步使用 u/v 与 ROI 精准映射。
        """
        _, det = self.detected_objects[-1]
        roi = det.roi

        # 基础健壮性检查
        if roi.width <= 0 or roi.height <= 0:
            return points_xyz, points_rgb

        # 简化版：先按深度进行粗裁剪（聚焦桌面目标）
        z = points_xyz[:, 2]
        valid = np.logical_and(z > 0.1, z < 1.5)

        filtered_xyz = points_xyz[valid]
        filtered_rgb = points_rgb[valid] if points_rgb is not None and len(points_rgb) == len(points_xyz) else points_rgb

        rospy.logdebug(
            "[Grasp] ROI(%d,%d,%d,%d) depth-filter: %d -> %d",
            roi.x_offset,
            roi.y_offset,
            roi.width,
            roi.height,
            len(points_xyz),
            len(filtered_xyz),
        )
        return filtered_xyz, filtered_rgb

    def _preprocess_pointcloud(self, points):
        """预处理点云（下采样、去噪等）"""
        if len(points) == 0:
            return points

        if o3d is None:
            return points

        try:
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)

            pcd = pcd.voxel_down_sample(voxel_size=0.005)
            pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

            return np.asarray(pcd.points)
        except Exception as e:
            rospy.logwarn(f"[Grasp] Preprocessing failed: {e}. Using raw points.")
            return points

    def _publish_grasp_candidate(self, pose, quality, frame_id):
        """发布单个抓取候选"""
        msg = GraspCandidate()

        msg.pose = PoseStamped()
        msg.pose.header.stamp = rospy.Time.now()
        msg.pose.header.frame_id = frame_id
        msg.pose.pose = pose

        msg.quality = float(quality)

        self.grasp_pub.publish(msg)

    def run(self):
        """保持节点运行"""
        rospy.spin()


if __name__ == '__main__':
    try:
        node = GraspEstimatorNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("[Grasp] Shutting down")
