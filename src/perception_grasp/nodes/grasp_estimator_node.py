#!/usr/bin/env python3
"""
GraspNet Estimator Node
Author: Muye Yuan
Environment: CUDA 11.3 + ROS Noetic (GraspNet-1Billion legacy requirements)
"""

import os
import sys

import rospy
import numpy as np
import torch
from cv_bridge import CvBridge, CvBridgeError
from scipy.spatial.transform import Rotation

from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import String
from std_msgs.msg import ColorRGBA
from common_msgs.msg import GraspCandidate, DetectedObject, TaskDecision
import sensor_msgs.point_cloud2 as pc2
from visualization_msgs.msg import Marker, MarkerArray

NODE_DIR = os.path.dirname(os.path.abspath(__file__))
if NODE_DIR not in sys.path:
    sys.path.insert(0, NODE_DIR)

from grasp_inference_core import GraspNetInferenceCore, filter_grasp_candidates_by_approach

try:
    import open3d as o3d
except ImportError:
    rospy.logwarn("[Grasp] open3d not installed. Install with: pip install open3d")


class GraspEstimatorNode:
    def __init__(self):
        rospy.init_node('grasp_estimator', anonymous=False)
        rospy.loginfo("=" * 60)
        rospy.loginfo("GraspNet Estimator Node Initializing")
        rospy.loginfo("=" * 60)
        
        # 设备检查（CUDA 11.3）
        self.device = self._setup_device()
        
        # 参数配置
        self.checkpoint_path = rospy.get_param('~checkpoint_path', '/workspace/weights/graspnet/checkpoint.tar')
        self.raw_top_k = int(rospy.get_param('~raw_top_k', 1000))
        self.num_grasp_candidates = rospy.get_param('~num_grasp_candidates', 5)
        self.pointcloud_topic = rospy.get_param('~pointcloud_topic', '/perception/yolo26_seg_cloud')
        self.graspnet_repo_path = rospy.get_param('~graspnet_repo_path', '/workspace/weights/graspnet/graspnet-baseline')
        self.num_point = rospy.get_param('~num_point', 20000)
        self.voxel_size = rospy.get_param('~voxel_size', 0.005)
        self.label_field_name = rospy.get_param('~label_field_name', 'label')
        self.task_decision_topic = rospy.get_param('~task_decision_topic', '/brain/task_decision')
        self.current_target_object_id = rospy.get_param('~target_object_id', -1)
        self.failure_topic = rospy.get_param('~failure_topic', '/perception/grasp_failure_reason')
        self.marker_topic = rospy.get_param('~marker_topic', '/perception/grasp_markers')
        self.approach_filter = rospy.get_param('~approach_filter', 'top_side')
        self.min_down_dot = float(rospy.get_param('~min_down_dot', 0.25))
        self.max_up_dot = float(rospy.get_param('~max_up_dot', 0.2))
        self.min_gripper_width = float(rospy.get_param('~min_gripper_width', 0.0178))
        self.max_gripper_width = float(rospy.get_param('~max_gripper_width', 0.1006))
        
        # 加载 GraspNet 模型
        self.inference_core = self._load_graspnet_model()
        
        # ROS 接口
        self.bridge = CvBridge()
        
        # 订阅点云数据
        self.pointcloud_sub = rospy.Subscriber(
            self.pointcloud_topic,
            PointCloud2,
            self.pointcloud_callback,
            queue_size=1,
            buff_size=2**24
        )
        
        # 订阅检测结果（可选：根据检测区域裁剪点云）
        self.detection_sub = rospy.Subscriber(
            '/perception/detected_objects',
            DetectedObject,
            self.detection_callback,
            queue_size=10
        )

        # 订阅 FSM 当前抓取目标（如果有）
        self.task_decision_sub = rospy.Subscriber(
            self.task_decision_topic,
            TaskDecision,
            self.task_decision_callback,
            queue_size=10
        )
        
        # 发布抓取候选
        self.grasp_pub = rospy.Publisher(
            '/perception/grasp_candidates',
            GraspCandidate,
            queue_size=10
        )

        self.failure_pub = rospy.Publisher(
            self.failure_topic,
            String,
            queue_size=10
        )

        self.marker_pub = rospy.Publisher(
            self.marker_topic,
            MarkerArray,
            queue_size=10
        )
        
        self.detected_objects = []
        self.latest_points = None
        self.latest_labels = None
        self.latest_frame_id = ""
        self.latest_stamp = None
        self.pending_grasp_request = False
        self.is_processing_request = False
        
        rospy.loginfo(f"[Grasp] Device: {self.device}")
        rospy.loginfo(f"[Grasp] Checkpoint: {self.checkpoint_path}")
        rospy.loginfo(f"[Grasp] Raw top-k: {self.raw_top_k}")
        rospy.loginfo(f"[Grasp] Subscribing to: {self.pointcloud_topic}")
        rospy.loginfo(f"[Grasp] TaskDecision topic: {self.task_decision_topic}")
        rospy.loginfo(f"[Grasp] Failure topic: {self.failure_topic}")
        rospy.loginfo(f"[Grasp] Marker topic: {self.marker_topic}")
        rospy.loginfo(
            f"[Grasp] Approach filter: mode={self.approach_filter}, "
            f"min_down_dot={self.min_down_dot}, max_up_dot={self.max_up_dot}"
        )
        rospy.loginfo(
            f"[Grasp] Gripper width limits: min={self.min_gripper_width:.4f} m, "
            f"max={self.max_gripper_width:.4f} m"
        )
        rospy.loginfo(f"[Grasp] Initial target_object_id: {self.current_target_object_id}")
        rospy.loginfo("[Grasp] Initialization complete. Ready to estimate grasps!")
        
    def _setup_device(self):
        """设置 CUDA 11.3 环境"""
        if torch.cuda.is_available():
            device = torch.device('cuda')
            cuda_version = torch.version.cuda
            rospy.loginfo(f"[Grasp] CUDA available: {torch.cuda.get_device_name(0)}")
            rospy.loginfo(f"[Grasp] CUDA version: {cuda_version}")
            
            # 警告：如果 CUDA 版本不是 11.3
            if cuda_version != "11.3":
                rospy.logwarn(f"[Grasp] Expected CUDA 11.3, but got {cuda_version}. GraspNet may have compatibility issues.")
        else:
            device = torch.device('cpu')
            rospy.logwarn("[Grasp] CUDA not available. Running on CPU (NOT RECOMMENDED for GraspNet)")
            
        return device
        
    def _load_graspnet_model(self):
        """加载 GraspNet-1Billion 模型"""
        rospy.loginfo("[Grasp] Loading GraspNet model...")
        core = GraspNetInferenceCore(
            checkpoint_path=self.checkpoint_path,
            device=str(self.device),
            num_point=self.num_point,
            voxel_size=self.voxel_size,
            graspnet_repo_path=self.graspnet_repo_path if self.graspnet_repo_path else None
        )
        # Force one-time model loading at node startup.
        core._build_model()
        rospy.loginfo("[Grasp] GraspNet model loaded successfully")
        return core
        
    def detection_callback(self, msg):
        """接收检测结果，用于裁剪点云"""
        self.detected_objects.append(msg)
        rospy.logdebug(f"[Grasp] Received detection: {msg.label}")

    def task_decision_callback(self, msg):
        """接收 FSM 当前抓取目标，并触发一次抓取请求"""
        if msg.task_type == TaskDecision.GRASP and msg.target_object_id >= 0:
            self.current_target_object_id = msg.target_object_id
            self.pending_grasp_request = True
            rospy.loginfo(
                f"[Grasp] Received grasp request from FSM, target_object_id(label_id)="
                f"{self.current_target_object_id}"
            )
            self._try_process_pending_request(trigger_source="fsm")
        else:
            rospy.logdebug(
                f"[Grasp] Ignoring TaskDecision task_type={msg.task_type}, "
                f"target_object_id={msg.target_object_id}"
            )
        
    def pointcloud_callback(self, msg):
        """持续缓存最新语义点云，在存在待处理请求时触发抓取"""
        try:
            points, labels = self._ros_pointcloud_to_numpy(msg)
            if points is None or len(points) == 0:
                rospy.logwarn("[Grasp] Empty point cloud received")
                return

            self.latest_points = points
            self.latest_labels = labels
            self.latest_frame_id = msg.header.frame_id
            self.latest_stamp = msg.header.stamp
            rospy.logdebug(f"[Grasp] Cached semantic point cloud with {len(points)} points")

            if self.pending_grasp_request:
                self._try_process_pending_request(trigger_source="pointcloud_update")
        except Exception as e:
            rospy.logerr(f"[Grasp] Error processing point cloud: {e}")

    def _try_process_pending_request(self, trigger_source):
        """在有缓存点云且收到抓取请求时执行一次抓取"""
        if not self.pending_grasp_request:
            return
        if self.is_processing_request:
            rospy.logwarn("[Grasp] Grasp request already in progress, skipping duplicate trigger")
            return
        if self.latest_points is None or len(self.latest_points) == 0:
            rospy.logwarn("[Grasp] No cached semantic point cloud yet; waiting for next point cloud")
            return

        self.is_processing_request = True
        try:
            self._execute_grasp_request(trigger_source)
            self.pending_grasp_request = False
        except Exception as e:
            self._publish_failure_reason(f"Grasp request failed: {e}")
            rospy.logerr(f"[Grasp] Error executing grasp request: {e}")
        finally:
            self.is_processing_request = False

    def _execute_grasp_request(self, trigger_source):
        """使用缓存语义点云执行一次抓取请求"""
        rospy.loginfo(
            f"[Grasp] Processing grasp request from {trigger_source}, "
            f"target_object_id(label_id)={self.current_target_object_id}"
        )

        points = np.array(self.latest_points, copy=True)
        labels = None if self.latest_labels is None else np.array(self.latest_labels, copy=True)
        frame_id = self.latest_frame_id or "camera_depth_optical_frame"

        points = self._filter_points_by_target(points, labels)
        if points is None or len(points) == 0:
            self._publish_failure_reason(
                f"No points matched target_object_id(label_id)={self.current_target_object_id}"
            )
            self._clear_grasp_markers(frame_id)
            return

        processed_points = self._preprocess_pointcloud(points)
        if processed_points is None or len(processed_points) == 0:
            self._publish_failure_reason("No valid points left after point cloud preprocessing")
            self._clear_grasp_markers(frame_id)
            return

        raw_grasp_results = self._estimate_grasps(processed_points)
        if not raw_grasp_results:
            self._publish_failure_reason("GraspNet returned no grasp candidates")
            self._clear_grasp_markers(frame_id)
            return

        grasp_results = filter_grasp_candidates_by_approach(
            raw_grasp_results,
            mode=self.approach_filter,
            min_down_dot=self.min_down_dot,
            max_up_dot=self.max_up_dot,
        )
        rospy.loginfo(
            f"[Grasp] Raw grasp candidates: {len(raw_grasp_results)}, "
            f"filtered candidates: {len(grasp_results)}"
        )
        if not grasp_results:
            self._publish_failure_reason(
                "No grasp candidates remained after approach-direction filtering"
            )
            self._clear_grasp_markers(frame_id)
            return

        grasp_results = self._filter_grasps_by_width(grasp_results)
        rospy.loginfo(
            f"[Grasp] Width-filtered candidates: {len(grasp_results)} "
            f"(min={self.min_gripper_width:.4f}, max={self.max_gripper_width:.4f})"
        )
        if not grasp_results:
            self._publish_failure_reason(
                "No grasp candidates remained after gripper-width filtering"
            )
            self._clear_grasp_markers(frame_id)
            return

        self._publish_grasp_markers(grasp_results[:self.num_grasp_candidates], frame_id)
        for i, result in enumerate(grasp_results[:self.num_grasp_candidates]):
            self._publish_grasp_candidate(result["pose"], result["quality"], frame_id)
            rospy.loginfo(
                f"[Grasp] Candidate {i+1}: quality={result['quality']:.3f}, width={result['width']:.3f}"
            )

    def _publish_failure_reason(self, reason):
        """发布抓取失败原因，供 FSM/调试侧消费"""
        rospy.logwarn(f"[Grasp] {reason}")
        self.failure_pub.publish(String(data=reason))

    def _filter_grasps_by_width(self, grasp_results):
        """按真实夹爪开合宽度范围过滤候选"""
        filtered = []
        for result in grasp_results:
            width = float(result.get("width", 0.0))
            if self.min_gripper_width <= width <= self.max_gripper_width:
                filtered.append(result)
        return filtered
            
    def _ros_pointcloud_to_numpy(self, msg):
        """将 ROS PointCloud2 转换为 NumPy 数组，并尽量读取 label 字段"""
        field_names = [field.name for field in msg.fields]
        label_field = None
        if self.label_field_name in field_names:
            label_field = self.label_field_name
        elif "l" in field_names:
            label_field = "l"

        points_list = []
        labels_list = []

        if label_field is not None:
            read_fields = ("x", "y", "z", label_field)
            for point in pc2.read_points(msg, skip_nans=True, field_names=read_fields):
                points_list.append([point[0], point[1], point[2]])
                labels_list.append(int(point[3]))
            labels = np.array(labels_list, dtype=np.int32)
            rospy.logdebug(f"[Grasp] Point cloud contains label field '{label_field}'")
        else:
            for point in pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z")):
                points_list.append([point[0], point[1], point[2]])
            labels = None
            rospy.logdebug("[Grasp] Point cloud does not contain label field; using all points")
            
        return np.array(points_list, dtype=np.float32), labels

    def _filter_points_by_target(self, points, labels):
        """按当前 target_object_id 过滤点云；如果没有目标或没有 labels，则返回原点云"""
        if labels is None:
            return points
        if self.current_target_object_id is None or self.current_target_object_id < 0:
            rospy.logdebug("[Grasp] No target_object_id set; using all labeled points")
            return points

        mask = labels == int(self.current_target_object_id)
        matched = int(np.count_nonzero(mask))
        if matched == 0:
            return np.empty((0, 3), dtype=np.float32)

        rospy.loginfo(
            f"[Grasp] Filtered point cloud by target_object_id={self.current_target_object_id}: "
            f"{matched}/{len(points)} points kept"
        )
        return points[mask]
        
    def _preprocess_pointcloud(self, points):
        """预处理点云（下采样、去噪等）"""
        if len(points) == 0:
            return points
            
        # 使用 Open3D 进行预处理
        try:
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            
            # 下采样
            pcd = pcd.voxel_down_sample(voxel_size=self.voxel_size)
            
            # 统计滤波去除离群点
            pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
            
            return np.asarray(pcd.points)
        except Exception as e:
            rospy.logwarn(f"[Grasp] Preprocessing failed: {e}. Using raw points.")
            return points
            
    def _estimate_grasps(self, points):
        """
        使用 GraspNet 估计抓取姿态
        返回: [{"pose": pose, "quality": quality, "width": width}, ...]
        """
        if self.inference_core is None:
            rospy.logerr("[Grasp] Inference core is not available")
            return []

        results = self.inference_core.predict(points, colors=None, top_k=self.raw_top_k)
        parsed = []
        for item in results:
            translation = np.asarray(item["translation"], dtype=np.float32).reshape(3)
            rotation_matrix = np.asarray(item["rotation"], dtype=np.float32).reshape(3, 3)
            quat = Rotation.from_matrix(rotation_matrix).as_quat()  # x,y,z,w

            pose = Pose()
            pose.position = Point(x=float(translation[0]), y=float(translation[1]), z=float(translation[2]))
            pose.orientation = Quaternion(
                x=float(quat[0]),
                y=float(quat[1]),
                z=float(quat[2]),
                w=float(quat[3]),
            )
            parsed.append(
                {
                    "pose": pose,
                    "quality": float(item["score"]),
                    "width": float(item.get("width", 0.02)),
                    "translation": translation,
                    "rotation": rotation_matrix,
                }
            )
        return parsed

    def _make_gripper_part(self, marker_id, frame_id, pose, local_offset, scale_xyz, color):
        quat = np.array(
            [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ],
            dtype=np.float32,
        )
        rotation_matrix = Rotation.from_quat(quat).as_matrix()
        translation = np.array(
            [pose.position.x, pose.position.y, pose.position.z],
            dtype=np.float32,
        )
        world_position = translation + rotation_matrix @ local_offset.astype(np.float32)

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "online_gripper"
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position = Point(
            x=float(world_position[0]),
            y=float(world_position[1]),
            z=float(world_position[2]),
        )
        marker.pose.orientation = pose.orientation
        marker.scale.x = float(scale_xyz[0])
        marker.scale.y = float(scale_xyz[1])
        marker.scale.z = float(scale_xyz[2])
        marker.color = color
        return marker

    def _publish_grasp_markers(self, grasp_results, frame_id):
        marker_array = MarkerArray()
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        max_quality = max((result["quality"] for result in grasp_results), default=1.0)
        for index, result in enumerate(grasp_results):
            quality = result["quality"]
            width = max(float(result["width"]), 0.005)
            color_ratio = 0.0 if max_quality <= 0 else min(max(quality / max_quality, 0.0), 1.0)
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

            palm_offset = np.array([-0.02, 0.0, 0.0], dtype=np.float32)
            left_finger_offset = np.array([0.01, finger_gap, 0.0], dtype=np.float32)
            right_finger_offset = np.array([0.01, -finger_gap, 0.0], dtype=np.float32)

            marker_array.markers.append(
                self._make_gripper_part(
                    base_id,
                    frame_id,
                    result["pose"],
                    palm_offset,
                    (palm_length, palm_width, palm_thickness),
                    color,
                )
            )
            marker_array.markers.append(
                self._make_gripper_part(
                    base_id + 1,
                    frame_id,
                    result["pose"],
                    left_finger_offset,
                    (finger_length, finger_width, finger_thickness),
                    color,
                )
            )
            marker_array.markers.append(
                self._make_gripper_part(
                    base_id + 2,
                    frame_id,
                    result["pose"],
                    right_finger_offset,
                    (finger_length, finger_width, finger_thickness),
                    color,
                )
            )

        self.marker_pub.publish(marker_array)

    def _clear_grasp_markers(self, frame_id):
        marker_array = MarkerArray()
        clear_marker = Marker()
        clear_marker.header.frame_id = frame_id
        clear_marker.header.stamp = rospy.Time.now()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)
        self.marker_pub.publish(marker_array)
        
    def _publish_grasp_candidate(self, pose, quality, frame_id):
        """发布单个抓取候选"""
        msg = GraspCandidate()
        
        msg.pose = PoseStamped()
        msg.pose.header.stamp = rospy.Time.now()
        msg.pose.header.frame_id = frame_id
        msg.pose.pose = pose
        
        msg.quality = quality
        
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
