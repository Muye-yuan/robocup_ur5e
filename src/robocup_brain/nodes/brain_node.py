#!/usr/bin/env python3
"""
RoboCup Brain Node - behavior-tree based task orchestration.

Current pick and place flow:
  1. Move to the overview joint pose once.
  2. Open gripper once.
  3. Pick the best YOLO target that is not blacklisted.
  4. Build a direct grasp pose from YOLO 3D position.
  5. Try path_planning first to approach the grasp pose.
  6. If path_planning fails, fall back to direct motion_control MOVE_TO_POSE.
  7. Close the gripper without blocking on holding verification.
  8. Return to the overview joint pose.
  9. Move to the selected trash bin joint pose and release.
"""

import json
import re

import actionlib
import actionlib_msgs.msg as action_msgs
import py_trees
import py_trees_ros
import rospy
import tf2_geometry_msgs
import tf2_ros
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion
from py_trees.common import Status
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf.transformations import quaternion_from_euler

from common_msgs.msg import (
    GraspResult,
    MotionCommand,
    PlanExecutePoseAction,
    PlanExecutePoseFeedback,
    PlanExecutePoseGoal,
    PlanExecutePoseResult,
)


FAILED_TARGET_KEYS_BB = "failed_target_keys"
TARGET_BB_KEYS = (
    "target_object",
    "target_point_base_link",
    "target_grasp_pose",
    "target_grasp_mode",
    "target_key",
    "target_score",
    "target_confidence",
)

OVERVIEW_JOINTS = [-0.0278, -0.0011, 0.0000, -0.3441, 0.0140, -0.0034]


def is_test_mode():
    return rospy.get_param("~test_mode", False)


def get_blackboard():
    return py_trees.blackboard.Blackboard()


def blackboard_get(key, default=None):
    try:
        value = get_blackboard().get(key)
    except Exception:
        return default
    return default if value is None else value


def log_stage(stage_name):
    rospy.loginfo("[Brain] Stage: %s", stage_name)


def clear_target_selection():
    blackboard = get_blackboard()
    for key in TARGET_BB_KEYS:
        blackboard.set(key, None)


def clear_terminal_failure():
    blackboard = get_blackboard()
    blackboard.set("task_terminal_failure", False)
    blackboard.set("task_terminal_failure_reason", "")
    blackboard.set("task_complete_no_targets", False)
    blackboard.set("task_complete_no_targets_reason", "")


def set_terminal_failure(reason):
    blackboard = get_blackboard()
    blackboard.set("task_terminal_failure", True)
    blackboard.set("task_terminal_failure_reason", reason)


def set_no_targets_complete(reason):
    blackboard = get_blackboard()
    blackboard.set("task_complete_no_targets", True)
    blackboard.set("task_complete_no_targets_reason", reason)
    blackboard.set("task_terminal_failure", False)
    blackboard.set("task_terminal_failure_reason", "")


def get_failed_target_keys():
    values = blackboard_get(FAILED_TARGET_KEYS_BB, [])
    return set(str(item) for item in values)


def set_failed_target_keys(keys):
    get_blackboard().set(FAILED_TARGET_KEYS_BB, sorted(str(item) for item in keys))


def build_target_key(label, point_base):
    return "{label}:{x:.2f}:{y:.2f}:{z:.2f}".format(
        label=str(label).lower(),
        x=float(point_base.point.x),
        y=float(point_base.point.y),
        z=float(point_base.point.z),
    )


def normalize_label(label):
    return str(label).strip().lower()


def rotate_vector_by_quaternion(quat, vector):
    x = quat.x
    y = quat.y
    z = quat.z
    w = quat.w
    norm = (x * x + y * y + z * z + w * w) ** 0.5
    if norm < 1e-9:
        x, y, z, w = 0.0, 0.0, 0.0, 1.0
    else:
        x /= norm
        y /= norm
        z /= norm
        w /= norm

    vx = vector[0]
    vy = vector[1]
    vz = vector[2]

    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)

    rx = vx + w * tx + (y * tz - z * ty)
    ry = vy + w * ty + (z * tx - x * tz)
    rz = vz + w * tz + (x * ty - y * tx)
    return rx, ry, rz


def blacklist_current_target(reason):
    blackboard = get_blackboard()
    target_key = blackboard_get("target_key")
    target_object = blackboard_get("target_object", {}) or {}
    target_label = (
        target_object.get("name", "unknown")
        if isinstance(target_object, dict)
        else str(target_object)
    )

    if target_key:
        failed = get_failed_target_keys()
        failed.add(str(target_key))
        set_failed_target_keys(failed)

    rospy.logwarn(
        "[Brain] Target failed, switching to another candidate | target=%s reason=%s",
        target_label,
        reason,
    )
    blackboard.set("last_target_failure_reason", reason)
    clear_target_selection()


def prepare_retry_from_overview(reason, blacklist=False):
    blackboard = get_blackboard()
    if blacklist:
        blacklist_current_target(reason)
    else:
        rospy.logwarn("[Brain] Recovery requested | reason=%s", reason)
        blackboard.set("last_target_failure_reason", reason)
        clear_target_selection()

    blackboard.set("overview_done", False)
    blackboard.set("holding_object", False)
    blackboard.set("executed_trajectory", None)
    blackboard.set("task_retry_requested", True)
    blackboard.set("task_retry_reason", reason)
    clear_terminal_failure()


def prepare_next_pick_cycle():
    blackboard = get_blackboard()
    blackboard.set("overview_done", False)
    blackboard.set("holding_object", False)
    blackboard.set("executed_trajectory", None)
    blackboard.set("placed_bin_color", "")
    blackboard.set("last_target_failure_reason", "")
    blackboard.set("task_retry_requested", False)
    blackboard.set("task_retry_reason", "")
    set_failed_target_keys([])
    clear_target_selection()
    clear_terminal_failure()


class MoveToOverviewBehavior(py_trees.behaviour.Behaviour):
    """Move to the fixed overview joint pose once, then open the gripper once."""

    def __init__(self, name="MoveToOverview"):
        super().__init__(name)
        self.motion_cmd_pub = rospy.Publisher("/motion/command", MotionCommand, queue_size=10)
        self.motion_result_sub = rospy.Subscriber(
            "/motion/result", GraspResult, self._motion_result_callback
        )
        self.gripper_cmd_pub = rospy.Publisher("/gripper/command", String, queue_size=1)
        self.yolo_enable_topic = rospy.get_param(
            "~yolo_enable_topic",
            "/perception/yolo26_seg_enabled",
        )
        self.yolo_enable_pub = rospy.Publisher(
            self.yolo_enable_topic,
            Bool,
            queue_size=1,
            latch=True,
        )
        self.gripper_release_srv = rospy.ServiceProxy("/gripper/release", Trigger)
        self.clear_target_pose_srv = rospy.ServiceProxy("/path_planning/clear_target_pose", Trigger)
        self.command_sent = False
        self.last_result = None
        self.release_done = False

    def _release_response_usable(self, message):
        text = str(message or "")
        lowered = text.lower()
        if "partial-open" in lowered or "accepted as partial-open" in lowered:
            return True

        min_closed_error = float(rospy.get_param("~release_min_closed_error", 0.25))
        match = re.search(r"goal error\s+([-\d\.eE]+)", text)
        if match is None:
            return False
        try:
            goal_error = float(match.group(1))
        except ValueError:
            return False
        return abs(goal_error) >= min_closed_error

    def _motion_result_callback(self, msg):
        self.last_result = msg

    def initialise(self):
        if blackboard_get("overview_done", False):
            return
        self.command_sent = False
        self.last_result = None
        self.release_done = False
        log_stage("MoveToOverview")
        self.yolo_enable_pub.publish(Bool(data=False))
        rospy.loginfo("[Brain] Stage action: disable YOLO perception")
        self._clear_path_planning_target_pose()

    def _clear_path_planning_target_pose(self):
        try:
            rospy.wait_for_service("/path_planning/clear_target_pose", timeout=0.5)
            response = self.clear_target_pose_srv()
            if response.success:
                rospy.loginfo("[Brain] Stage action: cleared stale prm_target_pose")
            else:
                rospy.logwarn(
                    "[Brain] Failed to clear prm_target_pose before detection: %s",
                    response.message,
                )
        except (rospy.ROSException, rospy.ServiceException):
            rospy.logwarn_throttle(
                5.0,
                "[Brain] /path_planning/clear_target_pose unavailable; continuing without clearing target pose",
            )

    def _open_gripper(self):
        try:
            rospy.wait_for_service("/gripper/release", timeout=0.5)
            response = self.gripper_release_srv()
            if response.success:
                rospy.loginfo("[Brain] Stage action: open gripper at poseToTakePics")
                return True
            if self._release_response_usable(response.message):
                rospy.logwarn(
                    "[Brain] Open gripper accepted as usable even though release returned failure: %s",
                    response.message,
                )
                return True
            rospy.logerr("[Brain] Failed to open gripper at poseToTakePics: %s", response.message)
            return False
        except (rospy.ROSException, rospy.ServiceException):
            if self.gripper_cmd_pub.get_num_connections() > 0:
                self.gripper_cmd_pub.publish(String(data="release"))
                rospy.logwarn(
                    "[Brain] /gripper/release service unavailable, published release command instead"
                )
                return True
            rospy.logerr("[Brain] Gripper release interface unavailable")
            return False

    def update(self):
        blackboard = get_blackboard()
        if blackboard_get("overview_done", False):
            return Status.SUCCESS

        if self.motion_cmd_pub.get_num_connections() == 0:
            return Status.RUNNING

        if not self.command_sent:
            clear_terminal_failure()
            cmd = MotionCommand()
            cmd.command_type = MotionCommand.MOVE_TO_JOINT
            cmd.joint_positions = OVERVIEW_JOINTS
            cmd.max_velocity = 1.0
            cmd.max_acceleration = 1.0
            cmd.collision_check = True
            self.motion_cmd_pub.publish(cmd)
            rospy.loginfo(
                "[Brain] Stage action: move to overview joints [%s]",
                ", ".join(f"{joint:.4f}" for joint in OVERVIEW_JOINTS),
            )
            self.command_sent = True
            return Status.RUNNING

        if self.last_result is None:
            return Status.RUNNING

        if self.last_result.status != GraspResult.SUCCESS:
            reason = self.last_result.message or "poseToTakePics motion failed"
            rospy.logerr("[Brain] Overview move failed: %s", reason)
            set_terminal_failure(reason)
            return Status.FAILURE

        if not self.release_done:
            if not self._open_gripper():
                reason = "Failed to open gripper at poseToTakePics"
                set_terminal_failure(reason)
                return Status.FAILURE
            self.release_done = True

        blackboard.set("overview_done", True)
        rospy.loginfo("[Brain] Stage complete: MoveToOverview")
        return Status.SUCCESS


class EvaluateTargetsBehavior(py_trees.behaviour.Behaviour):
    """Receive YOLO detections, transform them, and pick the best non-blacklisted target."""

    def __init__(self, name="EvaluateTargets"):
        super().__init__(name)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.latest_detections = []
        self.latest_frame_id = ""
        self.latest_stamp = rospy.Time(0)
        self.sub = None
        self.mock_done = False
        self.yolo_enable_topic = rospy.get_param(
            "~yolo_enable_topic",
            "/perception/yolo26_seg_enabled",
        )
        self.yolo_enable_pub = rospy.Publisher(
            self.yolo_enable_topic,
            Bool,
            queue_size=1,
            latch=True,
        )
        self.target_pose_preview_pub = rospy.Publisher(
            "/path_planning/target_pose_preview",
            PoseStamped,
            queue_size=1,
            latch=True,
        )
        self.wait_start_time = None

    def setup(self, timeout=None):
        if is_test_mode():
            return True
        self.sub = rospy.Subscriber(
            "/perception/yolo26_seg_detections",
            String,
            self._yolo_callback,
        )
        return True

    def _yolo_callback(self, msg):
        try:
            detections = json.loads(msg.data)
        except (TypeError, ValueError):
            return

        if not isinstance(detections, list):
            return

        self.latest_detections = detections
        blackboard = get_blackboard()
        blackboard.set("detected_objects", list(detections))

        if not detections:
            return

        first = detections[0]
        self.latest_frame_id = str(first.get("frame_id", "")) or rospy.get_param(
            "~perception_frame_id", "camera_rgb_optical_frame"
        )
        stamp_dict = first.get("gazebo_stamp", {})
        secs = int(stamp_dict.get("secs", 0))
        nsecs = int(stamp_dict.get("nsecs", 0))
        self.latest_stamp = (
            rospy.Time(secs=secs, nsecs=nsecs) if (secs or nsecs) else rospy.Time(0)
        )

    def _score_object(self, obj_label):
        score_map = {
            "cube": 1000,
            "green_cube": 100,
            "purple_cube": 100,
            "red_can": 80,
            "red_bottle": 80,
            "yellow_can": 60,
            "spam": 60,
            "green_can": 40,
            "blue_bottle": 40,
        }
        return score_map.get(normalize_label(obj_label), 1)

    def _canonical_label(self, obj_label):
        label = normalize_label(obj_label)
        cube_aliases = {
            normalize_label(item)
            for item in rospy.get_param(
                "~cube_class_labels",
                ["class_1", "class_3", "class_5", "class_7"],
            )
        }
        if label in cube_aliases:
            return "cube"
        return label

    def initialise(self):
        self.mock_done = False
        self.latest_detections = []
        self.latest_frame_id = ""
        self.latest_stamp = rospy.Time(0)
        self.wait_start_time = rospy.Time.now()
        clear_target_selection()
        log_stage("EvaluateTargets")
        self.yolo_enable_pub.publish(Bool(data=True))
        rospy.loginfo("[Brain] Stage action: enable YOLO perception")

    def _lookup_transform(self, frame_id, stamp):
        try:
            return self.tf_buffer.lookup_transform(
                "base_link",
                frame_id,
                stamp,
                rospy.Duration(0.5),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            return self.tf_buffer.lookup_transform(
                "base_link",
                frame_id,
                rospy.Time(0),
                rospy.Duration(0.5),
            )

    def update(self):
        if is_test_mode():
            if not self.mock_done:
                mock_point = PointStamped()
                mock_point.header.frame_id = "base_link"
                mock_point.point.x = 0.35
                mock_point.point.y = -0.10
                mock_point.point.z = 0.30
                blackboard = get_blackboard()
                blackboard.set("target_object", {"name": "mock_target", "confidence": 1.0})
                blackboard.set("target_point_base_link", mock_point)
                blackboard.set("target_key", build_target_key("mock_target", mock_point))
                self.mock_done = True
            return Status.SUCCESS

        no_target_timeout = float(rospy.get_param("~no_target_timeout", 3.0))
        elapsed = 0.0
        if self.wait_start_time is not None:
            elapsed = max(0.0, (rospy.Time.now() - self.wait_start_time).to_sec())

        if not self.latest_detections:
            if elapsed >= no_target_timeout:
                self.yolo_enable_pub.publish(Bool(data=False))
                reason = "No YOLO detections observed for %.1fs" % elapsed
                rospy.loginfo("[Brain] %s. Treating table as empty.", reason)
                set_no_targets_complete(reason)
                return Status.FAILURE
            return Status.RUNNING

        failed_target_keys = get_failed_target_keys()
        preferred_label = normalize_label(rospy.get_param("~preferred_target_label", "cube"))
        frame_id = self.latest_frame_id or rospy.get_param(
            "~perception_frame_id", "camera_rgb_optical_frame"
        )
        latest_labels = [
            str(item.get("name", "unknown"))
            for item in self.latest_detections
            if isinstance(item, dict)
        ]

        try:
            trans = self._lookup_transform(frame_id, self.latest_stamp)
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            rospy.logwarn_throttle(2.0, "[Brain] Waiting for TF to base_link: %s", exc)
            return Status.RUNNING

        best_target = None
        best_point_base = None
        best_target_key = None
        highest_score = -1.0
        highest_confidence = -1.0

        for obj in self.latest_detections:
            if not isinstance(obj, dict):
                continue

            center = obj.get("center_3d")
            if not isinstance(center, dict):
                continue

            pt_camera = PointStamped()
            pt_camera.header.frame_id = frame_id
            pt_camera.header.stamp = trans.header.stamp
            pt_camera.point.x = float(center.get("x", 0.0))
            pt_camera.point.y = float(center.get("y", 0.0))
            pt_camera.point.z = float(center.get("z", 0.0))

            pt_base = tf2_geometry_msgs.do_transform_point(pt_camera, trans)
            label = str(obj.get("name", "unknown"))
            canonical_label = self._canonical_label(label)
            if preferred_label and canonical_label != preferred_label:
                continue
            target_key = build_target_key(label, pt_base)
            if target_key in failed_target_keys:
                continue

            confidence = float(obj.get("confidence", 0.0))
            score = self._score_object(canonical_label)
            if score > highest_score or (
                score == highest_score and confidence > highest_confidence
            ):
                highest_score = score
                highest_confidence = confidence
                best_target = obj
                best_point_base = pt_base
                best_target_key = target_key

        if best_target is None:
            if preferred_label:
                rospy.logwarn_throttle(
                    2.0,
                    "[Brain] Waiting for preferred target '%s' | visible=%s blacklisted=%d",
                    preferred_label,
                    ", ".join(latest_labels),
                    len(failed_target_keys),
                )
            else:
                rospy.logwarn_throttle(
                    2.0,
                    "[Brain] No selectable YOLO target right now | labels=%s blacklisted=%d",
                    ", ".join(latest_labels),
                    len(failed_target_keys),
                )
            if elapsed >= no_target_timeout:
                self.yolo_enable_pub.publish(Bool(data=False))
                if preferred_label:
                    reason = (
                        "No selectable target '%s' observed for %.1fs"
                        % (preferred_label, elapsed)
                    )
                else:
                    reason = "No selectable YOLO target observed for %.1fs" % elapsed
                rospy.loginfo("[Brain] %s. Treating table as empty.", reason)
                set_no_targets_complete(reason)
                return Status.FAILURE
            return Status.RUNNING

        clear_terminal_failure()
        blackboard = get_blackboard()
        blackboard.set("target_object", best_target)
        blackboard.set("target_point_base_link", best_point_base)
        blackboard.set("target_key", best_target_key)
        blackboard.set("target_score", highest_score)
        blackboard.set("target_confidence", highest_confidence)
        selected_label = str(best_target.get("name", "unknown"))
        selected_canonical_label = self._canonical_label(selected_label)

        preview_pose = PoseStamped()
        preview_pose.header.frame_id = "base_link"
        preview_pose.header.stamp = rospy.Time.now()
        preview_roll = float(rospy.get_param("~direct_grasp_roll", 3.141592653589793))
        preview_pitch = float(rospy.get_param("~direct_grasp_pitch", 1.5707963267948966))
        preview_yaw = float(rospy.get_param("~direct_grasp_yaw", 0.0))
        qx, qy, qz, qw = quaternion_from_euler(preview_roll, preview_pitch, preview_yaw)
        preview_pose.pose.orientation = Quaternion(qx, qy, qz, qw)
        preview_local_x_offset = float(rospy.get_param("~direct_grasp_local_x_offset", 0.15))
        preview_dx, preview_dy, preview_dz = rotate_vector_by_quaternion(
            preview_pose.pose.orientation,
            (preview_local_x_offset, 0.0, 0.0),
        )
        preview_pose.pose.position.x = float(best_point_base.point.x) + preview_dx
        preview_pose.pose.position.y = float(best_point_base.point.y) + preview_dy
        preview_pose.pose.position.z = float(best_point_base.point.z) + preview_dz
        self.target_pose_preview_pub.publish(preview_pose)

        self.yolo_enable_pub.publish(Bool(data=False))
        rospy.loginfo("[Brain] Stage action: disable YOLO perception")
        rospy.loginfo(
            "[Brain] Stage complete: EvaluateTargets | selected=%s semantic=%s score=%s confidence=%.3f preview=(%.3f, %.3f, %.3f) local_x_offset=%.3f",
            selected_label,
            selected_canonical_label,
            highest_score,
            highest_confidence,
            preview_pose.pose.position.x,
            preview_pose.pose.position.y,
            preview_pose.pose.position.z,
            preview_local_x_offset,
        )
        return Status.SUCCESS


class RequestGraspPoseBehavior(py_trees.behaviour.Behaviour):
    """Build a simple grasp pose directly from the selected YOLO 3D point."""

    def __init__(self, name="RequestGraspPose"):
        super().__init__(name)
        self.request_sent = False

    def initialise(self):
        self.request_sent = False
        log_stage("BuildDirectGraspPose")

    def update(self):
        if is_test_mode():
            if not self.request_sent:
                mock_pose = PoseStamped()
                mock_pose.header.frame_id = "base_link"
                mock_pose.pose.position.x = 0.35
                mock_pose.pose.position.y = -0.10
                mock_pose.pose.position.z = 0.30
                mock_pose.pose.orientation.w = 1.0
                get_blackboard().set("target_grasp_pose", mock_pose)
                self.request_sent = True
            return Status.SUCCESS

        blackboard = get_blackboard()
        target_point = blackboard_get("target_point_base_link")
        if target_point is None:
            return Status.RUNNING

        if self.request_sent:
            return Status.SUCCESS

        target_object = blackboard_get("target_object", {}) or {}
        target_label = (
            target_object.get("name", "unknown")
            if isinstance(target_object, dict)
            else str(target_object)
        )

        grasp_offset_x = float(rospy.get_param("~direct_grasp_offset_x", 0.0))
        grasp_offset_y = float(rospy.get_param("~direct_grasp_offset_y", 0.0))
        grasp_offset_z = float(rospy.get_param("~direct_grasp_offset_z", 0.0))
        grasp_local_x_offset = float(rospy.get_param("~direct_grasp_local_x_offset", 0.15))
        grasp_min_z = float(rospy.get_param("~direct_grasp_min_z", 0.05))
        grasp_max_z = float(rospy.get_param("~direct_grasp_max_z", 1.20))
        grasp_roll = float(rospy.get_param("~direct_grasp_roll", 3.141592653589793))
        grasp_pitch = float(rospy.get_param("~direct_grasp_pitch", 1.5707963267948966))
        grasp_yaw = float(rospy.get_param("~direct_grasp_yaw", 0.0))

        raw_z = float(target_point.point.z) + grasp_offset_z
        clamped_z = max(grasp_min_z, min(grasp_max_z, raw_z))
        qx, qy, qz, qw = quaternion_from_euler(grasp_roll, grasp_pitch, grasp_yaw)
        grasp_orientation = Quaternion(qx, qy, qz, qw)
        local_dx, local_dy, local_dz = rotate_vector_by_quaternion(
            grasp_orientation,
            (grasp_local_x_offset, 0.0, 0.0),
        )

        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = "base_link"
        grasp_pose.header.stamp = rospy.Time.now()
        grasp_pose.pose.position.x = float(target_point.point.x) + grasp_offset_x + local_dx
        grasp_pose.pose.position.y = float(target_point.point.y) + grasp_offset_y + local_dy
        grasp_pose.pose.position.z = clamped_z + local_dz
        grasp_pose.pose.orientation = grasp_orientation

        blackboard.set("target_grasp_pose", grasp_pose)
        blackboard.set("target_grasp_mode", "direct_yolo_point")

        if abs(clamped_z - raw_z) > 1e-6:
            rospy.logwarn(
                "[Brain] Clamped target z from %.3f to %.3f for %s",
                raw_z,
                clamped_z,
                target_label,
            )

        rospy.loginfo(
            "[Brain] Stage complete: BuildDirectGraspPose | target=%s pos=(%.3f, %.3f, %.3f) rpy=(%.3f, %.3f, %.3f) local_x_offset=%.3f",
            target_label,
            grasp_pose.pose.position.x,
            grasp_pose.pose.position.y,
            grasp_pose.pose.position.z,
            grasp_roll,
            grasp_pitch,
            grasp_yaw,
            grasp_local_x_offset,
        )
        self.request_sent = True
        return Status.SUCCESS


class ExecutePickAndPlaceBehavior(py_trees.behaviour.Behaviour):
    """Approach, grasp, return to overview, move to bin, then release."""

    def __init__(self, name="ExecutePickAndPlace"):
        super().__init__(name)
        self.client = None
        self.motion_cmd_pub = rospy.Publisher("/motion/command", MotionCommand, queue_size=10)
        self.motion_result_sub = rospy.Subscriber(
            "/motion/result", GraspResult, self._motion_result_callback
        )
        self.gripper_cmd_pub = rospy.Publisher("/gripper/command", String, queue_size=1)
        self.gripper_grasp_srv = rospy.ServiceProxy("/gripper/grasp", Trigger)
        self.gripper_release_srv = rospy.ServiceProxy("/gripper/release", Trigger)
        self.gripper_is_holding_srv = rospy.ServiceProxy("/gripper/is_holding", Trigger)
        self.goal_sent = False
        self.direct_motion_sent = False
        self.return_overview_sent = False
        self.bin_motion_sent = False
        self.mock_done = False
        self.last_feedback_stage = None
        self.last_motion_result = None
        self.motion_result_count = 0
        self.direct_motion_start_count = 0
        self.return_overview_start_count = 0
        self.bin_motion_start_count = 0
        self.phase = "approach"
        self.place_bin_color = "green"
        self.place_bin_joints = []

    def setup(self, timeout=None):
        if is_test_mode():
            return True
        self.client = actionlib.SimpleActionClient(
            "/path_planning/plan_execute_pose", PlanExecutePoseAction
        )
        return True

    def _motion_result_callback(self, msg):
        self.last_motion_result = msg
        self.motion_result_count += 1

    def _feedback_callback(self, feedback):
        stage_name = (
            "Planning"
            if feedback.stage == PlanExecutePoseFeedback.PLANNING
            else "Executing"
        )
        if stage_name == self.last_feedback_stage:
            return
        self.last_feedback_stage = stage_name
        rospy.loginfo("[Brain] Stage: %s", stage_name)

    def initialise(self):
        self.goal_sent = False
        self.direct_motion_sent = False
        self.return_overview_sent = False
        self.bin_motion_sent = False
        self.mock_done = False
        self.last_feedback_stage = None
        self.direct_motion_start_count = self.motion_result_count
        self.return_overview_start_count = self.motion_result_count
        self.bin_motion_start_count = self.motion_result_count
        self.phase = "approach"
        self.place_bin_color = normalize_label(rospy.get_param("~place_bin_color", "green")) or "green"
        self.place_bin_joints = self._get_bin_joints(self.place_bin_color)
        log_stage("ApproachTarget")
        target_object = blackboard_get("target_object", {}) or {}
        target_label = (
            target_object.get("name", "unknown")
            if isinstance(target_object, dict)
            else str(target_object)
        )
        rospy.loginfo(
            "[Brain] Current grasp target label: %s | place_bin=%s",
            target_label,
            self.place_bin_color,
        )

    def _get_bin_joints(self, bin_color):
        defaults = {
            "green": [-0.4696, -1.1530, -1.6126, 1.1743, 1.5815, -0.4750],
            "blue": [1.4744, -1.0557, -1.4196, 0.9190, 1.5769, 1.4693],
        }
        resolved_color = normalize_label(bin_color)
        if resolved_color not in defaults:
            rospy.logwarn(
                "[Brain] Unknown place_bin_color '%s', falling back to green",
                bin_color,
            )
            resolved_color = "green"
        values = rospy.get_param("~%s_bin_joints" % resolved_color, defaults[resolved_color])
        joints = [float(v) for v in values]
        if len(joints) != 6:
            raise ValueError("%s_bin_joints must contain 6 joint values" % resolved_color)
        self.place_bin_color = resolved_color
        return joints

    @staticmethod
    def _is_interface_failure(message):
        text = str(message).lower()
        return (
            "unavailable" in text
            or "not reachable" in text
            or "timeout" in text and "gripper" in text
            or "no gripper" in text
            or text.startswith("unknown:")
        )

    def _call_gripper_grasp(self):
        try:
            rospy.wait_for_service("/gripper/grasp", timeout=0.5)
            response = self.gripper_grasp_srv()
            if response.success:
                rospy.loginfo("[Brain] Stage action: command gripper grasp")
                return True, response.message or "grasp_success"
            return False, response.message or "grasp_failed"
        except (rospy.ROSException, rospy.ServiceException):
            if self.gripper_cmd_pub.get_num_connections() > 0:
                self.gripper_cmd_pub.publish(String(data="grasp"))
                rospy.logwarn(
                    "[Brain] /gripper/grasp service unavailable, published grasp command instead"
                )
                return True, "grasp_command_published"
            return False, "gripper grasp interface unavailable"

    def _call_gripper_release(self):
        try:
            rospy.wait_for_service("/gripper/release", timeout=0.5)
            response = self.gripper_release_srv()
            if response.success:
                rospy.loginfo("[Brain] Stage action: command gripper release")
                return True, response.message or "release_success"
            return False, response.message or "release_failed"
        except (rospy.ROSException, rospy.ServiceException):
            if self.gripper_cmd_pub.get_num_connections() > 0:
                self.gripper_cmd_pub.publish(String(data="release"))
                rospy.logwarn(
                    "[Brain] /gripper/release service unavailable, published release command instead"
                )
                return True, "release_command_published"
            return False, "gripper release interface unavailable"

    def _send_direct_motion_goal(self, target_pose, reason):
        if self.motion_cmd_pub.get_num_connections() == 0:
            return False

        cmd = MotionCommand()
        cmd.command_type = MotionCommand.MOVE_TO_POSE
        cmd.target_pose = target_pose
        cmd.max_velocity = float(rospy.get_param("~direct_motion_max_velocity", 0.6))
        cmd.max_acceleration = float(
            rospy.get_param("~direct_motion_max_acceleration", 0.6)
        )
        cmd.collision_check = False

        log_stage("DirectMotionFallback")
        rospy.loginfo(
            "[Brain] Stage action: send pose directly to motion_control | reason=%s",
            reason,
        )
        self.last_motion_result = None
        self.direct_motion_start_count = self.motion_result_count
        self.motion_cmd_pub.publish(cmd)
        self.direct_motion_sent = True
        return True

    def _send_overview_joint_goal(self):
        if self.motion_cmd_pub.get_num_connections() == 0:
            return False

        cmd = MotionCommand()
        cmd.command_type = MotionCommand.MOVE_TO_JOINT
        cmd.joint_positions = list(OVERVIEW_JOINTS)
        cmd.max_velocity = float(rospy.get_param("~return_overview_max_velocity", 1.0))
        cmd.max_acceleration = float(
            rospy.get_param("~return_overview_max_acceleration", 1.0)
        )
        cmd.collision_check = True

        log_stage("ReturnToOverview")
        rospy.loginfo(
            "[Brain] Stage action: return to overview joints [%s]",
            ", ".join(f"{joint:.4f}" for joint in OVERVIEW_JOINTS),
        )
        self.last_motion_result = None
        self.return_overview_start_count = self.motion_result_count
        self.motion_cmd_pub.publish(cmd)
        self.return_overview_sent = True
        return True

    def _send_bin_joint_goal(self):
        if self.motion_cmd_pub.get_num_connections() == 0:
            return False

        cmd = MotionCommand()
        cmd.command_type = MotionCommand.MOVE_TO_JOINT
        cmd.joint_positions = list(self.place_bin_joints)
        cmd.max_velocity = float(rospy.get_param("~place_motion_max_velocity", 1.0))
        cmd.max_acceleration = float(rospy.get_param("~place_motion_max_acceleration", 1.0))
        cmd.collision_check = True

        log_stage("MoveToPlaceBin")
        rospy.loginfo(
            "[Brain] Stage action: move to %s bin joints [%s]",
            self.place_bin_color,
            ", ".join(f"{joint:.4f}" for joint in self.place_bin_joints),
        )
        self.last_motion_result = None
        self.bin_motion_start_count = self.motion_result_count
        self.motion_cmd_pub.publish(cmd)
        self.bin_motion_sent = True
        return True

    def _recover(self, reason, blacklist=False):
        prepare_retry_from_overview(reason, blacklist=blacklist)
        return Status.FAILURE

    def _after_target_reached(self, completion_label, trajectory=None):
        blackboard = get_blackboard()
        if trajectory is not None:
            blackboard.set("executed_trajectory", trajectory)

        success, message = self._call_gripper_grasp()
        if not success:
            if self._is_interface_failure(message):
                set_terminal_failure(f"Gripper grasp failed: {message}")
                rospy.logerr("[Brain] Gripper grasp failed: %s", message)
                return Status.FAILURE
            return self._recover(f"gripper grasp command failed: {message}", blacklist=False)

        blackboard.set("holding_object", True)
        rospy.loginfo("[Brain] Stage complete: %s", completion_label)
        rospy.sleep(float(rospy.get_param("~post_grasp_settle_time", 0.3)))
        self.phase = "return_overview"
        return Status.RUNNING

    def update(self):
        if is_test_mode():
            if not self.mock_done:
                self.mock_done = True
            return Status.SUCCESS

        blackboard = get_blackboard()
        target_pose = blackboard_get("target_grasp_pose")
        if target_pose is None:
            return Status.RUNNING
        if not isinstance(target_pose, PoseStamped):
            set_terminal_failure("target_grasp_pose is not a PoseStamped")
            rospy.logerr(
                "[Brain] target_grasp_pose is not a PoseStamped: %r", type(target_pose)
            )
            return Status.FAILURE

        if self.phase == "return_overview":
            if not self.return_overview_sent:
                if self._send_overview_joint_goal():
                    return Status.RUNNING
                set_terminal_failure("motion_control is not reachable for return_overview")
                rospy.logerr("[Brain] motion_control is not reachable for return_overview")
                return Status.FAILURE

            if self.motion_result_count <= self.return_overview_start_count:
                return Status.RUNNING

            if self.last_motion_result is not None and self.last_motion_result.status == GraspResult.SUCCESS:
                self.return_overview_sent = False
                rospy.loginfo("[Brain] Stage complete: ReturnToOverview")
                self.phase = "move_to_bin"
                return Status.RUNNING

            failure_message = (
                self.last_motion_result.message
                if self.last_motion_result is not None
                else "return_overview motion failed"
            )
            return self._recover(f"return to overview failed: {failure_message}", blacklist=False)

        if self.phase == "move_to_bin":
            if not self.bin_motion_sent:
                if self._send_bin_joint_goal():
                    return Status.RUNNING
                set_terminal_failure("motion_control is not reachable for bin placement")
                rospy.logerr("[Brain] motion_control is not reachable for bin placement")
                return Status.FAILURE

            if self.motion_result_count <= self.bin_motion_start_count:
                return Status.RUNNING

            self.bin_motion_sent = False
            if self.last_motion_result is not None and self.last_motion_result.status == GraspResult.SUCCESS:
                self.phase = "release"
                log_stage("ReleaseToBin")
                return Status.RUNNING

            failure_message = (
                self.last_motion_result.message
                if self.last_motion_result is not None
                else "bin motion failed"
            )
            return self._recover(
                f"{self.place_bin_color} bin motion failed: {failure_message}",
                blacklist=False,
            )

        if self.phase == "release":
            success, message = self._call_gripper_release()
            if not success:
                if self._is_interface_failure(message):
                    set_terminal_failure(f"Gripper release failed: {message}")
                    rospy.logerr("[Brain] Gripper release failed: %s", message)
                    return Status.FAILURE
                return self._recover(f"release at {self.place_bin_color} bin failed: {message}")

            blackboard.set("holding_object", False)
            blackboard.set("placed_bin_color", self.place_bin_color)
            rospy.loginfo("[Brain] Stage complete: PlaceTo%sBin", self.place_bin_color.capitalize())
            return Status.SUCCESS

        if self.direct_motion_sent:
            if self.motion_result_count <= self.direct_motion_start_count:
                return Status.RUNNING
            self.direct_motion_sent = False
            if self.last_motion_result is not None and self.last_motion_result.status == GraspResult.SUCCESS:
                return self._after_target_reached("DirectMotionFallback")

            failure_message = (
                self.last_motion_result.message
                if self.last_motion_result is not None
                else "motion_control direct move failed"
            )
            return self._recover(f"direct motion failed: {failure_message}", blacklist=True)

        if not self.goal_sent:
            if self.client is not None and self.client.wait_for_server(rospy.Duration(0.1)):
                goal = PlanExecutePoseGoal()
                goal.target_pose = target_pose
                goal.position_only = rospy.get_param("~path_planning_position_only", False)

                rospy.loginfo("[Brain] Stage action: send pose to path_planning")
                self.client.send_goal(goal, feedback_cb=self._feedback_callback)
                self.goal_sent = True
                return Status.RUNNING

            if self._send_direct_motion_goal(target_pose, "path_planning server unavailable"):
                return Status.RUNNING

            set_terminal_failure("Neither path_planning nor motion_control is reachable")
            rospy.logerr("[Brain] No path_planning server or direct motion_control connection available")
            return Status.FAILURE

        state = self.client.get_state()
        if state in (action_msgs.GoalStatus.ACTIVE, action_msgs.GoalStatus.PENDING):
            return Status.RUNNING

        result = self.client.get_result()
        self.goal_sent = False

        if state == action_msgs.GoalStatus.SUCCEEDED and result is not None and result.success:
            return self._after_target_reached("PathPlanning", trajectory=result.trajectory)

        if result is not None and result.status == PlanExecutePoseResult.PREEMPTED:
            failure_message = result.message or "path_planning preempted"
        elif result is not None:
            failure_message = result.message or "path_planning failed"
        else:
            failure_message = "path_planning action failed without a result"

        rospy.logwarn(
            "[Brain] Path planning failed, trying direct motion fallback: %s",
            failure_message,
        )
        if self._send_direct_motion_goal(target_pose, failure_message):
            return Status.RUNNING

        set_terminal_failure("Direct motion fallback could not be started")
        rospy.logerr("[Brain] Direct motion fallback could not be started")
        return Status.FAILURE


def create_behavior_tree():
    main_sequence = py_trees.composites.Sequence(
        name="MainTaskSequence",
        memory=True,
        children=[
            MoveToOverviewBehavior(),
            EvaluateTargetsBehavior(),
            RequestGraspPoseBehavior(),
            ExecutePickAndPlaceBehavior(),
        ],
    )
    root = py_trees.composites.Selector(
        name="TaskOrRecovery",
        children=[main_sequence],
    )
    return root


def initialise_blackboard_state():
    blackboard = get_blackboard()
    blackboard.set("overview_done", False)
    blackboard.set("holding_object", False)
    blackboard.set("placed_bin_color", "")
    blackboard.set("detected_objects", [])
    blackboard.set(FAILED_TARGET_KEYS_BB, [])
    blackboard.set("last_target_failure_reason", "")
    blackboard.set("task_retry_requested", False)
    blackboard.set("task_retry_reason", "")
    clear_terminal_failure()
    clear_target_selection()


def main():
    rospy.init_node("robocup_brain", anonymous=False)
    rospy.loginfo("=" * 50)
    rospy.loginfo("RoboCup Brain Node Starting")
    rospy.loginfo("=" * 50)
    rospy.loginfo("[Brain] test_mode=%s", is_test_mode())

    single_cycle = rospy.get_param("~single_cycle", True)
    loop_until_no_targets = rospy.get_param("~loop_until_no_targets", True)
    initialise_blackboard_state()

    root = create_behavior_tree()
    behaviour_tree = py_trees_ros.trees.BehaviourTree(root)
    if hasattr(behaviour_tree, "setup"):
        behaviour_tree.setup(timeout=15)

    rate = rospy.Rate(10)
    task_finished = False

    rospy.loginfo("[Brain] Behavior Tree initialized. Starting main loop...")

    try:
        while not rospy.is_shutdown():
            if task_finished:
                rate.sleep()
                continue

            behaviour_tree.tick()

            if root.status == Status.SUCCESS:
                if loop_until_no_targets:
                    rospy.loginfo("[Brain] Cycle complete. Returning to overview for next target.")
                    prepare_next_pick_cycle()
                    root.stop(Status.INVALID)
                elif single_cycle:
                    rospy.loginfo("[Brain] Task complete. Holding after single cycle.")
                    task_finished = True
            elif root.status == Status.FAILURE:
                if blackboard_get("task_complete_no_targets", False):
                    reason = blackboard_get("task_complete_no_targets_reason", "No targets remain")
                    rospy.loginfo("[Brain] Task complete. %s", reason)
                    task_finished = True
                elif blackboard_get("task_terminal_failure", False):
                    reason = blackboard_get("task_terminal_failure_reason", "unknown")
                    rospy.logwarn(
                        "[Brain] Terminal failure. Holding after single cycle: %s",
                        reason,
                    )
                    task_finished = True
                else:
                    reason = blackboard_get(
                        "task_retry_reason",
                        blackboard_get("last_target_failure_reason", "unknown"),
                    )
                    rospy.logwarn(
                        "[Brain] Non-terminal failure. Returning to overview and retrying: %s",
                        reason,
                    )
                    get_blackboard().set("task_retry_requested", False)
                    get_blackboard().set("task_retry_reason", "")
                    root.stop(Status.INVALID)

            rate.sleep()
    except KeyboardInterrupt:
        rospy.loginfo("[Brain] Shutting down...")


if __name__ == "__main__":
    main()
