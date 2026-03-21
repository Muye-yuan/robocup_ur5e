#!/usr/bin/env python3
import re
import sys
import copy
import json

import rospy
import moveit_commander

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import (
    CollisionObject,
    AttachedCollisionObject,
    PlanningScene,
    PlanningSceneComponents,
    AllowedCollisionEntry,
)
from moveit_msgs.srv import (
    ApplyPlanningScene,
    ApplyPlanningSceneRequest,
    GetPlanningScene,
    GetPlanningSceneRequest,
)
from std_srvs.srv import Empty, Trigger, TriggerResponse
from std_msgs.msg import String

from common_msgs.msg import ObjectScore, TaskDecision, GraspResult


def _sanitize_label(label: str) -> str:
    return re.sub(r"[^a-zA-Z0-9_]+", "_", label.strip().lower()) or "obj"


class PlanningSceneAdapter:
    """
    Planning scene bridge for Perception/Brain -> MoveIt.

    Responsibilities:
      - add/update collision objects from perception
      - clear octomap after object insertion
      - manage ACM during grasp/release
      - attach/detach object after successful grasp
    """

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("planning_scene_adapter", anonymous=False)

        self.scene = moveit_commander.PlanningSceneInterface(synchronous=True)
        self.robot = moveit_commander.RobotCommander()

        self.frame_id = rospy.get_param("~frame", "base_link")
        self.ee_link = rospy.get_param("~ee_link", "gripper_tip_link")

        self.touch_links = rospy.get_param(
            "~touch_links",
            [
                "wrist_3_link",
                "tool0",
                "robotiq_coupler",
                "robotiq_85_base_link",
                "robotiq_85_left_knuckle_link",
                "robotiq_85_left_finger_link",
                "robotiq_85_left_inner_knuckle_link",
                "robotiq_85_left_finger_tip_link",
                "robotiq_85_right_knuckle_link",
                "robotiq_85_right_finger_link",
                "robotiq_85_right_inner_knuckle_link",
                "robotiq_85_right_finger_tip_link",
            ],
        )

        self.gripper_links_for_acm = rospy.get_param(
            "~gripper_links_for_acm", self.touch_links
        )
        self.pointcloud_topic = rospy.get_param("~pointcloud_topic", "/camera/depth/points")
        self.motion_result_topic = rospy.get_param("~motion_result_topic", "/motion/result")
        self.planning_scene_command_topic = rospy.get_param(
            "~planning_scene_command_topic", "/brain/planning_scene_command"
        )
        self.auto_attach_on_grasp_success = rospy.get_param("~auto_attach_on_grasp_success", True)
        self.clear_octomap_on_target_add = rospy.get_param("~clear_octomap_on_target_add", True)
        self.configure_octomap_params = rospy.get_param("~configure_octomap_params", True)
        self.octomap_resolution = float(rospy.get_param("~octomap_resolution", 0.015))
        self.octomap_min_resolution = float(rospy.get_param("~octomap_min_resolution", 0.01))
        self.octomap_max_resolution = float(rospy.get_param("~octomap_max_resolution", 0.02))
        self.wait_for_pointcloud = rospy.get_param("~wait_for_pointcloud", False)
        self.pointcloud_wait_sec = float(rospy.get_param("~pointcloud_wait_sec", 2.0))

        self.object_shape_map = rospy.get_param(
            "~object_shape_map",
            {
                "can": "cylinder",
                "bottle": "cylinder",
                "cube": "box",
                "box": "box",
                "spam": "box",
            },
        )

        self.object_size_map = rospy.get_param(
            "~object_size_map",
            {
                "default": [0.05, 0.05, 0.10],   # box x,y,z
                "can": [0.035, 0.12],            # cylinder radius,height
                "bottle": [0.04, 0.20],
                "cube": [0.05, 0.05, 0.05],
                "box": [0.08, 0.06, 0.10],
                "spam": [0.10, 0.06, 0.03],
            },
        )

        self.position_is_table_contact = rospy.get_param("~position_is_table_contact", True)

        # object_id -> metadata
        self.objects = {}
        self.current_target_name = ""
        self.awaiting_grasp_success_attach = False
        self.target_is_attached = False
        self.current_task = ""

        self._configure_octomap_params()
        self._log_octomap_configuration()
        if self.wait_for_pointcloud:
            self._wait_for_pointcloud()

        rospy.loginfo("[Scene] Waiting for MoveIt services...")
        rospy.wait_for_service("/apply_planning_scene")
        rospy.wait_for_service("/get_planning_scene")
        rospy.wait_for_service("/clear_octomap")

        self.apply_scene_srv = rospy.ServiceProxy("/apply_planning_scene", ApplyPlanningScene)
        self.get_scene_srv = rospy.ServiceProxy("/get_planning_scene", GetPlanningScene)
        self.clear_octomap_srv = rospy.ServiceProxy("/clear_octomap", Empty)

        self.object_score_topic = rospy.get_param("~object_score_topic", "/perception/object_score")
        self.task_decision_topic = rospy.get_param("~task_decision_topic", "/brain/task_decision")

        rospy.Subscriber(self.object_score_topic, ObjectScore, self._on_object_score, queue_size=20)
        rospy.Subscriber(self.task_decision_topic, TaskDecision, self._on_task_decision, queue_size=10)
        rospy.Subscriber(self.motion_result_topic, GraspResult, self._on_motion_result, queue_size=10)
        rospy.Subscriber(self.planning_scene_command_topic, String, self._on_scene_command, queue_size=20)

        rospy.Service("~clear_scene", Trigger, self._srv_clear_scene)
        rospy.Service("~attach_target", Trigger, self._srv_attach_target)
        rospy.Service("~detach_target", Trigger, self._srv_detach_target)
        rospy.Service("~allow_target_collision", Trigger, self._srv_allow_target_collision)
        rospy.Service("~forbid_target_collision", Trigger, self._srv_forbid_target_collision)

        rospy.loginfo("[Scene] planning_scene_adapter ready.")

    def _configure_octomap_params(self):
        if not self.configure_octomap_params:
            return

        resolution = self.octomap_resolution
        if resolution < self.octomap_min_resolution or resolution > self.octomap_max_resolution:
            rospy.logwarn(
                "[Scene] octomap_resolution=%.4f outside [%.4f, %.4f], clamping.",
                resolution,
                self.octomap_min_resolution,
                self.octomap_max_resolution,
            )
            resolution = max(self.octomap_min_resolution, min(self.octomap_max_resolution, resolution))
            self.octomap_resolution = resolution

        rospy.set_param("/move_group/octomap_resolution", float(resolution))

        if not rospy.has_param("/move_group/sensors"):
            rospy.set_param("/move_group/sensors", ["point_cloud_sensor"])

        updater_key = "/move_group/point_cloud_sensor"
        if not rospy.has_param(updater_key):
            rospy.set_param(
                updater_key,
                {
                    "sensor_plugin": "occupancy_map_monitor/PointCloudOctomapUpdater",
                    "point_cloud_topic": self.pointcloud_topic,
                    "max_range": 3.5,
                    "point_subsample": 1,
                    "padding_offset": 0.02,
                    "padding_scale": 1.0,
                    "filtered_cloud_topic": "/move_group/filtered_cloud",
                },
            )

    def _log_octomap_configuration(self):
        sensors = rospy.get_param("/move_group/sensors", [])
        resolution = rospy.get_param("/move_group/octomap_resolution", None)
        updater = rospy.get_param("/move_group/point_cloud_sensor", {})
        rospy.loginfo(
            "[Scene] Octomap config: sensors=%s resolution=%s topic=%s",
            str(sensors),
            str(resolution),
            str(updater.get("point_cloud_topic", "N/A")),
        )
        if isinstance(resolution, (float, int)):
            res = float(resolution)
            if res < self.octomap_min_resolution or res > self.octomap_max_resolution:
                rospy.logwarn(
                    "[Scene] Octomap resolution %.4f is outside recommended [%.4f, %.4f]",
                    res,
                    self.octomap_min_resolution,
                    self.octomap_max_resolution,
                )

    def _wait_for_pointcloud(self):
        try:
            rospy.wait_for_message(self.pointcloud_topic, PointCloud2, timeout=self.pointcloud_wait_sec)
            rospy.loginfo("[Scene] Point cloud stream detected on %s", self.pointcloud_topic)
        except rospy.ROSException:
            rospy.logwarn(
                "[Scene] No point cloud received on %s within %.1fs",
                self.pointcloud_topic,
                self.pointcloud_wait_sec,
            )

    # ---------------------------
    # Object helpers
    # ---------------------------
    def _size_and_shape_for_label(self, label):
        lower = label.lower()
        shape = "box"
        size = self.object_size_map.get("default", [0.05, 0.05, 0.10])

        for key, mapped_shape in self.object_shape_map.items():
            if key in lower:
                shape = mapped_shape
                size = self.object_size_map.get(key, size)
                break
        return shape, size

    def _pose_from_object_score(self, msg, shape, size):
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = rospy.Time.now()
        pose.pose.orientation.w = 1.0
        pose.pose.position.x = msg.position.x
        pose.pose.position.y = msg.position.y
        pose.pose.position.z = msg.position.z

        # If perception gives table-contact point, lift to primitive center.
        if self.position_is_table_contact:
            if shape == "cylinder":
                pose.pose.position.z += float(size[1]) * 0.5
            else:
                pose.pose.position.z += float(size[2]) * 0.5

        return pose

    def _build_collision_object(self, object_name, pose_stamped, shape, size):
        co = CollisionObject()
        co.id = object_name
        co.header = pose_stamped.header

        primitive = SolidPrimitive()
        if shape == "cylinder":
            primitive.type = SolidPrimitive.CYLINDER
            primitive.dimensions = [float(size[1]), float(size[0])]  # [height, radius]
        else:
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [float(size[0]), float(size[1]), float(size[2])]  # [x,y,z]

        co.primitives.append(primitive)
        co.primitive_poses.append(copy.deepcopy(pose_stamped.pose))
        co.operation = CollisionObject.ADD
        return co

    # ---------------------------
    # Scene updates
    # ---------------------------
    def add_object(self, label, pose_stamped, size, shape="box", object_name=None, clear_octomap_under=True):
        if object_name is None:
            object_name = "obj_%s" % _sanitize_label(label)

        pose_stamped.header.frame_id = pose_stamped.header.frame_id or self.frame_id
        pose_stamped.header.stamp = rospy.Time.now()

        co = self._build_collision_object(object_name, pose_stamped, shape, size)

        scene_msg = PlanningScene()
        scene_msg.is_diff = True
        scene_msg.world.collision_objects.append(co)

        req = ApplyPlanningSceneRequest()
        req.scene = scene_msg
        ok = self.apply_scene_srv(req).success
        if not ok:
            rospy.logwarn("[Scene] Failed to add object %s", object_name)
            return ""

        self.objects[object_name] = {
            "label": label,
            "shape": shape,
            "size": list(size),
            "pose": pose_stamped,
        }

        rospy.loginfo("[Scene] Added object %s label=%s shape=%s size=%s",
                      object_name, label, shape, size)

        # Practical workaround to remove stale occupancy where the target now exists.
        if clear_octomap_under:
            self.clear_octomap()

        return object_name

    def remove_object(self, object_name):
        co = CollisionObject()
        co.id = object_name
        co.header.frame_id = self.frame_id
        co.operation = CollisionObject.REMOVE

        scene_msg = PlanningScene()
        scene_msg.is_diff = True
        scene_msg.world.collision_objects.append(co)

        req = ApplyPlanningSceneRequest(scene=scene_msg)
        ok = self.apply_scene_srv(req).success
        if ok and object_name in self.objects:
            del self.objects[object_name]
        return ok

    def clear_scene(self):
        for name in list(self.objects.keys()):
            self.remove_object(name)
        self.objects = {}
        self.current_target_name = ""
        self.awaiting_grasp_success_attach = False
        self.target_is_attached = False
        rospy.loginfo("[Scene] Cleared all objects.")

    def clear_octomap(self):
        try:
            self.clear_octomap_srv()
            rospy.loginfo("[Scene] clear_octomap requested.")
        except Exception as exc:
            rospy.logwarn("[Scene] clear_octomap failed: %s", exc)

    # ---------------------------
    # ACM logic
    # ---------------------------
    def set_target_collision_allowed(self, object_name, allowed):
        if not object_name:
            return False, "No target object selected"

        try:
            req = GetPlanningSceneRequest()
            req.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
            resp = self.get_scene_srv(req)
            acm = resp.scene.allowed_collision_matrix
        except Exception as exc:
            return False, "Failed to get ACM: %s" % exc

        names = list(acm.entry_names)
        entries = list(acm.entry_values)

        def ensure_name(name):
            if name in names:
                return names.index(name)

            names.append(name)

            for entry in entries:
                entry.enabled.append(False)

            new_entry = AllowedCollisionEntry()
            new_entry.enabled = [False] * len(names)
            entries.append(new_entry)
            return len(names) - 1

        target_idx = ensure_name(object_name)
        link_indices = [ensure_name(link) for link in self.gripper_links_for_acm]

        for idx in link_indices:
            entries[target_idx].enabled[idx] = bool(allowed)
            entries[idx].enabled[target_idx] = bool(allowed)

        scene_msg = PlanningScene()
        scene_msg.is_diff = True
        scene_msg.allowed_collision_matrix.entry_names = names
        scene_msg.allowed_collision_matrix.entry_values = entries

        try:
            ok = self.apply_scene_srv(ApplyPlanningSceneRequest(scene=scene_msg)).success
            if not ok:
                return False, "apply_planning_scene failed"
        except Exception as exc:
            return False, "Failed to apply ACM: %s" % exc

        rospy.loginfo("[Scene] ACM %s for gripper <-> %s",
                      "ALLOW" if allowed else "FORBID", object_name)
        return True, "ACM updated"

    # ---------------------------
    # Attach / detach
    # ---------------------------
    def attach_object(self, object_name):
        if not object_name:
            return False, "No target object selected"
        if object_name not in self.objects:
            return False, "Unknown object: %s" % object_name

        meta = self.objects[object_name]
        co = self._build_collision_object(
            object_name=object_name,
            pose_stamped=meta["pose"],
            shape=meta["shape"],
            size=meta["size"],
        )

        aco = AttachedCollisionObject()
        aco.link_name = self.ee_link
        aco.object = co
        aco.touch_links = list(self.touch_links)
        aco.object.operation = CollisionObject.ADD

        scene_msg = PlanningScene()
        scene_msg.is_diff = True
        scene_msg.robot_state.attached_collision_objects.append(aco)

        try:
            ok = self.apply_scene_srv(ApplyPlanningSceneRequest(scene=scene_msg)).success
            if not ok:
                return False, "Failed to attach object"
        except Exception as exc:
            return False, "Attach failed: %s" % exc

        self.target_is_attached = True
        rospy.loginfo("[Scene] Attached %s to %s", object_name, self.ee_link)
        return True, "Attached %s" % object_name

    def detach_object(self, object_name):
        if not object_name:
            return False, "No target object selected"

        aco = AttachedCollisionObject()
        aco.link_name = self.ee_link
        aco.object.id = object_name
        aco.object.operation = CollisionObject.REMOVE

        scene_msg = PlanningScene()
        scene_msg.is_diff = True
        scene_msg.robot_state.attached_collision_objects.append(aco)

        try:
            ok = self.apply_scene_srv(ApplyPlanningSceneRequest(scene=scene_msg)).success
            if not ok:
                return False, "Failed to detach object"
        except Exception as exc:
            return False, "Detach failed: %s" % exc

        self.target_is_attached = False
        rospy.loginfo("[Scene] Detached %s from %s", object_name, self.ee_link)
        return True, "Detached %s" % object_name

    # ---------------------------
    # Subscribers
    # ---------------------------
    def _on_object_score(self, msg):
        shape, size = self._size_and_shape_for_label(msg.label)
        object_name = "obj_%d_%s" % (msg.object_id, _sanitize_label(msg.label))
        pose = self._pose_from_object_score(msg, shape, size)

        # replace/update same object id
        if object_name in self.objects:
            self.remove_object(object_name)

        added_name = self.add_object(
            label=msg.label,
            pose_stamped=pose,
            size=size,
            shape=shape,
            object_name=object_name,
            clear_octomap_under=self.clear_octomap_on_target_add,
        )

        if added_name:
            self.current_target_name = added_name if self.current_target_name == "" else self.current_target_name

    def _on_task_decision(self, msg):
        # Adjust these based on your actual message definition
        task_str = str(msg.task_type).lower()
        self.current_task = task_str
        target_id = getattr(msg, "target_object_id", None)

        if target_id is not None:
            name_prefix = "obj_%d_" % target_id
            for name in self.objects.keys():
                if name.startswith(name_prefix):
                    self.current_target_name = name
                    break

        if not self.current_target_name:
            rospy.logwarn("[Scene] No current target selected.")
            return

        if task_str in ["grasp", "pick", "1"]:
            ok, info = self.set_target_collision_allowed(self.current_target_name, True)
            if not ok:
                rospy.logwarn("[Scene] GRASP ACM failed: %s", info)
            self.awaiting_grasp_success_attach = bool(self.auto_attach_on_grasp_success)
            if self.awaiting_grasp_success_attach:
                rospy.loginfo("[Scene] Waiting for grasp success to attach target.")

        elif task_str in ["transport", "move_to_bin", "2"]:
            if not self.target_is_attached:
                ok, info = self.attach_object(self.current_target_name)
                if not ok:
                    rospy.logwarn("[Scene] TRANSPORT attach failed: %s", info)
            self.awaiting_grasp_success_attach = False

        elif task_str in ["release", "place", "3"]:
            ok1, info1 = self.detach_object(self.current_target_name)
            if not ok1:
                rospy.logwarn("[Scene] RELEASE detach failed: %s", info1)

            ok2, info2 = self.set_target_collision_allowed(self.current_target_name, False)
            if not ok2:
                rospy.logwarn("[Scene] RELEASE ACM reset failed: %s", info2)
            self.awaiting_grasp_success_attach = False

    def _on_motion_result(self, msg):
        if not self.auto_attach_on_grasp_success:
            return
        if not self.awaiting_grasp_success_attach:
            return
        if self.current_task not in ["grasp", "pick", "1"]:
            return

        if msg.status == GraspResult.SUCCESS:
            ok, info = self.attach_object(self.current_target_name)
            if not ok:
                rospy.logwarn("[Scene] attach-after-grasp failed: %s", info)
            else:
                rospy.loginfo("[Scene] attach-after-grasp succeeded.")
            self.awaiting_grasp_success_attach = False
        elif msg.status in [
            GraspResult.NO_GRASP_FOUND,
            GraspResult.UNREACHABLE,
            GraspResult.COLLISION,
            GraspResult.EXECUTION_FAILED,
        ]:
            rospy.logwarn("[Scene] Grasp failed (%d), canceling pending attach.", int(msg.status))
            self.awaiting_grasp_success_attach = False

    def _on_scene_command(self, msg):
        """
        Optional high-level command channel from Brain.
        Payload is JSON in std_msgs/String, e.g.:
        {"command":"attach","object_name":"obj_1_can"}
        {"command":"allow_collision","object_name":"obj_1_can"}
        {"command":"add_object","label":"can","shape":"cylinder","size":[0.035,0.12],
         "pose":{"frame_id":"base_link","position":{"x":0.4,"y":0.0,"z":0.3},
                 "orientation":{"x":0,"y":0,"z":0,"w":1}}}
        """
        try:
            payload = json.loads(msg.data)
        except Exception as exc:
            rospy.logwarn("[Scene] planning_scene_command invalid JSON: %s", exc)
            return

        command = str(payload.get("command", "")).strip().lower()
        object_name = payload.get("object_name", self.current_target_name)

        if command == "clear_scene":
            self.clear_scene()
            return
        if command == "clear_octomap":
            self.clear_octomap()
            return
        if command == "attach":
            ok, info = self.attach_object(object_name)
            if not ok:
                rospy.logwarn("[Scene] command attach failed: %s", info)
            return
        if command == "detach":
            ok, info = self.detach_object(object_name)
            if not ok:
                rospy.logwarn("[Scene] command detach failed: %s", info)
            return
        if command == "allow_collision":
            ok, info = self.set_target_collision_allowed(object_name, True)
            if not ok:
                rospy.logwarn("[Scene] command allow_collision failed: %s", info)
            return
        if command == "forbid_collision":
            ok, info = self.set_target_collision_allowed(object_name, False)
            if not ok:
                rospy.logwarn("[Scene] command forbid_collision failed: %s", info)
            return
        if command == "remove_object":
            self.remove_object(object_name)
            return
        if command == "set_target":
            if object_name:
                self.current_target_name = object_name
            return
        if command == "add_object":
            self._handle_add_object_command(payload)
            return

        rospy.logwarn("[Scene] Unknown planning_scene_command: %s", command)

    def _handle_add_object_command(self, payload):
        label = str(payload.get("label", "obj"))
        shape = str(payload.get("shape", "box")).lower()
        size = payload.get("size", self.object_size_map.get("default", [0.05, 0.05, 0.10]))
        object_name = payload.get("object_name", "obj_%s" % _sanitize_label(label))
        clear_octomap_under = bool(payload.get("clear_octomap_under", True))

        pose_data = payload.get("pose", {})
        frame_id = pose_data.get("frame_id", self.frame_id)
        pos = pose_data.get("position", {})
        ori = pose_data.get("orientation", {})

        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = float(pos.get("x", 0.0))
        pose.pose.position.y = float(pos.get("y", 0.0))
        pose.pose.position.z = float(pos.get("z", 0.0))
        pose.pose.orientation.x = float(ori.get("x", 0.0))
        pose.pose.orientation.y = float(ori.get("y", 0.0))
        pose.pose.orientation.z = float(ori.get("z", 0.0))
        pose.pose.orientation.w = float(ori.get("w", 1.0))

        added = self.add_object(
            label=label,
            pose_stamped=pose,
            size=size,
            shape=shape,
            object_name=object_name,
            clear_octomap_under=clear_octomap_under,
        )
        if added:
            self.current_target_name = added

    # ---------------------------
    # Services
    # ---------------------------
    def _srv_clear_scene(self, _req):
        self.clear_scene()
        return TriggerResponse(success=True, message="Scene cleared")

    def _srv_attach_target(self, _req):
        ok, msg = self.attach_object(self.current_target_name)
        return TriggerResponse(success=ok, message=msg)

    def _srv_detach_target(self, _req):
        ok, msg = self.detach_object(self.current_target_name)
        return TriggerResponse(success=ok, message=msg)

    def _srv_allow_target_collision(self, _req):
        ok, msg = self.set_target_collision_allowed(self.current_target_name, True)
        return TriggerResponse(success=ok, message=msg)

    def _srv_forbid_target_collision(self, _req):
        ok, msg = self.set_target_collision_allowed(self.current_target_name, False)
        return TriggerResponse(success=ok, message=msg)


def main():
    PlanningSceneAdapter()
    rospy.spin()


if __name__ == "__main__":
    main()
