#!/usr/bin/env python3
import copy
import sys
import os
import threading
import math
import rospy
import moveit_commander
import actionlib
import tf2_ros

from geometry_msgs.msg import PoseStamped, Point, TransformStamped
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents, AllowedCollisionEntry
from moveit_msgs.msg import DisplayTrajectory
from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest
from moveit_msgs.srv import ApplyPlanningScene, ApplyPlanningSceneRequest, GetPlanningScene, GetPlanningSceneRequest
from std_srvs.srv import Empty
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import ColorRGBA
from common_msgs.msg import (
    ObjectScore,
    ExecuteTrajectoryGoal,
    ExecuteTrajectoryResult,
    ExecuteTrajectoryActionGoal,
    ExecuteTrajectoryActionResult,
    ExecuteTrajectoryActionFeedback,
    PlanExecutePoseGoal,
    PlanExecutePoseResult,
    PlanExecutePoseFeedback,
    PlanExecutePoseActionGoal,
    PlanExecutePoseActionResult,
    PlanExecutePoseActionFeedback,
)


# actionlib 用 type(a.action_goal) 取消息类，故需在实例上提供消息实例（不能是类属性）
class _ExecuteTrajectoryActionSpec:
    def __init__(self):
        self.action_goal = ExecuteTrajectoryActionGoal()
        self.action_result = ExecuteTrajectoryActionResult()
        self.action_feedback = ExecuteTrajectoryActionFeedback()


class _PlanExecutePoseActionSpec:
    def __init__(self):
        self.action_goal = PlanExecutePoseActionGoal()
        self.action_result = PlanExecutePoseActionResult()
        self.action_feedback = PlanExecutePoseActionFeedback()
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker
import actionlib_msgs.msg as action_msgs


def publish_display_trajectory(robot, plan):
    pub = rospy.Publisher(
        "/move_group/display_planned_path",
        DisplayTrajectory,
        queue_size=1,
        latch=True,
    )
    rospy.sleep(0.3)

    msg = DisplayTrajectory()
    msg.trajectory_start = robot.get_current_state()
    msg.trajectory.append(plan)
    pub.publish(msg)


def extract_plan(plan_result):
    """
    Handles different MoveIt commander return formats.
    Returns (success: bool, robot_traj)
    """
    if isinstance(plan_result, tuple) and len(plan_result) >= 2:
        return bool(plan_result[0]), plan_result[1]

    robot_traj = plan_result
    if hasattr(robot_traj, "joint_trajectory") and len(robot_traj.joint_trajectory.points) > 0:
        return True, robot_traj

    return False, robot_traj


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


def quaternion_to_rpy(quat):
    x = quat.x
    y = quat.y
    z = quat.z
    w = quat.w
    norm = (x * x + y * y + z * z + w * w) ** 0.5
    if norm < 1e-9:
        return 0.0, 0.0, 0.0
    return euler_from_quaternion((x / norm, y / norm, z / norm, w / norm))


class PRMPlannerNode:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("prm_planner_node", anonymous=False)

        # Static-ish config
        self.group_name = rospy.get_param("~group", "manipulator")
        self.planner_id = rospy.get_param("~planner", "PRMstar")
        self.frame_id = rospy.get_param("~frame", "base_link")
        self.ee_link = rospy.get_param("~ee_link", "gripper_tip_link")

        planning_time = rospy.get_param("~planning_time", 10.0)
        attempts = rospy.get_param("~attempts", 10)
        self.planning_time = float(planning_time)
        self.planning_attempts = int(attempts)
        self.execute_motion = rospy.get_param("~execute", False)
        self.clear_octomap_before_plan = rospy.get_param("~clear_octomap_before_plan", True)
        self.clear_octomap_on_failure = rospy.get_param("~clear_octomap_on_failure", True)
        self.publish_plan_markers = rospy.get_param("~publish_plan_markers", True)
        self.save_plan_plot = rospy.get_param("~save_plan_plot", True)
        self.compute_ee_path_points = rospy.get_param(
            "~compute_ee_path_points",
            self.publish_plan_markers or self.save_plan_plot,
        )
        self.plan_plot_path = rospy.get_param("~plan_plot_path", "/tmp/prm_plan_plot.png")
        self.allow_gripper_internal_collisions = rospy.get_param("~allow_gripper_internal_collisions", True)
        self.gripper_internal_collision_pairs = rospy.get_param(
            "~gripper_internal_collision_pairs",
            [
                ["robotiq_coupler", "wrist_3_link"],
                ["robotiq_coupler", "robotiq_85_base_link"],
                ["robotiq_85_left_finger_tip_link", "robotiq_85_left_inner_knuckle_link"],
                ["robotiq_85_right_finger_tip_link", "robotiq_85_right_inner_knuckle_link"],
                ["robotiq_85_left_finger_link", "robotiq_85_left_finger_tip_link"],
                ["robotiq_85_right_finger_link", "robotiq_85_right_finger_tip_link"],
                ["robotiq_85_left_finger_link", "robotiq_85_left_knuckle_link"],
                ["robotiq_85_right_finger_link", "robotiq_85_right_knuckle_link"],
                ["robotiq_85_left_inner_knuckle_link", "robotiq_85_base_link"],
                ["robotiq_85_right_inner_knuckle_link", "robotiq_85_base_link"],
                ["robotiq_85_left_knuckle_link", "robotiq_85_base_link"],
                ["robotiq_85_right_knuckle_link", "robotiq_85_base_link"],
            ],
        )
        self.goal_position_tolerance = rospy.get_param("~goal_position_tolerance", 0.01)
        self.goal_orientation_tolerance = rospy.get_param("~goal_orientation_tolerance", 0.05)
        self.goal_joint_tolerance = rospy.get_param("~goal_joint_tolerance", 0.01)
        self.publish_target_tf = rospy.get_param("~publish_target_tf", True)
        self.target_tf_frame = rospy.get_param("~target_tf_frame", "prm_target_pose")
        self.target_tf_rate = float(rospy.get_param("~target_tf_rate", 10.0))
        self.target_axis_length = float(rospy.get_param("~target_axis_length", 0.08))
        self.target_axis_width = float(rospy.get_param("~target_axis_width", 0.006))
        self.cleared_target_pose_z = float(rospy.get_param("~cleared_target_pose_z", -10.0))
        self.plan_velocity_scaling = float(rospy.get_param("~plan_velocity_scaling", 1.0))
        self.plan_acceleration_scaling = float(rospy.get_param("~plan_acceleration_scaling", 1.0))
        self.bin_position_only = rospy.get_param("~bin_position_only", False)
        self.use_object_score_target = rospy.get_param("~use_object_score_target", True)
        self.object_score_topic = rospy.get_param("~object_score_topic", "/perception/object_score")
        self.target_object_id = rospy.get_param("~target_object_id", -1)
        self.target_label = rospy.get_param("~target_label", "")
        self.object_default_height = float(rospy.get_param("~object_default_height", 0.08))
        self.position_is_table_contact = rospy.get_param("~position_is_table_contact", True)
        self.stage3_mode = rospy.get_param("~stage3_mode", "object")  # "object" or "bin"
        self.stage3_object_id = rospy.get_param("~stage3_object_id", -1)
        self.stage3_label = rospy.get_param("~stage3_label", "")
        self.stage3_manual_fallback = rospy.get_param("~stage3_manual_fallback", True)
        self.stage3_x = rospy.get_param("~stage3_x", 0.45)
        self.stage3_y = rospy.get_param("~stage3_y", -0.05)
        self.stage3_z = rospy.get_param("~stage3_z", 0.30)
        self.fallback_planners = rospy.get_param(
            "~fallback_planners",
            ["RRTConnectkConfigDefault", "RRTstarkConfigDefault", "PRMkConfigDefault"],
        )
        self.execute_max_velocity = float(rospy.get_param("~execute_max_velocity", 1.0))
        self.execute_max_acceleration = float(rospy.get_param("~execute_max_acceleration", 1.0))
        self.action_planning_time = float(
            rospy.get_param("~action_planning_time", min(self.planning_time, 10.0))
        )
        self.action_attempts = int(rospy.get_param("~action_attempts", 1))
        self.action_use_fallback_planners = bool(
            rospy.get_param("~action_use_fallback_planners", False)
        )
        self.action_clear_octomap_before_plan = bool(
            rospy.get_param("~action_clear_octomap_before_plan", False)
        )
        self.action_clear_octomap_on_failure = bool(
            rospy.get_param("~action_clear_octomap_on_failure", False)
        )
        self.latest_objects = {}
        self.selected_object = None

        self.robot = moveit_commander.RobotCommander()
        self.group = moveit_commander.MoveGroupCommander(self.group_name)
        self.clear_octomap_srv = None
        self.state_validity_srv = None
        self.compute_fk_srv = None
        self.apply_scene_srv = None
        self.get_scene_srv = None
        self.last_failure_detail = "none"
        self._target_tf_lock = threading.Lock()
        self._latest_target_pose = None

        self.path_marker_pub = rospy.Publisher("~ee_path_marker", Marker, queue_size=1, latch=True)
        self.target_marker_pub = rospy.Publisher("~target_marker", Marker, queue_size=1, latch=True)
        self.target_pose_pub = rospy.Publisher("~target_pose", PoseStamped, queue_size=1, latch=True)
        self.target_axis_pub = rospy.Publisher("~target_pose_axis", Marker, queue_size=1, latch=True)
        self.status_marker_pub = rospy.Publisher("~status_marker", Marker, queue_size=1, latch=True)
        self.target_pose_preview_sub = rospy.Subscriber(
            "/path_planning/target_pose_preview",
            PoseStamped,
            self._target_pose_preview_cb,
            queue_size=1,
        )
        self.target_tf_broadcaster = tf2_ros.TransformBroadcaster() if self.publish_target_tf else None
        self.target_tf_timer = None
        self.motion_control_client = actionlib.SimpleActionClient(
            "/motion_control/execute_trajectory",
            _ExecuteTrajectoryActionSpec,
        )
        self.plan_execute_pose_server = actionlib.SimpleActionServer(
            "/path_planning/plan_execute_pose",
            _PlanExecutePoseActionSpec,
            execute_cb=self._execute_pose_action_cb,
            auto_start=False,
        )

        self.group.set_pose_reference_frame(self.frame_id)
        self.group.set_planner_id(self.planner_id)
        self.group.set_planning_time(planning_time)
        self.group.set_num_planning_attempts(attempts)
        self.group.set_goal_position_tolerance(self.goal_position_tolerance)
        self.group.set_goal_orientation_tolerance(self.goal_orientation_tolerance)
        self.group.set_goal_joint_tolerance(self.goal_joint_tolerance)
        self.group.set_max_velocity_scaling_factor(max(0.01, min(1.0, self.plan_velocity_scaling)))
        self.group.set_max_acceleration_scaling_factor(max(0.01, min(1.0, self.plan_acceleration_scaling)))
        self.group.allow_replanning(True)
        self.group.set_start_state_to_current_state()

        rospy.loginfo(
            "[PRM] Config: group=%s planner=%s frame=%s ee_link=%s planning_time=%.2f attempts=%d execute=%s",
            self.group_name,
            self.planner_id,
            self.frame_id,
            self.ee_link,
            planning_time,
            attempts,
            str(self.execute_motion),
        )
        rospy.loginfo(
            "[PRM] Tolerances: pos=%.4f orient=%.4f joint=%.4f bin_position_only=%s",
            self.goal_position_tolerance,
            self.goal_orientation_tolerance,
            self.goal_joint_tolerance,
            str(self.bin_position_only),
        )
        rospy.loginfo(
            "[PRM] MoveIt scaling: plan_velocity=%.2f plan_acceleration=%.2f execute_velocity=%.2f execute_acceleration=%.2f",
            self.plan_velocity_scaling,
            self.plan_acceleration_scaling,
            self.execute_max_velocity,
            self.execute_max_acceleration,
        )
        rospy.loginfo(
            "[PRM] Action planning policy: time=%.2f attempts=%d use_fallback_planners=%s clear_octomap_before_plan=%s clear_octomap_on_failure=%s",
            self.action_planning_time,
            self.action_attempts,
            str(self.action_use_fallback_planners),
            str(self.action_clear_octomap_before_plan),
            str(self.action_clear_octomap_on_failure),
        )
        rospy.loginfo(
            "[PRM] Octomap clearing: before_plan=%s on_failure=%s",
            str(self.clear_octomap_before_plan),
            str(self.clear_octomap_on_failure),
        )
        rospy.loginfo(
            "[PRM] Visualization: publish_markers=%s save_plot=%s compute_ee_path_points=%s plot_path=%s",
            str(self.publish_plan_markers),
            str(self.save_plan_plot),
            str(self.compute_ee_path_points),
            self.plan_plot_path,
        )
        rospy.loginfo(
            "[PRM] ACM internal gripper collisions allowed=%s",
            str(self.allow_gripper_internal_collisions),
        )
        rospy.loginfo(
            "[PRM] Object target mode: enabled=%s topic=%s target_object_id=%d target_label='%s'",
            str(self.use_object_score_target),
            self.object_score_topic,
            int(self.target_object_id),
            self.target_label,
        )
        rospy.loginfo(
            "[PRM] Stage3 target mode: mode=%s stage3_object_id=%d stage3_label='%s'",
            self.stage3_mode,
            int(self.stage3_object_id),
            self.stage3_label,
        )
        rospy.loginfo(
            "[PRM] Stage3 manual fallback: enabled=%s stage3=(%.3f, %.3f, %.3f)",
            str(self.stage3_manual_fallback),
            self.stage3_x,
            self.stage3_y,
            self.stage3_z,
        )
        rospy.loginfo("[PRM] Fallback planners: %s", str(self.fallback_planners))

        if self.use_object_score_target:
            rospy.Subscriber(self.object_score_topic, ObjectScore, self._on_object_score, queue_size=20)

        rospy.Service("~plan_pregrasp", Trigger, self._srv_plan_pregrasp)
        rospy.Service("~plan_grasp", Trigger, self._srv_plan_grasp)
        rospy.Service("~plan_to_bin", Trigger, self._srv_plan_to_bin)
        rospy.Service("/path_planning/clear_target_pose", Trigger, self._srv_clear_target_pose)

        if self.allow_gripper_internal_collisions:
            self._allow_gripper_internal_collisions()

        if self.publish_target_tf and self.target_tf_rate > 0.0:
            self.target_tf_timer = rospy.Timer(
                rospy.Duration(1.0 / self.target_tf_rate),
                self._target_tf_timer_cb,
            )

        self._clear_target_visuals(log_message=False)
        self.plan_execute_pose_server.start()
        rospy.loginfo("[PRM] PRM planner node ready.")

    def _publish_plan_execute_feedback(self, stage, message):
        feedback = PlanExecutePoseFeedback()
        feedback.stage = stage
        feedback.message = message
        self.plan_execute_pose_server.publish_feedback(feedback)

    def _make_plan_execute_result(self, success, status, message, trajectory=None):
        result = PlanExecutePoseResult()
        result.success = success
        result.status = status
        result.message = message
        if trajectory is not None:
            result.trajectory = trajectory
        return result

    def _validate_plan_execute_goal(self, goal):
        target_pose = goal.target_pose
        if not target_pose.header.frame_id:
            return False, "target_pose.header.frame_id is required"
        if goal.position_only:
            return True, ""

        q = target_pose.pose.orientation
        if q.x == 0.0 and q.y == 0.0 and q.z == 0.0 and q.w == 0.0:
            return False, "target_pose orientation is invalid (zero quaternion)"
        return True, ""

    def _set_group_planning_config(self, planning_time, attempts):
        self.group.set_planning_time(float(planning_time))
        self.group.set_num_planning_attempts(int(attempts))

    def _plan_action_target(self, goal):
        saved_clear_before = self.clear_octomap_before_plan
        saved_clear_on_failure = self.clear_octomap_on_failure
        saved_fallback_planners = list(self.fallback_planners)

        self.clear_octomap_before_plan = self.action_clear_octomap_before_plan
        self.clear_octomap_on_failure = self.action_clear_octomap_on_failure
        if not self.action_use_fallback_planners:
            self.fallback_planners = []
        self._set_group_planning_config(self.action_planning_time, self.action_attempts)

        try:
            return self.plan_to_pose(
                goal.target_pose,
                label="action_target",
                position_only=goal.position_only,
                execute_motion=False,
            )
        finally:
            self.clear_octomap_before_plan = saved_clear_before
            self.clear_octomap_on_failure = saved_clear_on_failure
            self.fallback_planners = saved_fallback_planners
            self._set_group_planning_config(self.planning_time, self.planning_attempts)

    def _execute_pose_action_cb(self, goal):
        is_valid, message = self._validate_plan_execute_goal(goal)
        if not is_valid:
            result = self._make_plan_execute_result(
                False,
                PlanExecutePoseResult.INVALID_GOAL,
                message,
            )
            self.plan_execute_pose_server.set_aborted(result, message)
            return

        if self.plan_execute_pose_server.is_preempt_requested():
            result = self._make_plan_execute_result(
                False,
                PlanExecutePoseResult.PREEMPTED,
                "Path planning goal preempted before planning started",
            )
            self.plan_execute_pose_server.set_preempted(result, result.message)
            return

        self._publish_plan_execute_feedback(
            PlanExecutePoseFeedback.PLANNING,
            "Planning with MoveIt/PRM",
        )
        rospy.loginfo(
            "[PRM] Action target planning budget: time=%.2fs attempts=%d fallback_planners=%s octomap_retry=%s",
            self.action_planning_time,
            self.action_attempts,
            str(self.action_use_fallback_planners),
            str(self.action_clear_octomap_on_failure),
        )
        success, robot_traj = self._plan_action_target(goal)

        if self.plan_execute_pose_server.is_preempt_requested():
            result = self._make_plan_execute_result(
                False,
                PlanExecutePoseResult.PREEMPTED,
                "Path planning goal preempted after planning",
                robot_traj.joint_trajectory if robot_traj is not None else None,
            )
            self.plan_execute_pose_server.set_preempted(result, result.message)
            return

        if not success or robot_traj is None or not hasattr(robot_traj, "joint_trajectory"):
            result = self._make_plan_execute_result(
                False,
                PlanExecutePoseResult.PLANNING_FAILED,
                f"Planning failed: {self.last_failure_detail}",
                robot_traj.joint_trajectory if robot_traj is not None else None,
            )
            self.plan_execute_pose_server.set_aborted(result, result.message)
            return

        if not self.motion_control_client.wait_for_server(timeout=rospy.Duration(1.0)):
            result = self._make_plan_execute_result(
                False,
                PlanExecutePoseResult.EXECUTION_FAILED,
                "motion_control action server is unavailable",
                robot_traj.joint_trajectory,
            )
            self.plan_execute_pose_server.set_aborted(result, result.message)
            return

        execute_goal = ExecuteTrajectoryGoal()
        execute_goal.trajectory = robot_traj.joint_trajectory
        execute_goal.max_velocity = self.execute_max_velocity
        execute_goal.max_acceleration = self.execute_max_acceleration
        traj_points = len(robot_traj.joint_trajectory.points)
        traj_duration = (
            robot_traj.joint_trajectory.points[-1].time_from_start.to_sec()
            if traj_points > 0
            else 0.0
        )

        self._publish_plan_execute_feedback(
            PlanExecutePoseFeedback.EXECUTING,
            "Executing trajectory via motion_control",
        )
        rospy.loginfo(
            "[PRM] Sending trajectory to motion_control: points=%d duration=%.2fs vel_scale=%.2f acc_scale=%.2f",
            traj_points,
            traj_duration,
            self.execute_max_velocity,
            self.execute_max_acceleration,
        )
        self.motion_control_client.send_goal(execute_goal)
        rospy.loginfo("[PRM] motion_control goal sent, waiting for execution result...")
        execute_wait_start = rospy.Time.now()
        last_wait_log = execute_wait_start

        while not rospy.is_shutdown():
            if self.plan_execute_pose_server.is_preempt_requested():
                self.motion_control_client.cancel_goal()
                result = self._make_plan_execute_result(
                    False,
                    PlanExecutePoseResult.PREEMPTED,
                    "Path planning goal preempted during execution",
                    robot_traj.joint_trajectory,
                )
                self.plan_execute_pose_server.set_preempted(result, result.message)
                return

            now = rospy.Time.now()
            if (now - last_wait_log).to_sec() >= 2.0:
                rospy.loginfo(
                    "[PRM] Waiting for motion_control execution result... elapsed=%.1fs",
                    (now - execute_wait_start).to_sec(),
                )
                last_wait_log = now

            if self.motion_control_client.wait_for_result(timeout=rospy.Duration(0.1)):
                motion_result = self.motion_control_client.get_result()
                motion_state = self.motion_control_client.get_state()
                rospy.loginfo(
                    "[PRM] motion_control execution finished: state=%d success=%s status=%s message=%s",
                    motion_state,
                    str(motion_result.success if motion_result is not None else False),
                    str(motion_result.status if motion_result is not None else "None"),
                    motion_result.message if motion_result is not None else "no result",
                )

                if motion_state == action_msgs.GoalStatus.SUCCEEDED and motion_result is not None and motion_result.success:
                    result = self._make_plan_execute_result(
                        True,
                        PlanExecutePoseResult.SUCCEEDED,
                        motion_result.message or "Trajectory executed successfully",
                        robot_traj.joint_trajectory,
                    )
                    self.plan_execute_pose_server.set_succeeded(result, result.message)
                    return

                if motion_result is not None and motion_result.status == ExecuteTrajectoryResult.PREEMPTED:
                    result = self._make_plan_execute_result(
                        False,
                        PlanExecutePoseResult.PREEMPTED,
                        motion_result.message or "Execution preempted",
                        robot_traj.joint_trajectory,
                    )
                    self.plan_execute_pose_server.set_preempted(result, result.message)
                    return

                failure_message = "Execution failed"
                if motion_result is not None and motion_result.message:
                    failure_message = motion_result.message
                else:
                    failure_message = f"Execution failed with motion_control state {motion_state}"
                result = self._make_plan_execute_result(
                    False,
                    PlanExecutePoseResult.EXECUTION_FAILED,
                    failure_message,
                    robot_traj.joint_trajectory,
                )
                self.plan_execute_pose_server.set_aborted(result, result.message)
                return

        result = self._make_plan_execute_result(
            False,
            PlanExecutePoseResult.PREEMPTED,
            "Path planning action interrupted",
            robot_traj.joint_trajectory,
        )
        self.plan_execute_pose_server.set_preempted(result, result.message)

    def _ensure_state_validity_srv(self):
        if self.state_validity_srv is not None:
            return True
        try:
            rospy.wait_for_service("/check_state_validity", timeout=1.0)
            self.state_validity_srv = rospy.ServiceProxy("/check_state_validity", GetStateValidity)
            return True
        except Exception:
            return False

    def _ensure_scene_srvs(self):
        if self.apply_scene_srv is not None and self.get_scene_srv is not None:
            return True
        try:
            rospy.wait_for_service("/apply_planning_scene", timeout=1.0)
            rospy.wait_for_service("/get_planning_scene", timeout=1.0)
            self.apply_scene_srv = rospy.ServiceProxy("/apply_planning_scene", ApplyPlanningScene)
            self.get_scene_srv = rospy.ServiceProxy("/get_planning_scene", GetPlanningScene)
            return True
        except Exception as exc:
            rospy.logwarn("[PRM] planning scene services unavailable: %s", exc)
            return False

    def _allow_gripper_internal_collisions(self):
        if not self._ensure_scene_srvs():
            return False
        try:
            req = GetPlanningSceneRequest()
            req.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
            resp = self.get_scene_srv(req)
            acm = resp.scene.allowed_collision_matrix
        except Exception as exc:
            rospy.logwarn("[PRM] Failed to fetch ACM: %s", exc)
            return False

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

        updated = False
        for pair in self.gripper_internal_collision_pairs:
            if not isinstance(pair, (list, tuple)) or len(pair) != 2:
                continue
            a, b = str(pair[0]), str(pair[1])
            ia = ensure_name(a)
            ib = ensure_name(b)
            if not entries[ia].enabled[ib] or not entries[ib].enabled[ia]:
                entries[ia].enabled[ib] = True
                entries[ib].enabled[ia] = True
                updated = True
                rospy.loginfo("[PRM] ACM allow internal pair: %s <-> %s", a, b)

        if not updated:
            return True

        scene_msg = PlanningScene()
        scene_msg.is_diff = True
        scene_msg.allowed_collision_matrix.entry_names = names
        scene_msg.allowed_collision_matrix.entry_values = entries
        try:
            ok = self.apply_scene_srv(ApplyPlanningSceneRequest(scene=scene_msg)).success
            if not ok:
                rospy.logwarn("[PRM] Failed to apply ACM internal collision pairs.")
            return ok
        except Exception as exc:
            rospy.logwarn("[PRM] apply_planning_scene exception: %s", exc)
            return False

    def _ensure_fk_srv(self):
        if self.compute_fk_srv is not None:
            return True
        try:
            rospy.wait_for_service("/compute_fk", timeout=1.0)
            self.compute_fk_srv = rospy.ServiceProxy("/compute_fk", GetPositionFK)
            return True
        except Exception:
            return False

    def _make_hidden_target_pose(self):
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = self.cleared_target_pose_z
        pose.pose.orientation.w = 1.0
        return pose

    def _publish_delete_marker(self, publisher, namespace, marker_id):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = namespace
        marker.id = marker_id
        marker.action = Marker.DELETE
        publisher.publish(marker)

    def _clear_target_visuals(self, log_message=True):
        hidden_pose = self._make_hidden_target_pose()
        with self._target_tf_lock:
            self._latest_target_pose = copy.deepcopy(hidden_pose)

        self.target_pose_pub.publish(hidden_pose)
        self._broadcast_target_tf()
        self._publish_delete_marker(self.target_marker_pub, "prm_target", 1)
        self._publish_delete_marker(self.path_marker_pub, "prm_ee_path", 2)
        self._publish_delete_marker(self.status_marker_pub, "prm_status", 3)
        self._publish_delete_marker(self.target_axis_pub, "prm_target_pose_axis", 4)

        if log_message:
            rospy.loginfo(
                "[PRM] Cleared target pose visualization and moved %s below view (z=%.3f).",
                self.target_tf_frame,
                self.cleared_target_pose_z,
            )

    def _srv_clear_target_pose(self, _req):
        self._clear_target_visuals(log_message=True)
        return TriggerResponse(success=True, message="target_pose_cleared")

    def _publish_target_preview(self, target_pose):
        self.target_pose_pub.publish(target_pose)
        self._update_target_tf(target_pose)

        if not self.publish_plan_markers:
            return

        now = rospy.Time.now()

        target_marker = Marker()
        target_marker.header.frame_id = self.frame_id
        target_marker.header.stamp = now
        target_marker.ns = "prm_target"
        target_marker.id = 1
        target_marker.type = Marker.SPHERE
        target_marker.action = Marker.ADD
        target_marker.pose = target_pose.pose
        target_marker.scale.x = 0.035
        target_marker.scale.y = 0.035
        target_marker.scale.z = 0.035
        target_marker.color.a = 1.0
        target_marker.color.r = 1.0
        target_marker.color.g = 0.8
        target_marker.color.b = 0.1
        self.target_marker_pub.publish(target_marker)

        axis_marker = Marker()
        axis_marker.header.frame_id = self.frame_id
        axis_marker.header.stamp = now
        axis_marker.ns = "prm_target_pose_axis"
        axis_marker.id = 4
        axis_marker.type = Marker.LINE_LIST
        axis_marker.action = Marker.ADD
        axis_marker.pose.orientation.w = 1.0
        axis_marker.scale.x = self.target_axis_width
        axis_marker.points = []
        axis_marker.colors = []

        origin = target_pose.pose.position
        quat = target_pose.pose.orientation
        axis_specs = [
            ((self.target_axis_length, 0.0, 0.0), ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)),
            ((0.0, self.target_axis_length, 0.0), ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)),
            ((0.0, 0.0, self.target_axis_length), ColorRGBA(r=0.0, g=0.4, b=1.0, a=1.0)),
        ]
        for axis_vec, color in axis_specs:
            dx, dy, dz = rotate_vector_by_quaternion(quat, axis_vec)
            start = Point()
            start.x = origin.x
            start.y = origin.y
            start.z = origin.z
            end = Point()
            end.x = origin.x + dx
            end.y = origin.y + dy
            end.z = origin.z + dz
            axis_marker.points.extend([start, end])
            axis_marker.colors.extend([color, color])
        self.target_axis_pub.publish(axis_marker)

        self._publish_delete_marker(self.path_marker_pub, "prm_ee_path", 2)
        self._publish_delete_marker(self.status_marker_pub, "prm_status", 3)

    def _target_pose_preview_cb(self, msg):
        if not msg.header.frame_id:
            return
        self._publish_target_preview(msg)
        rospy.loginfo(
            "[PRM] Preview target pose updated: frame=%s pos=(%.3f, %.3f, %.3f)",
            msg.header.frame_id,
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        )

    def _update_target_tf(self, target_pose):
        if not self.publish_target_tf:
            return
        with self._target_tf_lock:
            self._latest_target_pose = copy.deepcopy(target_pose)
        self._broadcast_target_tf()

    def _broadcast_target_tf(self):
        if not self.publish_target_tf or self.target_tf_broadcaster is None:
            return
        with self._target_tf_lock:
            target_pose = copy.deepcopy(self._latest_target_pose)
        if target_pose is None:
            return

        tf_msg = TransformStamped()
        tf_msg.header.stamp = rospy.Time.now()
        tf_msg.header.frame_id = target_pose.header.frame_id or self.frame_id
        tf_msg.child_frame_id = self.target_tf_frame
        tf_msg.transform.translation.x = target_pose.pose.position.x
        tf_msg.transform.translation.y = target_pose.pose.position.y
        tf_msg.transform.translation.z = target_pose.pose.position.z
        tf_msg.transform.rotation = target_pose.pose.orientation
        self.target_tf_broadcaster.sendTransform(tf_msg)

    def _target_tf_timer_cb(self, _event):
        self._broadcast_target_tf()

    def _start_state_validity_info(self):
        if not self._ensure_state_validity_srv():
            return True, "state validity service unavailable"
        try:
            req = GetStateValidityRequest()
            req.robot_state = self.robot.get_current_state()
            req.group_name = self.group_name
            resp = self.state_validity_srv(req)
            if resp.valid:
                return True, "start state valid"
            if not resp.contacts:
                return False, "start state invalid (collision contact list empty)"
            unique_pairs = []
            seen_pairs = set()
            for contact in resp.contacts:
                pair = tuple(sorted((contact.contact_body_1, contact.contact_body_2)))
                if pair in seen_pairs:
                    continue
                seen_pairs.add(pair)
                unique_pairs.append("%s <-> %s" % pair)
            detail = "start state in collision: %s" % "; ".join(unique_pairs[:8])
            return False, detail
        except Exception as exc:
            return True, "state validity check failed: %s" % str(exc)

    def _clear_octomap(self):
        if self.clear_octomap_srv is None:
            try:
                rospy.wait_for_service("/clear_octomap", timeout=1.0)
                self.clear_octomap_srv = rospy.ServiceProxy("/clear_octomap", Empty)
            except Exception:
                return False
        try:
            self.clear_octomap_srv()
            rospy.loginfo("[PRM] clear_octomap requested before planning.")
            return True
        except Exception as exc:
            rospy.logwarn("[PRM] clear_octomap failed: %s", exc)
            return False

    def _ee_points_from_traj(self, robot_traj):
        if robot_traj is None or not hasattr(robot_traj, "joint_trajectory"):
            return []
        jt = robot_traj.joint_trajectory
        if len(jt.points) == 0:
            return []
        if not self._ensure_fk_srv():
            rospy.logwarn("[PRM] /compute_fk unavailable; cannot build 3D EE path points.")
            return []

        points = []
        for p in jt.points:
            req = GetPositionFKRequest()
            req.header.frame_id = self.frame_id
            req.fk_link_names = [self.ee_link]
            req.robot_state = self.robot.get_current_state()
            req.robot_state.joint_state.name = list(jt.joint_names)
            req.robot_state.joint_state.position = list(p.positions)
            try:
                resp = self.compute_fk_srv(req)
                if resp.pose_stamped:
                    pose = resp.pose_stamped[0].pose.position
                    points.append((pose.x, pose.y, pose.z))
            except Exception:
                continue
        return points

    def _publish_plan_markers(self, ee_points, target_pose, success, label, status_text):
        self.target_pose_pub.publish(target_pose)
        self._update_target_tf(target_pose)
        if not self.publish_plan_markers:
            return

        now = rospy.Time.now()

        target_marker = Marker()
        target_marker.header.frame_id = self.frame_id
        target_marker.header.stamp = now
        target_marker.ns = "prm_target"
        target_marker.id = 1
        target_marker.type = Marker.SPHERE
        target_marker.action = Marker.ADD
        target_marker.pose = target_pose.pose
        target_marker.scale.x = 0.035
        target_marker.scale.y = 0.035
        target_marker.scale.z = 0.035
        target_marker.color.a = 1.0
        target_marker.color.r = 0.1 if success else 1.0
        target_marker.color.g = 1.0 if success else 0.2
        target_marker.color.b = 0.1
        self.target_marker_pub.publish(target_marker)

        axis_marker = Marker()
        axis_marker.header.frame_id = self.frame_id
        axis_marker.header.stamp = now
        axis_marker.ns = "prm_target_pose_axis"
        axis_marker.id = 4
        axis_marker.type = Marker.LINE_LIST
        axis_marker.action = Marker.ADD
        axis_marker.pose.orientation.w = 1.0
        axis_marker.scale.x = self.target_axis_width
        axis_marker.points = []
        axis_marker.colors = []

        origin = target_pose.pose.position
        quat = target_pose.pose.orientation
        axis_specs = [
            ((self.target_axis_length, 0.0, 0.0), ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)),
            ((0.0, self.target_axis_length, 0.0), ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)),
            ((0.0, 0.0, self.target_axis_length), ColorRGBA(r=0.0, g=0.4, b=1.0, a=1.0)),
        ]
        for axis_vec, color in axis_specs:
            dx, dy, dz = rotate_vector_by_quaternion(quat, axis_vec)
            start = Point()
            start.x = origin.x
            start.y = origin.y
            start.z = origin.z
            end = Point()
            end.x = origin.x + dx
            end.y = origin.y + dy
            end.z = origin.z + dz
            axis_marker.points.extend([start, end])
            axis_marker.colors.extend([color, color])
        self.target_axis_pub.publish(axis_marker)

        path_marker = Marker()
        path_marker.header.frame_id = self.frame_id
        path_marker.header.stamp = now
        path_marker.ns = "prm_ee_path"
        path_marker.id = 2
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.scale.x = 0.01
        path_marker.color.a = 1.0
        path_marker.color.r = 0.1 if success else 1.0
        path_marker.color.g = 0.8 if success else 0.2
        path_marker.color.b = 1.0 if success else 0.2
        path_marker.points = []
        for x, y, z in ee_points:
            pt = Point()
            pt.x = x
            pt.y = y
            pt.z = z
            path_marker.points.append(pt)
        self.path_marker_pub.publish(path_marker)

        status_marker = Marker()
        status_marker.header.frame_id = self.frame_id
        status_marker.header.stamp = now
        status_marker.ns = "prm_status"
        status_marker.id = 3
        status_marker.type = Marker.TEXT_VIEW_FACING
        status_marker.action = Marker.ADD
        status_marker.pose.position.x = target_pose.pose.position.x
        status_marker.pose.position.y = target_pose.pose.position.y
        status_marker.pose.position.z = target_pose.pose.position.z + 0.08
        status_marker.pose.orientation.w = 1.0
        status_marker.scale.z = 0.04
        status_marker.color.a = 1.0
        status_marker.color.r = 0.1 if success else 1.0
        status_marker.color.g = 1.0 if success else 0.2
        status_marker.color.b = 0.1
        status_marker.text = "%s: %s" % (label, status_text)
        self.status_marker_pub.publish(status_marker)

    def _save_plan_plot(self, ee_points, target_pose, success, label, status_text):
        if not self.save_plan_plot:
            return
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
        except Exception as exc:
            rospy.logwarn("[PRM] Matplotlib unavailable for plot save: %s", exc)
            return

        xs, ys, zs = [], [], []
        for x, y, z in ee_points:
            xs.append(x)
            ys.append(y)
            zs.append(z)

        fig = plt.figure(figsize=(8, 6))
        ax = fig.add_subplot(111, projection="3d")
        if xs:
            ax.plot(xs, ys, zs, color="tab:blue" if success else "tab:red", linewidth=2.5, label="EE path")
            ax.scatter(xs[0], ys[0], zs[0], color="black", s=30, label="start")
            ax.scatter(xs[-1], ys[-1], zs[-1], color="tab:green" if success else "tab:red", s=35, label="stop")
        tx = target_pose.pose.position.x
        ty = target_pose.pose.position.y
        tz = target_pose.pose.position.z
        ax.scatter(tx, ty, tz, color="gold", s=60, label="target")
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.set_title("PRM %s | %s" % (label, "SUCCESS" if success else "FAILED"))
        ax.legend(loc="best")
        ax.text2D(0.02, 0.02, status_text, transform=ax.transAxes, fontsize=9)
        fig.tight_layout()

        out_path = self.plan_plot_path
        out_dir = os.path.dirname(out_path) or "."
        os.makedirs(out_dir, exist_ok=True)
        fig.savefig(out_path, dpi=130)
        plt.close(fig)
        rospy.loginfo("[PRM] Saved 3D plan plot: %s", out_path)

    def _read_runtime_params(self):
        """
        Re-read runtime parameters before each plan call so you can test
        different targets from terminal without restarting the node.
        """
        # Safer defaults inside UR5e reachable workspace (table pick zone)
        self.target_x = rospy.get_param("~target_x", 0.35)
        self.target_y = rospy.get_param("~target_y", -0.10)
        self.target_z = rospy.get_param("~target_z", 0.30)

        self.pregrasp_offset_z = rospy.get_param("~pregrasp_offset_z", 0.12)
        self.grasp_offset_z = rospy.get_param("~grasp_offset_z", 0.02)

        # Default bin pose tuned for the current UR5e drop motion tests.
        self.bin_x = rospy.get_param("~bin_x", 0.50)
        self.bin_y = rospy.get_param("~bin_y", -0.30)
        self.bin_z = rospy.get_param("~bin_z", 0.30)
        self.bin_rx = rospy.get_param("~bin_rx", math.pi / 2.0)
        self.bin_ry = rospy.get_param("~bin_ry", math.pi)
        self.bin_rz = rospy.get_param("~bin_rz", 0.0)

        # Prefer roll/pitch/yaw params for human-facing workflows. Legacy qx/qy/qz/qw
        # are still accepted as fallback so older scripts do not break.
        rx = rospy.get_param("~rx", None)
        ry = rospy.get_param("~ry", None)
        rz = rospy.get_param("~rz", None)
        if rx is None or ry is None or rz is None:
            qx = rospy.get_param("~qx", None)
            qy = rospy.get_param("~qy", None)
            qz = rospy.get_param("~qz", None)
            qw = rospy.get_param("~qw", None)
            if None not in (qx, qy, qz, qw):
                class _Quat:
                    pass

                quat = _Quat()
                quat.x = float(qx)
                quat.y = float(qy)
                quat.z = float(qz)
                quat.w = float(qw)
                self.rx, self.ry, self.rz = quaternion_to_rpy(quat)
            else:
                self.rx, self.ry, self.rz = 0.0, 0.0, 0.0
        else:
            self.rx = float(rx)
            self.ry = float(ry)
            self.rz = float(rz)
        self.use_object_score_target = rospy.get_param("~use_object_score_target", self.use_object_score_target)
        self.target_object_id = rospy.get_param("~target_object_id", self.target_object_id)
        self.target_label = rospy.get_param("~target_label", self.target_label)
        self.stage3_mode = rospy.get_param("~stage3_mode", self.stage3_mode)
        self.stage3_object_id = rospy.get_param("~stage3_object_id", self.stage3_object_id)
        self.stage3_label = rospy.get_param("~stage3_label", self.stage3_label)
        self.stage3_manual_fallback = rospy.get_param("~stage3_manual_fallback", self.stage3_manual_fallback)
        self.stage3_x = rospy.get_param("~stage3_x", self.stage3_x)
        self.stage3_y = rospy.get_param("~stage3_y", self.stage3_y)
        self.stage3_z = rospy.get_param("~stage3_z", self.stage3_z)
        self.bin_rx = rospy.get_param("~bin_rx", self.bin_rx)
        self.bin_ry = rospy.get_param("~bin_ry", self.bin_ry)
        self.bin_rz = rospy.get_param("~bin_rz", self.bin_rz)

        rospy.loginfo(
            "[PRM] Runtime params:"
            " target=(%.3f, %.3f, %.3f)"
            " pregrasp_offset_z=%.3f grasp_offset_z=%.3f"
            " target_rpy=(%.3f, %.3f, %.3f)"
            " bin=(%.3f, %.3f, %.3f)"
            " bin_rpy=(%.3f, %.3f, %.3f)",
            self.target_x,
            self.target_y,
            self.target_z,
            self.pregrasp_offset_z,
            self.grasp_offset_z,
            self.rx,
            self.ry,
            self.rz,
            self.bin_x,
            self.bin_y,
            self.bin_z,
            self.bin_rx,
            self.bin_ry,
            self.bin_rz,
        )

    def _on_object_score(self, msg):
        self.latest_objects[msg.object_id] = msg

        if self.target_object_id >= 0 and msg.object_id == self.target_object_id:
            self.selected_object = msg
            return
        if self.target_label and msg.label.lower() == self.target_label.lower():
            self.selected_object = msg
            return
        if self.target_object_id < 0 and not self.target_label:
            self.selected_object = msg

    def _object_target_xyz(self):
        if not self.use_object_score_target:
            return None

        msg = None
        if self.target_object_id >= 0:
            msg = self.latest_objects.get(self.target_object_id, None)
        elif self.target_label:
            for obj in self.latest_objects.values():
                if obj.label.lower() == self.target_label.lower():
                    msg = obj
                    break
        else:
            msg = self.selected_object

        if msg is None:
            return None

        x = msg.position.x
        y = msg.position.y
        z = msg.position.z
        if self.position_is_table_contact:
            z += self.object_default_height * 0.5
        return x, y, z, msg.object_id, msg.label

    def _object_xyz_from_msg(self, msg):
        x = msg.position.x
        y = msg.position.y
        z = msg.position.z
        if self.position_is_table_contact:
            z += self.object_default_height * 0.5
        return x, y, z, msg.object_id, msg.label

    def _stage3_object_xyz(self, excluded_object_id=-1):
        if self.stage3_object_id >= 0:
            msg = self.latest_objects.get(self.stage3_object_id, None)
            if msg is not None:
                return self._object_xyz_from_msg(msg)
            return None

        if self.stage3_label:
            for obj in self.latest_objects.values():
                if obj.label.lower() == self.stage3_label.lower() and obj.object_id != excluded_object_id:
                    return self._object_xyz_from_msg(obj)
            return None

        candidates = [obj for obj in self.latest_objects.values() if obj.object_id != excluded_object_id]
        if not candidates:
            return None
        # Prefer high-score objects; stable tie-break by object_id.
        candidates.sort(key=lambda o: (-float(getattr(o, "total_score", 0.0)), int(o.object_id)))
        return self._object_xyz_from_msg(candidates[0])

    def _make_pose(self, x, y, z, rx=None, ry=None, rz=None):
        target = PoseStamped()
        target.header.frame_id = self.frame_id
        target.header.stamp = rospy.Time.now()
        target.pose.position.x = x
        target.pose.position.y = y
        target.pose.position.z = z
        if rx is None:
            rx = self.rx
        if ry is None:
            ry = self.ry
        if rz is None:
            rz = self.rz
        qx, qy, qz, qw = quaternion_from_euler(rx, ry, rz)
        target.pose.orientation.x = qx
        target.pose.orientation.y = qy
        target.pose.orientation.z = qz
        target.pose.orientation.w = qw
        return target

    def _plan_once(self):
        plan_result = self.group.plan()
        success, robot_traj = extract_plan(plan_result)
        return success, robot_traj

    def _plan_with_fallbacks(self):
        planner_order = [self.planner_id] + [p for p in self.fallback_planners if p != self.planner_id]
        for planner in planner_order:
            try:
                self.group.set_planner_id(planner)
                success, robot_traj = self._plan_once()
                if success:
                    if planner != self.planner_id:
                        rospy.loginfo("[PRM] Planned successfully with fallback planner: %s", planner)
                    return True, robot_traj
                rospy.logwarn("[PRM] Planner %s returned success=False", planner)
            except Exception as exc:
                rospy.logwarn("[PRM] Planner %s raised exception: %s", planner, exc)
        return False, None

    def plan_to_pose(self, pose_stamped, label="target", position_only=False, execute_motion=None):
        self.last_failure_detail = "none"
        should_execute_motion = self.execute_motion if execute_motion is None else execute_motion
        if self.allow_gripper_internal_collisions:
            self._allow_gripper_internal_collisions()
        if self.clear_octomap_before_plan:
            self._clear_octomap()

        self.group.set_start_state_to_current_state()
        valid_start, start_detail = self._start_state_validity_info()
        if not valid_start:
            rospy.logwarn("[PRM] Pre-check for %s: %s", label, start_detail)
        if position_only:
            rospy.logwarn("[PRM] %s is using position_only=True, so orientation will not be enforced.", label)
            self.group.set_position_target(
                [
                    pose_stamped.pose.position.x,
                    pose_stamped.pose.position.y,
                    pose_stamped.pose.position.z,
                ],
                self.ee_link,
            )
        else:
            self.group.set_pose_target(pose_stamped, self.ee_link)

        rx, ry, rz = quaternion_to_rpy(pose_stamped.pose.orientation)
        rospy.loginfo(
            "[PRM] Planning to %s: frame=%s pos=(%.3f, %.3f, %.3f) rpy=(%.3f, %.3f, %.3f) position_only=%s",
            label,
            pose_stamped.header.frame_id,
            pose_stamped.pose.position.x,
            pose_stamped.pose.position.y,
            pose_stamped.pose.position.z,
            rx,
            ry,
            rz,
            str(position_only),
        )

        try:
            success, robot_traj = self._plan_with_fallbacks()
        except Exception as exc:
            self.group.clear_pose_targets()
            self.group.clear_path_constraints()
            rospy.logerr("[PRM] Exception during planning to %s: %s", label, exc)
            return False, None

        self.group.clear_pose_targets()
        self.group.clear_path_constraints()

        if not success:
            if self.clear_octomap_on_failure:
                rospy.logwarn("[PRM] First attempt failed, clearing octomap and retrying once.")
                self._clear_octomap()
                self.group.set_start_state_to_current_state()
                if position_only:
                    self.group.set_position_target(
                        [
                            pose_stamped.pose.position.x,
                            pose_stamped.pose.position.y,
                            pose_stamped.pose.position.z,
                        ],
                        self.ee_link,
                    )
                else:
                    self.group.set_pose_target(pose_stamped, self.ee_link)
                try:
                    success, robot_traj = self._plan_with_fallbacks()
                except Exception:
                    success, robot_traj = False, None
                self.group.clear_pose_targets()
                self.group.clear_path_constraints()

                if success and robot_traj is not None and hasattr(robot_traj, "joint_trajectory") \
                        and len(robot_traj.joint_trajectory.points) > 0:
                    pts = robot_traj.joint_trajectory.points
                    rospy.loginfo(
                        "[PRM] Path to %s found after octomap retry. points=%d duration=%.2fs",
                        label,
                        len(pts),
                        pts[-1].time_from_start.to_sec(),
                    )
                    publish_display_trajectory(self.robot, robot_traj)
                    return True, robot_traj

            valid_after, after_detail = self._start_state_validity_info()
            if not valid_after:
                self.last_failure_detail = after_detail
            else:
                self.last_failure_detail = "planner returned success=False (no collision-free path found)"
            self._publish_plan_markers([], pose_stamped, False, label, self.last_failure_detail)
            self._save_plan_plot([], pose_stamped, False, label, self.last_failure_detail)
            rospy.logerr("[PRM] Planning to %s failed: MoveIt returned success=False", label)
            rospy.logerr("[PRM] Failure detail: %s", self.last_failure_detail)
            return False, None

        if robot_traj is None:
            rospy.logerr("[PRM] Planning to %s failed: robot_traj is None", label)
            return False, None

        if not hasattr(robot_traj, "joint_trajectory"):
            rospy.logerr("[PRM] Planning to %s failed: returned object has no joint_trajectory", label)
            return False, None

        if len(robot_traj.joint_trajectory.points) == 0:
            rospy.logerr("[PRM] Planning to %s failed: trajectory has 0 points", label)
            return False, robot_traj

        pts = robot_traj.joint_trajectory.points
        rospy.loginfo(
            "[PRM] Path to %s found. points=%d duration=%.2fs",
            label,
            len(pts),
            pts[-1].time_from_start.to_sec(),
        )

        publish_display_trajectory(self.robot, robot_traj)
        rospy.loginfo("[PRM] Published planned trajectory to /move_group/display_planned_path")
        ee_points = self._ee_points_from_traj(robot_traj) if self.compute_ee_path_points else []
        self._publish_plan_markers(ee_points, pose_stamped, True, label, "planned")
        self._save_plan_plot(ee_points, pose_stamped, True, label, "planned")

        if should_execute_motion:
            rospy.loginfo("[PRM] Executing trajectory to %s...", label)
            ok = self.group.execute(robot_traj, wait=True)
            self.group.stop()
            if not ok:
                rospy.logwarn("[PRM] Execution to %s failed", label)
                return False, robot_traj
            rospy.loginfo("[PRM] Execution to %s completed", label)

        return True, robot_traj

    def plan_pregrasp(self):
        self._read_runtime_params()
        obj_xyz = self._object_target_xyz()
        if obj_xyz is not None:
            self.target_x, self.target_y, self.target_z = obj_xyz[0], obj_xyz[1], obj_xyz[2]
            rospy.loginfo(
                "[PRM] Using object target for pregrasp: id=%d label=%s pos=(%.3f, %.3f, %.3f)",
                obj_xyz[3], obj_xyz[4], self.target_x, self.target_y, self.target_z,
            )
        pose = self._make_pose(
            self.target_x,
            self.target_y,
            self.target_z + self.pregrasp_offset_z,
        )
        return self.plan_to_pose(pose, label="pregrasp")

    def plan_grasp(self):
        self._read_runtime_params()
        obj_xyz = self._object_target_xyz()
        if obj_xyz is not None:
            self.target_x, self.target_y, self.target_z = obj_xyz[0], obj_xyz[1], obj_xyz[2]
            rospy.loginfo(
                "[PRM] Using object target for grasp: id=%d label=%s pos=(%.3f, %.3f, %.3f)",
                obj_xyz[3], obj_xyz[4], self.target_x, self.target_y, self.target_z,
            )
        pose = self._make_pose(
            self.target_x,
            self.target_y,
            self.target_z + self.grasp_offset_z,
        )
        return self.plan_to_pose(pose, label="grasp")

    def plan_to_bin(self):
        self._read_runtime_params()
        if str(self.stage3_mode).lower() == "object":
            pick_obj = self._object_target_xyz() if self.use_object_score_target else None
            excluded_id = pick_obj[3] if pick_obj is not None else -1
            place_obj = self._stage3_object_xyz(excluded_object_id=excluded_id)
            if place_obj is None:
                if not self.stage3_manual_fallback:
                    rospy.logerr("[PRM] Stage3 object mode enabled but no secondary object is available.")
                    return False, None
                rospy.logwarn(
                    "[PRM] No secondary object available. Falling back to manual stage3 pose (%.3f, %.3f, %.3f).",
                    self.stage3_x, self.stage3_y, self.stage3_z,
                )
                pose = self._make_pose(self.stage3_x, self.stage3_y, self.stage3_z + self.pregrasp_offset_z)
                ok, traj = self.plan_to_pose(pose, label="stage3_manual_fallback", position_only=True)
                if not ok:
                    return self.plan_to_pose(pose, label="stage3_manual_fallback_full_pose", position_only=False)
                return ok, traj

            rospy.loginfo(
                "[PRM] Stage3 using table object: id=%d label=%s pos=(%.3f, %.3f, %.3f)",
                place_obj[3], place_obj[4], place_obj[0], place_obj[1], place_obj[2],
            )
            pose = self._make_pose(place_obj[0], place_obj[1], place_obj[2] + self.pregrasp_offset_z)
            ok, traj = self.plan_to_pose(pose, label="stage3_object", position_only=True)
            if not ok:
                rospy.logwarn("[PRM] Stage3 object position-only planning failed, retrying full pose.")
                return self.plan_to_pose(pose, label="stage3_object_full_pose", position_only=False)
            return ok, traj

        pose = self._make_pose(self.bin_x, self.bin_y, self.bin_z, self.bin_rx, self.bin_ry, self.bin_rz)
        ok, traj = self.plan_to_pose(pose, label="bin", position_only=self.bin_position_only)
        if not ok and self.bin_position_only:
            rospy.logwarn("[PRM] Bin position-only planning failed, retrying with full pose target.")
            return self.plan_to_pose(pose, label="bin_full_pose", position_only=False)
        return ok, traj

    def _srv_plan_pregrasp(self, _req):
        ok, _ = self.plan_pregrasp()
        msg = "pregrasp planned" if ok else ("pregrasp planning failed: %s" % self.last_failure_detail)
        return TriggerResponse(success=ok, message=msg)

    def _srv_plan_grasp(self, _req):
        ok, _ = self.plan_grasp()
        msg = "grasp planned" if ok else ("grasp planning failed: %s" % self.last_failure_detail)
        return TriggerResponse(success=ok, message=msg)

    def _srv_plan_to_bin(self, _req):
        ok, _ = self.plan_to_bin()
        if str(self.stage3_mode).lower() == "object":
            msg = "stage3 object planned" if ok else ("stage3 object planning failed: %s" % self.last_failure_detail)
        else:
            msg = "bin planned" if ok else ("bin planning failed: %s" % self.last_failure_detail)
        return TriggerResponse(success=ok, message=msg)


def main():
    PRMPlannerNode()
    rospy.spin()


if __name__ == "__main__":
    main()
