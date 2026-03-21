#!/usr/bin/env python3
"""
Motion Control Node - IK/FK/Dynamics for UR5e Robot
Author: Jiaxin Liang
Responsibilities:
  - Forward Kinematics (FK): Joint angles -> End-effector pose
  - Inverse Kinematics (IK): End-effector pose -> Joint angles
  - Dynamics: Compute joint torques, velocities, accelerations
  - Low-level motion execution interface with robot controller
"""

import rospy
import numpy as np
import copy
import threading
from typing import List, Optional, Tuple

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Float64MultiArray, Bool, String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
    JointTrajectoryControllerState,
)
from std_srvs.srv import Trigger, TriggerResponse
from common_msgs.msg import (
    MotionCommand,
    GraspResult,
    ExecuteTrajectoryAction,
    ExecuteTrajectoryResult,
    ExecuteTrajectoryFeedback,
)
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, RobotState, MoveItErrorCodes

import tf2_ros
import tf2_geometry_msgs
import actionlib
import actionlib_msgs.msg as action_msgs

try:
    import PyKDL
    from urdf_parser_py.urdf import URDF
    from kdl_parser_py import urdf as kdl_urdf
    _HAS_KDL = True
except ImportError:
    _HAS_KDL = False


class MotionControlNode:
    """
    Motion Control Node for UR5e Robot
    
    This node provides low-level motion control services including:
    - IK/FK computation
    - Trajectory generation
    - Joint space and Cartesian space control
    - Velocity and force control interfaces
    """
    
    def __init__(self):
        rospy.init_node('motion_control', anonymous=False)
        rospy.loginfo("=" * 60)
        rospy.loginfo("Motion Control Node Initializing")
        rospy.loginfo("=" * 60)
        
        # Robot configuration
        self.robot_name = rospy.get_param('~robot_name', 'ur5e')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.ee_frame = rospy.get_param('~ee_frame', 'tool0')
        
        # Control parameters
        self.joint_names = rospy.get_param('~joint_names', [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ])
        self.num_joints = len(self.joint_names)
        
        # Current robot state (positions in same order as joint_names)
        self.current_joint_state = None
        self.current_joint_positions = None  # List[float] in joint_names order
        self.current_ee_pose = None
        
        # UR5e DH parameters (standard order j1..j6, from UR documentation)
        self.dh_params = {
            'a': [0, -0.425, -0.39225, 0, 0, 0],
            'd': [0.1625, 0, 0, 0.1333, 0.0997, 0.0996],
            'alpha': [np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0],
            'offset': [0, -np.pi/2, 0, -np.pi/2, 0, 0]
        }
        self.joint_limits = {
            'shoulder_pan_joint': [-np.pi, np.pi],
            'shoulder_lift_joint': [-np.pi, np.pi],
            'elbow_joint': [-np.pi, np.pi],
            'wrist_1_joint': [-np.pi, np.pi],
            'wrist_2_joint': [-np.pi, np.pi],
            'wrist_3_joint': [-np.pi, np.pi]
        }
        self.home_joints = rospy.get_param(
            '~home_position',
            [0.0, -1.57, 1.57, -1.57, -1.57, 0.0],
        )
        if len(self.home_joints) != self.num_joints:
            rospy.logwarn(
                "[MotionControl] Invalid ~home_position length %d, falling back to default.",
                len(self.home_joints),
            )
            self.home_joints = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
        self.home_joints = [float(v) for v in self.home_joints]
        
        # TF2 for transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # TCP link: the actual end-effector frame used by UI and IK targets
        self.tcp_link = rospy.get_param('~tcp_link', 'gripper_tip_link')
        self.T_tool0_to_tcp = np.eye(4)  # 4x4 fixed transform, updated from TF during init

        # KDL chain for URDF-based FK (preferred over DH parameters)
        self.kdl_chain = None
        self.kdl_fk_solver = None
        self.kdl_jac_solver = None
        self.kdl_num_joints = 0
        self._use_kdl = False
        self._kdl_includes_tcp = True  # True if chain goes all the way to tcp_link
        self._kdl_joint_names = []     # joint names in KDL chain order
        self._cfg_to_kdl = []          # mapping: cfg index -> kdl index
        self._kdl_to_cfg = []          # mapping: kdl index -> cfg index

        # MoveIt IK: service, group, planning frame and tip link (must match your MoveIt config)
        self.moveit_ik_service = rospy.get_param('~moveit_ik_service', '/compute_ik')
        self.moveit_ik_group = rospy.get_param('~moveit_ik_group', 'manipulator')
        self.moveit_planning_frame = rospy.get_param('~moveit_planning_frame', 'world')
        self.moveit_ik_link = rospy.get_param('~moveit_ik_link', 'gripper_tip_link')
        self._moveit_ik_proxy = None  # lazy init when first needed
        self._execution_lock = threading.RLock()
        self._active_execution_source = None
        self._gripper_lock = threading.RLock()

        # Action client for trajectory execution (Gazebo UR5e controller)
        self.trajectory_client = actionlib.SimpleActionClient(
            '/pos_joint_traj_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
        )
        rospy.loginfo("[MotionControl] Waiting for trajectory action server...")
        if not self.trajectory_client.wait_for_server(timeout=rospy.Duration(5.0)):
            rospy.logwarn("[MotionControl] Trajectory action server not found. Gazebo UR5e may not be running.")

        # Gripper control uses the real controller interface exposed by Gazebo.
        self.gripper_action_name = rospy.get_param(
            '~gripper_action_name',
            '/gripper_controller/follow_joint_trajectory',
        )
        self.gripper_joint_name = rospy.get_param(
            '~gripper_joint_name',
            'robotiq_85_left_knuckle_joint',
        )
        self.gripper_open_position = float(rospy.get_param('~gripper_open_position', 0.0))
        self.gripper_closed_position = float(rospy.get_param('~gripper_closed_position', 0.72))
        self.gripper_motion_time = float(rospy.get_param('~gripper_motion_time', 1.0))
        self.gripper_wait_timeout = float(rospy.get_param('~gripper_wait_timeout', 3.0))
        self.gripper_position_tolerance = float(rospy.get_param('~gripper_position_tolerance', 0.03))
        self.gripper_grasp_retry_count = int(rospy.get_param('~gripper_grasp_retry_count', 1))
        self.gripper_post_grasp_settle_time = float(
            rospy.get_param('~gripper_post_grasp_settle_time', 0.20)
        )
        self.gripper_skip_holding_check = bool(
            rospy.get_param('~gripper_skip_holding_check', True)
        )
        self.gripper_accept_partial_grasp = bool(
            rospy.get_param('~gripper_accept_partial_grasp', True)
        )
        self.gripper_release_min_closed_error = float(
            rospy.get_param('~gripper_release_min_closed_error', 0.25)
        )
        self.gripper_empty_grasp_closed_tolerance = float(
            rospy.get_param('~gripper_empty_grasp_closed_tolerance', 0.05)
        )
        self.gripper_left_tip_link = rospy.get_param(
            '~gripper_left_tip_link',
            'robotiq_85_left_finger_tip_link',
        )
        self.gripper_right_tip_link = rospy.get_param(
            '~gripper_right_tip_link',
            'robotiq_85_right_finger_tip_link',
        )
        self.gripper_holding_min_distance = float(
            rospy.get_param('~gripper_holding_min_distance', 0.018)
        )
        self.gripper_holding_max_distance = float(
            rospy.get_param('~gripper_holding_max_distance', 0.095)
        )
        self.gripper_holding_closed_margin = float(
            rospy.get_param('~gripper_holding_closed_margin', 0.05)
        )
        self.gripper_client = actionlib.SimpleActionClient(
            self.gripper_action_name,
            FollowJointTrajectoryAction,
        )
        self._gripper_server_reported = False
        self._latest_gripper_state = None
        rospy.loginfo(
            "[MotionControl] Gripper config: action=%s joint=%s open=%.3f closed=%.3f motion_time=%.2fs tol=%.3f retries=%d settle=%.2fs skip_holding=%s accept_partial=%s release_min_closed_err=%.3f empty_tol=%.3f hold_dist=[%.3f, %.3f]",
            self.gripper_action_name,
            self.gripper_joint_name,
            self.gripper_open_position,
            self.gripper_closed_position,
            self.gripper_motion_time,
            self.gripper_position_tolerance,
            self.gripper_grasp_retry_count,
            self.gripper_post_grasp_settle_time,
            self.gripper_skip_holding_check,
            self.gripper_accept_partial_grasp,
            self.gripper_release_min_closed_error,
            self.gripper_empty_grasp_closed_tolerance,
            self.gripper_holding_min_distance,
            self.gripper_holding_max_distance,
        )
        
        # ROS interfaces
        self._setup_subscribers()
        self._setup_publishers()
        self._setup_services()
        self._setup_action_servers()
        
        # Communication test: wait for first joint_states from Gazebo
        rospy.loginfo("[MotionControl] Communication test: waiting for /joint_states (timeout 10s)...")
        try:
            msg = rospy.wait_for_message('/joint_states', JointState, timeout=10.0)
            positions = []
            for name in self.joint_names:
                if name in msg.name:
                    positions.append(msg.position[msg.name.index(name)])
            if len(positions) == self.num_joints:
                self.current_joint_positions = positions
                rospy.loginfo("[MotionControl] Communication OK (joint_states received from Gazebo)")
            else:
                rospy.logwarn("[MotionControl] Communication test: joint_states received but missing joints (got %d, need %d)" % (len(positions), self.num_joints))
        except rospy.ROSException as e:
            rospy.logwarn("[MotionControl] Communication test FAILED: %s. Check Gazebo and /joint_states." % str(e))
        
        # Initialize FK method: prefer KDL (URDF-based), fallback to DH + TF
        self._init_kdl_chain()
        if not self._use_kdl:
            self._init_tool0_to_tcp_transform()
        
        rospy.loginfo("Motion Control Node Ready  (FK mode: %s)", "KDL/URDF" if self._use_kdl else "DH+TF")
        rospy.loginfo("=" * 60)

        # Validate FK vs TF at startup
        self._validate_fk_vs_tf()

        # Compute and publish dexterous workspace bounds (FK sampling)
        self._publish_workspace_bounds()

    def _compute_workspace_bounds(self, num_samples_per_joint: int = 12) -> Optional[Tuple[float, float, float, float, float, float]]:
        """
        Compute dexterous workspace bounds via FK sampling over joint limits.
        Returns (x_min, x_max, y_min, y_max, z_min, z_max) or None if FK unavailable.
        """
        limits = []
        for name in self.joint_names:
            lo, hi = self.joint_limits.get(name, [-np.pi, np.pi])
            limits.append((float(lo), float(hi)))
        if len(limits) != self.num_joints:
            return None

        xs, ys, zs = [], [], []
        # Sample joint space: for each joint, take num_samples values
        n = num_samples_per_joint
        for i0 in np.linspace(limits[0][0], limits[0][1], min(n, 8)):
            for i1 in np.linspace(limits[1][0], limits[1][1], min(n, 8)):
                for i2 in np.linspace(limits[2][0], limits[2][1], min(n, 6)):
                    for i3 in np.linspace(limits[3][0], limits[3][1], min(n, 5)):
                        for i4 in np.linspace(limits[4][0], limits[4][1], min(n, 4)):
                            for i5 in np.linspace(limits[5][0], limits[5][1], min(n, 4)):
                                q = [i0, i1, i2, i3, i4, i5]
                                T = self._fk_gripper_matrix(q)
                                if T is not None:
                                    xs.append(T[0, 3])
                                    ys.append(T[1, 3])
                                    zs.append(T[2, 3])

        if not xs:
            return None
        return (min(xs), max(xs), min(ys), max(ys), min(zs), max(zs))

    def _publish_workspace_bounds(self):
        """Compute workspace bounds via FK sampling and publish to /motion/workspace_bounds.
        Intersects FK envelope with conservative UR5e usable region (x>0 in front of base)
        to avoid IK-unreachable boundary points."""
        bounds = self._compute_workspace_bounds(num_samples_per_joint=10)
        if bounds is None:
            rospy.logwarn("[MotionControl] Workspace bounds computation failed")
            return
        x_min, x_max, y_min, y_max, z_min, z_max = bounds
        # UR5e 保守可用区域：基座前方 x>0，避免 x<0 后方点 IK 失败
        usable = (0.15, 0.85, -0.55, 0.55, 0.15, 0.90)
        x_min = max(x_min, usable[0])
        x_max = min(x_max, usable[1])
        y_min = max(y_min, usable[2])
        y_max = min(y_max, usable[3])
        z_min = max(z_min, usable[4])
        z_max = min(z_max, usable[5])
        rospy.loginfo("[MotionControl] Dexterous workspace (usable): X[%.3f,%.3f] Y[%.3f,%.3f] Z[%.3f,%.3f]",
                      x_min, x_max, y_min, y_max, z_min, z_max)
        msg = Float64MultiArray()
        msg.data = [x_min, x_max, y_min, y_max, z_min, z_max]
        self.workspace_bounds_pub.publish(msg)

    def _setup_subscribers(self):
        """Setup ROS subscribers"""
        self.joint_state_sub = rospy.Subscriber(
            '/joint_states',
            JointState,
            self.joint_state_callback,
            queue_size=1
        )
        
        self.motion_command_sub = rospy.Subscriber(
            '/motion/command',
            MotionCommand,
            self.motion_command_callback,
            queue_size=10
        )

        self.gripper_command_sub = rospy.Subscriber(
            '/gripper/command',
            String,
            self._gripper_command_callback,
            queue_size=10,
        )

        self.gripper_state_sub = rospy.Subscriber(
            '/gripper_controller/state',
            JointTrajectoryControllerState,
            self._gripper_state_callback,
            queue_size=1,
        )

        self.capture_result_sub = rospy.Subscriber(
            '/camera/capture_result',
            Bool,
            self._capture_result_callback,
            queue_size=10
        )


    def _setup_publishers(self):
        """Setup ROS publishers"""
        self.motion_result_pub = rospy.Publisher(
            '/motion/result',
            GraspResult,
            queue_size=10
        )
        
        self.trajectory_pub = rospy.Publisher(
            '/joint_trajectory',
            JointTrajectory,
            queue_size=10
        )

        self.workspace_bounds_pub = rospy.Publisher(
            '/motion/workspace_bounds',
            Float64MultiArray,
            queue_size=1,
            latch=True
        )

        self.gripper_result_pub = rospy.Publisher(
            '/gripper/grasp_result',
            Bool,
            queue_size=1,
        )


    def _setup_services(self):
        """
        Setup ROS services for synchronous IK/FK queries
        
        TODO: Implement service servers for:
        - FK service: joint_angles -> end_effector_pose
        - IK service: end_effector_pose -> joint_angles
        - Jacobian service: compute Jacobian matrix
        """
        self.gripper_grasp_srv = rospy.Service(
            '/gripper/grasp',
            Trigger,
            self._handle_gripper_grasp,
        )
        self.gripper_release_srv = rospy.Service(
            '/gripper/release',
            Trigger,
            self._handle_gripper_release,
        )
        self.gripper_is_holding_srv = rospy.Service(
            '/gripper/is_holding',
            Trigger,
            self._handle_gripper_is_holding,
        )

    def _setup_action_servers(self):
        self.execute_trajectory_server = actionlib.SimpleActionServer(
            '/motion_control/execute_trajectory',
            ExecuteTrajectoryAction,
            execute_cb=self._execute_trajectory_action_cb,
            auto_start=False,
        )
        self.execute_trajectory_server.start()

    def _try_begin_execution(self, source: str):
        with self._execution_lock:
            if self._active_execution_source is not None:
                return False, self._active_execution_source
            self._active_execution_source = source
            return True, None

    def _finish_execution(self, source: str):
        with self._execution_lock:
            if self._active_execution_source == source:
                self._active_execution_source = None

    def _publish_execute_feedback(self, stage: int, message: str):
        feedback = ExecuteTrajectoryFeedback()
        feedback.stage = stage
        feedback.message = message
        self.execute_trajectory_server.publish_feedback(feedback)

    def _validate_fk_vs_tf(self):
        """Compare FK output against TF to verify kinematic model correctness."""
        if self.current_joint_positions is None:
            rospy.logwarn("[MotionControl] No joint positions yet, skipping FK validation")
            return
        T_fk = self._fk_gripper_matrix(self.current_joint_positions)
        if T_fk is None:
            rospy.logwarn("[MotionControl] FK returned None, skipping validation")
            return
        try:
            trans = self.tf_buffer.lookup_transform(
                self.base_frame, self.tcp_link, rospy.Time(0), rospy.Duration(2.0))
            t = trans.transform.translation
            fk_pos = T_fk[0:3, 3]
            tf_pos = np.array([t.x, t.y, t.z])
            diff = np.linalg.norm(fk_pos - tf_pos)
            rospy.loginfo("[MotionControl] ===== FK vs TF Validation =====")
            rospy.loginfo("[MotionControl] FK  pos: [%.4f, %.4f, %.4f]", fk_pos[0], fk_pos[1], fk_pos[2])
            rospy.loginfo("[MotionControl] TF  pos: [%.4f, %.4f, %.4f]", tf_pos[0], tf_pos[1], tf_pos[2])
            rospy.loginfo("[MotionControl] Position diff: %.6f m", diff)
            if diff > 0.01:
                rospy.logerr("[MotionControl] FK vs TF mismatch > 10mm! FK model may be wrong.")
            else:
                rospy.loginfo("[MotionControl] FK vs TF OK (< 10mm)")
            rospy.loginfo("[MotionControl] joints: %s",
                          ["%.4f" % j for j in self.current_joint_positions])
        except Exception as e:
            rospy.logwarn("[MotionControl] FK validation TF lookup failed: %s", e)

    def _init_kdl_chain(self):
        """Initialize KDL kinematic chain from URDF for FK/Jacobian computation.
        Tries base_link -> tcp_link first; if joint count doesn't match the 6-DOF arm,
        falls back to base_link -> ee_frame (tool0) and uses TF for the extra offset."""
        if not _HAS_KDL:
            rospy.logwarn("[MotionControl] PyKDL / kdl_parser_py not available, using DH fallback")
            return
        try:
            robot = URDF.from_parameter_server()
            (ok, tree) = kdl_urdf.treeFromUrdfModel(robot)
            if not ok:
                rospy.logerr("[MotionControl] Failed to build KDL tree from URDF")
                return

            # Prefer direct chain to tcp_link (no extra transform needed)
            chain = tree.getChain(self.base_frame, self.tcp_link)
            nj = chain.getNrOfJoints()
            if nj == self.num_joints:
                self.kdl_chain = chain
                self.kdl_num_joints = nj
                self._kdl_includes_tcp = True
                rospy.loginfo("[MotionControl] KDL chain OK: %s -> %s (%d joints, direct)",
                              self.base_frame, self.tcp_link, nj)
            else:
                # Extra joints (e.g. gripper finger); use chain to tool0 + TF offset
                chain2 = tree.getChain(self.base_frame, self.ee_frame)
                nj2 = chain2.getNrOfJoints()
                if nj2 == self.num_joints:
                    self.kdl_chain = chain2
                    self.kdl_num_joints = nj2
                    self._kdl_includes_tcp = False
                    rospy.loginfo("[MotionControl] KDL chain OK: %s -> %s (%d joints, +TF offset to %s)",
                                  self.base_frame, self.ee_frame, nj2, self.tcp_link)
                    self._init_tool0_to_tcp_transform()
                else:
                    rospy.logwarn("[MotionControl] KDL joint mismatch (tcp=%d, ee=%d, expected=%d), DH fallback",
                                  nj, nj2, self.num_joints)
                    return

            # Extract joint names from KDL chain segments (in chain order)
            # PyKDL uses Joint.None (value 8) for fixed joints, not Joint.Fixed
            _KDL_JOINT_FIXED = getattr(PyKDL.Joint, 'None', 8)
            self._kdl_joint_names = []
            for seg_idx in range(self.kdl_chain.getNrOfSegments()):
                jnt = self.kdl_chain.getSegment(seg_idx).getJoint()
                if jnt.getType() != _KDL_JOINT_FIXED:
                    self._kdl_joint_names.append(jnt.getName())
            rospy.loginfo("[MotionControl] KDL joint order: %s", self._kdl_joint_names)
            rospy.loginfo("[MotionControl] Config joint order: %s", list(self.joint_names))

            # Build bidirectional mapping between config order and KDL order
            self._cfg_to_kdl = []
            for cfg_name in self.joint_names:
                if cfg_name in self._kdl_joint_names:
                    self._cfg_to_kdl.append(self._kdl_joint_names.index(cfg_name))
                else:
                    rospy.logerr("[MotionControl] Config joint '%s' not found in KDL chain!", cfg_name)
                    return
            self._kdl_to_cfg = [0] * self.kdl_num_joints
            for cfg_i, kdl_i in enumerate(self._cfg_to_kdl):
                self._kdl_to_cfg[kdl_i] = cfg_i

            self.kdl_fk_solver = PyKDL.ChainFkSolverPos_recursive(self.kdl_chain)
            self.kdl_jac_solver = PyKDL.ChainJntToJacSolver(self.kdl_chain)
            self._use_kdl = True
        except Exception as e:
            rospy.logwarn("[MotionControl] KDL init failed: %s, using DH fallback", e)

    def _init_tool0_to_tcp_transform(self):
        """Get the fixed 4x4 transform from tool0 to tcp_link (e.g. gripper_tip_link) via TF."""
        if self.tcp_link == self.ee_frame:
            self.T_tool0_to_tcp = np.eye(4)
            rospy.loginfo("[MotionControl] tcp_link == ee_frame (%s), no extra transform needed", self.ee_frame)
            return
        rospy.loginfo("[MotionControl] Looking up fixed transform: %s -> %s ...", self.ee_frame, self.tcp_link)
        for attempt in range(20):
            try:
                trans = self.tf_buffer.lookup_transform(
                    self.ee_frame, self.tcp_link, rospy.Time(0), rospy.Duration(1.0)
                )
                t = trans.transform.translation
                r = trans.transform.rotation
                T = np.eye(4)
                T[0:3, 0:3] = self._quat_to_rotation_matrix([r.x, r.y, r.z, r.w])
                T[0, 3], T[1, 3], T[2, 3] = t.x, t.y, t.z
                self.T_tool0_to_tcp = T
                rospy.loginfo("[MotionControl] tool0->tcp transform acquired: translation=[%.4f, %.4f, %.4f]",
                              t.x, t.y, t.z)
                return
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.sleep(0.5)
        rospy.logwarn("[MotionControl] Could not get %s->%s transform, falling back to identity. "
                      "Numerical IK may be inaccurate!", self.ee_frame, self.tcp_link)
        self.T_tool0_to_tcp = np.eye(4)

    def _cfg_to_kdl_jntarray(self, joint_angles: List[float]) -> 'PyKDL.JntArray':
        """Convert joint_angles (in config/joint_names order) to KDL JntArray (in KDL chain order)."""
        n = self.kdl_num_joints
        q = PyKDL.JntArray(n)
        for cfg_i, kdl_i in enumerate(self._cfg_to_kdl):
            if cfg_i < len(joint_angles):
                q[kdl_i] = joint_angles[cfg_i]
        return q

    def _fk_kdl(self, joint_angles: List[float]) -> Optional[np.ndarray]:
        """FK via KDL/URDF: joint_angles (config order) -> 4x4 transform (base_link -> tcp_link)."""
        q = self._cfg_to_kdl_jntarray(joint_angles)
        frame = PyKDL.Frame()
        if self.kdl_fk_solver.JntToCart(q, frame) < 0:
            return None
        T = np.eye(4)
        for i in range(3):
            for j in range(3):
                T[i, j] = frame.M[i, j]
        T[0, 3], T[1, 3], T[2, 3] = frame.p.x(), frame.p.y(), frame.p.z()
        return T

    def _jacobian_kdl(self, joint_angles: List[float]) -> np.ndarray:
        """6xN Jacobian via KDL. Input: config order. Output: columns in config order."""
        q = self._cfg_to_kdl_jntarray(joint_angles)
        n = self.kdl_num_joints
        jac = PyKDL.Jacobian(n)
        self.kdl_jac_solver.JntToJac(q, jac)
        J = np.zeros((6, n))
        for cfg_i in range(n):
            kdl_i = self._cfg_to_kdl[cfg_i]
            for row in range(6):
                J[row, cfg_i] = jac[row, kdl_i]
        return J

    def _fk_gripper_matrix(self, joint_angles: List[float]) -> Optional[np.ndarray]:
        """Forward kinematics returning 4x4 transform: base_link -> tcp_link (gripper_tip_link).
        Uses KDL (URDF) when available; falls back to DH + tool0->tcp transform."""
        if self._use_kdl:
            T = self._fk_kdl(joint_angles)
            if T is None:
                return None
            if not self._kdl_includes_tcp:
                T = T @ self.T_tool0_to_tcp
            return T
        T_tool0 = self._fk_matrix(joint_angles)
        if T_tool0 is None:
            return None
        return T_tool0 @ self.T_tool0_to_tcp


    def _rpy_to_quaternion(self, roll: float, pitch: float, yaw: float) -> List[float]:
        """Convert RPY Euler angles to quaternion [qx, qy, qz, qw]"""
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return [qx, qy, qz, qw]

    def _fk_matrix(self, joint_angles: List[float]) -> Optional[np.ndarray]:
        """Forward kinematics returning 4x4 transform matrix (base_link -> tool0)."""
        if joint_angles is None or len(joint_angles) != self.num_joints:
            return None
        a, d, alpha, offset = self.dh_params['a'], self.dh_params['d'], self.dh_params['alpha'], self.dh_params['offset']
        T = np.eye(4)
        for i in range(6):
            theta = joint_angles[i] + offset[i]
            ct, st = np.cos(theta), np.sin(theta)
            ca, sa = np.cos(alpha[i]), np.sin(alpha[i])
            A = np.array([
                [ct, -st*ca, st*sa, a[i]*ct],
                [st, ct*ca, -ct*sa, a[i]*st],
                [0, sa, ca, d[i]],
                [0, 0, 0, 1]
            ])
            T = T @ A
        return T

    def _quat_to_rotation_matrix(self, q: List[float]) -> np.ndarray:
        """Quaternion [qx,qy,qz,qw] -> 3x3 rotation matrix."""
        qx, qy, qz, qw = float(q[0]), float(q[1]), float(q[2]), float(q[3])
        return np.array([
            [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)]
        ])

    def _pose_to_matrix(self, pose: Pose) -> np.ndarray:
        """Geometry Pose (position + quaternion) -> 4x4 transform."""
        T = np.eye(4)
        T[0:3, 0:3] = self._quat_to_rotation_matrix([
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        T[0, 3], T[1, 3], T[2, 3] = pose.position.x, pose.position.y, pose.position.z
        return T

    def _rotation_to_axis_angle(self, R: np.ndarray) -> np.ndarray:
        """Rotation matrix 3x3 -> axis-angle vector (length = angle in rad)."""
        angle = np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))
        if angle < 1e-6:
            return np.zeros(3)
        axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
        return axis * (angle / (2 * np.sin(angle)))

    def _compute_jacobian(self, q: List[float]) -> np.ndarray:
        """6xN Jacobian at tcp_link. Uses KDL when available, else numerical finite-differences."""
        if self._use_kdl:
            return self._jacobian_kdl(q)
        return self._compute_jacobian_numerical(q)

    def _compute_jacobian_numerical(self, q: List[float], eps: float = 1e-6) -> np.ndarray:
        """6x6 numerical Jacobian (finite differences) at tcp_link."""
        J = np.zeros((6, self.num_joints))
        T0 = self._fk_gripper_matrix(q)
        if T0 is None:
            return J
        p0 = T0[0:3, 3]
        R0 = T0[0:3, 0:3]
        for j in range(self.num_joints):
            qp = list(q)
            qp[j] += eps
            T1 = self._fk_gripper_matrix(qp)
            if T1 is None:
                continue
            J[0:3, j] = (T1[0:3, 3] - p0) / eps
            R1 = T1[0:3, 0:3]
            J[3:6, j] = self._rotation_to_axis_angle(R1 @ R0.T) / eps
        return J

    def _get_moveit_ik_proxy(self):
        """Lazy init and return MoveIt /compute_ik service proxy, or None if unavailable."""
        if self._moveit_ik_proxy is not None:
            return self._moveit_ik_proxy
        try:
            rospy.wait_for_service(self.moveit_ik_service, timeout=0.5)
            self._moveit_ik_proxy = rospy.ServiceProxy(self.moveit_ik_service, GetPositionIK)
            rospy.loginfo("[MotionControl] MoveIt IK service connected: %s", self.moveit_ik_service)
            return self._moveit_ik_proxy
        except (rospy.ROSException, rospy.ROSInterruptException):
            return None

    # =========================================================================
    # Callbacks
    # =========================================================================

    def joint_state_callback(self, msg: JointState):
        """Update current joint state (positions in joint_names order)."""
        self.current_joint_state = msg
        positions = []
        for name in self.joint_names:
            if name in msg.name:
                idx = msg.name.index(name)
                positions.append(msg.position[idx])
        if len(positions) == self.num_joints:
            self.current_joint_positions = positions
            self.current_ee_pose = self.compute_forward_kinematics(positions)

    def _capture_result_callback(self, msg: Bool):
        """收到相机拍照结果，在终端打印"""
        if msg.data:
            rospy.loginfo("[MotionControl] Successful Shot")
        else:
            rospy.logwarn("[MotionControl] Failed")

    def _gripper_state_callback(self, msg: JointTrajectoryControllerState):
        self._latest_gripper_state = msg

    def _get_gripper_state(
        self,
        timeout_sec: float = 0.5,
        min_seq: Optional[int] = None,
    ) -> Optional[JointTrajectoryControllerState]:
        state = self._latest_gripper_state
        if state is not None:
            seq = int(getattr(state.header, "seq", 0))
            if min_seq is None or seq > int(min_seq):
                return state

        deadline = rospy.Time.now() + rospy.Duration(max(0.0, timeout_sec))
        while not rospy.is_shutdown():
            remaining = (deadline - rospy.Time.now()).to_sec()
            if remaining <= 0.0:
                break
            try:
                state = rospy.wait_for_message(
                    '/gripper_controller/state',
                    JointTrajectoryControllerState,
                    timeout=remaining,
                )
            except rospy.ROSException:
                return None
            self._latest_gripper_state = state
            seq = int(getattr(state.header, "seq", 0))
            if min_seq is None or seq > int(min_seq):
                return state
        return None

    def _get_gripper_actual_position(self, timeout_sec: float = 0.5) -> Optional[float]:
        state = self._get_gripper_state(timeout_sec=timeout_sec)
        if state is not None and state.actual.positions:
            return float(state.actual.positions[0])
        return None

    def _get_gripper_fingertip_distance(self, timeout_sec: float = 0.25) -> Optional[float]:
        deadline = rospy.Time.now() + rospy.Duration(max(0.0, timeout_sec))
        while not rospy.is_shutdown():
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.gripper_left_tip_link,
                    self.gripper_right_tip_link,
                    rospy.Time(0),
                    rospy.Duration(0.05),
                )
                t = transform.transform.translation
                return float((t.x ** 2 + t.y ** 2 + t.z ** 2) ** 0.5)
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ):
                if rospy.Time.now() >= deadline:
                    break
                rospy.sleep(0.02)
        return None

    @staticmethod
    def _wrapped_position_error(actual: float, target: float) -> float:
        direct_error = abs(actual - target)
        wrapped_error = abs(((actual - target + np.pi) % (2.0 * np.pi)) - np.pi)
        return min(direct_error, wrapped_error)

    def _gripper_reached_position(
        self,
        target: float,
        retries: int = 3,
        timeout_sec: float = 0.4,
        min_seq: Optional[int] = None,
    ) -> Tuple[bool, Optional[float]]:
        for _ in range(retries):
            state = self._get_gripper_state(timeout_sec=timeout_sec, min_seq=min_seq)
            actual = None
            desired = None
            error = None
            if state is not None:
                if state.actual.positions:
                    actual = float(state.actual.positions[0])
                if state.desired.positions:
                    desired = float(state.desired.positions[0])
                if state.error.positions:
                    error = float(state.error.positions[0])

            desired_matches = desired is None or (
                self._wrapped_position_error(desired, target)
                <= self.gripper_position_tolerance
            )
            actual_matches = actual is not None and (
                self._wrapped_position_error(actual, target)
                <= self.gripper_position_tolerance
            )
            error_matches = error is not None and abs(error) <= self.gripper_position_tolerance

            if desired_matches and (actual_matches or error_matches):
                return True, actual
            rospy.sleep(0.1)
        return False, actual if 'actual' in locals() else None

    def _get_gripper_holding_state(self, timeout_sec: float = 0.25) -> Tuple[Optional[bool], str]:
        actual = self._get_gripper_actual_position(timeout_sec=timeout_sec)
        if actual is not None:
            closed_error = self._wrapped_position_error(actual, self.gripper_closed_position)
            if closed_error <= self.gripper_empty_grasp_closed_tolerance:
                return False, (
                    "mode=fully_closed actual=%.4f closed_err=%.4f tol=%.4f"
                    % (
                        actual,
                        closed_error,
                        self.gripper_empty_grasp_closed_tolerance,
                    )
                )

        fingertip_distance = self._get_gripper_fingertip_distance(timeout_sec=timeout_sec)
        if fingertip_distance is not None:
            holding = (
                self.gripper_holding_min_distance
                <= fingertip_distance
                <= self.gripper_holding_max_distance
            )
            return holding, (
                "mode=fingertip_distance distance=%.4fm range=[%.4f, %.4f]"
                % (
                    fingertip_distance,
                    self.gripper_holding_min_distance,
                    self.gripper_holding_max_distance,
                )
            )

        if actual is None:
            return None, "unknown: no fingertip TF or gripper controller state"

        open_error = self._wrapped_position_error(actual, self.gripper_open_position)
        closed_error = self._wrapped_position_error(actual, self.gripper_closed_position)
        holding = (
            open_error > self.gripper_position_tolerance
            and closed_error > self.gripper_holding_closed_margin
        )
        return holding, (
            "mode=joint_position actual=%.4f open_err=%.4f closed_err=%.4f"
            % (actual, open_error, closed_error)
        )

    def _ensure_gripper_server(self, timeout_sec: Optional[float] = None) -> bool:
        timeout = self.gripper_wait_timeout if timeout_sec is None else float(timeout_sec)
        if self.gripper_client.wait_for_server(timeout=rospy.Duration(timeout)):
            if not self._gripper_server_reported:
                rospy.loginfo(
                    "[MotionControl] Gripper action server connected: %s",
                    self.gripper_action_name,
                )
                self._gripper_server_reported = True
            return True

        rospy.logwarn_throttle(
            5.0,
            "[MotionControl] Gripper action server unavailable: %s",
            self.gripper_action_name,
        )
        return False

    @staticmethod
    def _gripper_failure_is_transport_related(message: str) -> bool:
        text = str(message or "").lower()
        return (
            "unavailable" in text
            or "timed out" in text
            or "timeout" in text
            or "not found" in text
        )

    def _send_gripper_position(self, position: float, label: str) -> Tuple[bool, str]:
        if not self._ensure_gripper_server():
            return False, f"Gripper action server unavailable: {self.gripper_action_name}"

        goal = FollowJointTrajectoryGoal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = [self.gripper_joint_name]

        point = JointTrajectoryPoint()
        point.positions = [float(position)]
        point.time_from_start = rospy.Duration(max(0.1, self.gripper_motion_time))
        goal.trajectory.points.append(point)

        timeout = point.time_from_start.to_sec() + self.gripper_wait_timeout
        pre_goal_seq = None
        if self._latest_gripper_state is not None:
            pre_goal_seq = int(getattr(self._latest_gripper_state.header, "seq", 0))
        with self._gripper_lock:
            self.gripper_client.send_goal(goal)
            if not self.gripper_client.wait_for_result(timeout=rospy.Duration(timeout)):
                self.gripper_client.cancel_goal()
                return False, f"{label} timed out"

            state = self.gripper_client.get_state()
            result = self.gripper_client.get_result()

        if state != action_msgs.GoalStatus.SUCCEEDED:
            reached, actual = self._gripper_reached_position(
                float(position),
                retries=5,
                timeout_sec=0.5,
                min_seq=pre_goal_seq,
            )
            if reached:
                return True, (
                    f"{label} reached target within tolerance "
                    f"(controller state {state}, actual={actual:.4f})"
                )
            error_text = result.error_string if result is not None and hasattr(result, "error_string") else ""
            return False, f"{label} failed with controller state {state} {error_text}".strip()

        return True, f"{label} completed"

    def _execute_gripper_grasp(self) -> Tuple[bool, str]:
        attempts = max(1, int(self.gripper_grasp_retry_count))
        last_message = "grasp_not_attempted"
        actual_before = self._get_gripper_actual_position(timeout_sec=0.2)
        rospy.loginfo(
            "[MotionControl] Gripper grasp requested: closed_target=%.4f actual_before=%s attempts=%d",
            self.gripper_closed_position,
            "None" if actual_before is None else f"{actual_before:.4f}",
            attempts,
        )

        for attempt in range(1, attempts + 1):
            success, message = self._send_gripper_position(
                self.gripper_closed_position,
                "Gripper grasp attempt %d/%d" % (attempt, attempts),
            )
            last_message = message
            if success:
                if attempt < attempts and self.gripper_post_grasp_settle_time > 0.0:
                    rospy.sleep(self.gripper_post_grasp_settle_time)
                    continue
                if self.gripper_post_grasp_settle_time > 0.0:
                    rospy.sleep(self.gripper_post_grasp_settle_time)

                if self.gripper_skip_holding_check:
                    self.gripper_result_pub.publish(Bool(data=True))
                    rospy.loginfo(
                        "[MotionControl] %s | holding check skipped by configuration",
                        message,
                    )
                    return True, f"{message} | holding check skipped"

                holding, holding_message = self._get_gripper_holding_state(
                    timeout_sec=max(0.25, self.gripper_post_grasp_settle_time + 0.1)
                )
                if holding is False:
                    full_message = f"{message} | empty grasp detected: {holding_message}"
                    self.gripper_result_pub.publish(Bool(data=False))
                    rospy.logwarn("[MotionControl] %s", full_message)
                    return False, full_message
                if holding is None:
                    rospy.logwarn(
                        "[MotionControl] Gripper motion succeeded but holding state is unknown: %s",
                        holding_message,
                    )
                    self.gripper_result_pub.publish(Bool(data=True))
                    return True, f"{message} | holding unknown: {holding_message}"

                self.gripper_result_pub.publish(Bool(data=True))
                rospy.loginfo("[MotionControl] %s | holding confirmed: %s", message, holding_message)
                return True, message

            if (
                self.gripper_accept_partial_grasp
                and not self._gripper_failure_is_transport_related(message)
            ):
                self.gripper_result_pub.publish(Bool(data=True))
                rospy.logwarn(
                    "[MotionControl] %s | accepted as usable partial grasp by configuration",
                    message,
                )
                return True, f"{message} | accepted as usable partial grasp"

            rospy.logwarn("[MotionControl] %s", message)
            if attempt < attempts and self.gripper_post_grasp_settle_time > 0.0:
                rospy.sleep(self.gripper_post_grasp_settle_time)

        self.gripper_result_pub.publish(Bool(data=False))
        return False, last_message

    def _execute_gripper_release(self) -> Tuple[bool, str]:
        actual_before = self._get_gripper_actual_position(timeout_sec=0.2)
        rospy.loginfo(
            "[MotionControl] Gripper release requested: open_target=%.4f actual_before=%s",
            self.gripper_open_position,
            "None" if actual_before is None else f"{actual_before:.4f}",
        )
        success, message = self._send_gripper_position(
            self.gripper_open_position,
            "Gripper release",
        )
        if success:
            self.gripper_result_pub.publish(Bool(data=False))
            rospy.loginfo("[MotionControl] %s", message)
            return success, message

        # Some runs return ABORTED even though fingers opened enough for the next pick.
        actual = self._get_gripper_actual_position(timeout_sec=0.5)
        if actual is not None:
            closed_error = self._wrapped_position_error(actual, self.gripper_closed_position)
            if closed_error >= self.gripper_release_min_closed_error:
                partial_message = (
                    "Gripper release accepted as partial-open: actual=%.4f closed_err=%.4f "
                    ">= min_closed_err=%.4f | original=%s"
                    % (
                        actual,
                        closed_error,
                        self.gripper_release_min_closed_error,
                        message,
                    )
                )
                self.gripper_result_pub.publish(Bool(data=False))
                rospy.logwarn("[MotionControl] %s", partial_message)
                return True, partial_message

        rospy.logwarn("[MotionControl] %s", message)
        return success, message

    def _gripper_command_callback(self, msg: String):
        command = msg.data.lower().strip()
        if command in ('grasp', 'close'):
            self._execute_gripper_grasp()
        elif command in ('release', 'open'):
            self._execute_gripper_release()
        else:
            rospy.logwarn("[MotionControl] Unknown gripper command: %s", msg.data)

    def _handle_gripper_grasp(self, _req):
        success, message = self._execute_gripper_grasp()
        return TriggerResponse(
            success=success,
            message=message if message else ("grasp_success" if success else "grasp_failed"),
        )

    def _handle_gripper_release(self, _req):
        success, message = self._execute_gripper_release()
        return TriggerResponse(
            success=success,
            message=message if message else ("release_success" if success else "release_failed"),
        )

    def _handle_gripper_is_holding(self, _req):
        holding, message = self._get_gripper_holding_state()
        if holding is None:
            return TriggerResponse(success=False, message=message)
        return TriggerResponse(
            success=holding,
            message=message if message else ("holding" if holding else "released"),
        )

    def motion_command_callback(self, msg: MotionCommand):
        """
        Execute motion command from higher-level planner
        
        Args:
            msg: MotionCommand specifying desired motion
            
        TODO: Implement command execution logic:
        - Parse command type (move_to_pose, move_to_joint, etc.)
        - Generate appropriate trajectory
        - Execute motion
        - Publish result
        """
        rospy.loginfo(f"[MotionControl] Received command: {msg.command_type}")

        if msg.command_type == MotionCommand.STOP:
            self._stop_motion()
            return

        execution_source = f"topic:{msg.command_type}"
        ok, active_source = self._try_begin_execution(execution_source)
        if not ok:
            self._publish_motion_result(
                GraspResult.EXECUTION_FAILED,
                f"Motion controller busy: {active_source}",
            )
            return

        try:
            if msg.command_type == MotionCommand.MOVE_TO_POSE:
                self._execute_cartesian_motion(msg.target_pose, msg)
            elif msg.command_type == MotionCommand.MOVE_TO_JOINT:
                joints = list(msg.joint_positions) if hasattr(msg.joint_positions, '__iter__') else []
                if len(joints) != self.num_joints:
                    self._publish_motion_result(GraspResult.EXECUTION_FAILED, "Invalid joint_positions length")
                else:
                    self._execute_joint_motion(joints, msg)
            elif msg.command_type == MotionCommand.EXECUTE_TRAJECTORY:
                success, _status, message = self._execute_trajectory_sync(msg.trajectory)
                result_status = GraspResult.SUCCESS if success else GraspResult.EXECUTION_FAILED
                self._publish_motion_result(result_status, message)
            elif msg.command_type == MotionCommand.HOME:
                self._move_to_home()
            else:
                rospy.logwarn(f"[MotionControl] Unknown command type: {msg.command_type}")
                self._publish_motion_result(GraspResult.EXECUTION_FAILED, f"Unknown command: {msg.command_type}")
        except Exception as e:
            rospy.logerr(f"[MotionControl] Motion execution failed: {e}")
            self._publish_motion_result(GraspResult.EXECUTION_FAILED, str(e))
        finally:
            self._finish_execution(execution_source)


    # =========================================================================
    # Kinematics - Forward Kinematics
    # =========================================================================

    def compute_forward_kinematics(self, joint_angles: List[float]) -> Optional[PoseStamped]:
        """Compute FK: joint angles -> tcp_link (gripper_tip_link) pose in base_link."""
        T = self._fk_gripper_matrix(joint_angles)
        if T is None:
            return None
        x, y, z = T[0, 3], T[1, 3], T[2, 3]
        roll = np.arctan2(T[2, 1], T[2, 2])
        pitch = np.arctan2(-T[2, 0], np.sqrt(T[2, 1]**2 + T[2, 2]**2))
        yaw = np.arctan2(T[1, 0], T[0, 0])
        q = self._rpy_to_quaternion(roll, pitch, yaw)
        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.header.stamp = rospy.Time.now()
        pose.pose.position = Point(x, y, z)
        pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
        return pose


    # =========================================================================
    # Kinematics - Inverse Kinematics
    # =========================================================================

    def compute_inverse_kinematics(
        self,
        target_pose: PoseStamped,
        seed_joints: Optional[List[float]] = None
    ) -> Optional[List[float]]:
        """
        Inverse kinematics: target pose -> joint angles.
        Prefer MoveIt /compute_ik service; fall back to numerical IK if unavailable or failed.
        """
        if target_pose.header.frame_id != self.base_frame:
            transformed = self._transform_pose(target_pose, self.base_frame)
            if transformed is None:
                rospy.logerr("[MotionControl] Failed to transform target pose to base_frame")
                return None
            target_pose = transformed

        # Try MoveIt IK first (pose must be in MoveIt's planning frame and for its tip link)
        proxy = self._get_moveit_ik_proxy()
        if proxy is not None:
            pose_for_moveit = self._transform_pose(target_pose, self.moveit_planning_frame)
            if pose_for_moveit is None:
                rospy.logwarn("[MotionControl] Cannot transform pose to MoveIt planning_frame=%s, fallback to numerical", self.moveit_planning_frame)
            else:
                req = PositionIKRequest()
                req.group_name = self.moveit_ik_group
                req.pose_stamped = pose_for_moveit
                req.ik_link_name = self.moveit_ik_link
                req.avoid_collisions = False
                req.robot_state = RobotState()
                req.robot_state.is_diff = True
                req.robot_state.joint_state = JointState()
                req.robot_state.joint_state.name = list(self.joint_names)
                seed = seed_joints if (seed_joints is not None and len(seed_joints) == self.num_joints) else self.current_joint_positions
                req.robot_state.joint_state.position = list(seed) if seed else [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
                req.timeout = rospy.Duration(2.0)   # 0.5s was too short for some poses
                try:
                    resp = proxy(req)
                    if resp.error_code.val == MoveItErrorCodes.SUCCESS and resp.solution.joint_state.name:
                        # Map solution to self.joint_names order
                        sol_map = dict(zip(resp.solution.joint_state.name, resp.solution.joint_state.position))
                        out = []
                        for name in self.joint_names:
                            if name in sol_map:
                                out.append(sol_map[name])
                            else:
                                rospy.logwarn("[MotionControl] MoveIt IK solution missing joint %s, fallback to numerical", name)
                                return self._compute_ik_numerical(target_pose, seed_joints)
                        if len(out) == self.num_joints:
                            rospy.logdebug("[MotionControl] IK from MoveIt")
                            return out
                    else:
                        rospy.logwarn("[MotionControl] MoveIt IK failed (error_code=%d, -31=no solution), fallback to numerical", resp.error_code.val)
                except Exception as e:
                    rospy.logwarn("[MotionControl] MoveIt IK call failed: %s, fallback to numerical", e)

        return self._compute_ik_numerical(target_pose, seed_joints)

    def _compute_ik_numerical(
        self,
        target_pose: PoseStamped,
        seed_joints: Optional[List[float]] = None
    ) -> Optional[List[float]]:
        """Numerical IK (Jacobian pseudo-inverse) for tcp_link (gripper_tip_link).
        FK uses _fk_gripper_matrix so the solver targets the actual TCP, not tool0.
        Tries multiple seeds for robustness."""
        T_target = self._pose_to_matrix(target_pose.pose)
        p_target = T_target[0:3, 3]
        R_target = T_target[0:3, 0:3]

        seeds = []
        if seed_joints is not None and len(seed_joints) == self.num_joints:
            seeds.append(list(seed_joints))
        if self.current_joint_positions is not None:
            seeds.append(list(self.current_joint_positions))
        seeds.append([0.0, -1.57, 1.57, -1.57, -1.57, 0.0])
        if self.current_joint_positions is not None:
            for offset in [0.3, -0.3, 0.6, -0.6]:
                seeds.append([j + offset for j in self.current_joint_positions])

        for seed_idx, seed in enumerate(seeds):
            result = self._ik_iterate(p_target, R_target, list(seed))
            if result is not None:
                if seed_idx > 0:
                    rospy.loginfo("[MotionControl] Numerical IK converged with seed #%d", seed_idx)
                return result

        rospy.logwarn("[MotionControl] Numerical IK failed with all %d seeds", len(seeds))
        return None

    def _ik_iterate(self, p_target: np.ndarray, R_target: np.ndarray, q: List[float]) -> Optional[List[float]]:
        """Single-seed numerical IK iteration (damped least-squares / Levenberg-Marquardt)."""
        step = 0.5
        pos_tol, ori_tol = 2e-3, 2e-2
        max_iter = 300
        lambda_damp = 0.005
        logged_init = False
        for it in range(max_iter):
            T = self._fk_gripper_matrix(q)
            if T is None:
                return None
            p = T[0:3, 3]
            R = T[0:3, 0:3]
            pos_err = p_target - p
            ori_err = self._rotation_to_axis_angle(R_target @ R.T)
            pe = np.linalg.norm(pos_err)
            oe = np.linalg.norm(ori_err)
            if not logged_init:
                rospy.loginfo("[IK] init err: pos=%.4f m, ori=%.4f rad | target=[%.3f,%.3f,%.3f]",
                              pe, oe, p_target[0], p_target[1], p_target[2])
                logged_init = True
            if pe < pos_tol and oe < ori_tol:
                rospy.loginfo("[IK] converged at iter %d: pos=%.5f ori=%.5f", it, pe, oe)
                return q
            err = np.concatenate([pos_err, ori_err])
            J = self._compute_jacobian(q)
            try:
                JJt = J @ J.T + (lambda_damp ** 2) * np.eye(6)
                dq = J.T @ np.linalg.solve(JJt, err)
            except Exception:
                return None
            dq_norm = np.linalg.norm(dq)
            if dq_norm > 0.5:
                dq = dq * (0.5 / dq_norm)
            q = list(np.array(q) + step * dq)
            for i, name in enumerate(self.joint_names):
                lo, hi = self.joint_limits.get(name, [-6.28, 6.28])
                q[i] = np.clip(q[i], lo, hi)
        rospy.logwarn("[IK] did NOT converge after %d iters: pos=%.4f ori=%.4f", max_iter, pe, oe)
        return None


    def compute_ik_with_collision_check(
        self,
        target_pose: PoseStamped,
        seed_joints: Optional[List[float]] = None
    ) -> Tuple[Optional[List[float]], bool]:
        """
        Compute IK with collision checking
        
        Args:
            target_pose: Desired pose
            seed_joints: IK seed
            
        Returns:
            (joint_angles, is_valid): Joint solution and validity flag
            
        TODO: Implement IK with collision checking
        - Compute IK solution
        - Check if solution is collision-free
        - If collision detected, try alternative IK solutions (UR5e has up to 8 solutions)
        """
        joint_solution = self.compute_inverse_kinematics(target_pose, seed_joints)
        
        if joint_solution is None:
            return None, False
        
        # TODO: Add collision checking
        is_collision_free = True  # Placeholder
        
        return joint_solution, is_collision_free


    # =========================================================================
    # Dynamics
    # =========================================================================

    def compute_jacobian(self, joint_angles: List[float]) -> Optional[np.ndarray]:
        """
        Compute geometric Jacobian matrix at given configuration
        
        Args:
            joint_angles: Joint configuration
            
        Returns:
            6x6 Jacobian matrix (linear and angular velocity)
            
        TODO: Implement Jacobian computation
        - Can be derived from FK using numerical differentiation
        - Or use analytical Jacobian from kinematics library
        
        Used for:
        - Cartesian velocity control: v_ee = J * q_dot
        - Force control: tau = J^T * F_ext
        """
        # TODO: Implement Jacobian calculation
        return None


    def compute_inverse_dynamics(
        self,
        joint_positions: List[float],
        joint_velocities: List[float],
        joint_accelerations: List[float]
    ) -> Optional[List[float]]:
        """
        Compute inverse dynamics: accelerations -> joint torques
        
        Args:
            joint_positions: Joint angles
            joint_velocities: Joint velocities
            joint_accelerations: Desired joint accelerations
            
        Returns:
            List of joint torques required
            
        TODO: Implement inverse dynamics
        - Use recursive Newton-Euler algorithm or
        - Use dynamics library (KDL, PyBullet, or robot-specific)
        
        Useful for feedforward torque control
        """
        # TODO: Implement inverse dynamics
        return None


    # =========================================================================
    # Motion Execution
    # =========================================================================

    def _execute_cartesian_motion(self, target_pose: PoseStamped, cmd: MotionCommand):
        """Execute Cartesian motion: IK then joint motion."""
        tp = target_pose.pose
        rospy.loginfo("[MotionControl] Cartesian target (frame=%s): pos=[%.4f,%.4f,%.4f] quat=[%.4f,%.4f,%.4f,%.4f]",
                      target_pose.header.frame_id,
                      tp.position.x, tp.position.y, tp.position.z,
                      tp.orientation.x, tp.orientation.y, tp.orientation.z, tp.orientation.w)
        if self.current_joint_positions is not None:
            T_cur = self._fk_gripper_matrix(self.current_joint_positions)
            if T_cur is not None:
                rospy.loginfo("[MotionControl] FK(current): pos=[%.4f,%.4f,%.4f]",
                              T_cur[0, 3], T_cur[1, 3], T_cur[2, 3])
        target_joints = self.compute_inverse_kinematics(target_pose)
        if target_joints is None:
            self._publish_motion_result(GraspResult.EXECUTION_FAILED, "IK failed for target pose")
            return
        # Base duration 2.0s for small teach-pendant steps (e.g. 10mm in 2s ~= 5mm/s); scale by max_velocity
        duration = 2.0
        if cmd.max_velocity > 0:
            duration = max(0.8, 2.0 * (1.0 / max(cmd.max_velocity, 0.01)))
        success = self._execute_joint_motion_impl(target_joints, duration)
        if success:
            self._publish_motion_result(GraspResult.SUCCESS, "Cartesian motion completed")
        else:
            self._publish_motion_result(GraspResult.EXECUTION_FAILED, "Cartesian motion failed")

    def _execute_joint_motion(self, target_joints: List[float], cmd: MotionCommand):
        """Execute joint space motion."""
        rospy.loginfo("[MotionControl] Executing joint motion...")
        if not self._check_joint_limits(target_joints):
            self._publish_motion_result(GraspResult.UNREACHABLE, "Joint limit violation")
            return
        duration = 4.0
        if cmd.max_velocity > 0:
            duration = max(1.0, 4.0 * (1.0 / max(cmd.max_velocity, 0.01)))
        success = self._execute_joint_motion_impl(target_joints, duration)
        if success:
            self._publish_motion_result(GraspResult.SUCCESS, "Joint motion completed")
        else:
            self._publish_motion_result(GraspResult.EXECUTION_FAILED, "Joint motion failed")

    def _execute_joint_motion_impl(self, target_joints: List[float], duration: float) -> bool:
        """Execute joint motion via action client. Single-point goal (same as test_ur5e_control). Returns True on success."""
        if self.current_joint_positions is None:
            rospy.logerr("[MotionControl] No current joint state")
            return False
        if len(target_joints) != self.num_joints:
            rospy.logerr("[MotionControl] Invalid joint count")
            return False
        # Match test_ur5e_control: single point, positions + time_from_start only (no vel/acc)
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = list(target_joints)
        point.time_from_start = rospy.Duration(duration)
        goal.trajectory.points.append(point)
        self.trajectory_client.send_goal(goal)
        if self.trajectory_client.wait_for_result(timeout=rospy.Duration(duration + 5.0)):
            return self.trajectory_client.get_result() is not None
        self.trajectory_client.cancel_goal()
        rospy.logerr("[MotionControl] Motion timed out")
        return False

    def _scale_trajectory_timing(self, trajectory: JointTrajectory, max_velocity: float, max_acceleration: float):
        """Apply coarse time scaling to a trajectory based on velocity/acceleration factors."""
        command = copy.deepcopy(trajectory)
        scale_factors = [1.0]

        if max_velocity > 0.0:
            scale_factors.append(1.0 / max_velocity)
        if max_acceleration > 0.0:
            scale_factors.append(1.0 / np.sqrt(max_acceleration))

        time_scale = max(scale_factors)
        if abs(time_scale - 1.0) < 1e-6:
            return command

        for point in command.points:
            scaled_time = max(0.05, point.time_from_start.to_sec() * time_scale)
            point.time_from_start = rospy.Duration(scaled_time)
            if point.velocities:
                point.velocities = [value / time_scale for value in point.velocities]
            if point.accelerations:
                point.accelerations = [value / (time_scale ** 2) for value in point.accelerations]
        return command

    def _execute_trajectory_sync(
        self,
        trajectory: JointTrajectory,
        max_velocity: float = 1.0,
        max_acceleration: float = 1.0,
        preempt_check=None,
        feedback_cb=None,
    ):
        """Execute a pre-computed trajectory and return (success, status, message)."""
        rospy.loginfo("[MotionControl] Executing trajectory...")
        if len(trajectory.points) == 0:
            return False, ExecuteTrajectoryResult.INVALID_TRAJECTORY, "Empty trajectory"

        if not self.trajectory_client.wait_for_server(timeout=rospy.Duration(1.0)):
            return False, ExecuteTrajectoryResult.EXECUTION_FAILED, "Trajectory action server unavailable"

        raw_duration = trajectory.points[-1].time_from_start.to_sec()
        command = self._scale_trajectory_timing(trajectory, max_velocity, max_acceleration)
        if not command.joint_names:
            command.joint_names = self.joint_names
        scaled_duration = command.points[-1].time_from_start.to_sec()
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = command
        rospy.loginfo(
            "[MotionControl] Sending FollowJointTrajectory goal: points=%d raw_duration=%.2fs scaled_duration=%.2fs vel_scale=%.2f acc_scale=%.2f",
            len(command.points),
            raw_duration,
            scaled_duration,
            max_velocity,
            max_acceleration,
        )
        self.trajectory_client.send_goal(goal)
        rospy.loginfo("[MotionControl] FollowJointTrajectory goal sent to controller")
        if feedback_cb is not None:
            feedback_cb(ExecuteTrajectoryFeedback.EXECUTING, "Trajectory executing")

        timeout_sec = max(60.0, command.points[-1].time_from_start.to_sec() + 5.0)
        start_time = rospy.Time.now()
        last_wait_log = start_time

        while not rospy.is_shutdown():
            if preempt_check is not None and preempt_check():
                self.trajectory_client.cancel_goal()
                return False, ExecuteTrajectoryResult.PREEMPTED, "Trajectory execution preempted"

            now = rospy.Time.now()
            if (now - last_wait_log).to_sec() >= 2.0:
                rospy.loginfo(
                    "[MotionControl] Waiting for controller result... elapsed=%.1fs / timeout=%.1fs",
                    (now - start_time).to_sec(),
                    timeout_sec,
                )
                last_wait_log = now

            if self.trajectory_client.wait_for_result(timeout=rospy.Duration(0.1)):
                state = self.trajectory_client.get_state()
                rospy.loginfo("[MotionControl] Controller finished with state=%d", state)
                if state == action_msgs.GoalStatus.SUCCEEDED:
                    return True, ExecuteTrajectoryResult.SUCCEEDED, "Trajectory completed"
                return False, ExecuteTrajectoryResult.EXECUTION_FAILED, (
                    f"Trajectory failed with controller state {state}"
                )

            if (rospy.Time.now() - start_time).to_sec() > timeout_sec:
                self.trajectory_client.cancel_goal()
                return False, ExecuteTrajectoryResult.EXECUTION_FAILED, "Trajectory failed (timeout)"

        self.trajectory_client.cancel_goal()
        return False, ExecuteTrajectoryResult.PREEMPTED, "Trajectory execution interrupted"

    def _execute_trajectory_action_cb(self, goal):
        execution_source = "action:execute_trajectory"
        ok, active_source = self._try_begin_execution(execution_source)
        result = ExecuteTrajectoryResult()

        if not ok:
            result.success = False
            result.status = ExecuteTrajectoryResult.BUSY
            result.message = f"Motion controller busy: {active_source}"
            self.execute_trajectory_server.set_aborted(result, result.message)
            return

        try:
            traj_points = len(goal.trajectory.points)
            traj_duration = (
                goal.trajectory.points[-1].time_from_start.to_sec()
                if traj_points > 0
                else 0.0
            )
            rospy.loginfo(
                "[MotionControl] /motion_control/execute_trajectory accepted: points=%d duration=%.2fs vel_scale=%.2f acc_scale=%.2f",
                traj_points,
                traj_duration,
                goal.max_velocity,
                goal.max_acceleration,
            )
            self._publish_execute_feedback(ExecuteTrajectoryFeedback.ACCEPTED, "Trajectory accepted")
            success, status, message = self._execute_trajectory_sync(
                goal.trajectory,
                max_velocity=goal.max_velocity,
                max_acceleration=goal.max_acceleration,
                preempt_check=self.execute_trajectory_server.is_preempt_requested,
                feedback_cb=self._publish_execute_feedback,
            )
            rospy.loginfo(
                "[MotionControl] /motion_control/execute_trajectory finished: success=%s status=%d message=%s",
                str(success),
                status,
                message,
            )
            result.success = success
            result.status = status
            result.message = message

            motion_status = GraspResult.SUCCESS if success else GraspResult.EXECUTION_FAILED
            self._publish_motion_result(motion_status, message)

            if status == ExecuteTrajectoryResult.PREEMPTED:
                self.execute_trajectory_server.set_preempted(result, message)
            elif success:
                self.execute_trajectory_server.set_succeeded(result, message)
            else:
                self.execute_trajectory_server.set_aborted(result, message)
        finally:
            self._finish_execution(execution_source)

    def _stop_motion(self):
        """Emergency stop: cancel all goals."""
        rospy.logwarn("[MotionControl] Stopping motion!")
        self.trajectory_client.cancel_all_goals()
        self._publish_motion_result(GraspResult.SUCCESS, "Motion stopped")

    def _move_to_home(self):
        """Move to home configuration (standard joint order)."""
        rospy.loginfo("[MotionControl] Moving to home position...")
        success = self._execute_joint_motion_impl(self.home_joints, 4.0)
        if success:
            self._publish_motion_result(GraspResult.SUCCESS, "Home position reached")
        else:
            self._publish_motion_result(GraspResult.EXECUTION_FAILED, "Home motion failed")


    # =========================================================================
    # Trajectory Generation
    # =========================================================================

    def generate_joint_trajectory(
        self,
        start_joints: List[float],
        goal_joints: List[float],
        duration: float,
        max_velocity: float = 1.0,
        max_acceleration: float = 1.0,
        num_points: int = 50
    ) -> JointTrajectory:
        """Generate smooth joint trajectory (cubic interpolation)."""
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        for i in range(num_points + 1):
            t = (i / num_points) * duration
            point = JointTrajectoryPoint()
            point.positions = []
            point.velocities = []
            point.accelerations = []
            for j in range(self.num_joints):
                a0, a1 = start_joints[j], 0.0
                a2 = 3.0 * (goal_joints[j] - start_joints[j]) / (duration ** 2)
                a3 = -2.0 * (goal_joints[j] - start_joints[j]) / (duration ** 3)
                point.positions.append(a0 + a1*t + a2*t**2 + a3*t**3)
                point.velocities.append(a1 + 2*a2*t + 3*a3*t**2)
                point.accelerations.append(2*a2 + 6*a3*t)
            point.time_from_start = rospy.Duration(t)
            trajectory.points.append(point)
        return trajectory


    def generate_cartesian_path(
        self,
        waypoints: List[PoseStamped],
        step_size: float = 0.01
    ) -> Optional[JointTrajectory]:
        """
        Generate trajectory through Cartesian waypoints
        
        Args:
            waypoints: List of Cartesian poses to pass through
            step_size: Step size for intermediate waypoints (meters)
            
        Returns:
            JointTrajectory in joint space
            
        TODO: Implement Cartesian path generation
        Steps:
        1. Interpolate between waypoints with fixed step size
        2. Compute IK for each intermediate point
        3. Check for IK failures and trajectory continuity
        4. Return joint space trajectory
        
        Useful for straight-line motions, circular paths, etc.
        """
        # TODO: Implement Cartesian path generation
        return None


    # =========================================================================
    # Utilities
    # =========================================================================

    def _publish_motion_result(self, status: int, message: str):
        """Publish motion execution result (GraspResult)."""
        result = GraspResult()
        result.status = status
        result.message = message
        result.execution_time = 0.0
        self.motion_result_pub.publish(result)


    def _check_joint_limits(self, joint_angles: List[float]) -> bool:
        """Check if joint angles are within limits (same order as joint_names)."""
        if len(joint_angles) != self.num_joints:
            return False
        for i, name in enumerate(self.joint_names):
            if name not in self.joint_limits:
                continue
            lo, hi = self.joint_limits[name]
            if not (lo <= joint_angles[i] <= hi):
                rospy.logwarn(f"[MotionControl] {name} = {joint_angles[i]:.3f} outside [{lo:.3f}, {hi:.3f}]")
                return False
        return True


    def _transform_pose(self, pose: PoseStamped, target_frame: str) -> Optional[PoseStamped]:
        """
        Transform pose to target frame using TF2
        
        Args:
            pose: Input pose
            target_frame: Target reference frame
            
        Returns:
            Transformed pose or None if transform fails
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                pose.header.frame_id,
                rospy.Time(0),
                rospy.Duration(1.0)
            )
            return tf2_geometry_msgs.do_transform_pose(pose, transform)
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"[MotionControl] TF transform failed: {e}")
            return None


    def run(self):
        """Main loop"""
        rospy.spin()


if __name__ == '__main__':
    try:
        node = MotionControlNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
