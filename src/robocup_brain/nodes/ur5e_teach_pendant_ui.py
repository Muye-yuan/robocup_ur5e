#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UR5e Teach Pendant UI - Soft Teach Pendant
UI-only script, motion control is in motion_control_node.py

Author: Suhang Xia
Usage:
  rosrun robocup_brain ur5e_teach_pendant_ui.py
"""

import sys
import threading
import rospy
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QDoubleSpinBox, QGroupBox, QGridLayout,
    QTextEdit, QCheckBox
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QFont

# ROS Messages
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger
from common_msgs.msg import MotionCommand, GraspResult
import tf2_ros
import tf2_geometry_msgs
import tf.transformations as tf_trans


class UR5eTeachPendantUI(QMainWindow):
    """
    UR5e Soft Teach Pendant UI.
    This class only handles UI display and button events.
    Motion control is implemented in motion_control_node.py.
    """

    pose_updated_signal = pyqtSignal(dict)
    status_updated_signal = pyqtSignal(str)
    joints_updated_signal = pyqtSignal(list)
    traverse_advance_signal = pyqtSignal()  # rospy thread -> main thread, advance traversal phase

    def __init__(self):
        super().__init__()
        self.setWindowTitle("UR5e Teach Pendant")
        self.setGeometry(100, 100, 800, 700)

        self.linear_step = 0.01
        self.rotation_step = 0.087
        self.current_pose = None      # dict with x,y,z,roll,pitch,yaw for display
        self.current_pose_stamped = None  # PoseStamped for sending to motion_control
        self.current_joints = None
        self.is_moving = False
        self.recorded_points_list = []  # list of {"pose": dict, "joints": list, "index": int}

        # Dexterous space traversal state
        self._traversing = False
        self._traverse_waypoints = []  # list of (x, y, z) position waypoints
        self._traverse_index = 0       # current position waypoint index
        self._traverse_phase = 'move'  # 'move' | 'roll' | 'pitch' | 'yaw' | 'restore'
        self._traverse_base_quat = None  # orientation at the position point

        # motion_control connected: set True when /motion/result received
        self._motion_control_connected = False
        # Dexterous workspace bounds [x_min, x_max, y_min, y_max, z_min, z_max] from /motion/workspace_bounds
        self._workspace_bounds = None

        self.init_ros()
        self.init_ui()

        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.on_update_timer)
        self.update_timer.start(100)

        self.pose_updated_signal.connect(self.on_pose_updated)
        self.status_updated_signal.connect(self.on_status_updated)
        self.joints_updated_signal.connect(self.on_joints_updated)
        self.traverse_advance_signal.connect(self._on_traverse_advance_requested)

    def init_ros(self):
        try:
            rospy.init_node('ur5e_teach_pendant_ui', anonymous=True)
            self.motion_cmd_pub = rospy.Publisher('/motion/command', MotionCommand, queue_size=10)
            self.gripper_cmd_pub = rospy.Publisher('/gripper/command', String, queue_size=1)
            self.joint_state_sub = rospy.Subscriber('/joint_states', JointState, self.on_joint_state_received)
            self.motion_result_sub = rospy.Subscriber('/motion/result', GraspResult, self.on_motion_result_received)
            self.workspace_bounds_sub = rospy.Subscriber(
                '/motion/workspace_bounds', Float64MultiArray, self._on_workspace_bounds_received
            )
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
            # Run rospy callbacks in a background thread (rospy.spin_once not in older rospy)
            self._ros_thread = threading.Thread(target=self._ros_spin_thread, daemon=True)
            self._ros_thread.start()
            rospy.loginfo("[TeachPendantUI] ROS initialized")
        except Exception as e:
            rospy.logerr(f"[TeachPendantUI] ROS init failed: {e}")

    def _ros_spin_thread(self):
        """Background thread: run rospy.spin() so callbacks (joint_states, motion/result, TF) run."""
        try:
            rospy.spin()
        except rospy.exceptions.ROSInterruptException:
            pass
        except Exception:
            pass

    def _get_current_pose_from_tf(self):
        """
        Get current end-effector pose (base_link -> tcp_link).
        tcp_link should match MoveIt's tip link (e.g. gripper_tip_link) so IK is consistent.
        Returns (PoseStamped or None, pose_dict or None). pose_dict has x,y,z,roll,pitch,yaw.
        """
        tcp_link = rospy.get_param('~tcp_link', 'gripper_tip_link')  # same as MoveIt ik_link
        try:
            trans = self.tf_buffer.lookup_transform(
                'base_link', tcp_link, rospy.Time(0), rospy.Duration(0.5)
            )
            pose = PoseStamped()
            pose.header = trans.header
            pose.pose.position = Point(
                trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.translation.z
            )
            pose.pose.orientation = Quaternion(
                trans.transform.rotation.x,
                trans.transform.rotation.y,
                trans.transform.rotation.z,
                trans.transform.rotation.w
            )
            r = trans.transform.rotation
            q = [r.x, r.y, r.z, r.w]
            roll, pitch, yaw = tf_trans.euler_from_quaternion(q)
            pose_dict = {
                'x': pose.pose.position.x, 'y': pose.pose.position.y, 'z': pose.pose.position.z,
                'roll': roll, 'pitch': pitch, 'yaw': yaw
            }
            return pose, pose_dict
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logdebug("[TeachPendantUI] TF lookup failed: %s" % str(e))
            return None, None

    @staticmethod
    def _extract_arm_joints_from_joint_state(msg):
        joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint',
        ]
        joints = []
        for name in joint_names:
            if name not in msg.name:
                return None
            joints.append(float(msg.position[msg.name.index(name)]))
        return joints

    def _get_current_joints_from_topic(self, timeout_sec=0.5, prefer_cached=True):
        if prefer_cached and self.current_joints and len(self.current_joints) == 6:
            return list(self.current_joints)
        try:
            msg = rospy.wait_for_message('/joint_states', JointState, timeout=timeout_sec)
        except rospy.ROSException:
            return None
        return self._extract_arm_joints_from_joint_state(msg)

    def _apply_joint_values_to_ui(self, joints, force=False):
        if joints is None:
            return
        self.current_joints = list(joints)
        if self._joint_edit_mode and not force:
            return
        for i, angle in enumerate(joints):
            if i < len(self.joint_spinboxes):
                self.joint_spinboxes[i].blockSignals(True)
                self.joint_spinboxes[i].setValue(angle)
                self.joint_spinboxes[i].blockSignals(False)

    def _send_move_to_pose(self, target_pose_stamped):
        """Publish MotionCommand MOVE_TO_POSE."""
        target_pose_stamped.header.stamp = rospy.Time.now()
        target_pose_stamped.header.frame_id = 'base_link'
        cmd = MotionCommand()
        cmd.command_type = MotionCommand.MOVE_TO_POSE
        cmd.target_pose = target_pose_stamped
        cmd.max_velocity = 1.0   # 1.0 => ~5s for Cartesian; 0.5 => ~10s (slower)
        cmd.max_acceleration = 1.0
        self.motion_cmd_pub.publish(cmd)
        self.is_moving = True
        self.status_updated_signal.emit("Status: Moving...")

    def _move_in_local_frame(self, dx=0.0, dy=0.0, dz=0.0, droll=0.0, dpitch=0.0, dyaw=0.0):
        """Move gripper_tip_link in its own local coordinate system.
        dx/dy/dz: translation along gripper's local X/Y/Z axes (meters).
        droll/dpitch/dyaw: rotation about gripper's local X/Y/Z axes (radians)."""
        pose_stamped, _ = self._get_current_pose_from_tf()
        if pose_stamped is None:
            self.log("No current pose (TF)")
            return
        o = pose_stamped.pose.orientation
        q_current = [o.x, o.y, o.z, o.w]
        R_current = tf_trans.quaternion_matrix(q_current)[:3, :3]

        p = pose_stamped.pose.position
        delta_local = [dx, dy, dz]
        delta_base = R_current @ delta_local
        pose_stamped.pose.position.x = p.x + delta_base[0]
        pose_stamped.pose.position.y = p.y + delta_base[1]
        pose_stamped.pose.position.z = p.z + delta_base[2]

        if droll != 0.0 or dpitch != 0.0 or dyaw != 0.0:
            q_delta = tf_trans.quaternion_from_euler(droll, dpitch, dyaw)
            q_new = tf_trans.quaternion_multiply(q_current, q_delta)
            pose_stamped.pose.orientation = Quaternion(q_new[0], q_new[1], q_new[2], q_new[3])

        self._send_move_to_pose(pose_stamped)

    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        left_panel = QWidget()
        right_panel = QWidget()
        main_layout.addWidget(left_panel, 2)
        main_layout.addWidget(right_panel, 1)
        left_layout = QVBoxLayout(left_panel)

        pose_group = QGroupBox("Current EE Pose (Base Link -> gripper_tip_link (TCP))")
        pose_layout = QGridLayout(pose_group)
        pose_layout.addWidget(QLabel("<b>Position (m):</b>"), 0, 0, 1, 6)
        self.pose_labels = {}
        for i, coord in enumerate(['X', 'Y', 'Z']):
            label_value = QLabel("--")
            label_value.setStyleSheet("font-family: monospace; font-size: 12px;")
            self.pose_labels[f'pos_{coord}'] = label_value
            pose_layout.addWidget(QLabel(f"{coord}:"), 1, i * 2)
            pose_layout.addWidget(label_value, 1, i * 2 + 1)
        pose_layout.addWidget(QLabel("<b>Orientation (rad):</b>"), 2, 0, 1, 6)
        for i, coord in enumerate(['Roll', 'Pitch', 'Yaw']):
            label_value = QLabel("--")
            label_value.setStyleSheet("font-family: monospace; font-size: 12px;")
            self.pose_labels[f'rot_{coord}'] = label_value
            pose_layout.addWidget(QLabel(f"{coord}:"), 3, i * 2)
            pose_layout.addWidget(label_value, 3, i * 2 + 1)
        self.btn_refresh_pose = QPushButton("Refresh Pose")
        self.btn_refresh_pose.clicked.connect(self.on_refresh_pose_clicked)
        pose_layout.addWidget(self.btn_refresh_pose, 4, 0, 1, 6)
        left_layout.addWidget(pose_group)

        target_group = QGroupBox("Targeted EE Pose (Base Link -> gripper_tip_link (TCP))")
        target_layout = QGridLayout(target_group)
        target_layout.addWidget(QLabel("<b>Position (m):</b>"), 0, 0, 1, 3)
        target_layout.addWidget(QLabel("<b>Orientation (rad):</b>"), 0, 3, 1, 3)
        self.target_inputs = {}
        for i, (label, key) in enumerate([('X:', 'x'), ('Y:', 'y'), ('Z:', 'z')]):
            target_layout.addWidget(QLabel(label), i + 1, 0)
            spinbox = QDoubleSpinBox()
            spinbox.setRange(-2.0, 2.0)
            spinbox.setDecimals(4)
            spinbox.setSingleStep(0.01)
            spinbox.setStyleSheet("font-family: monospace;")
            self.target_inputs[key] = spinbox
            target_layout.addWidget(spinbox, i + 1, 1)
        for i, (label, key) in enumerate([('Roll:', 'roll'), ('Pitch:', 'pitch'), ('Yaw:', 'yaw')]):
            target_layout.addWidget(QLabel(label), i + 1, 3)
            spinbox = QDoubleSpinBox()
            spinbox.setRange(-6.2832, 6.2832)
            spinbox.setDecimals(4)
            spinbox.setSingleStep(0.01)
            spinbox.setStyleSheet("font-family: monospace;")
            self.target_inputs[key] = spinbox
            target_layout.addWidget(spinbox, i + 1, 4)
        self.btn_go = QPushButton("GO")
        self.btn_go.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold; font-size: 14px; padding: 6px;")
        self.btn_go.clicked.connect(self.on_go_clicked)
        target_layout.addWidget(self.btn_go, 4, 0, 1, 6)
        left_layout.addWidget(target_group)

        step_group = QGroupBox("Step Size")
        step_layout = QGridLayout(step_group)
        step_layout.addWidget(QLabel("Linear step (m):"), 0, 0)
        self.linear_spinbox = QDoubleSpinBox()
        self.linear_spinbox.setRange(0.001, 0.1)
        self.linear_spinbox.setValue(self.linear_step)
        self.linear_spinbox.setSingleStep(0.001)
        self.linear_spinbox.setDecimals(3)
        self.linear_spinbox.valueChanged.connect(self.on_linear_step_changed)
        step_layout.addWidget(self.linear_spinbox, 0, 1)
        step_layout.addWidget(QLabel("Rotation step (deg):"), 1, 0)
        self.rotation_spinbox = QDoubleSpinBox()
        self.rotation_spinbox.setRange(0.1, 45.0)
        self.rotation_spinbox.setValue(self.rotation_step * 180 / 3.14159)
        self.rotation_spinbox.setSingleStep(0.5)
        self.rotation_spinbox.setDecimals(1)
        self.rotation_spinbox.valueChanged.connect(self.on_rotation_step_changed)
        step_layout.addWidget(self.rotation_spinbox, 1, 1)
        left_layout.addWidget(step_group)

        trans_group = QGroupBox("Translation")
        trans_layout = QGridLayout(trans_group)
        for row, (axis, neg_lbl, pos_lbl, on_neg, on_pos) in enumerate([
            ("X", "-X ←", "+X →", self.on_x_negative_clicked, self.on_x_positive_clicked),
            ("Y", "-Y ↓", "+Y ↑", self.on_y_negative_clicked, self.on_y_positive_clicked),
            ("Z", "-Z ↓", "+Z ↑", self.on_z_negative_clicked, self.on_z_positive_clicked),
        ]):
            trans_layout.addWidget(QLabel(f"<b>{axis}:</b>"), row, 0)
            b1, b2 = QPushButton(neg_lbl), QPushButton(pos_lbl)
            b1.setStyleSheet("background-color: #ffcccc;")
            b2.setStyleSheet("background-color: #ccffcc;")
            b1.clicked.connect(on_neg)
            b2.clicked.connect(on_pos)
            trans_layout.addWidget(b1, row, 1)
            trans_layout.addWidget(b2, row, 2)
        left_layout.addWidget(trans_group)

        rot_group = QGroupBox("Rotation")
        rot_layout = QGridLayout(rot_group)
        for row, (name, neg_lbl, pos_lbl, on_neg, on_pos) in enumerate([
            ("Roll", "-Roll ↺", "+Roll ↻", self.on_roll_negative_clicked, self.on_roll_positive_clicked),
            ("Pitch", "-Pitch ↺", "+Pitch ↻", self.on_pitch_negative_clicked, self.on_pitch_positive_clicked),
            ("Yaw", "-Yaw ↺", "+Yaw ↻", self.on_yaw_negative_clicked, self.on_yaw_positive_clicked),
        ]):
            rot_layout.addWidget(QLabel(f"<b>{name}:</b>"), row, 0)
            b1, b2 = QPushButton(neg_lbl), QPushButton(pos_lbl)
            b1.setStyleSheet("background-color: #ffcccc;")
            b2.setStyleSheet("background-color: #ccffcc;")
            b1.clicked.connect(on_neg)
            b2.clicked.connect(on_pos)
            rot_layout.addWidget(b1, row, 1)
            rot_layout.addWidget(b2, row, 2)
        left_layout.addWidget(rot_group)

        func_group = QGroupBox("Actions")
        func_layout = QHBoxLayout(func_group)
        self.btn_home = QPushButton("Home")
        self.btn_home.clicked.connect(self.on_home_clicked)
        self.btn_stop = QPushButton("E-Stop")
        self.btn_stop.setStyleSheet("background-color: #ff6666; font-weight: bold;")
        self.btn_stop.clicked.connect(self.on_stop_clicked)
        self.btn_record = QPushButton("Record Point")
        self.btn_record.clicked.connect(self.on_record_point_clicked)
        func_layout.addWidget(self.btn_home)
        func_layout.addWidget(self.btn_stop)
        func_layout.addWidget(self.btn_record)
        left_layout.addWidget(func_group)

        self.status_label = QLabel("Status: Ready | motion_control: waiting...")
        self.status_label.setStyleSheet("padding: 5px; background-color: #f0f0f0;")
        self.status_label.setAlignment(Qt.AlignCenter)
        left_layout.addWidget(self.status_label)

        right_layout = QVBoxLayout(right_panel)
        joints_group = QGroupBox("Joint Angles (rad)")
        joints_layout = QVBoxLayout(joints_group)
        self.joint_spinboxes = []
        self.joint_labels = []
        joint_names = ['J1 (Shoulder Pan)', 'J2 (Shoulder Lift)', 'J3 (Elbow)', 'J4 (Wrist 1)', 'J5 (Wrist 2)', 'J6 (Wrist 3)']
        for name in joint_names:
            h = QHBoxLayout()
            h.addWidget(QLabel(f"{name}:"))
            spinbox = QDoubleSpinBox()
            spinbox.setRange(-6.2832, 6.2832)
            spinbox.setDecimals(4)
            spinbox.setSingleStep(0.01)
            spinbox.setReadOnly(True)
            spinbox.setStyleSheet("font-family: monospace;")
            self.joint_spinboxes.append(spinbox)
            self.joint_labels.append(spinbox)
            h.addWidget(spinbox)
            joints_layout.addLayout(h)
        btn_row = QHBoxLayout()
        self._joint_edit_mode = False
        self.btn_joint_edit = QPushButton("Edit")
        self.btn_joint_edit.setStyleSheet("background-color: #FF9800; color: white; font-weight: bold; padding: 6px;")
        self.btn_joint_edit.clicked.connect(self._toggle_joint_edit_mode)
        btn_row.addWidget(self.btn_joint_edit)
        self.btn_joint_go = QPushButton("GO (Joint)")
        self.btn_joint_go.setStyleSheet("background-color: #2196F3; color: white; font-weight: bold; font-size: 14px; padding: 6px;")
        self.btn_joint_go.clicked.connect(self.on_joint_go_clicked)
        self.btn_joint_go.setEnabled(False)
        btn_row.addWidget(self.btn_joint_go)
        joints_layout.addLayout(btn_row)
        right_layout.addWidget(joints_group)

        traverse_group = QGroupBox("Dexterous Space Traversal")
        traverse_layout = QGridLayout(traverse_group)
        traverse_layout.addWidget(QLabel("Height Z (m):"), 0, 0)
        self.traverse_z_spinbox = QDoubleSpinBox()
        self.traverse_z_spinbox.setRange(0.1, 1.0)
        self.traverse_z_spinbox.setValue(0.45)
        self.traverse_z_spinbox.setDecimals(3)
        self.traverse_z_spinbox.setSingleStep(0.01)
        traverse_layout.addWidget(self.traverse_z_spinbox, 0, 1)
        traverse_layout.addWidget(QLabel("Target points:"), 1, 0)
        self.traverse_points_spinbox = QDoubleSpinBox()
        self.traverse_points_spinbox.setRange(50, 2000)
        self.traverse_points_spinbox.setValue(500)
        self.traverse_points_spinbox.setDecimals(0)
        self.traverse_points_spinbox.setSingleStep(50)
        traverse_layout.addWidget(self.traverse_points_spinbox, 1, 1)
        traverse_layout.addWidget(QLabel("Workspace:"), 2, 0)
        self.traverse_workspace_label = QLabel("Waiting for motion_control...")
        self.traverse_workspace_label.setStyleSheet("color: #666; font-size: 10px;")
        traverse_layout.addWidget(self.traverse_workspace_label, 2, 1)
        traverse_layout.addWidget(QLabel("Move time (s):"), 3, 0)
        self.traverse_time_spinbox = QDoubleSpinBox()
        self.traverse_time_spinbox.setRange(0.5, 20.0)
        self.traverse_time_spinbox.setValue(3.0)
        self.traverse_time_spinbox.setDecimals(1)
        self.traverse_time_spinbox.setSingleStep(0.5)
        traverse_layout.addWidget(self.traverse_time_spinbox, 3, 1)
        self.traverse_fixed_down_check = QCheckBox("Fixed downward pose (camera faces table)")
        self.traverse_fixed_down_check.setChecked(True)
        self.traverse_fixed_down_check.setToolTip("Checked: fixed downward pose; Unchecked: use current pose")
        traverse_layout.addWidget(self.traverse_fixed_down_check, 4, 0, 1, 2)
        traverse_btn_row = QHBoxLayout()
        self.btn_traverse_start = QPushButton("Start")
        self.btn_traverse_start.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold; padding: 6px;")
        self.btn_traverse_start.clicked.connect(self._start_traversal)
        traverse_btn_row.addWidget(self.btn_traverse_start)
        self.btn_traverse_stop = QPushButton("Stop")
        self.btn_traverse_stop.setStyleSheet("background-color: #f44336; color: white; font-weight: bold; padding: 6px;")
        self.btn_traverse_stop.clicked.connect(self._stop_traversal)
        self.btn_traverse_stop.setEnabled(False)
        traverse_btn_row.addWidget(self.btn_traverse_stop)
        traverse_layout.addLayout(traverse_btn_row, 5, 0, 1, 2)
        self.traverse_status_label = QLabel("Idle")
        self.traverse_status_label.setAlignment(Qt.AlignCenter)
        self.traverse_status_label.setStyleSheet("font-family: monospace; color: #666;")
        traverse_layout.addWidget(self.traverse_status_label, 6, 0, 1, 2)
        right_layout.addWidget(traverse_group)

        log_group = QGroupBox("Log")
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        # QTextEdit has no setMaximumBlockCount (only QPlainTextEdit does)
        log_group_layout = QVBoxLayout(log_group)
        log_group_layout.addWidget(self.log_text)
        right_layout.addWidget(log_group)

        gripper_group = QGroupBox("Gripper Control")
        gripper_layout = QHBoxLayout(gripper_group)
        self.btn_grasp = QPushButton("Grasp")
        self.btn_grasp.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold; padding: 6px;")
        self.btn_grasp.clicked.connect(self.on_grasp_clicked)
        self.btn_release = QPushButton("Release")
        self.btn_release.setStyleSheet("background-color: #2196F3; color: white; font-weight: bold; padding: 6px;")
        self.btn_release.clicked.connect(self.on_release_clicked)
        gripper_layout.addWidget(self.btn_grasp)
        gripper_layout.addWidget(self.btn_release)
        right_layout.addWidget(gripper_group)

        points_group = QGroupBox("Recorded Points")
        points_layout = QVBoxLayout(points_group)
        self.recorded_points = QTextEdit()
        self.recorded_points.setReadOnly(True)
        self.recorded_points.setMaximumHeight(150)
        points_layout.addWidget(self.recorded_points)
        self.btn_clear_points = QPushButton("Clear Points")
        self.btn_clear_points.clicked.connect(self.on_clear_points_clicked)
        points_layout.addWidget(self.btn_clear_points)
        right_layout.addWidget(points_group)

    def on_go_clicked(self):
        """Send robot to the targeted TCP pose entered by the user."""
        x = self.target_inputs['x'].value()
        y = self.target_inputs['y'].value()
        z = self.target_inputs['z'].value()
        roll = self.target_inputs['roll'].value()
        pitch = self.target_inputs['pitch'].value()
        yaw = self.target_inputs['yaw'].value()
        q = tf_trans.quaternion_from_euler(roll, pitch, yaw)
        target = PoseStamped()
        target.header.frame_id = 'base_link'
        target.header.stamp = rospy.Time.now()
        target.pose.position = Point(x, y, z)
        target.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
        self.log(f"GO -> pos=[{x:.4f}, {y:.4f}, {z:.4f}] rpy=[{roll:.4f}, {pitch:.4f}, {yaw:.4f}]")
        self._send_move_to_pose(target)

    def on_linear_step_changed(self, value):
        self.linear_step = value
        self.log(f"Linear step set: {value*1000:.1f} mm")

    def on_rotation_step_changed(self, value):
        self.rotation_step = value * 3.14159 / 180.0
        self.log(f"Rotation step set: {value:.1f} deg")

    def on_x_negative_clicked(self):
        self._move_in_local_frame(dx=-self.linear_step)
        self.log(f"X- {-self.linear_step*1000:.1f} mm (local)")
    def on_x_positive_clicked(self):
        self._move_in_local_frame(dx=self.linear_step)
        self.log(f"X+ +{self.linear_step*1000:.1f} mm (local)")
    def on_y_negative_clicked(self):
        self._move_in_local_frame(dy=-self.linear_step)
        self.log(f"Y- -{self.linear_step*1000:.1f} mm (local)")
    def on_y_positive_clicked(self):
        self._move_in_local_frame(dy=self.linear_step)
        self.log(f"Y+ +{self.linear_step*1000:.1f} mm (local)")
    def on_z_negative_clicked(self):
        self._move_in_local_frame(dz=-self.linear_step)
        self.log(f"Z- -{self.linear_step*1000:.1f} mm (local)")
    def on_z_positive_clicked(self):
        self._move_in_local_frame(dz=self.linear_step)
        self.log(f"Z+ +{self.linear_step*1000:.1f} mm (local)")
    def on_roll_negative_clicked(self):
        self._move_in_local_frame(droll=-self.rotation_step)
        self.log(f"Roll- -{self.rotation_step*180/3.14159:.1f} deg (local)")
    def on_roll_positive_clicked(self):
        self._move_in_local_frame(droll=self.rotation_step)
        self.log(f"Roll+ +{self.rotation_step*180/3.14159:.1f} deg (local)")
    def on_pitch_negative_clicked(self):
        self._move_in_local_frame(dpitch=-self.rotation_step)
        self.log(f"Pitch- -{self.rotation_step*180/3.14159:.1f} deg (local)")
    def on_pitch_positive_clicked(self):
        self._move_in_local_frame(dpitch=self.rotation_step)
        self.log(f"Pitch+ +{self.rotation_step*180/3.14159:.1f} deg (local)")
    def on_yaw_negative_clicked(self):
        self._move_in_local_frame(dyaw=-self.rotation_step)
        self.log(f"Yaw- -{self.rotation_step*180/3.14159:.1f} deg (local)")
    def on_yaw_positive_clicked(self):
        self._move_in_local_frame(dyaw=self.rotation_step)
        self.log(f"Yaw+ +{self.rotation_step*180/3.14159:.1f} deg (local)")
    def on_home_clicked(self):
        cmd = MotionCommand()
        cmd.command_type = MotionCommand.HOME
        self.motion_cmd_pub.publish(cmd)
        self.is_moving = True
        self.status_updated_signal.emit("Status: Going Home...")
        self.log("Go to Home")
    def on_stop_clicked(self):
        cmd = MotionCommand()
        cmd.command_type = MotionCommand.STOP
        self.motion_cmd_pub.publish(cmd)
        self.is_moving = False
        self.status_updated_signal.emit("Status: E-Stop sent")
        self.log("E-Stop")
    def on_record_point_clicked(self):
        _, pose_dict = self._get_current_pose_from_tf()
        if pose_dict is None:
            self.log("Cannot record: current pose unknown (TF)")
            return
        self.current_pose = pose_dict
        idx = len(self.recorded_points_list) + 1
        self.recorded_points_list.append({
            "pose": dict(pose_dict),
            "joints": list(self.current_joints) if self.current_joints else None,
            "index": idx
        })
        self._update_recorded_points_display()
        self.log("Record point #%d" % idx)
    def on_clear_points_clicked(self):
        self.recorded_points_list.clear()
        self._update_recorded_points_display()
        self.log("Clear points")

    def on_grasp_clicked(self):
        """Call /gripper/grasp service."""
        try:
            grasp_srv = rospy.ServiceProxy('/gripper/grasp', Trigger)
            resp = grasp_srv()
            if resp.success:
                self.log("Gripper: Grasp success")
            else:
                self.log("Gripper: Grasp empty (no object)")
        except rospy.ServiceException as e:
            self.log("Gripper: Grasp failed - %s" % str(e))
        except Exception as e:
            self.log("Gripper: Grasp error - %s" % str(e))

    def on_release_clicked(self):
        """Publish release to /gripper/command."""
        try:
            msg = String(data="release")
            self.gripper_cmd_pub.publish(msg)
            self.log("Gripper: Release sent")
        except Exception as e:
            self.log("Gripper: Release error - %s" % str(e))
    def _update_recorded_points_display(self):
        lines = []
        for rec in self.recorded_points_list:
            p = rec["pose"]
            lines.append("P#%d: x=%.3f y=%.3f z=%.3f" % (rec["index"], p["x"], p["y"], p["z"]))
        self.recorded_points.setPlainText("\n".join(lines) if lines else "(none)")
    def on_refresh_pose_clicked(self):
        pose_stamped, pose_dict = self._get_current_pose_from_tf()
        if pose_dict is None:
            self.log("Refresh pose: TF failed (base_link->tool0)")
            return

        joints = self._get_current_joints_from_topic(timeout_sec=0.5, prefer_cached=False)
        self.current_pose = pose_dict
        self.current_pose_stamped = pose_stamped
        self.pose_updated_signal.emit(pose_dict)
        if joints is not None:
            self._apply_joint_values_to_ui(joints, force=True)
            self.log(
                "Refresh joints: [%s]"
                % ", ".join(f"{joint:.4f}" for joint in joints)
            )
            self.log("Refresh pose + joints OK")
        else:
            self.log("Refresh pose OK (joint states unavailable)")

    def on_joint_state_received(self, msg):
        joints = self._extract_arm_joints_from_joint_state(msg)
        if joints is not None:
            self.current_joints = joints
            self.joints_updated_signal.emit(joints)

    def _on_workspace_bounds_received(self, msg):
        """Cache workspace bounds from motion_control: [x_min, x_max, y_min, y_max, z_min, z_max]."""
        if len(msg.data) >= 6:
            self._workspace_bounds = (
                float(msg.data[0]), float(msg.data[1]),
                float(msg.data[2]), float(msg.data[3]),
                float(msg.data[4]), float(msg.data[5])
            )
            x_min, x_max, y_min, y_max, z_min, z_max = self._workspace_bounds
            if hasattr(self, 'traverse_workspace_label'):
                self.traverse_workspace_label.setText(
                    "X[%.2f,%.2f] Y[%.2f,%.2f] Z[%.2f,%.2f]" % (x_min, x_max, y_min, y_max, z_min, z_max)
                )
                self.traverse_workspace_label.setStyleSheet("color: #4CAF50; font-size: 10px;")

    def on_motion_result_received(self, msg):
        was_first = not self._motion_control_connected
        self._motion_control_connected = True
        if msg.status == GraspResult.SUCCESS:
            txt = "Status: Motion OK"
            if was_first:
                txt += " | motion_control connected"
            self.status_updated_signal.emit(txt)
            self.log("OK %s" % msg.message)
        else:
            txt = "Status: Motion Failed"
            if was_first:
                txt += " | motion_control connected"
            self.status_updated_signal.emit(txt)
            self.log("Failed %s" % msg.message)
        self.is_moving = False
        if self._traversing:
            self.traverse_advance_signal.emit()

    def _on_traverse_advance_requested(self):
        """Main thread slot: advance traversal phase after 500ms (QTimer must run in main thread)"""
        if self._traversing:
            QTimer.singleShot(500, self._advance_traverse_phase)

    def on_pose_updated(self, pose_dict):
        self.pose_labels['pos_X'].setText(f"{pose_dict['x']:.4f}")
        self.pose_labels['pos_Y'].setText(f"{pose_dict['y']:.4f}")
        self.pose_labels['pos_Z'].setText(f"{pose_dict['z']:.4f}")
        self.pose_labels['rot_Roll'].setText(f"{pose_dict['roll']:.4f}")
        self.pose_labels['rot_Pitch'].setText(f"{pose_dict['pitch']:.4f}")
        self.pose_labels['rot_Yaw'].setText(f"{pose_dict['yaw']:.4f}")

    def on_status_updated(self, status_text):
        self.status_label.setText(status_text)

    def _toggle_joint_edit_mode(self):
        """Toggle between Live (auto-update) and Edit (user can modify) modes."""
        self._joint_edit_mode = not self._joint_edit_mode
        if self._joint_edit_mode:
            self.btn_joint_edit.setText("Live")
            self.btn_joint_edit.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold; padding: 6px;")
            self.btn_joint_go.setEnabled(True)
            for sb in self.joint_spinboxes:
                sb.setReadOnly(False)
                sb.setStyleSheet("font-family: monospace; background-color: #FFFDE7;")
            self.log("Joint edit mode ON (auto-update paused)")
        else:
            self.btn_joint_edit.setText("Edit")
            self.btn_joint_edit.setStyleSheet("background-color: #FF9800; color: white; font-weight: bold; padding: 6px;")
            self.btn_joint_go.setEnabled(False)
            for sb in self.joint_spinboxes:
                sb.setReadOnly(True)
                sb.setStyleSheet("font-family: monospace;")
            self.log("Joint live mode ON (auto-update resumed)")

    def on_joint_go_clicked(self):
        """Send robot to the joint angles entered in the spinboxes."""
        joint_positions = [sb.value() for sb in self.joint_spinboxes]
        self.log(f"GO (Joint) -> [{', '.join(f'{j:.4f}' for j in joint_positions)}]")
        cmd = MotionCommand()
        cmd.command_type = MotionCommand.MOVE_TO_JOINT
        cmd.joint_positions = joint_positions
        cmd.max_velocity = 1.0
        cmd.max_acceleration = 1.0
        self.motion_cmd_pub.publish(cmd)
        self.is_moving = True
        self.status_updated_signal.emit("Status: Moving (Joint)...")
        if self._joint_edit_mode:
            self._toggle_joint_edit_mode()

    def on_joints_updated(self, joints):
        self._apply_joint_values_to_ui(joints, force=False)

    # =========================================================================
    # Dexterous Space Traversal
    # =========================================================================
    #
    # Sequence per position waypoint:
    #   1. 'move'    — move to (x, y, z) with base orientation
    #   2. 'roll'    — tilt roll by +30 deg (photo angle 1)
    #   3. 'pitch'   — tilt pitch by +30 deg from base (photo angle 2)
    #   4. 'yaw'     — tilt yaw by +30 deg from base (photo angle 3)
    #   5. 'restore' — return to base orientation
    #   then advance to next position waypoint and repeat

    _ANGLE_DELTA = 0.5236  # ~30 degrees in radians

    # Fixed downward pose: gripper/camera faces table (base_link -Z)
    # [0,1,0,0] = 180 deg around Y, flips +Z to -Z (camera down)
    _TRAVERSE_FIXED_DOWN_QUAT = [0.0, 1.0, 0.0, 0.0]

    def _start_traversal(self):
        """Generate XY grid waypoints in the dexterous workspace at fixed Z and start."""
        import math
        z = self.traverse_z_spinbox.value()
        target_points = int(self.traverse_points_spinbox.value())

        use_fixed_down = self.traverse_fixed_down_check.isChecked()
        if use_fixed_down:
            self._traverse_base_quat = list(self._TRAVERSE_FIXED_DOWN_QUAT)
            self.log("Traversal: using fixed downward pose (camera faces table)")
        else:
            pose_stamped, _ = self._get_current_pose_from_tf()
            if pose_stamped is None:
                self.log("Traversal: cannot get current pose from TF")
                return
            o = pose_stamped.pose.orientation
            self._traverse_base_quat = [o.x, o.y, o.z, o.w]

        # Use dexterous workspace bounds, or UR5e default if not available
        bounds = self._workspace_bounds
        if bounds is None:
            # UR5e approximate reachable workspace (base_link)
            bounds = (0.10, 0.85, -0.55, 0.55, 0.05, 0.90)
            self.log("Traversal: using default workspace (motion_control not publishing bounds)")
        x_min, x_max, y_min, y_max, z_min, z_max = bounds
        z_fixed = max(z_min, min(z_max, z))

        # Compute xy_step from target point count
        x_range = x_max - x_min
        y_range = y_max - y_min
        if x_range <= 0 or y_range <= 0:
            self.log("Traversal: invalid workspace")
            return
        area = x_range * y_range
        xy_step = math.sqrt(area / max(1, target_points))
        xy_step = max(0.01, min(0.2, xy_step))

        # XY grid traversal
        self._traverse_waypoints = []
        x_vals = []
        y_vals = []
        x = x_min
        while x <= x_max:
            x_vals.append(x)
            x += xy_step
        if x_max > x_min and (not x_vals or x_vals[-1] < x_max - 1e-6):
            x_vals.append(x_max)
        y = y_min
        while y <= y_max:
            y_vals.append(y)
            y += xy_step
        if y_max > y_min and (not y_vals or y_vals[-1] < y_max - 1e-6):
            y_vals.append(y_max)
        # Snake pattern to reduce empty travel
        for i, xi in enumerate(x_vals):
            yi_list = y_vals if i % 2 == 0 else list(reversed(y_vals))
            for yi in yi_list:
                self._traverse_waypoints.append((xi, yi, z_fixed))
        n_points = len(self._traverse_waypoints)
        if n_points == 0:
            self.log("Traversal: no valid grid points in workspace, adjust Z or target points")
            return

        self._traverse_index = 0
        self._traverse_phase = 'move'
        self._traversing = True
        self.btn_traverse_start.setEnabled(False)
        self.btn_traverse_stop.setEnabled(True)
        self.log(f"Traversal started: {n_points} XY points, z={z_fixed:.3f}m, step={xy_step:.3f}m (target={target_points})")
        if not self._motion_control_connected:
            self.log("Tip: if arm does not move, ensure 1) arm_gazebo is running 2) motion_control_node is running")
        self._execute_traverse_step()

    def _stop_traversal(self):
        """Stop the traversal sequence."""
        self._traversing = False
        self._traverse_waypoints = []
        self._traverse_index = 0
        self._traverse_phase = 'move'
        self.btn_traverse_start.setEnabled(True)
        self.btn_traverse_stop.setEnabled(False)
        self.traverse_status_label.setText("Stopped")
        self.traverse_status_label.setStyleSheet("font-family: monospace; color: #f44336;")
        self.log("Traversal stopped")
        cmd = MotionCommand()
        cmd.command_type = MotionCommand.STOP
        self.motion_cmd_pub.publish(cmd)

    def _execute_traverse_step(self):
        """Execute the current phase of the traversal sequence."""
        if not self._traversing or not self._traverse_waypoints:
            return
        if self._traverse_index >= len(self._traverse_waypoints):
            self._traverse_index = 0  # loop

        x, y, z = self._traverse_waypoints[self._traverse_index]
        phase = self._traverse_phase
        q_base = list(self._traverse_base_quat)
        n_total = len(self._traverse_waypoints)

        if phase == 'move':
            q_send = q_base
            label = f"Point {self._traverse_index + 1}/{n_total} — moving"
        elif phase == 'roll':
            q_delta = tf_trans.quaternion_from_euler(self._ANGLE_DELTA, 0, 0)
            q_send = tf_trans.quaternion_multiply(q_base, q_delta).tolist()
            label = f"Point {self._traverse_index + 1}/{n_total} — roll +30°"
        elif phase == 'pitch':
            q_delta = tf_trans.quaternion_from_euler(0, self._ANGLE_DELTA, 0)
            q_send = tf_trans.quaternion_multiply(q_base, q_delta).tolist()
            label = f"Point {self._traverse_index + 1}/{n_total} — pitch +30°"
        elif phase == 'yaw':
            q_delta = tf_trans.quaternion_from_euler(0, 0, self._ANGLE_DELTA)
            q_send = tf_trans.quaternion_multiply(q_base, q_delta).tolist()
            label = f"Point {self._traverse_index + 1}/{n_total} — yaw +30°"
        elif phase == 'restore':
            q_send = q_base
            label = f"Point {self._traverse_index + 1}/{n_total} — restore"
        else:
            return

        self.traverse_status_label.setText(label)
        self.traverse_status_label.setStyleSheet("font-family: monospace; color: #4CAF50;")

        move_time = self.traverse_time_spinbox.value()
        velocity_factor = max(0.1, min(1.0, 2.0 / move_time))

        target = PoseStamped()
        target.header.frame_id = 'base_link'
        target.header.stamp = rospy.Time.now()
        target.pose.position = Point(x, y, z)
        target.pose.orientation = Quaternion(q_send[0], q_send[1], q_send[2], q_send[3])

        cmd = MotionCommand()
        cmd.command_type = MotionCommand.MOVE_TO_POSE
        cmd.target_pose = target
        cmd.max_velocity = velocity_factor
        cmd.max_acceleration = 1.0
        self.motion_cmd_pub.publish(cmd)
        self.is_moving = True
        self.log(f"Traverse [{phase}] point {self._traverse_index + 1}/{n_total} "
                 f"pos=[{x:.3f},{y:.3f},{z:.3f}]")

    def _advance_traverse_phase(self):
        """Advance to the next phase/waypoint after a motion completes."""
        if not self._traversing:
            return
        phase_order = ['move', 'roll', 'pitch', 'yaw', 'restore']
        idx = phase_order.index(self._traverse_phase) if self._traverse_phase in phase_order else 0
        if idx < len(phase_order) - 1:
            self._traverse_phase = phase_order[idx + 1]
        else:
            self._traverse_phase = 'move'
            self._traverse_index += 1
        self._execute_traverse_step()

    def on_update_timer(self):
        pass

    def log(self, message):
        from datetime import datetime
        self.log_text.append(f"[{datetime.now().strftime('%H:%M:%S')}] {message}")

    def closeEvent(self, event):
        self.update_timer.stop()
        rospy.signal_shutdown("Teach Pendant UI closed")
        event.accept()


def main():
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    app.setFont(QFont("Sans", 9))
    window = UR5eTeachPendantUI()
    window.show()
    try:
        sys.exit(app.exec_())
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
