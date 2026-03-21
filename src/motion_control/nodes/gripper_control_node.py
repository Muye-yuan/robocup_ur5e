#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
夹爪控制调试节点 - 使用 Gazebo 暴露的单关节 FollowJointTrajectory 接口
- 默认走 /debug_gripper/*，避免和 motion_control 内置夹爪接口冲突
- 提供 grasp / release 命令（Topic + Service）
- 对接 /gripper_controller/follow_joint_trajectory
- 发布 grasp_result (Bool)
"""

import rospy
import actionlib
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
    JointTrajectoryControllerState,
)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger, TriggerResponse
import actionlib_msgs.msg as action_msgs

# Robotiq 85 单关节控制量，当前仿真里 0=张开，0.72 为更柔和的默认闭合量
GRIPPER_CLOSED = 0.72
GRIPPER_OPEN = 0.0
DEFAULT_MOVE_TIME = 1.0


class GripperControlNode:
    def __init__(self):
        rospy.init_node('gripper_control', anonymous=False)

        self.action_name = rospy.get_param(
            '~gripper_action', '/gripper_controller/follow_joint_trajectory'
        )
        self.gripper_joint_name = rospy.get_param(
            '~gripper_joint_name',
            'robotiq_85_left_knuckle_joint',
        )
        self.move_time = float(rospy.get_param('~move_time', DEFAULT_MOVE_TIME))
        self.position_tolerance = float(rospy.get_param('~position_tolerance', 0.03))
        self.command_topic = rospy.get_param('~command_topic', '/debug_gripper/command')
        self.grasp_service_name = rospy.get_param('~grasp_service', '/debug_gripper/grasp')
        self.release_service_name = rospy.get_param('~release_service', '/debug_gripper/release')
        self.result_topic = rospy.get_param('~result_topic', '/debug_gripper/grasp_result')
        self.state_topic = rospy.get_param('~state_topic', '/gripper_controller/state')

        self.client = actionlib.SimpleActionClient(
            self.action_name, FollowJointTrajectoryAction
        )
        self.server_connected = False
        self.latest_state = None

        self.result_pub = rospy.Publisher(
            self.result_topic, Bool, queue_size=1
        )
        self.cmd_sub = rospy.Subscriber(
            self.command_topic, String, self._cmd_callback, queue_size=1
        )
        self.grasp_srv = rospy.Service(
            self.grasp_service_name, Trigger, self._grasp_service
        )
        self.release_srv = rospy.Service(
            self.release_service_name, Trigger, self._release_service
        )
        self.state_sub = rospy.Subscriber(
            self.state_topic,
            JointTrajectoryControllerState,
            self._state_callback,
            queue_size=1,
        )

        rospy.loginfo(
            "[GripperControl] Ready. action=%s joint=%s command=%s grasp_srv=%s release_srv=%s",
            self.action_name,
            self.gripper_joint_name,
            self.command_topic,
            self.grasp_service_name,
            self.release_service_name,
        )

    def _ensure_server(self, timeout_sec=0.2):
        if self.server_connected:
            return True

        if self.client.wait_for_server(rospy.Duration(timeout_sec)):
            self.server_connected = True
            rospy.loginfo("[GripperControl] Connected to action server: %s", self.action_name)
            return True

        rospy.logwarn_throttle(5.0, "[GripperControl] Action server unavailable: %s", self.action_name)
        return False

    def _state_callback(self, msg):
        self.latest_state = msg

    def _get_actual_position(self, timeout_sec=0.5):
        if self.latest_state is not None and self.latest_state.actual.positions:
            return float(self.latest_state.actual.positions[0])
        try:
            msg = rospy.wait_for_message(
                self.state_topic,
                JointTrajectoryControllerState,
                timeout=timeout_sec,
            )
            self.latest_state = msg
            if msg.actual.positions:
                return float(msg.actual.positions[0])
        except rospy.ROSException:
            return None
        return None

    @staticmethod
    def _wrapped_position_error(actual, target):
        direct_error = abs(actual - target)
        wrapped_error = abs(((actual - target + 3.141592653589793) % (2.0 * 3.141592653589793)) - 3.141592653589793)
        return min(direct_error, wrapped_error)

    def _reached_target(self, target):
        for _ in range(3):
            actual = self._get_actual_position(timeout_sec=0.4)
            if actual is not None and self._wrapped_position_error(actual, target) <= self.position_tolerance:
                return True
            rospy.sleep(0.1)
        return False

    def _cmd_callback(self, msg):
        cmd = msg.data.lower().strip()
        if cmd in ('grasp', 'close'):
            self._do_grasp()
        elif cmd in ('release', 'open'):
            self._do_release()

    def _grasp_service(self, req):
        success = self._do_grasp()
        return TriggerResponse(
            success=success,
            message="grasp_success" if success else "grasp_empty"
        )

    def _release_service(self, req):
        success = self._do_release()
        return TriggerResponse(
            success=success,
            message="release_success" if success else "release_failed"
        )

    def _do_grasp(self):
        """执行力控制夹取，返回 True 表示夹到物体，False 表示空夹"""
        if not self._ensure_server():
            self.result_pub.publish(Bool(data=False))
            return False

        goal = FollowJointTrajectoryGoal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = [self.gripper_joint_name]
        point = JointTrajectoryPoint()
        point.positions = [GRIPPER_CLOSED]
        point.time_from_start = rospy.Duration(max(0.1, self.move_time))
        goal.trajectory.points.append(point)

        self.client.send_goal(goal)
        if not self.client.wait_for_result(rospy.Duration(point.time_from_start.to_sec() + 2.0)):
            self.client.cancel_goal()
            self.result_pub.publish(Bool(data=False))
            rospy.logwarn("[GripperControl] Grasp timed out.")
            return False

        state = self.client.get_state()
        grasp_success = state == action_msgs.GoalStatus.SUCCEEDED
        if not grasp_success:
            grasp_success = self._reached_target(GRIPPER_CLOSED)
        self.result_pub.publish(Bool(data=grasp_success))

        rospy.loginfo(
            "[GripperControl] Grasp: %s (state=%s)",
            "SUCCESS" if grasp_success else "FAILED",
            state,
        )
        return grasp_success

    def _do_release(self):
        """张开夹爪"""
        if not self._ensure_server():
            return False

        goal = FollowJointTrajectoryGoal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = [self.gripper_joint_name]
        point = JointTrajectoryPoint()
        point.positions = [GRIPPER_OPEN]
        point.time_from_start = rospy.Duration(max(0.1, self.move_time))
        goal.trajectory.points.append(point)

        self.client.send_goal(goal)
        if not self.client.wait_for_result(rospy.Duration(point.time_from_start.to_sec() + 2.0)):
            self.client.cancel_goal()
            rospy.logwarn("[GripperControl] Release timed out.")
            return False
        state = self.client.get_state()
        release_success = state == action_msgs.GoalStatus.SUCCEEDED
        if not release_success:
            release_success = self._reached_target(GRIPPER_OPEN)
        if release_success:
            rospy.loginfo("[GripperControl] Release completed.")
        else:
            rospy.logwarn("[GripperControl] Release failed (state=%s).", state)
        return release_success


def main():
    try:
        node = GripperControlNode()
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
