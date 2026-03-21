#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
夹爪开合度监控节点
- 订阅 TF，计算两指尖距离（夹爪间距）
- 阈值 18mm：距离 >= 18mm 为夹持中（有物体），< 18mm 为未夹持/空闭合
- 发布 /gripper/fingertip_distance (Float32, 米)
- 发布 /gripper/grasp_status (String: "grasping" | "released")
- 每次夹取状态变化或定期打印夹取情况到终端
"""

import rospy
from std_msgs.msg import Float32, String
import tf2_ros

# 夹持判定阈值 (米)，空载闭合约 17mm，夹持物体通常 > 18mm
GRASP_THRESHOLD_M = 0.018  # 18mm


class GripperMonitorNode:
    def __init__(self):
        rospy.init_node('gripper_monitor', anonymous=False)

        self.threshold = rospy.get_param('~grasp_threshold_m', GRASP_THRESHOLD_M)
        self.left_tip = rospy.get_param('~left_finger_tip_link', 'robotiq_85_left_finger_tip_link')
        self.right_tip = rospy.get_param('~right_finger_tip_link', 'robotiq_85_right_finger_tip_link')
        self.print_interval = rospy.get_param('~print_interval_s', 1.0)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.distance_pub = rospy.Publisher('/gripper/fingertip_distance', Float32, queue_size=1)
        self.status_pub = rospy.Publisher('/gripper/grasp_status', String, queue_size=1)

        self._last_status = None
        self._last_print_time = 0.0

        rospy.loginfo("[GripperMonitor] 夹持阈值: %.1f mm, 指尖 links: %s <-> %s",
                      self.threshold * 1000, self.left_tip, self.right_tip)

        self.timer = rospy.Timer(rospy.Duration(0.1), self._timer_callback)

    def _get_fingertip_distance(self):
        """从 TF 获取两指尖距离（米），失败返回 None"""
        try:
            trans = self.tf_buffer.lookup_transform(
                self.left_tip, self.right_tip, rospy.Time(0), rospy.Duration(0.1)
            )
            t = trans.transform.translation
            dist = (t.x ** 2 + t.y ** 2 + t.z ** 2) ** 0.5
            return dist
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None

    def _timer_callback(self, event):
        dist = self._get_fingertip_distance()
        if dist is None:
            return

        status = "grasping" if dist >= self.threshold else "released"
        dist_mm = dist * 1000

        self.distance_pub.publish(Float32(data=dist))
        self.status_pub.publish(String(data=status))

        now = rospy.Time.now().to_sec()

        if self._last_status != status:
            self._print_grasp_situation(status, dist_mm, changed=True)
            self._last_status = status
            self._last_print_time = now
        elif now - self._last_print_time >= self.print_interval:
            self._print_grasp_situation(status, dist_mm, changed=False)
            self._last_print_time = now

    def _print_grasp_situation(self, status, dist_mm, changed=False):
        """打印夹取情况到终端"""
        thresh_mm = self.threshold * 1000
        if status == "grasping":
            msg_cn = "夹持中"
            detail = "指尖距离 %.2f mm (>= 阈值 %.1f mm)" % (dist_mm, thresh_mm)
        else:
            msg_cn = "未夹持/掉落"
            detail = "指尖距离 %.2f mm (< 阈值 %.1f mm)" % (dist_mm, thresh_mm)

        if changed:
            rospy.loginfo("[GripperMonitor] 夹持状态变化: %s | %s", msg_cn, detail)
        else:
            rospy.loginfo("[GripperMonitor] 夹持状态: %s | %s", msg_cn, detail)


def main():
    try:
        node = GripperMonitorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
