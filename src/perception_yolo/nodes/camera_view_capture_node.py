#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
获取 Gazebo 相机视角并保存图像
订阅: /camera/rgb/image_raw, /motion/result
当机械臂每次姿态或位置改变（motion 完成）时触发拍照，不再使用固定时间间隔
发布: /camera/capture_result (std_msgs/Bool) 供 motion_control 打印结果
"""

import os
import time
import atexit
import sys
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from common_msgs.msg import GraspResult

# #region agent log
def _debug_log(hypothesis_id, location, message, data=None, run_id="run1"):
    try:
        payload = {
            "id": f"log_{int(time.time()*1000)}",
            "timestamp": int(time.time() * 1000),
            "location": location,
            "message": message,
            "data": data or {},
            "runId": run_id,
            "hypothesisId": hypothesis_id,
        }
        log_path = "/home/liufazhan/robocup_ur5e/.cursor/debug.log"
        os.makedirs(os.path.dirname(log_path), exist_ok=True)
        with open(log_path, "a", encoding="utf-8") as f:
            f.write(str(payload).replace("'", "\"") + "\n")
    except Exception:
        pass
# #endregion agent log

# #region agent log
def _log_uncaught(exc_type, exc, tb):
    _debug_log("H1", "camera_view_capture_node.py:excepthook", "uncaught_exception", {
        "type": str(exc_type),
        "error": str(exc)
    })
    sys.__excepthook__(exc_type, exc, tb)

def _log_exit():
    _debug_log("H1", "camera_view_capture_node.py:atexit", "process_exit")

sys.excepthook = _log_uncaught
atexit.register(_log_exit)
# #endregion agent log


class CameraViewCapture:
    def __init__(self):
        # #region agent log
        _debug_log("H1", "camera_view_capture_node.py:__init__", "init_start")
        # #endregion agent log
        # #region agent log
        _debug_log("H1", "camera_view_capture_node.py:__init__", "env_vars", {
            "ROS_MASTER_URI": os.environ.get("ROS_MASTER_URI"),
            "ROS_IP": os.environ.get("ROS_IP")
        })
        # #endregion agent log
        try:
            rospy.init_node('camera_view_capture', anonymous=True)
        except BaseException as e:
            # #region agent log
            _debug_log("H1", "camera_view_capture_node.py:__init__", "init_node_failed", {"error": str(e)})
            # #endregion agent log
            raise
        # #region agent log
        _debug_log("H1", "camera_view_capture_node.py:__init__", "after_init_node")
        # #endregion agent log
        self.bridge = CvBridge()
        # #region agent log
        _debug_log("H2", "camera_view_capture_node.py:__init__", "bridge_created")
        # #endregion agent log
        self.image_topic = rospy.get_param('~image_topic', '/camera/rgb/image_raw')
        self.save_dir = rospy.get_param('~save_dir', '/workspace/saved_images')
        self._last_frame = None
        # #region agent log
        _debug_log("H2", "camera_view_capture_node.py:__init__", "params_loaded", {
            "image_topic": self.image_topic,
            "save_dir": self.save_dir
        })
        # #endregion agent log

        os.makedirs(self.save_dir, exist_ok=True)
        rospy.loginfo("订阅图像话题: %s", self.image_topic)
        rospy.loginfo("保存目录: %s", self.save_dir)
        rospy.loginfo("拍照模式: 运动完成触发 (无固定间隔)")
        # #region agent log
        _debug_log("H2", "camera_view_capture_node.py:config", "config_loaded", {
            "topic": self.image_topic,
            "save_dir": self.save_dir
        })
        # #endregion agent log

        self.capture_result_pub = rospy.Publisher('/camera/capture_result', Bool, queue_size=1)
        self.sub = rospy.Subscriber(self.image_topic, Image, self._image_callback, queue_size=1)
        self.motion_result_sub = rospy.Subscriber('/motion/result', GraspResult, self._motion_result_callback, queue_size=10)
        # #region agent log
        _debug_log("H3", "camera_view_capture_node.py:__init__", "subscriber_created", {"topic": self.image_topic})
        # #endregion agent log
        self._received_count = 0
        # #region agent log
        _debug_log("H3", "camera_view_capture_node.py:__init__", "init_complete")
        # #endregion agent log

        def _check_messages(_event):
            if self._received_count == 0:
                try:
                    topics = rospy.get_published_topics()
                    topic_names = [t[0] for t in topics]
                except Exception as e:
                    topic_names = []
                    _debug_log("H4", "camera_view_capture_node.py:check_messages", "topics_error", {"error": str(e)})
                _debug_log("H4", "camera_view_capture_node.py:check_messages", "no_messages", {
                    "topic": self.image_topic,
                    "published_topics_count": len(topic_names),
                    "published_topics_sample": topic_names[:10]
                })
        rospy.Timer(rospy.Duration(5.0), _check_messages, oneshot=True)

    def _image_callback(self, msg):
        """缓存最新帧，用于运动完成时保存"""
        try:
            self._received_count += 1
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self._last_frame = frame
        except CvBridgeError as e:
            rospy.logerr("CvBridge 转换失败: %s", str(e))
            return

    def _motion_result_callback(self, msg):
        """运动完成时触发拍照：仅当 SUCCESS 时保存（姿态/位置已改变）"""
        if msg.status != GraspResult.SUCCESS:
            return
        self._trigger_capture()

    def _trigger_capture(self):
        """保存当前帧并发布拍照结果"""
        success = False
        if self._last_frame is not None:
            try:
                now = rospy.Time.now().to_sec()
                filename = os.path.join(self.save_dir, f"camera_view_{int(now*1000)}.jpg")
                if cv2.imwrite(filename, self._last_frame):
                    success = True
                    rospy.loginfo("已保存图像: %s", filename)
            except Exception as e:
                rospy.logerr("保存图像失败: %s", str(e))
        self.capture_result_pub.publish(Bool(data=success))


if __name__ == '__main__':
    try:
        node = CameraViewCapture()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass




# docker exec -it perception_yolo bash -c "source /opt/ros/noetic/setup.bash && export ROS_MASTER_URI=http://127.0.0.1:11311 && source /workspace/devel/setup.bash && python3 /workspace/src/perception_yolo/nodes/camera_view_capture_node.py _save_dir:=/workspace/pics"