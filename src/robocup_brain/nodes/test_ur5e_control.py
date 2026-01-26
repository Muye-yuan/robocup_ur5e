#!/usr/bin/env python3
"""
UR5e Joint Control Test Script
Simple test to control UR5e arm in Gazebo

Author: Suhang Xia
Usage: 
  docker-compose exec brain bash
  source /workspace/devel/setup.bash
  rosrun robocup_brain test_ur5e_control.py
"""

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

class UR5eController:
    """Simple UR5e controller for testing"""
    
    def __init__(self):
        rospy.init_node('ur5e_test_controller', anonymous=False)
        
        # UR5e joint names
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint', 
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        # Current joint states
        self.current_joints = None
        
        # Subscribe to joint states
        self.joint_sub = rospy.Subscriber(
            '/joint_states',
            JointState,
            self.joint_state_callback
        )
        
        # Action client for trajectory control
        rospy.loginfo("[Test] Connecting to /arm_controller/follow_joint_trajectory...")
        self.client = actionlib.SimpleActionClient(
            '/arm_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
        )
        
        rospy.loginfo("[Test] Waiting for action server...")
        if self.client.wait_for_server(timeout=rospy.Duration(10.0)):
            rospy.loginfo("[Test] Connected to UR5e controller!")
        else:
            rospy.logerr("[Test] Failed to connect to action server!")
            rospy.logerr("[Test] Make sure Gazebo UR5e is running!")
    
    def joint_state_callback(self, msg):
        """Callback to receive current joint positions"""
        self.current_joints = dict(zip(msg.name, msg.position))
    
    def move_to_joint_positions(self, positions, duration=3.0):
        """
        Move robot to specified joint positions
        
        Args:
            positions: List of 6 joint angles in radians
            duration: Time to complete the movement (seconds)
        """
        if len(positions) != 6:
            rospy.logerr("[Test] Need exactly 6 joint positions!")
            return False
        
        # Create trajectory goal
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = self.joint_names
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = rospy.Duration(duration)
        
        goal.trajectory.points.append(point)
        
        # Send goal
        rospy.loginfo(f"[Test] Sending goal: {positions}")
        self.client.send_goal(goal)
        
        # Wait for result
        rospy.loginfo("[Test] Waiting for movement to complete...")
        self.client.wait_for_result()
        
        result = self.client.get_result()
        if result:
            rospy.loginfo("[Test] Movement completed successfully!")
            return True
        else:
            rospy.logerr("[Test] Movement failed!")
            return False
    
    def print_current_joints(self):
        """Print current joint positions"""
        if self.current_joints:
            rospy.loginfo("=" * 60)
            rospy.loginfo("Current Joint Positions:")
            for joint_name in self.joint_names:
                if joint_name in self.current_joints:
                    pos = self.current_joints[joint_name]
                    rospy.loginfo(f"  {joint_name}: {pos:.3f} rad ({pos*180/3.14159:.1f} deg)")
            rospy.loginfo("=" * 60)
        else:
            rospy.logwarn("[Test] No joint states received yet")
    
    def run_test_sequence(self):
        """Run a test sequence of movements"""
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("UR5e Joint Control Test Starting")
        rospy.loginfo("=" * 60)
        
        # Wait for initial joint states
        rospy.loginfo("[Test] Waiting for joint states...")
        rospy.sleep(2.0)
        
        # Show current position
        self.print_current_joints()
        
        # Test Position 1: Home position (all zeros)
        rospy.loginfo("\n[Test] Moving to HOME position...")
        home_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.move_to_joint_positions(home_pos, duration=4.0)
        rospy.sleep(1.0)
        
        # Test Position 2: Simple bent arm
        rospy.loginfo("\n[Test] Moving to BENT position...")
        bent_pos = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]  # 90 degrees at several joints
        self.move_to_joint_positions(bent_pos, duration=4.0)
        rospy.sleep(1.0)
        
        # Test Position 3: Rotate base
        rospy.loginfo("\n[Test] Rotating BASE...")
        rotated_pos = [1.57, -1.57, 1.57, -1.57, -1.57, 0.0]  # 90 degree base rotation
        self.move_to_joint_positions(rotated_pos, duration=4.0)
        rospy.sleep(1.0)
        
        # Test Position 4: Return to home
        rospy.loginfo("\n[Test] Returning to HOME...")
        self.move_to_joint_positions(home_pos, duration=4.0)
        rospy.sleep(1.0)
        
        # Show final position
        self.print_current_joints()
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("Test sequence completed!")
        rospy.loginfo("=" * 60)


def main():
    try:
        controller = UR5eController()
        controller.run_test_sequence()
    except rospy.ROSInterruptException:
        rospy.loginfo("[Test] Interrupted by user")
    except Exception as e:
        rospy.logerr(f"[Test] Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()