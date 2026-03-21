#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Matplotlib 规划结果可视化（无 X11/RViz，保存 PNG）
适用于 headless 环境或 RViz 无法启动时

Usage:
  python3 plot_path_planning.py [--subscribe] [--output path.png]
  --subscribe: 订阅 ROS 话题实时更新（需 roscore + path_planning 节点运行）
  --output: 输出路径，默认 /workspace/viz_output/path_planning.png
"""

import argparse
import os
import sys

try:
    import matplotlib
    matplotlib.use('Agg')  # 无显示器
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False


def plot_planning_results(obstacles, aco_path_xyz, rrt_path_xyz, virtual_grasp, start_xyz,
                          output_path, fk_fn=None):
    """绘制障碍物、ACO 路径、RRT* 路径、虚拟抓取点
    aco_path_xyz, rrt_path_xyz: list of (x,y,z)
    start_xyz: (x,y,z) 或 None
    """
    if not HAS_MATPLOTLIB:
        print("需要 matplotlib: pip install matplotlib")
        return False

    fig = plt.figure(figsize=(12, 5))
    ax1 = fig.add_subplot(121)
    ax2 = fig.add_subplot(122)

    from matplotlib.patches import Rectangle
    for (cx, cy, cz), (hx, hy, hz) in obstacles:
        ax1.add_patch(Rectangle((cx - hx, cy - hy), 2*hx, 2*hy, facecolor='red', alpha=0.5))
        ax2.add_patch(Rectangle((cx - hx, cz - hz), 2*hx, 2*hz, facecolor='red', alpha=0.5))

    if aco_path_xyz and len(aco_path_xyz) >= 2:
        xs, ys, zs = zip(*aco_path_xyz)
        ax1.plot(xs, ys, 'g-', linewidth=2, label='ACO')
        ax2.plot(xs, zs, 'g-', linewidth=2, label='ACO')

    if rrt_path_xyz and len(rrt_path_xyz) >= 2:
        xs, ys, zs = zip(*rrt_path_xyz)
        ax1.plot(xs, ys, 'b-', linewidth=2, label='RRT*')
        ax2.plot(xs, zs, 'b-', linewidth=2, label='RRT*')

    if virtual_grasp and len(virtual_grasp) >= 3:
        gx, gy, gz = virtual_grasp[0], virtual_grasp[1], virtual_grasp[2]
        ax1.plot(gx, gy, 'yo', markersize=12, label='Grasp')
        ax2.plot(gx, gz, 'yo', markersize=12, label='Grasp')

    if start_xyz and len(start_xyz) >= 3:
        sx, sy, sz = start_xyz[0], start_xyz[1], start_xyz[2]
        ax1.plot(sx, sy, 'k^', markersize=10, label='Start')
        ax2.plot(sx, sz, 'k^', markersize=10, label='Start')

    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_title('XY 平面')
    ax1.legend(loc='upper right', fontsize=8)
    ax1.axis('equal')
    ax1.grid(True)

    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Z (m)')
    ax2.set_title('XZ 平面')
    ax2.legend(loc='upper right', fontsize=8)
    ax2.axis('equal')
    ax2.grid(True)

    os.makedirs(os.path.dirname(output_path) or '.', exist_ok=True)
    fig.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()
    print("[plot_path_planning] 已保存: %s" % output_path)
    return True


def main_subscribe(output_path):
    """订阅 ROS 话题，规划完成后自动绘图"""
    try:
        import rospy
        from visualization_msgs.msg import Marker, MarkerArray
    except ImportError:
        print("需要 ROS: source /opt/ros/noetic/setup.bash")
        return 1

    _dir = os.path.dirname(os.path.abspath(__file__))
    if _dir not in sys.path:
        sys.path.insert(0, _dir)
    from aco_rrtstar_planner_node import UR5eFK, GAZEBO_DEFAULT_OBSTACLES

    rospy.init_node('plot_path_planning', anonymous=True)
    obstacles = list(GAZEBO_DEFAULT_OBSTACLES)
    aco_path_xyz = []
    rrt_path_xyz = []
    virtual_grasp = [0.35, 0.0, 0.2]  # 桌面中心
    start_xyz = None

    def on_obstacles(msg):
        obstacles.clear()
        for m in msg.markers:
            if m.type == Marker.CUBE:
                obstacles.append(
                    ((m.pose.position.x, m.pose.position.y, m.pose.position.z),
                     (m.scale.x/2, m.scale.y/2, m.scale.z/2))
                )

    def on_aco_path(msg):
        if msg.points:
            aco_path_xyz[:] = [(p.x, p.y, p.z) for p in msg.points]

    def on_rrt_path(msg):
        if msg.points:
            rrt_path_xyz[:] = [(p.x, p.y, p.z) for p in msg.points]

    def on_grasp(msg):
        virtual_grasp[:] = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]

    def on_timer(event):
        if aco_path_xyz or rrt_path_xyz:
            sxyz = UR5eFK.fk([0, -1.5708, 0, -1.5708, 0, 0]) if start_xyz is None else start_xyz
            plot_planning_results(
                list(obstacles), list(aco_path_xyz), list(rrt_path_xyz),
                list(virtual_grasp), tuple(sxyz) if sxyz is not None else None, output_path
            )

    rospy.Subscriber('/path_planning/obstacles', MarkerArray, on_obstacles)
    rospy.Subscriber('/path_planning/aco_path', Marker, on_aco_path)
    rospy.Subscriber('/path_planning/rrt_path', Marker, on_rrt_path)
    rospy.Subscriber('/path_planning/virtual_grasp_point', Marker, on_grasp)
    rospy.Timer(rospy.Duration(5.0), on_timer)
    rospy.loginfo("[plot_path_planning] 订阅中，每 5 秒保存 PNG 到 %s", output_path)
    rospy.spin()
    return 0


def main_static(output_path):
    """使用内置示例数据绘图（无需 ROS）"""
    _dir = os.path.dirname(os.path.abspath(__file__))
    if _dir not in sys.path:
        sys.path.insert(0, _dir)
    from aco_rrtstar_planner_node import UR5eFK, GAZEBO_DEFAULT_OBSTACLES
    obstacles = list(GAZEBO_DEFAULT_OBSTACLES)
    home = [0, -1.5708, 0, -1.5708, 0, 0]
    goal = [-1.5708, -1.5708, -1.5708, -1.5708, 1.5708, 0]
    aco_path = [UR5eFK.fk(home), UR5eFK.fk(goal)]
    rrt_path = [UR5eFK.fk(home), UR5eFK.fk(goal)]
    grasp = [0.35, 0.0, 0.2]
    start_xyz = UR5eFK.fk(home)
    plot_planning_results(obstacles, aco_path, rrt_path, grasp, tuple(start_xyz), output_path)
    return 0


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--subscribe', '-s', action='store_true', help='订阅 ROS 话题')
    parser.add_argument('--output', '-o', default='/workspace/viz_output/path_planning.png')
    parser.add_argument('--demo', action='store_true', help='无 ROS 演示')
    args = parser.parse_args()
    if args.subscribe:
        sys.exit(main_subscribe(args.output))
    else:
        sys.exit(main_static(args.output))
