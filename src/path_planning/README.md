# Path Planning - ACO+RRT* 混合规划器

Sarvin & Chang Gao - 路径规划与碰撞避障

## 功能

- 订阅 3D 点云 (`/camera/depth/points`)，全部转化为障碍物
- **虚拟抓取点**：作为路径规划目标，与点云协同工作——点云全部转为障碍物，虚拟抓取点作为终点进行规划
- 两阶段规划：ACO 粗栅格全局通道 + RRT* 信息素引导采样
- RViz 可视化：障碍物、ACO 路径（绿色）、RRT* 路径（蓝色）、虚拟抓取点（黄色球体）
- 机械臂站位与 Gazebo arm_gazebo 相同 (x=-0.10, y=0, z=0.615)

## 使用方式

### 方式一：docker exec（与 robocup_brain/test_ur5e_control 格式一致）

```bash
docker exec -it path_planning bash -c "source /workspace/scripts/setup_container_env.sh && python3 /workspace/src/path_planning/nodes/test_aco_rrtstar.py"
```

或使用 rosrun：

```bash
docker exec -it path_planning bash -c "source /workspace/scripts/setup_container_env.sh && rosrun path_planning test_aco_rrtstar.py"
```

### 方式二：直接启动规划节点

```bash
docker exec -it path_planning bash -c "source /workspace/scripts/setup_container_env.sh && roslaunch path_planning aco_rrtstar_planning.launch"
```

### 启动前准备

1. **启动 arm_gazebo 或 MoveIt demo**（提供 roscore、joint_states、robot_description）
   ```bash
   # 终端1: arm_gazebo
   cd arm_gazebo/docker && sudo ./run.bash
   # 或: roslaunch ur5e_moveit_config demo.launch
   ```

2. **启动 RoboCup 容器**（含 path_planning）
   ```bash
   cd robocup_ur5e && ./scripts/start.sh  # 选 1
   ```

3. **RViz 中需添加**：
   - MarkerArray：Topic `/path_planning/obstacles`（障碍物，红色）
   - Marker：Topic `/path_planning/aco_path`（ACO 路径，绿色）
   - Marker：Topic `/path_planning/rrt_path`（RRT* 路径，蓝色）
   - Marker：Topic `/path_planning/virtual_grasp_point`（虚拟抓取点，黄色球体）

## 与 motion_control 集成

Path planning 优先使用 **motion_control** 的 IK/FK 服务，不可用时自动回退到本地 DH 实现。

| 接口 | 说明 |
|------|------|
| `/motion/compute_ik` | 逆运动学服务 (path_planning/srv/ComputeIK.srv) |
| `/motion/compute_fk` | 正运动学服务 (path_planning/srv/ComputeFK.srv) |
| `/motion/ee_pose` | （可选）motion_control 发布的当前末端位姿 PoseStamped |

参数 `use_motion_control_kinematics: true`（默认）启用服务调用。motion_control 需实现上述服务，否则 path_planning 使用本地 IK/FK。

## 话题

| 订阅 | 说明 |
|------|------|
| /camera/depth/points | 3D 点云输入 |
| /joint_states | 关节状态（可选） |
| /motion/ee_pose | （可选）motion_control 当前末端位姿 |

| 发布 | 说明 |
|------|------|
| /path_planning/obstacles | 障碍物 MarkerArray |
| /path_planning/aco_path | ACO 路径 LineStrip |
| /path_planning/rrt_path | RRT* 路径 LineStrip |
| /path_planning/virtual_grasp_point | 虚拟抓取点 Marker |
| /joint_states | 机械臂关节状态（home 位姿） |

## 配置（planning_config.yaml）

- `virtual_grasp_point`: 虚拟抓取点 [x, y, z]，默认 `[0.35, 0.0, 0.2]`（桌面中心、桌面上表面高度）
- `use_virtual_grasp_only`: 为 true 时，无点云也执行规划（使用 Gazebo 默认障碍物）

## RViz 无法启动（故障排除）

RViz 为 GUI，在 WSL2/Docker 下需正确配置 X11 显示。若 RViz 启动失败或白屏，依次尝试：

### 1. 启动前设置 DISPLAY（在运行 docker compose 的终端）

```bash
export DISPLAY=:0
```

### 2. 允许 X11 本地连接（若仍失败）

```bash
# 需要 host 上已运行 X server（WSLg 或 VcXsrv）
xhost +local:
```

### 3. 从 path_planning 容器启动 RViz

```bash
docker exec -it path_planning bash -c "source /workspace/scripts/setup_container_env.sh && rosrun rviz rviz -d /workspace/src/path_planning/config/path_planning.rviz"
```

### 4. 或从 arm_gazebo 容器启动 RViz

```bash
docker exec -it armgazebo tmux a
# Ctrl+b c 新建窗口，然后：
rosrun rviz rviz -d /path/to/path_planning.rviz  # 或手动添加 Marker 话题
```

### 5. WSL2 下 DISPLAY 可能的值

- Windows 11 + WSLg：通常 `export DISPLAY=:0`
- WSL2 + VcXsrv：`export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0`

### 6. RViz 有配置但右侧不显示内容

- 确认 **path_planning 节点在运行**（`test_aco_rrtstar.py` 或 `rosrun path_planning test_aco_rrtstar.py`）
- 验证话题是否有数据：`rostopic echo /path_planning/virtual_grasp_point`
- 若 Fixed Frame 报错，尝试改为 `base_link`

## 替代 RViz：Matplotlib 保存 PNG（无需 X11）

当 RViz 无法启动或无法显示时，可用 Matplotlib 将规划结果保存为 PNG 图片。

### 方式一：规划节点内置保存（推荐）

在 `planning_config.yaml` 中设置 `save_matplotlib_viz: true`（默认已开启）。每次规划完成后自动保存到 `path_planning_viz/path_planning.png`。

### 方式二：订阅 ROS 话题自动绘图

```bash
# 终端 1：启动 path_planning 节点
docker exec -it path_planning bash -c "source /workspace/scripts/setup_container_env.sh && python3 /workspace/src/path_planning/nodes/test_aco_rrtstar.py"

# 终端 2：启动绘图脚本（订阅话题，每 5 秒保存一次 PNG）
docker exec -it path_planning bash -c "source /workspace/scripts/setup_container_env.sh && python3 /workspace/src/path_planning/nodes/plot_path_planning.py -s -o /workspace/viz_output/path_planning.png"
```

### 方式二：无 ROS 演示

```bash
docker exec -it path_planning python3 /workspace/src/path_planning/nodes/plot_path_planning.py -o /workspace/viz_output/demo.png
```

### 查看结果

PNG 保存在 `robocup_ur5e/path_planning_viz/`（通过 volume 挂载）：

```bash
ls path_planning_viz/
# 在文件管理器中打开或: xdg-open path_planning_viz/path_planning.png
```
