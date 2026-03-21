## one_shot_planner 说明（一次性规划方案）

本模块实现 **一次性完整轨迹规划** 的封装，基于 `path_planning` 包中的 ACO + RRT* 规划器，将「从起点到目标」的路径转为 `JointTrajectory`，通过 `/motion/command` 交给 `motion_control` 执行。

**与 `ur5e_teach_pendant_ui` 一致**：不在此处直接调用 IK/FK，相信 `motion_control_node`；当前末端位姿由 **TF** 获取，当前关节由 **`/joint_states`** 获取，目标关节由 **配置/上层** 提供，仅通过 **`MotionCommand`** 与 motion_control 交互。

**碰撞检测**：RRT* 在路径上除检查连杆上 7 个关节原点与障碍物的碰撞外，还支持 **末端执行器在当前姿态下的包围盒** 与障碍物相交检测（见 `ee_half_extents` / `~end_effector_collision_box`），从而保证整条路径上末端按当前姿态不会撞上障碍物。

---

### 1. 相关文件

- **`nodes/one_shot_planner.py`**  
  一次性规划核心函数，供其他节点/业务逻辑调用；提供 `plan_one_shot`（目标为 xyz + goal_joints）、`plan_one_shot_from_goal_pose`（目标为 4×4 末端位姿，内部调 IK）、`plan_one_shot_grasped_object_to_goal`（夹住物体后使物体到达目标位姿，可选 ee_to_object 换算）。
- **`nodes/demo_one_shot_planning.py`**  
  演示节点：用 TF 取当前位姿、用 `/joint_states` 取当前关节、用参数取目标关节，规划后向 `/motion/command` 发布 `EXECUTE_TRAJECTORY`。

---

### 2. one_shot_planner 提供的核心接口

#### 2.1 `get_default_virtual_grasp_point()`

- 返回默认虚拟抓取点位置（base_link 系）`[x, y, z]`，与 `config/planning_config.yaml` 中的 `virtual_grasp_point` 一致（默认 `[0.35, 0.0, 0.2]`，桌面中心、桌面上表面高度），一般用于 ACO 的目标笛卡尔点。

#### 2.2 `plan_one_shot(...)`

**当前函数签名：**

```python
path_joints, trajectory = plan_one_shot(
    start_xyz,      # [x, y, z] 起点笛卡尔位置，由 TF 获取（同示教器）
    start_joints,   # 起点关节角 (6,)，由 /joint_states 获取
    goal_xyz,       # [x, y, z] 目标笛卡尔位置，用于 ACO，一般即 virtual_grasp_point
    goal_joints,    # 目标关节角 (6,)，由配置/上层提供，不在此做 IK
    obstacles=None,
    bounds=None,
    frame_id="base_link",
    aco_grid_res=0.08,
    aco_n_ants=40,
    aco_n_iters=30,
    aco_greedy_prob=0.3,
    aco_elite_deposit_ratio=2.0,
    rrt_step_size=0.12,
    rrt_max_iter=4000,
    return_vis_data=False,
    ee_half_extents=None,  # 末端 EE 系包围盒半长 (hx, hy, hz)；None 时从 ~end_effector_collision_box 读取
)
```

- **`start_xyz`**（必选）  
  - 起点笛卡尔位置 `[x, y, z]`（base_link）。  
  - 与示教器一致：由 **TF** `base_link` → `tcp_link`（如 `gripper_tip_link`）查当前末端位置得到。

- **`start_joints`**（必选）  
  - 起点关节角（6 维），与 `motion_control` 的关节顺序一致。  
  - 由 **`/joint_states`** 订阅得到。

- **`goal_xyz`**（必选）  
  - 目标笛卡尔位置 `[x, y, z]`，供 ACO 路径使用，一般用 `virtual_grasp_point` 或 `get_default_virtual_grasp_point()`。

- **`goal_joints`**（必选）  
  - 目标关节角（6 维）。  
  - **不在此做 IK**，由配置（如 `~goal_joints`）或上层逻辑提供，相信 motion_control 侧对目标姿态的约定。

- **`obstacles`**（可选）  
  - 格式：`[((cx, cy, cz), (hx, hy, hz)), ...]`。  
  - `None` 时使用 `GAZEBO_DEFAULT_OBSTACLES` 并加地面障碍。

- **`ee_half_extents`**（可选）  
  - 末端执行器在 **EE 系**下的包围盒半长 `(hx, hy, hz)`（米），用于路径上每一段的姿态/几何碰撞检测；RRT* 会检查该包围盒 8 个角点在当前末端位姿下是否与障碍物相交。  
  - `None` 时从 ROS 参数 `~end_effector_collision_box` 读取（如 `planning_config.yaml` 中的 `end_effector_collision_box: [0.02, 0.02, 0.05]`）；设为 `(0, 0, 0)` 可禁用末端盒检测，仅保留连杆 7 点碰撞。

- **返回值**  
  - `path_joints`：RRT* 关节路径，失败为 `None`。  
  - `trajectory`：`trajectory_msgs/JointTrajectory`，可封装为 `MotionCommand(EXECUTE_TRAJECTORY)` 发到 `/motion/command`。

**配置**：`config/planning_config.yaml` 中的 `end_effector_collision_box: [0.02, 0.02, 0.05]` 对应默认末端包围盒半长（约 4×4×10 cm）；可按实际夹爪/末端尺寸修改，或设为 `[0, 0, 0]` 关闭末端几何检测。

#### 2.2.1 `plan_one_shot_from_goal_pose(goal_pose_4x4, start_xyz, start_joints, ...)`

以 **目标 4×4 笛卡尔矩阵（含目标点 + 旋转矩阵）** 为输入的封装：内部将 4×4 转为位姿，调用 `/motion/compute_ik` 求得 `goal_joints`，再调用 `plan_one_shot`。适合上层只给目标末端位姿、不直接给关节角的场景。

**函数签名要点：**

```python
path_joints, trajectory = plan_one_shot_from_goal_pose(
    goal_pose_4x4,     # 4x4 齐次变换（numpy 或 list），base 系下目标末端位姿
    start_xyz,         # [x, y, z] 起点笛卡尔位置
    start_joints,      # 起点关节角 (6,)
    obstacles=None,
    bounds=None,
    frame_id="base_link",
    # ... 其余 ACO/RRT 参数同 plan_one_shot ...
    return_vis_data=False,
    ee_half_extents=None,
    seed_joints_for_ik=None,  # 可选，IK 种子关节角；None 时用 start_joints
    ik_timeout=2.0,           # 等待 /motion/compute_ik 的超时（秒）
)
```

- **`goal_pose_4x4`**（必选）：4×4 齐次变换矩阵，`T[0:3, 3]` 为目标位置，`T[0:3, 0:3]` 为旋转矩阵；内部会转为 `PoseStamped` 后调用 IK。
- **`seed_joints_for_ik`**（可选）：IK 求解的初值；缺省使用 `start_joints`。
- **返回值**：与 `plan_one_shot` 相同；若 IK 失败或矩阵非 4×4，返回 `(None, None)`（或 `return_vis_data=True` 时为 `(None, None, None)`）。

**依赖**：需 `motion_control` 提供 `/motion/compute_ik` 服务（请求为 `PoseStamped`，含位置与四元数姿态）。

**示例**：目标位置 `[0.35, 0, 0.2]`、姿态为单位旋转矩阵：

```python
import numpy as np
from one_shot_planner import plan_one_shot_from_goal_pose, build_motion_command_execute_trajectory

goal_4x4 = np.eye(4)
goal_4x4[0:3, 3] = [0.35, 0.0, 0.2]   # 目标点；需要任意姿态时修改 goal_4x4[:3, :3]

path_joints, traj = plan_one_shot_from_goal_pose(
    goal_pose_4x4=goal_4x4,
    start_xyz=[0.2, 0.0, 0.3],
    start_joints=[0, -1.57, 0, -1.57, 0, 0],
)
if path_joints and traj:
    cmd = build_motion_command_execute_trajectory(traj)
    pub.publish(cmd)
```

#### 2.2.2 `plan_one_shot_grasped_object_to_goal(object_goal_4x4, start_xyz, start_joints, ee_to_object_4x4=None, ...)`

夹住物品后规划路径，使夹持的物体到达指定位姿。**`ee_to_object_4x4` 为可选**：不传时直接将 `object_goal_4x4` 当作末端目标位姿，不做换算；传入时由「物体目标位姿」与「EE→物体变换」反推末端目标。**位姿/碰撞约束**：不传 `ee_to_object_4x4` 时仍会用默认物体大小（假定物体中心在 EE 原点）与末端盒合并做碰撞，默认大小为 `DEFAULT_GRASPED_OBJECT_HALF_EXTENTS = (0.02, 0.02, 0.03)`（米）；传了 `ee_to_object_4x4` 时用末端+物体变换合并。

**函数签名要点：**

```python
path_joints, trajectory = plan_one_shot_grasped_object_to_goal(
    object_goal_4x4,   # 4x4 齐次变换（base 系）；ee_to_object_4x4 为 None 时即末端目标，否则为物体目标位姿
    start_xyz,         # [x, y, z] 起点笛卡尔位置
    start_joints,      # 起点关节角 (6,)
    ee_to_object_4x4=None,  # 可选。None 时不换算；否则为末端系→物体系，用于反推末端目标并合并碰撞盒
    obstacles=None,
    # ... 其余 ACO/RRT 参数同 plan_one_shot_from_goal_pose ...
    ee_half_extents=None,
    grasped_object_half_extents=None,  # 物体在 EE 系下包围盒半长；不传时 ee_to_object_4x4 为 None 用 DEFAULT_GRASPED_OBJECT_HALF_EXTENTS
    seed_joints_for_ik=None,
    ik_timeout=2.0,
)
```

- **`object_goal_4x4`**（必选）：4×4 齐次变换。`ee_to_object_4x4=None` 时直接作为末端目标；否则为夹持物体要到达的目标位姿。
- **`ee_to_object_4x4`**（可选）：末端系→物体系 4×4；物体中心在 EE 系下为 `T[0:3, 3]`。为 `None` 时不换算，直接用 `object_goal_4x4` 作为末端目标。
- **`grasped_object_half_extents`**（可选）：夹持物体在 EE 系下的包围盒半长 `(ox, oy, oz)`。不传时：若 `ee_to_object_4x4` 为 `None` 则用默认 `DEFAULT_GRASPED_OBJECT_HALF_EXTENTS = (0.02, 0.02, 0.03)` 与末端盒合并（物体中心假定在 EE 原点）；若传了 `ee_to_object_4x4` 则仅在传了该参数时参与合并。
- **返回值**：与 `plan_one_shot` 相同。

**示例**（不换算，直接末端目标）：

```python
import numpy as np
from one_shot_planner import plan_one_shot_grasped_object_to_goal, build_motion_command_execute_trajectory

# 末端目标位姿（不传 ee_to_object_4x4 时直接使用）
ee_goal_4x4 = np.eye(4)
ee_goal_4x4[0:3, 3] = [0.4, 0.0, 0.15]

path_joints, traj = plan_one_shot_grasped_object_to_goal(
    object_goal_4x4=ee_goal_4x4,
    start_xyz=[0.2, 0.0, 0.3],
    start_joints=[0, -1.57, 0, -1.57, 0, 0],
)
```

**示例**（物体目标 + EE→物体变换，带物体碰撞盒）：

```python
object_goal_4x4 = np.eye(4)
object_goal_4x4[0:3, 3] = [0.4, 0.0, 0.15]
ee_to_object_4x4 = np.eye(4)
ee_to_object_4x4[0:3, 3] = [0.05, 0.0, 0.0]   # 物体中心相对 EE 的偏移
path_joints, traj = plan_one_shot_grasped_object_to_goal(
    object_goal_4x4=object_goal_4x4,
    start_xyz=[0.2, 0.0, 0.3],
    start_joints=[0, -1.57, 0, -1.57, 0, 0],
    ee_to_object_4x4=ee_to_object_4x4,
    grasped_object_half_extents=(0.02, 0.02, 0.03),
)
```

#### 2.3 `joints_to_trajectory(path_joints, joint_names, time_step=0.5)`

- 将关节路径转为 `JointTrajectory`，`time_step` 为相邻点时间间隔（秒）。

#### 2.4 `build_motion_command_execute_trajectory(trajectory)`

- 构造 `MotionCommand`，`command_type=EXECUTE_TRAJECTORY`，`max_velocity=1.0`，`max_acceleration=1.0`，与示教器发布方式一致，可直接发布到 `/motion/command`。

#### 2.5 `build_obstacles_from_yolo_instance_cloud(instance_cloud, target_center_xyz, ...)`

- 基于 YOLO 分割实例点云构造一次性规划可用的**环境障碍物**，同时返回目标物体的类别与几何信息，便于抓取阶段使用。典型输入为 `yolo26_seg_xyzl_instance_cloud_node.py` 发布的实例点云：
  - 点云字段至少包含：`x, y, z, label`（同一实例的点 label 一致），后续可扩展加入 `class_id` 字段。
  - `target_center_xyz` 为上层估计/选择的**目标物体中心点**（与点云同一坐标系，一般为 `base_link`）。

- **返回值为 dict**，主要字段：

  - `class_id`：目标物体的 YOLO 类别 id（若点云中存在 `class_id` 字段，否则为 `None`）。
  - `center`：目标物体中心点 `(cx, cy, cz)`，即传入的 `target_center_xyz`。
  - `half_extents`：目标物体的包围盒半轴 `(hx, hy, hz)`，通过 `items_list.json` 中的 `class_id` / `class_name` 查表；若查不到则使用 `DEFAULT_GRASPED_OBJECT_HALF_EXTENTS` 作为兜底。
  - `obstacles`：仅包含**环境障碍物**的列表 `[((cx, cy, cz), (hx, hy, hz)), ...]`，不含目标物体本身。可直接传给 `plan_one_shot` 的 `obstacles` 参数。
  - `env_cloud`：环境点云（`PointCloud2`，仅含 x,y,z），为去除目标实例点之后的剩余点云；可用于可视化或进一步处理。

- **匹配逻辑摘要**：

  1. 按 `label` 将实例点云聚类，统计每个实例到 `target_center_xyz` 的最近点距离；
  2. 距离最近的实例视为“目标物体”；若最近距离仍大于阈值（`center_match_radius`，默认 0.10 m），则视为匹配失败，函数返回 `None`；
  3. 从该实例的点中读取 `class_id`（若存在字段），并通过 `items_list.json` 查找对应的 `half_extents`；
  4. 将该实例的全部点从点云中剔除，其余点体素化为环境障碍物（调用现有的 `pointcloud_to_obstacles`）。

---

### 3. demo_one_shot_planning 使用说明

`nodes/demo_one_shot_planning.py` 为可运行 Demo，逻辑与示教器对齐：TF + `/joint_states` + 参数，不调用 IK/FK。

#### 3.1 前置条件

- `roscore` 已启动。  
- `motion_control_node` 已运行（订阅 `/motion/command`）。  
- TF 中存在 `base_link` → `gripper_tip_link`（或 `~tcp_link` 指定）。  
- 已配置目标关节 `~goal_joints` 或使用默认 `~goal_joints_default`。  
- 可选：Gazebo/RViz 与 `/joint_states`、点云话题，便于完整验证。

#### 3.2 启动 Demo

```bash
cd ~/robocup_ur5e
source devel/setup.bash
rosrun path_planning demo_one_shot_planning.py
```

带参数示例：

```bash
# 指定目标关节（必选其一：参数或默认）
rosrun path_planning demo_one_shot_planning.py _goal_joints:="[0, -2.0, 1.2, -1.57, -1.57, 0]"

# 指定虚拟抓取点（用于 ACO 的 goal_xyz）
rosrun path_planning demo_one_shot_planning.py _virtual_grasp_point:="[0.35, 0.0, 0.2]"

# 指定末端碰撞盒半长（EE 系 [hx, hy, hz] 米）；[0,0,0] 表示不检查末端几何
rosrun path_planning demo_one_shot_planning.py _end_effector_collision_box:="[0.02, 0.02, 0.05]"
```

#### 3.3 Demo 内部流程

1. **起点位姿**  
   - 与示教器一致：**TF** 查询 `base_link` → `tcp_link`，得到当前末端位置 `start_xyz`；失败则用 `~start_xyz`。

2. **起点关节**  
   - 订阅 **`/joint_states`** 取一次当前关节为 `start_joints`；超时则用 `~home_joints`。

3. **目标**  
   - `goal_xyz`：由 `~virtual_grasp_point` 或 `get_default_virtual_grasp_point()`。  
   - `goal_joints`：由 `~goal_joints` 解析；未设置则用 `~goal_joints_default`（不在此做 IK）。

4. **障碍物**  
   - 点云话题（如 `/camera/depth/points`）有数据则用 `pointcloud_to_obstacles`；否则用 `GAZEBO_DEFAULT_OBSTACLES`。

5. **规划与下发**  
   - 调用 `plan_one_shot(start_xyz, start_joints, goal_xyz, goal_joints, obstacles)`；末端碰撞盒由 `~end_effector_collision_box` 提供（默认启用）。  
   - 用 `build_motion_command_execute_trajectory(trajectory)` 构造 `MotionCommand`，发布到 **`/motion/command`**，由 motion_control 执行。

---

### 4. demo_one_shot_with_target_removal（基于 YOLO 分割点云挖去目标区域）

`nodes/demo_one_shot_with_target_removal.py` 是在一次性规划前**挖掉指定目标物体点云**的 Demo，支持两种来源：

- 方式 A：原始深度点云 + 语义分割点云 `/perception/yolo26_seg_cloud`（通过 `target_class_id` 按类别挖空，使用 `pointcloud_target_removal.remove_target_region_from_pointcloud`）。
- 方式 B：YOLO 实例分割点云 `/perception/yolo26_seg_instance_cloud`（由 `yolo26_seg_xyzl_instance_cloud_node.py` 发布），结合目标中心点 `~target_center`，通过 `build_obstacles_from_yolo_instance_cloud` 自动识别目标物体并构造环境障碍物。

#### 4.1 方式 B：基于实例点云的目标选择与环境障碍物构造

- 关键参数：
  - `~instance_cloud_topic`：YOLO 实例点云话题，默认不启用；例如 `/perception/yolo26_seg_instance_cloud`。
  - `~target_center`：目标物体中心点（字符串或列表形式），例如 `"[0.35, 0.0, 0.2]"`，在 `~frame_id`（默认 `base_link`）下。

- Demo 会：

  1. 从 `~instance_cloud_topic` 订阅一帧 `PointCloud2`，并通过 TF 变换到 `~frame_id`；
  2. 调用 `build_obstacles_from_yolo_instance_cloud(seg_instance_in_frame, target_center_xyz, ...)`，得到：
     - 目标物体的 `class_id`、`center`、`half_extents`；
     - 仅包含环境部分的 `obstacles`（去掉目标实例后的体素化结果）；
  3. 使用返回的 `obstacles` 直接作为 `plan_one_shot` 的障碍物列表；
  4. 若匹配或点云获取失败，则自动回退到方式 A（基于 `target_class_id` 的类别挖空）。

- 运行示例：

```bash
rosrun path_planning demo_one_shot_with_target_removal.py \
  _instance_cloud_topic:=/perception/yolo26_seg_instance_cloud \
  _target_center:="[0.35, 0.0, 0.2]" \
  _voxel_resolution:=0.05 \
  _frame_id:=base_link
```

此时：

- 目标物体**不再当作障碍**参与碰撞检测（因为它本来就是末端要到达/抓取的目标），只用作抓取参数和逻辑决策；
- 环境中其它物体通过点云体素化转为 AABB，作为 `plan_one_shot` 的障碍物输入。

#### 4.2 方式 A：兼容旧逻辑（基于 target_class_id 的点云挖空）

- 若未设置 `~instance_cloud_topic` 或 `~target_center`，或方式 B 失败：
  - Demo 会回退到原来的流程：订阅原始点云与语义分割点云，调用 `remove_target_region_from_pointcloud` 按 `target_class_id` 挖掉一类物体，再用 `pointcloud_to_obstacles` 将剩余点云体素化为障碍物。

---

### 5. 在其他节点中复用一次性规划

与示教器相同用法：不直接使用 IK/FK，只使用 TF、`/joint_states` 和配置，并通过 `MotionCommand` 与 motion_control 交互。

1. 用 **TF** 取当前末端位置 `start_xyz`（如 `base_link` → `gripper_tip_link`）。  
2. 从 **`/joint_states`** 取当前关节 `start_joints`。  
3. 从 **配置或上层** 取目标：若为关节角 + 笛卡尔点，则调用 `plan_one_shot(...)`；若为 **4×4 末端目标位姿**，则调用 `plan_one_shot_from_goal_pose(goal_pose_4x4, ...)`；若为 **夹住物体后使物体到达目标**，则调用 `plan_one_shot_grasped_object_to_goal(object_goal_4x4, start_xyz, start_joints, ee_to_object_4x4=None, ...)`（不传 `ee_to_object_4x4` 时直接按末端目标规划）。  
4. 将返回的轨迹用 `build_motion_command_execute_trajectory` 发布到 `/motion/command`。

示例（伪代码）：

```python
import tf2_ros
from one_shot_planner import (
    plan_one_shot,
    get_default_virtual_grasp_point,
    build_motion_command_execute_trajectory,
)
from common_msgs.msg import MotionCommand

# 与示教器一致：TF 取当前位姿
tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)
trans = tf_buffer.lookup_transform("base_link", "gripper_tip_link", rospy.Time(0), rospy.Duration(0.5))
start_xyz = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]

# /joint_states 取当前关节（实际代码中可订阅一次或从回调得到）
start_joints = [...]  # 6 维

# 目标：配置或上层提供，不在此做 IK
goal_xyz = get_default_virtual_grasp_point()  # 或自定义 [x, y, z]
goal_joints = rospy.get_param("~goal_joints", [0, -2.0, 1.2, -1.57, -1.57, 0])

path_joints, traj = plan_one_shot(
    start_xyz=start_xyz,
    start_joints=start_joints,
    goal_xyz=goal_xyz,
    goal_joints=goal_joints,
    obstacles=None,
    ee_half_extents=None,  # None 时从 ~end_effector_collision_box 读取；或 (0,0,0) 禁用末端盒检测
)

if path_joints and traj:
    cmd = build_motion_command_execute_trajectory(traj)
    pub = rospy.Publisher("/motion/command", MotionCommand, queue_size=10)
    pub.publish(cmd)
```

这样在 RoboCup 行为树或任务规划节点中即可复用一次性规划，与示教器、motion_control 的用法保持一致。
