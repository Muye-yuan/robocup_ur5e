# Path Planning - ACO+RRT* Hybrid Planner

Sarvin & Chang Gao - Path planning and collision avoidance

## Features

- Subscribes to 3D point cloud (`/camera/depth/points`), converts all points into obstacles
- **Virtual grasp point**: Used as the path planning goal; works with point cloud—point cloud is entirely converted to obstacles, virtual grasp point serves as the planning target
- Two-stage planning: ACO coarse grid global channel + RRT* pheromone-guided sampling
- RViz visualization: obstacles, ACO path (green), RRT* path (blue), virtual grasp point (yellow sphere)
- Robot base pose matches Gazebo arm_gazebo (x=-0.10, y=0, z=0.615)

## Usage

### Method 1: docker exec (same format as robocup_brain/test_ur5e_control)

```bash
docker exec -it path_planning bash -c "source /workspace/scripts/setup_container_env.sh && python3 /workspace/src/path_planning/nodes/test_aco_rrtstar.py"
```

Or with rosrun:

```bash
docker exec -it path_planning bash -c "source /workspace/scripts/setup_container_env.sh && rosrun path_planning test_aco_rrtstar.py"
```

### Method 2: Launch planning node directly

```bash
docker exec -it path_planning bash -c "source /workspace/scripts/setup_container_env.sh && roslaunch path_planning aco_rrtstar_planning.launch"
```

### Prerequisites

1. **Start arm_gazebo or MoveIt demo** (provides roscore, joint_states, robot_description)
   ```bash
   # Terminal 1: arm_gazebo
   cd arm_gazebo/docker && sudo ./run.bash
   # Or: roslaunch ur5e_moveit_config demo.launch
   ```

2. **Start RoboCup containers** (includes path_planning)
   ```bash
   cd robocup_ur5e && ./scripts/start.sh  # Select 1
   ```

3. **Add in RViz**:
   - MarkerArray: Topic `/path_planning/obstacles` (obstacles, red)
   - Marker: Topic `/path_planning/aco_path` (ACO path, green)
   - Marker: Topic `/path_planning/rrt_path` (RRT* path, blue)
   - Marker: Topic `/path_planning/virtual_grasp_point` (virtual grasp point, yellow sphere)

## Motion Control Integration

Path planning prefers **motion_control**'s IK/FK services; falls back to local DH implementation when unavailable.

| Interface | Description |
|-----------|-------------|
| `/motion/compute_ik` | Inverse kinematics service (path_planning/srv/ComputeIK.srv) |
| `/motion/compute_fk` | Forward kinematics service (path_planning/srv/ComputeFK.srv) |
| `/motion/ee_pose` | (Optional) Current end-effector pose PoseStamped published by motion_control |

Parameter `use_motion_control_kinematics: true` (default) enables service calls. motion_control must implement the above services; otherwise path_planning uses local IK/FK.

## Topics

| Subscribe | Description |
|-----------|-------------|
| /camera/depth/points | 3D point cloud input |
| /joint_states | Joint state (optional) |
| /motion/ee_pose | (Optional) Current end-effector pose from motion_control |

| Publish | Description |
|---------|-------------|
| /path_planning/obstacles | Obstacles MarkerArray |
| /path_planning/aco_path | ACO path LineStrip |
| /path_planning/rrt_path | RRT* path LineStrip |
| /path_planning/virtual_grasp_point | Virtual grasp point Marker |
| /joint_states | Robot joint state (home pose) |

## Configuration (planning_config.yaml)

- `virtual_grasp_point`: Virtual grasp point [x, y, z], default `[0.35, 0.0, 0.2]` (table center, table top height)
- `use_virtual_grasp_only`: When true, planning runs without point cloud (uses Gazebo default obstacles)

## RViz Won't Start (Troubleshooting)

RViz is a GUI; under WSL2/Docker it requires correct X11 display configuration. If RViz fails to start or shows a white screen, try the following:

### 1. Set DISPLAY before launch (in the terminal running docker compose)

```bash
export DISPLAY=:0
```

### 2. Allow local X11 connections (if it still fails)

```bash
# Requires X server running on host (WSLg or VcXsrv)
xhost +local:
```

### 3. Launch RViz from path_planning container

```bash
docker exec -it path_planning bash -c "source /workspace/scripts/setup_container_env.sh && rosrun rviz rviz -d /workspace/src/path_planning/config/path_planning.rviz"
```

### 4. Or launch RViz from arm_gazebo container

```bash
docker exec -it armgazebo tmux a
# Ctrl+b c to create new window, then:
rosrun rviz rviz -d /path/to/path_planning.rviz  # Or manually add Marker topics
```

### 5. Possible DISPLAY values under WSL2

- Windows 11 + WSLg: usually `export DISPLAY=:0`
- WSL2 + VcXsrv: `export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0`

### 6. RViz has config but right panel shows nothing

- Confirm **path_planning node is running** (`test_aco_rrtstar.py` or `rosrun path_planning test_aco_rrtstar.py`)
- Check topic data: `rostopic echo /path_planning/virtual_grasp_point`
- If Fixed Frame errors, try changing to `base_link`

## Alternative to RViz: Matplotlib Save PNG (No X11)

When RViz cannot start or display, use Matplotlib to save planning results as PNG images.

### Method 1: Built-in save in planning node (recommended)

Set `save_matplotlib_viz: true` in `planning_config.yaml` (enabled by default). Automatically saves to `path_planning_viz/path_planning.png` after each planning run.

### Method 2: Subscribe to ROS topics for automatic plotting

```bash
# Terminal 1: Start path_planning node
docker exec -it path_planning bash -c "source /workspace/scripts/setup_container_env.sh && python3 /workspace/src/path_planning/nodes/test_aco_rrtstar.py"

# Terminal 2: Start plotting script (subscribes to topics, saves PNG every 5 seconds)
docker exec -it path_planning bash -c "source /workspace/scripts/setup_container_env.sh && python3 /workspace/src/path_planning/nodes/plot_path_planning.py -s -o /workspace/viz_output/path_planning.png"
```

### Method 3: Demo without ROS

```bash
docker exec -it path_planning python3 /workspace/src/path_planning/nodes/plot_path_planning.py -o /workspace/viz_output/demo.png
```

### View results

PNG files are saved in `robocup_ur5e/path_planning_viz/` (via volume mount):

```bash
ls path_planning_viz/
# Open in file manager or: xdg-open path_planning_viz/path_planning.png
```
