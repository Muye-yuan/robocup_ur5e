#!/usr/bin/env bash
# =============================================================================
# 仅启动 Path Planning 容器并进入 shell（代码通过 compose 挂载）
# 在仓库根目录执行: ./docker/run_path_planning_docker.bash
# =============================================================================
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
SERVICE_NAME="path_planning"
CONTAINER_NAME="path_planning"
DOCKER_COMPOSE="docker compose"
command -v docker-compose &>/dev/null && DOCKER_COMPOSE="docker-compose"

cd "$REPO_ROOT"
$DOCKER_COMPOSE up -d "$SERVICE_NAME"
sleep 1
docker exec -it "$CONTAINER_NAME" bash -c '
  source /opt/ros/noetic/setup.bash 2>/dev/null || true
  [ -f /workspace/devel/setup.bash ] && source /workspace/devel/setup.bash
  for extra_pkg in /workspace/src/ur_description /workspace/src/robotiq_description; do
    [ -d "$extra_pkg" ] && export ROS_PACKAGE_PATH="${extra_pkg}:${ROS_PACKAGE_PATH}"
  done
  export ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311}
  export ROS_IP=${ROS_IP:-}
  cd /workspace 2>/dev/null || true
  exec bash
'
