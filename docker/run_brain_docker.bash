#!/usr/bin/env bash
# =============================================================================
# 启动/进入 RoboCup Brain 容器（代码通过 compose 挂载）
# 默认：不重建镜像，仅确保容器运行并进入 shell
# 可选：--build 先重建镜像再启动
# 在宿主机仓库目录执行: ./docker/run_brain_docker.bash [--build] [--force-recreate]
# =============================================================================
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
SERVICE_NAME="robocup_brain"
CONTAINER_NAME="robocup_brain"
ENV_FILE="$REPO_ROOT/scripts/.env"

BUILD_IMAGE=0
FORCE_RECREATE=0
for arg in "$@"; do
  case "$arg" in
    --build)
      BUILD_IMAGE=1
      ;;
    --force-recreate)
      FORCE_RECREATE=1
      ;;
    -h|--help)
      echo "Usage: ./docker/run_brain_docker.bash [--build] [--force-recreate]"
      exit 0
      ;;
    *)
      echo "[run_brain] Unknown option: $arg" >&2
      echo "Usage: ./docker/run_brain_docker.bash [--build] [--force-recreate]" >&2
      exit 1
      ;;
  esac
done

# 兼容 docker-compose（旧）与 docker compose（新）
DOCKER_COMPOSE="docker compose"
command -v docker-compose &>/dev/null && DOCKER_COMPOSE="docker-compose"

compose_ps_q() {
  if [ "$DOCKER_COMPOSE" = "docker-compose" ]; then
    docker-compose ps -q "$SERVICE_NAME"
  else
    docker compose ps -q "$SERVICE_NAME"
  fi
}

resolve_container() {
  local cid
  cid="$(compose_ps_q | head -n 1)"
  if [ -n "$cid" ]; then
    echo "$cid"
    return 0
  fi

  docker ps -a -q -f "label=com.docker.compose.service=${SERVICE_NAME}" | head -n 1
}

# 加载仓库内的 ROS 网络配置，避免默认回落到 localhost:11311
if [ -f "$ENV_FILE" ]; then
  set -a
  # shellcheck disable=SC1090
  source "$ENV_FILE"
  set +a
fi

tcp_reachable() {
  local host="$1"
  local port="$2"
  timeout 2 bash -lc "echo >/dev/tcp/$host/$port" >/dev/null 2>&1
}

parse_ros_master_uri() {
  local uri="$1"
  local without_proto="${uri#http://}"
  without_proto="${without_proto#https://}"
  ROS_MASTER_HOST="${without_proto%%:*}"
  ROS_MASTER_PORT="${without_proto##*:}"
  if [ "$ROS_MASTER_HOST" = "$ROS_MASTER_PORT" ]; then
    ROS_MASTER_PORT="11311"
  fi
}

if [ -n "${ROS_MASTER_URI:-}" ]; then
  parse_ros_master_uri "$ROS_MASTER_URI"
  if ! tcp_reachable "$ROS_MASTER_HOST" "$ROS_MASTER_PORT"; then
    if tcp_reachable "127.0.0.1" "11311"; then
      echo "[run_brain] ROS master $ROS_MASTER_URI is unreachable, falling back to http://127.0.0.1:11311"
      export ROS_MASTER_URI="http://127.0.0.1:11311"
      export ROS_IP="127.0.0.1"
    else
      echo "[run_brain] Warning: ROS master $ROS_MASTER_URI is unreachable and no local master was detected on 127.0.0.1:11311" >&2
    fi
  fi
fi

cd "$REPO_ROOT"

is_running() {
  local cid
  cid="$(resolve_container)"
  [ -n "$cid" ] && docker inspect -f '{{.State.Running}}' "$cid" 2>/dev/null | grep -q true
}

compose_up() {
  local cmd=("$DOCKER_COMPOSE" up -d)
  [ "$BUILD_IMAGE" -eq 1 ] && cmd+=(--build)
  [ "$FORCE_RECREATE" -eq 1 ] && cmd+=(--force-recreate)
  cmd+=("$SERVICE_NAME")
  "${cmd[@]}"
}

cleanup_stale_containers() {
  docker rm -f "$CONTAINER_NAME" 2>/dev/null || true
  while IFS= read -r stale; do
    [ -n "$stale" ] && docker rm -f "$stale" >/dev/null 2>&1 || true
  done < <(docker ps -a --format '{{.Names}}' | grep -E "_${CONTAINER_NAME}$" || true)
}

if [ "$FORCE_RECREATE" -eq 1 ]; then
  cleanup_stale_containers
fi

if [ "$BUILD_IMAGE" -eq 1 ] || [ "$FORCE_RECREATE" -eq 1 ] || ! is_running; then
  if ! compose_up; then
    echo "[run_brain] compose up failed, cleaning stale containers and retrying..."
    cleanup_stale_containers
    compose_up
  fi
fi
sleep 1
TARGET_CONTAINER="$(resolve_container)"
if [ -z "$TARGET_CONTAINER" ]; then
  echo "[run_brain] Unable to resolve a container for service ${SERVICE_NAME}" >&2
  exit 1
fi

docker exec -it "$TARGET_CONTAINER" bash -c '
  source /opt/ros/noetic/setup.bash 2>/dev/null || true
  [ -f /workspace/devel/setup.bash ] && source /workspace/devel/setup.bash
  export ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311}
  export ROS_IP=${ROS_IP:-}
  cd /workspace 2>/dev/null || true
  exec bash
'
