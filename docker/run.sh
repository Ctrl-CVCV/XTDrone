#!/usr/bin/env bash
set -e

# 镜像在系统 docker（default 上下文），不是 Docker Desktop；必须显式指定 socket。
export DOCKER_HOST="${DOCKER_HOST:-unix:///var/run/docker.sock}"

IMAGE_NAME="xtdrone-noetic-px4:1.13.2-v1.3"
CONTAINER_NAME="xtdrone-dev"

mkdir -p "$HOME/xtdrone_docker/workspace"

# 开发阶段简化 X11 权限；团队正式环境可改 Xauthority。
xhost +local: >/dev/null

docker run -it \
    --name "${CONTAINER_NAME}" \
    --network host \
    --gpus all \
    -e DISPLAY="${DISPLAY}" \
    -e QT_X11_NO_MITSHM=1 \
    -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,display \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$HOME/xtdrone_docker/workspace:/workspace" \
    --shm-size=2g \
    "${IMAGE_NAME}"
