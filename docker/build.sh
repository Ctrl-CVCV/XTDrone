#!/usr/bin/env bash
set -e

IMAGE_NAME="xtdrone-noetic-px4:1.13.2-v0"

# 可选：本地直连 GitHub 不稳定时，用代理加速构建。
# 用法: HTTP_PROXY=http://127.0.0.1:7897 HTTPS_PROXY=http://127.0.0.1:7897 ./build.sh
# --network host 让构建容器可访问宿主机 127.0.0.1 上的代理。
# --ssh 转发 ~/.ssh/id_ed25519：PX4/XTDrone 主仓库与子模块走 SSH 克隆（防 GFW 中断）。
NO_PROXY_DEFAULT="localhost,127.0.0.1,archive.ubuntu.com,security.ubuntu.com,*.ubuntu.com"

docker build \
    --network host \
    --ssh default="${HOME}/.ssh/id_ed25519" \
    --build-arg HTTP_PROXY="${HTTP_PROXY:-}" \
    --build-arg HTTPS_PROXY="${HTTPS_PROXY:-}" \
    --build-arg NO_PROXY="${NO_PROXY:-$NO_PROXY_DEFAULT}" \
    --build-arg USER_UID="$(id -u)" \
    --build-arg USER_GID="$(id -g)" \
    --build-arg USERNAME=dev \
    --build-arg XTDRONE_REF=master \
    -t "${IMAGE_NAME}" \
    .
