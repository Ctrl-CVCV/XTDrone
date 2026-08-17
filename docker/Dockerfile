FROM osrf/ros:noetic-desktop-full-focal

SHELL ["/bin/bash", "-c"]

ARG DEBIAN_FRONTEND=noninteractive
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=1000
ARG XTDRONE_REF=master
# 可选代理：构建网络走本地代理时，git/apt/pip/wget 自动使用
ARG HTTP_PROXY
ARG HTTPS_PROXY
ARG NO_PROXY

ENV TZ=Asia/Shanghai
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# 1. 基础开发工具
RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo git git-lfs openssh-client wget curl ca-certificates gnupg lsb-release \
    software-properties-common build-essential cmake ninja-build ccache \
    pkg-config vim nano tmux gdb rsync unzip zip net-tools iproute2 \
    iputils-ping procps mesa-utils x11-apps \
    python3 python3-dev python3-pip python3-setuptools python3-wheel \
    python3-catkin-tools python3-rosdep python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# 2. Gazebo Classic 11 与 PX4 Gazebo 编译依赖
RUN apt-get update && apt-get install -y --no-install-recommends \
    gazebo11 libgazebo11-dev bc protobuf-compiler libeigen3-dev \
    libopencv-dev libxml2-utils libimage-exiftool-perl \
    gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav libgstreamer-plugins-base1.0-dev \
    && rm -rf /var/lib/apt/lists/*

# 3. ROS/MAVROS/Gazebo ROS/Navigation
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-gazebo-ros ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    ros-noetic-mavros ros-noetic-mavros-extras ros-noetic-mavros-msgs \
    ros-noetic-navigation ros-noetic-move-base ros-noetic-map-server \
    ros-noetic-amcl ros-noetic-global-planner \
    ros-noetic-dwa-local-planner \
    ros-noetic-pcl-ros ros-noetic-cv-bridge \
    ros-noetic-image-transport ros-noetic-tf ros-noetic-tf2-ros \
    ros-noetic-tf2-geometry-msgs ros-noetic-ddynamic-reconfigure \
    && rm -rf /var/lib/apt/lists/*

# 4. EGO/EGO-Swarm 常用依赖
RUN apt-get update && apt-get install -y --no-install-recommends \
    libarmadillo-dev libpcl-dev libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

# 5. 开发用户
RUN groupadd --gid ${USER_GID} ${USERNAME} && \
    useradd --uid ${USER_UID} --gid ${USER_GID} \
            --create-home --shell /bin/bash ${USERNAME} && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" \
        > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME}

# 5.5 SSH HTTP CONNECT 代理隧道脚本（SSH 克隆走本地代理，绕过 GFW 限速）
# 构建用 --network host，容器内 127.0.0.1:7897 即宿主机代理；用 python3 实现，免装 netcat。
RUN cat > /usr/local/bin/proxy_connect.py <<PYEOF
#!/usr/bin/env python3
import os, select, socket, sys
PROXY = ("127.0.0.1", 7897)
host, port = sys.argv[1], int(sys.argv[2])
s = socket.create_connection(PROXY, timeout=20)
s.sendall("CONNECT {}:{} HTTP/1.1\r\nHost: {}:{}\r\n\r\n".format(host, port, host, port).encode())
resp = b""
while b"\r\n\r\n" not in resp:
    c = s.recv(4096)
    if not c:
        break
    resp += c
if not resp.startswith(b"HTTP/1.1 200"):
    sys.stderr.write("CONNECT failed: %r\n" % resp[:200]); sys.exit(1)
stdin_fd = sys.stdin.fileno()
while True:
    watch = [s.fileno()]
    if stdin_fd >= 0:
        watch.append(stdin_fd)
    r, _, _ = select.select(watch, [], [])
    for fd in r:
        if fd == stdin_fd:
            data = os.read(stdin_fd, 65536)
            if not data:
                try:
                    s.shutdown(socket.SHUT_WR)
                except OSError:
                    pass
                stdin_fd = -1
            else:
                s.sendall(data)
        else:
            data = s.recv(65536)
            if not data:
                sys.exit(0)
            os.write(sys.stdout.fileno(), data)
PYEOF

RUN chmod +x /usr/local/bin/proxy_connect.py

# 6. PX4 v1.13.2
# 直连 GitHub 大流量会被 GFW 中断（GnuTLS recv error -110）。
# 主仓库与子模块走 SSH（--mount=type=ssh 复用宿主机密钥，构建时需加 --ssh）；
# LFS 对象走 HTTPS，传入 HTTP_PROXY/HTTPS_PROXY 时自动走代理；并做 3 次重试。
RUN --mount=type=ssh git lfs install --system && \
    git config --global url."git@github.com:".insteadOf "https://github.com/" && \
    git config --global core.sshCommand "ssh -o StrictHostKeyChecking=accept-new -o ProxyCommand='python3 /usr/local/bin/proxy_connect.py %h %p'" && \
    if [ -n "${HTTPS_PROXY}${HTTP_PROXY}" ]; then \
        git config --global http.proxy "${HTTP_PROXY:-$HTTPS_PROXY}"; \
        git config --global https.proxy "${HTTPS_PROXY:-$HTTP_PROXY}"; \
    fi && \
    for i in 1 2 3; do \
        rm -rf /home/${USERNAME}/PX4-Autopilot; \
        git clone --branch v1.13.2 --recursive \
            git@github.com:PX4/PX4-Autopilot.git \
            /home/${USERNAME}/PX4-Autopilot && break; \
        echo "PX4 clone attempt ${i} failed, retrying in 5s..."; \
        sleep 5; \
    done && \
    cd /home/${USERNAME}/PX4-Autopilot && \
    git lfs pull && \
    git submodule foreach --recursive 'git lfs pull' && \
    cd / && \
    chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}/PX4-Autopilot

# PX4 官方 setup 脚本支持 Ubuntu20.04 和 Docker。
# ROS Desktop Full 已提供 Gazebo，所以禁用脚本自己的 sim-tools。
# 脚本在 Docker 中假设 root，故以 root 执行；
# HOME 指向 dev，让 pip --user 安装到 /home/dev/.local。
RUN HOME=/home/${USERNAME} USER=${USERNAME} \
    bash /home/${USERNAME}/PX4-Autopilot/Tools/setup/ubuntu.sh \
        --no-sim-tools --no-nuttx && \
    chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}/.local

# 7. GeographicLib datasets（MAVROS）
RUN wget -q \
    https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh \
    -O /tmp/install_geographiclib_datasets.sh && \
    chmod +x /tmp/install_geographiclib_datasets.sh && \
    bash /tmp/install_geographiclib_datasets.sh && \
    rm /tmp/install_geographiclib_datasets.sh

# 8. XTDrone
# V0 允许 master；验证成功后必须改成固定 commit SHA。
RUN --mount=type=ssh git clone git@github.com:robin-shaun/XTDrone.git \
    /home/${USERNAME}/XTDrone && \
    cd /home/${USERNAME}/XTDrone && \
    git checkout ${XTDRONE_REF} && \
    chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}/XTDrone

# 9. 工作目录
RUN mkdir -p /workspace /home/${USERNAME}/ego_ws/src && \
    chown -R ${USERNAME}:${USERNAME} \
        /workspace /home/${USERNAME}/ego_ws

ENV PATH=/home/${USERNAME}/.local/bin:${PATH}

USER ${USERNAME}
WORKDIR /home/${USERNAME}

# 10. 预编译 PX4 Gazebo SITL，但不启动
RUN cd /home/${USERNAME}/PX4-Autopilot && \
    DONT_RUN=1 make px4_sitl_default gazebo

# 11. Bash 环境
RUN echo "source /opt/ros/noetic/setup.bash" \
        >> /home/${USERNAME}/.bashrc && \
    echo "export PATH=/home/${USERNAME}/.local/bin:\$PATH" \
        >> /home/${USERNAME}/.bashrc

USER root
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

USER ${USERNAME}
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
