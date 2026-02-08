# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT
#

ARG ROS_VERSION=jazzy
ARG UBUNTU_VERSION=noble

FROM ros:${ROS_VERSION}-ros-base-${UBUNTU_VERSION} AS base

ENV WORKSPACE=/data/workspace
ENV LANG=en_US.UTF-8

WORKDIR $WORKSPACE

# Setup time zone and locale data (necessary for SSL and HTTPS packages)
RUN apt-get update \
    && DEBIAN_FRONTEND="noninteractive" apt-get -y install tzdata locales keyboard-configuration curl \
    && sed -i -e 's/# en_US.UTF-8 UTF-8/en_US.UTF-8 UTF-8/' /etc/locale.gen \
    && dpkg-reconfigure --frontend=noninteractive locales \
    && update-locale LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# Install the required ubuntu packages
RUN apt-get update \
    && apt-get install -y \
            binutils \
            clang \
            cmake \
            git \
            git-lfs \
            libglu1-mesa-dev \
            libevdev2 \
            libxcb-xinerama0 \
            libfontconfig1-dev \
            libxcb-xkb-dev \
            libxkbcommon-x11-dev \
            libxkbcommon-dev \
            libvulkan1 \
            libxcb-xfixes0-dev \
            libxcb-xinput-dev \
            libxcb-xinput0 \
            libxcb-keysyms1-dev \
            mesa-vulkan-drivers \
            libpcre2-16-0 \
            ninja-build \
            python3-pip \
            software-properties-common \
            vulkan-tools \
            ros-${ROS_DISTRO}-ackermann-msgs \
            ros-${ROS_DISTRO}-control-toolbox \
            ros-${ROS_DISTRO}-gazebo-msgs \
            ros-${ROS_DISTRO}-navigation2 \
            ros-${ROS_DISTRO}-nav2-bringup \
            ros-${ROS_DISTRO}-rviz2 \
            ros-${ROS_DISTRO}-tf2-ros \
            ros-${ROS_DISTRO}-slam-toolbox \
            ros-${ROS_DISTRO}-pointcloud-to-laserscan \
            ros-${ROS_DISTRO}-cyclonedds \
            ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
            && rm -rf /var/lib/apt/lists/* \
            && pip install --break-system-packages python-statemachine

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# ============================================================================
# Stage 2: O3DE Build
# ============================================================================
FROM base AS o3de-builder

# O3DE Variables
ARG O3DE_VERSION=2510.2
ARG O3DE_INSTALL_VERSION=25.10.2
ARG O3DE_DEB_SHA256=
ENV O3DE_ROOT=/opt/O3DE/${O3DE_INSTALL_VERSION}

# O3DE Extras Variables
ARG O3DE_EXTRAS_VERSION=2510.2
ARG O3DE_EXTRAS_REPO=https://github.com/o3de/o3de-extras.git
ENV O3DE_EXTRAS_ROOT=$WORKSPACE/o3de-extras

# Project Variables
ENV PROJECT_ROOT=$WORKSPACE/Project

# Copy optional host-side O3DE .deb cache
COPY docker/cache/ /tmp/o3de-deb-cache/

# Install O3DE from local cache when present, otherwise download from o3debinaries.org
RUN set -eux; \
    IFS='.'; set -- $O3DE_VERSION; \
    if [ "$#" -eq 3 ]; then \
        deb_suffix="${1}${2}_${3}"; \
    elif [ "$#" -eq 2 ]; then \
        deb_suffix="${1}_${2}"; \
    else \
        deb_suffix="$(echo "$O3DE_VERSION" | tr '.' '_')"; \
    fi; \
    deb_file="o3de_${deb_suffix}.deb"; \
    local_deb="/tmp/o3de-deb-cache/${deb_file}"; \
    local_sha="${local_deb}.sha256"; \
    remote_url="https://o3debinaries.org/main/Latest/Linux/${deb_file}"; \
    remote_sha_url="${remote_url}.sha256"; \
    selected_deb="/tmp/${deb_file}"; \
    selected_sha="/tmp/${deb_file}.sha256"; \
    expected_sha=""; \
    if [ -f "$local_deb" ]; then \
        cp "$local_deb" "$selected_deb"; \
        if [ -f "$local_sha" ]; then \
            cp "$local_sha" "$selected_sha"; \
        else \
            curl -fL "$remote_sha_url" -o "$selected_sha"; \
        fi; \
    else \
        curl -fL "$remote_url" -o "$selected_deb"; \
        curl -fL "$remote_sha_url" -o "$selected_sha"; \
    fi; \
    expected_sha="$(awk '{print $1}' "$selected_sha")"; \
    echo "$expected_sha  $selected_deb" | sha256sum -c -; \
    if [ -n "$O3DE_DEB_SHA256" ]; then \
        echo "$O3DE_DEB_SHA256  $selected_deb" | sha256sum -c -; \
    fi; \
    apt-get update; \
    dpkg -i "$selected_deb" || apt-get install -f -y; \
    rm -f "$selected_deb" "$selected_sha"; \
    rm -rf /var/lib/apt/lists/*

# Clone O3DE extras and register gems (cache-friendly - no project files needed)
RUN git clone --single-branch -b ${O3DE_EXTRAS_VERSION} $O3DE_EXTRAS_REPO $O3DE_EXTRAS_ROOT \
    && git -C $O3DE_EXTRAS_ROOT lfs install \
    && git -C $O3DE_EXTRAS_ROOT lfs pull

# Register engine and gems (depends on install/clone layers only)
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && $O3DE_ROOT/python/get_python.sh \
    && $O3DE_ROOT/scripts/o3de.sh register --this-engine \
    && $O3DE_ROOT/scripts/o3de.sh register -gp $O3DE_EXTRAS_ROOT/Gems/LevelGeoreferencing \
    && $O3DE_ROOT/scripts/o3de.sh register -gp $O3DE_EXTRAS_ROOT/Gems/ROS2 \
    && $O3DE_ROOT/scripts/o3de.sh register -gp $O3DE_EXTRAS_ROOT/Gems/ROS2Sensors \
    && $O3DE_ROOT/scripts/o3de.sh register -gp $O3DE_EXTRAS_ROOT/Gems/ROS2Controllers

# Copy project files and register (changes here don't invalidate clone/register layers)
COPY Project/ $PROJECT_ROOT/
COPY ros2_ws/ $WORKSPACE/ros2_ws/

RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && $O3DE_ROOT/scripts/o3de.sh register -pp $PROJECT_ROOT

# Configure O3DE project
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && cmake -B $PROJECT_ROOT/build/linux -G "Ninja Multi-Config" \
                            -S $PROJECT_ROOT \
                            -DLY_DISABLE_TEST_MODULES=ON \
                            -DLY_STRIP_DEBUG_SYMBOLS=ON \
                            -DAZ_USE_PHYSX5=ON

# Build O3DE project (separate layer - cmake configure cached above)
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && cmake --build $PROJECT_ROOT/build/linux --config profile \
             --target Playground.GameLauncher Editor Playground.Assets

# Build colcon workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && cd $WORKSPACE/ros2_ws \
    && colcon build --symlink-install

# ============================================================================
# Stage 3: Final Runtime Image
# ============================================================================
FROM base AS runtime

ARG O3DE_VERSION=2510.2
ARG O3DE_INSTALL_VERSION=25.10.2

# Copy built artifacts from builder
COPY --from=o3de-builder /opt/O3DE/${O3DE_INSTALL_VERSION} /opt/O3DE/${O3DE_INSTALL_VERSION}
COPY --from=o3de-builder $WORKSPACE/o3de-extras $WORKSPACE/o3de-extras
COPY --from=o3de-builder $WORKSPACE/Project $WORKSPACE/Project
COPY --from=o3de-builder $WORKSPACE/ros2_ws $WORKSPACE/ros2_ws
COPY --from=o3de-builder /root/.o3de /root/.o3de

# Set environment variables
ENV O3DE_ROOT=/opt/O3DE/${O3DE_INSTALL_VERSION}
ENV O3DE_EXTRAS_ROOT=$WORKSPACE/o3de-extras
ENV PROJECT_ROOT=$WORKSPACE/Project
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Setup ROS2 environment
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc \
    && echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /etc/bash.bashrc \
    && printf "pcm.!default {\n    type null\n}\nctl.!default {\n    type null\n}\n" > /etc/asound.conf

# Create entrypoint script
RUN echo '#!/bin/bash' > /entrypoint.sh \
    && echo 'set -e' >> /entrypoint.sh \
    && echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /entrypoint.sh \
    && echo 'if [ -d "$WORKSPACE/ros2_ws/install" ]; then' >> /entrypoint.sh \
    && echo '  source $WORKSPACE/ros2_ws/install/setup.bash' >> /entrypoint.sh \
    && echo 'fi' >> /entrypoint.sh \
    && echo 'exec "$@"' >> /entrypoint.sh \
    && chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]
