# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT
#

ARG ROS_VERSION=jazzy
ARG UBUNTU_VERSION=noble
ARG USERNAME=o3de
ARG USER_ID=1000
ARG GROUP_ID=1000

FROM ros:${ROS_VERSION}-ros-base-${UBUNTU_VERSION} AS base

ARG USERNAME
ARG USER_ID
ARG GROUP_ID
ENV WORKSPACE=/home/${USERNAME}/workspace
ENV LANG=en_US.UTF-8

WORKDIR $WORKSPACE

# Setup time zone and locale data (necessary for SSL and HTTPS packages)
RUN apt-get update \
    && DEBIAN_FRONTEND="noninteractive" apt-get -y install tzdata locales keyboard-configuration curl \
    && sed -i -e 's/# en_US.UTF-8 UTF-8/en_US.UTF-8 UTF-8/' /etc/locale.gen \
    && dpkg-reconfigure --frontend=noninteractive locales \
    && update-locale LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# Install O3DE build dependencies, rosdep, and sudo
RUN apt-get update \
    && apt-get install -y \
            binutils \
            clang \
            cmake \
            git \
            git-lfs \
            libglu1-mesa-dev \
            libevdev2 \
            libevdev-dev \
            ffmpeg \
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
            python3-rosdep \
            software-properties-common \
            sudo \
            vulkan-tools \
            ros-${ROS_DISTRO}-cyclonedds \
            ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/* \
    && pip install --break-system-packages python-statemachine

# Create user with passwordless sudo
# Check if UID exists, if so modify it; otherwise create new user
RUN groupadd -f -g ${GROUP_ID} ${USERNAME} 2>/dev/null || true \
    && (id -u ${USER_ID} >/dev/null 2>&1 && usermod -l ${USERNAME} -g ${GROUP_ID} -d /home/${USERNAME} $(id -nu ${USER_ID}) 2>/dev/null) || useradd -u ${USER_ID} -g ${GROUP_ID} -d /home/${USERNAME} -m -s /bin/bash ${USERNAME} 2>/dev/null || true \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME} \
    && mkdir -p $WORKSPACE \
    && chown -R ${USER_ID}:${GROUP_ID} /home/${USERNAME}

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# ============================================================================
# Stage 2: ROS2 Workspace Dependencies
# ============================================================================
FROM base AS ros-deps

ARG USERNAME
ARG USER_ID
ARG GROUP_ID

# Copy only ros2_ws to install dependencies
COPY --chown=${USER_ID}:${GROUP_ID} ros2_ws/ $WORKSPACE/ros2_ws/

USER ${USERNAME}

# Initialize rosdep and install all ROS2 workspace dependencies
# Note: rosdep init is likely already done in base image or needs sudo.
# We skip init if file exists.
RUN (ls /etc/ros/rosdep/sources.list.d/20-default.list >/dev/null 2>&1 || sudo rosdep init) \
    && rosdep update \
    && . /opt/ros/${ROS_DISTRO}/setup.sh \
    && sudo apt-get update \
    && rosdep install --from-paths $WORKSPACE/ros2_ws/src \
                      --ignore-src \
                      --rosdistro ${ROS_DISTRO} \
                      --skip-keys="gazebo_ros gazebo_ros_pkgs hls_lfcd_lds_driver" \
                      -y \
    && sudo rm -rf /var/lib/apt/lists/*

# ============================================================================
# Stage 3: O3DE Dependencies Cache
# ============================================================================
FROM ros-deps AS o3de-deps

ARG USERNAME
ARG USER_ID
ARG GROUP_ID

# O3DE Variables
ARG O3DE_VERSION=2510.2
ARG O3DE_INSTALL_VERSION=25.10.2
ARG O3DE_DEB_SHA256=
ENV O3DE_ROOT=/opt/O3DE/${O3DE_INSTALL_VERSION}

# O3DE Extras Variables
ARG O3DE_EXTRAS_VERSION=2510.2
ARG O3DE_EXTRAS_REPO=https://github.com/o3de/o3de-extras.git
ENV O3DE_EXTRAS_ROOT=$WORKSPACE/o3de-extras

# Copy optional host-side O3DE .deb cache
COPY --chown=${USER_ID}:${GROUP_ID} docker/cache/ /tmp/o3de-deb-cache/

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
    unset IFS; \
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
    sudo apt-get update; \
    sudo dpkg -i "$selected_deb" || sudo apt-get install -f -y; \
    sudo rm -f "$selected_deb" "$selected_sha"; \
    sudo rm -rf /var/lib/apt/lists/*; \
    sudo chown -R ${USER_ID}:${GROUP_ID} /opt/O3DE/${O3DE_INSTALL_VERSION}

# Clone O3DE extras
RUN git clone --single-branch -b ${O3DE_EXTRAS_VERSION} $O3DE_EXTRAS_REPO $O3DE_EXTRAS_ROOT \
    && git -C $O3DE_EXTRAS_ROOT lfs install \
    && git -C $O3DE_EXTRAS_ROOT lfs pull

# Register engine and gems
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && $O3DE_ROOT/python/get_python.sh \
    && $O3DE_ROOT/scripts/o3de.sh register --this-engine \
    && $O3DE_ROOT/scripts/o3de.sh register -gp $O3DE_EXTRAS_ROOT/Gems/LevelGeoreferencing \
    && $O3DE_ROOT/scripts/o3de.sh register -gp $O3DE_EXTRAS_ROOT/Gems/ROS2 \
    && $O3DE_ROOT/scripts/o3de.sh register -gp $O3DE_EXTRAS_ROOT/Gems/ROS2Sensors \
    && $O3DE_ROOT/scripts/o3de.sh register -gp $O3DE_EXTRAS_ROOT/Gems/ROS2Controllers \
    && $O3DE_ROOT/scripts/o3de.sh register -gp $O3DE_EXTRAS_ROOT/Gems/ROS2RobotImporter \
    && $O3DE_ROOT/scripts/o3de.sh register -gp $O3DE_EXTRAS_ROOT/Gems/ROS2SampleRobots \
    && $O3DE_ROOT/scripts/o3de.sh register -gp $O3DE_EXTRAS_ROOT/Gems/WarehouseAssets \
    && $O3DE_ROOT/scripts/o3de.sh register -tp $O3DE_EXTRAS_ROOT/Templates/Ros2ProjectTemplate

# Pre-download common 3rdParty packages by running cmake configure on engine SDK
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && mkdir -p /tmp/cache-warmup \
    && cd /tmp/cache-warmup \
    && cmake -B build -G "Ninja Multi-Config" \
             -S $O3DE_ROOT \
             -DLY_PROJECTS="" \
             -DLY_DISABLE_TEST_MODULES=ON 2>&1 | head -100 || true \
    && cd / \
    && rm -rf /tmp/cache-warmup

# ============================================================================
# Stage 4: ROS2 Workspace Build
# ============================================================================
FROM o3de-deps AS ros2-builder

ARG USERNAME
ARG USER_ID
ARG GROUP_ID

USER ${USERNAME}
ENV HOME=/home/${USERNAME}

COPY --chown=${USER_ID}:${GROUP_ID} ros2_ws/ $WORKSPACE/ros2_ws/

# Build colcon workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && cd $WORKSPACE/ros2_ws \
    && colcon build --symlink-install

# ============================================================================
# Stage 5: Final Runtime Image
# ============================================================================
FROM ros-deps AS runtime

ARG O3DE_VERSION=2510.2
ARG O3DE_INSTALL_VERSION=25.10.2
ARG USERNAME
ARG USER_ID
ARG GROUP_ID

# Copy built artifacts from builder stages
COPY --chown=${USER_ID}:${GROUP_ID} --from=o3de-deps /opt/O3DE/${O3DE_INSTALL_VERSION} /opt/O3DE/${O3DE_INSTALL_VERSION}
COPY --chown=${USER_ID}:${GROUP_ID} --from=o3de-deps $WORKSPACE/o3de-extras $WORKSPACE/o3de-extras
COPY --chown=${USER_ID}:${GROUP_ID} --from=ros2-builder $WORKSPACE/ros2_ws $WORKSPACE/ros2_ws
COPY --chown=${USER_ID}:${GROUP_ID} --from=o3de-deps /home/${USERNAME}/.o3de /home/${USERNAME}/.o3de

RUN mkdir -p $WORKSPACE/projects \
    && if [ -f /home/${USERNAME}/.o3de/o3de_manifest.json ]; then \
        sed -i 's|"default_projects_folder":.*|"default_projects_folder": "'$WORKSPACE'/projects",|' /home/${USERNAME}/.o3de/o3de_manifest.json; \
    fi

# Set environment variables
ENV O3DE_ROOT=/opt/O3DE/${O3DE_INSTALL_VERSION}
ENV O3DE_EXTRAS_ROOT=$WORKSPACE/o3de-extras
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV HOME=/home/${USERNAME}
ENV PATH="/opt/O3DE/${O3DE_INSTALL_VERSION}/bin/Linux/profile/Default:${PATH}"

# Setup ROS2 environment and add O3DE binaries to PATH
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" | sudo tee -a /etc/bash.bashrc \
    && echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" | sudo tee -a /etc/bash.bashrc \
    && echo "export PATH=\"/opt/O3DE/${O3DE_INSTALL_VERSION}/bin/Linux/profile/Default:\$PATH\"" | sudo tee -a /etc/bash.bashrc \
    && echo "alias sudo='sudo '" | sudo tee -a /etc/bash.bashrc \
    && printf "pcm.!default {\n    type null\n}\nctl.!default {\n    type null\n}\n" | sudo tee /etc/asound.conf \
    && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /home/${USERNAME}/.bashrc \
    && echo "export PATH=\"/opt/O3DE/${O3DE_INSTALL_VERSION}/bin/Linux/profile/Default:\$PATH\"" >> /home/${USERNAME}/.bashrc

# Create entrypoint script
RUN sudo sh -c "echo '#!/bin/bash' > /entrypoint.sh" \
    && sudo sh -c "echo 'set -e' >> /entrypoint.sh" \
    && sudo sh -c "echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /entrypoint.sh" \
    && sudo sh -c "echo 'if [ -d \"\$WORKSPACE/ros2_ws/install\" ]; then' >> /entrypoint.sh" \
    && sudo sh -c "echo '  source \$WORKSPACE/ros2_ws/install/setup.bash' >> /entrypoint.sh" \
    && sudo sh -c "echo 'fi' >> /entrypoint.sh" \
    && sudo sh -c "echo 'exec \"\$@\"' >> /entrypoint.sh" \
    && sudo chmod +x /entrypoint.sh

USER ${USERNAME}
WORKDIR /home/${USERNAME}

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]
