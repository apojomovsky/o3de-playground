#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
IMAGE_NAME="o3de-playground"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

usage() {
    cat <<EOF
Usage: $(basename "$0") [command] [options]

Commands:
  shell     Interactive bash shell (attaches if running)
  editor    Launch O3DE Editor
  game      Launch GameLauncher
  nav       Launch navigation stack
  attach    Attach to running container (alias for shell)

Options:
  --nvidia  Use NVIDIA GPU (default if nvidia-smi found)
  --amd     Use AMD GPU
  --audio   Enable host audio passthrough (/dev/snd)
  --tag     Image tag (default: latest)
  --no-mounts  Disable host project bind mounts
  --process-assets  Force AssetProcessorBatch before game launch
  --skip-process-assets  Never run AssetProcessorBatch before game launch

Examples:
  ./scripts/run.sh shell --nvidia
  ./scripts/run.sh editor --amd
  ./scripts/run.sh nav
  ./scripts/run.sh game --amd --process-assets
EOF
}

detect_gpu() {
    if command -v nvidia-smi &> /dev/null && nvidia-smi &> /dev/null; then
        echo "nvidia"
    elif [[ -e /dev/kfd ]]; then
        echo "amd"
    else
        echo "none"
    fi
}

TAG="latest"
GPU=""
COMMAND="shell"
AUDIO_MODE="dummy"
O3DE_INSTALL_VERSION="${O3DE_INSTALL_VERSION:-25.10.2}"
USE_MOUNTS=1
PROCESS_ASSETS_MODE="auto"
MOUNT_OPTS=()
CONTAINER_USER="o3de"
CONTAINER_WORKSPACE="/home/${CONTAINER_USER}/workspace"

seed_host_cache_from_image() {
    if [[ "$USE_MOUNTS" != "1" ]] || [[ "$COMMAND" != "game" ]] || [[ "$PROCESS_ASSETS_MODE" != "auto" ]]; then
        return
    fi

    local host_cache_linux="$PROJECT_ROOT/Project/Cache/linux"
    local stamp_file="$host_cache_linux/.ap_autorun.stamp"

    mkdir -p "$host_cache_linux"

    if [[ -f "$stamp_file" ]]; then
        return
    fi

    echo "[INFO] First run: seeding host cache from image..."

    local container_id
    container_id=$(docker create "${IMAGE_NAME}:${TAG}" /bin/true)

    if docker cp "$container_id:${CONTAINER_WORKSPACE}/Project/Cache/linux/." "$host_cache_linux/" >/dev/null 2>&1; then
        touch "$stamp_file"
        echo "[INFO] Host cache seeded from image. Skipping initial AssetProcessorBatch."
    else
        echo "[WARN] Could not seed host cache from image. Falling back to AssetProcessorBatch."
    fi

    docker rm -f "$container_id" >/dev/null 2>&1 || true
}

build_mount_opts() {
    if [[ "$USE_MOUNTS" != "1" ]]; then
        return
    fi

    local host_project="$PROJECT_ROOT/Project"
    local host_ros2="$PROJECT_ROOT/ros2_ws"

    if [[ -f "$host_project/project.json" ]]; then
        MOUNT_OPTS+=(-v "$host_project/project.json:${CONTAINER_WORKSPACE}/Project/project.json:rw")
    fi
    if [[ -d "$host_project/Levels" ]]; then
        MOUNT_OPTS+=(-v "$host_project/Levels:${CONTAINER_WORKSPACE}/Project/Levels:rw")
    fi
    if [[ -d "$host_project/Registry" ]]; then
        MOUNT_OPTS+=(-v "$host_project/Registry:${CONTAINER_WORKSPACE}/Project/Registry:rw")
    fi
    if [[ -d "$host_project/Assets" ]]; then
        MOUNT_OPTS+=(-v "$host_project/Assets:${CONTAINER_WORKSPACE}/Project/Assets:rw")
    fi
    if [[ -d "$host_project/Scripts" ]]; then
        MOUNT_OPTS+=(-v "$host_project/Scripts:${CONTAINER_WORKSPACE}/Project/Scripts:rw")
    fi
    mkdir -p "$host_project/Cache"
    MOUNT_OPTS+=(-v "$host_project/Cache:${CONTAINER_WORKSPACE}/Project/Cache:rw")
    mkdir -p "$host_ros2/src"
    MOUNT_OPTS+=(-v "$host_ros2/src:${CONTAINER_WORKSPACE}/ros2_ws/src:rw")
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        shell|editor|game|nav|attach)
            COMMAND="$1"
            shift
            ;;
        --nvidia)
            GPU="nvidia"
            shift
            ;;
        --amd)
            GPU="amd"
            shift
            ;;
        --audio)
            AUDIO_MODE="host"
            shift
            ;;
        --tag)
            TAG="$2"
            shift 2
            ;;
        --no-mounts)
            USE_MOUNTS=0
            shift
            ;;
        --process-assets)
            PROCESS_ASSETS_MODE="always"
            shift
            ;;
        --skip-process-assets)
            PROCESS_ASSETS_MODE="never"
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            usage
            exit 1
            ;;
    esac
done

if [[ -z "$GPU" ]]; then
    GPU=$(detect_gpu)
    echo "Auto-detected GPU: $GPU"
fi

if ! docker image inspect "${IMAGE_NAME}:${TAG}" &> /dev/null; then
    echo -e "${RED}Image ${IMAGE_NAME}:${TAG} not found. Run ./scripts/build.sh first.${NC}"
    exit 1
fi

build_mount_opts
seed_host_cache_from_image

CONTAINER_NAME="o3de-playground-run"

ensure_container_running() {
    if docker container inspect "${CONTAINER_NAME}" >/dev/null 2>&1; then
        if [[ "$(docker inspect -f '{{.State.Running}}' "${CONTAINER_NAME}")" == "true" ]]; then
            return 0
        else
            echo -e "${YELLOW}Starting stopped container: ${CONTAINER_NAME}${NC}"
            docker start "${CONTAINER_NAME}" >/dev/null
            return 0
        fi
    fi
    
    echo -e "${GREEN}Creating new container: ${CONTAINER_NAME}${NC}"
    
    case "$GPU" in
        nvidia)
            start_nvidia_container
            ;;
        amd)
            start_amd_container
            ;;
        *)
            echo -e "${RED}GPU not specified${NC}"
            exit 1
            ;;
    esac
}

start_nvidia_container() {
    local audio_opts=()
    if [[ "$AUDIO_MODE" == "host" ]]; then
        if [[ -e /dev/snd ]]; then
            audio_opts+=(--device=/dev/snd:/dev/snd --group-add audio -e JACK_NO_START_SERVER=1)
        else
            audio_opts+=(-e SDL_AUDIODRIVER=dummy -e AUDIODEV=null -e JACK_NO_START_SERVER=1)
        fi
    else
        audio_opts+=(-e SDL_AUDIODRIVER=dummy -e AUDIODEV=null -e JACK_NO_START_SERVER=1)
    fi
    
    xhost +local:docker 2>/dev/null || true
    local xauth_opts=()
    if [[ -f "$HOME/.Xauthority" ]]; then
        xauth_opts+=(-v "$HOME/.Xauthority:/home/$(whoami)/.Xauthority:ro" -e XAUTHORITY="/home/$(whoami)/.Xauthority")
    fi
    
    docker run -d \
        --init \
        --runtime=nvidia --gpus all \
        "${MOUNT_OPTS[@]}" \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        "${xauth_opts[@]}" \
        -e DISPLAY="$DISPLAY" \
        -e NVIDIA_VISIBLE_DEVICES=all \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
        "${audio_opts[@]}" \
        --name "${CONTAINER_NAME}" \
        "${IMAGE_NAME}:${TAG}" \
        sleep infinity >/dev/null
}

start_amd_container() {
    local vk_icd="/usr/share/vulkan/icd.d/radeon_icd.json"
    local audio_opts=()
    if [[ "$AUDIO_MODE" == "host" ]]; then
        if [[ -e /dev/snd ]]; then
            audio_opts+=(--device=/dev/snd:/dev/snd --group-add audio -e JACK_NO_START_SERVER=1)
        else
            audio_opts+=(-e SDL_AUDIODRIVER=dummy -e AUDIODEV=null -e JACK_NO_START_SERVER=1)
        fi
    else
        audio_opts+=(-e SDL_AUDIODRIVER=dummy -e AUDIODEV=null -e JACK_NO_START_SERVER=1)
    fi
    
    # Dynamically detect host group IDs for GPU devices to ensure access
    # This fixes issues where host 'render' GID (e.g. 992) != container 'render' GID (e.g. 991)
    local group_opts=()
    
    if [[ -e /dev/dri/renderD128 ]]; then
        local render_gid=$(stat -c '%g' /dev/dri/renderD128)
        group_opts+=(--group-add "$render_gid")
    else
        group_opts+=(--group-add render)
    fi
    
    if [[ -e /dev/kfd ]]; then
        local kfd_gid=$(stat -c '%g' /dev/kfd)
        # Avoid duplicate if same as render
        if [[ "$kfd_gid" != "${render_gid:-}" ]]; then
            group_opts+=(--group-add "$kfd_gid")
        fi
    fi
    
    if [[ -e /dev/dri/card0 ]]; then
        local video_gid=$(stat -c '%g' /dev/dri/card0)
        group_opts+=(--group-add "$video_gid")
    else
        group_opts+=(--group-add video)
    fi

    xhost +local:docker 2>/dev/null || true
    local xauth_opts=()
    if [[ -f "$HOME/.Xauthority" ]]; then
        xauth_opts+=(-v "$HOME/.Xauthority:/home/$(whoami)/.Xauthority:ro" -e XAUTHORITY="/home/$(whoami)/.Xauthority")
    fi
    
    docker run -d \
        --init \
        --device=/dev/kfd --device=/dev/dri \
        "${MOUNT_OPTS[@]}" \
        "${group_opts[@]}" \
        --security-opt seccomp=unconfined \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        "${xauth_opts[@]}" \
        -e DISPLAY="$DISPLAY" \
        -e XDG_RUNTIME_DIR=/tmp/runtime-root \
        -e QT_QPA_PLATFORM=xcb \
        -e VK_ICD_FILENAMES="$vk_icd" \
        "${audio_opts[@]}" \
        --name "${CONTAINER_NAME}" \
        "${IMAGE_NAME}:${TAG}" \
        sleep infinity >/dev/null
}

case "$COMMAND" in
    attach|shell)
        ensure_container_running
        if [[ "$COMMAND" == "shell" ]]; then
            echo -e "${GREEN}Opening shell in container: ${CONTAINER_NAME}${NC}"
        else
            echo -e "${GREEN}Attaching to container: ${CONTAINER_NAME}${NC}"
        fi
        exec docker exec -it "${CONTAINER_NAME}" /bin/bash
        ;;
    editor)
        ensure_container_running
        CMD="source /opt/ros/jazzy/setup.bash; if [[ -f ${CONTAINER_WORKSPACE}/ros2_ws/install/setup.bash ]]; then source ${CONTAINER_WORKSPACE}/ros2_ws/install/setup.bash; fi; echo \"AMENT_PREFIX_PATH=\$AMENT_PREFIX_PATH\"; /opt/O3DE/${O3DE_INSTALL_VERSION}/bin/Linux/profile/Default/Editor --project-path ${CONTAINER_WORKSPACE}/Project"
        echo -e "${GREEN}Running: $CMD${NC}"
        exec docker exec -it "${CONTAINER_NAME}" bash -lc "$CMD"
        ;;
    game)
        ensure_container_running
        AP_BIN="/opt/O3DE/${O3DE_INSTALL_VERSION}/bin/Linux/profile/Default/AssetProcessorBatch"
        GAME_BIN="${CONTAINER_WORKSPACE}/Project/build/linux/bin/profile/Playground.GameLauncher"
        STAMP_PATH="${CONTAINER_WORKSPACE}/Project/Cache/linux/.ap_autorun.stamp"
        if [[ "$PROCESS_ASSETS_MODE" == "always" ]]; then
            CMD="$AP_BIN --project-path ${CONTAINER_WORKSPACE}/Project && $GAME_BIN --project-path ${CONTAINER_WORKSPACE}/Project"
        elif [[ "$PROCESS_ASSETS_MODE" == "never" ]]; then
            CMD="$GAME_BIN --project-path ${CONTAINER_WORKSPACE}/Project"
        else
            if [[ "$USE_MOUNTS" == "1" && -f "$PROJECT_ROOT/Project/Cache/linux/.ap_autorun.stamp" ]]; then
                CMD="echo '[INFO] Asset cache already prepared. Use --process-assets to force.'; $GAME_BIN --project-path ${CONTAINER_WORKSPACE}/Project"
            else
                CMD="mkdir -p ${CONTAINER_WORKSPACE}/Project/Cache/linux; echo '[INFO] Processing assets (auto mode)...'; $AP_BIN --project-path ${CONTAINER_WORKSPACE}/Project && touch $STAMP_PATH; $GAME_BIN --project-path ${CONTAINER_WORKSPACE}/Project"
            fi
        fi
        echo -e "${GREEN}Running: $CMD${NC}"
        exec docker exec -it "${CONTAINER_NAME}" bash -lc "$CMD"
        ;;
    nav)
        ensure_container_running
        CMD="source /opt/ros/jazzy/setup.bash && source ${CONTAINER_WORKSPACE}/ros2_ws/install/setup.bash && ros2 launch playground_nav navigation.launch.py"
        echo -e "${GREEN}Running: $CMD${NC}"
        exec docker exec -it "${CONTAINER_NAME}" bash -lc "$CMD"
        ;;
esac

case "$GPU" in
    nvidia)
        run_nvidia "$CMD"
        ;;
    amd)
        run_amd "$CMD"
        ;;
    none)
        echo -e "${RED}No GPU detected. Running without GPU support (will likely fail for Editor).${NC}"
        docker run -it --rm --init "${MOUNT_OPTS[@]}" "${IMAGE_NAME}:${TAG}" bash -lc "$CMD"
        ;;
esac
