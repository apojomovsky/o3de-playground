#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
IMAGE_NAME="o3de-playground"

RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m'

usage() {
    cat <<EOF
Usage: $(basename "$0") [command] [options]

Commands:
  shell     Interactive bash shell (default)
  editor    Launch O3DE Editor
  game      Launch GameLauncher
  nav       Launch navigation stack

Options:
  --nvidia  Use NVIDIA GPU (default if nvidia-smi found)
  --amd     Use AMD GPU
  --tag     Image tag (default: latest)

Examples:
  ./scripts/run.sh shell --nvidia
  ./scripts/run.sh editor --amd
  ./scripts/run.sh nav
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

run_nvidia() {
    local cmd="$1"
    xhost +local:docker 2>/dev/null || true
    docker run -it --rm \
        --runtime=nvidia --gpus all \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -e DISPLAY="$DISPLAY" \
        -e NVIDIA_VISIBLE_DEVICES=all \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
        --name o3de-playground-run \
        "${IMAGE_NAME}:${TAG}" \
        $cmd
}

run_amd() {
    local cmd="$1"
    xhost +local:docker 2>/dev/null || true
    docker run -it --rm \
        --device=/dev/kfd --device=/dev/dri \
        --group-add video \
        --security-opt seccomp=unconfined \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -e DISPLAY="$DISPLAY" \
        -e QT_QPA_PLATFORM=xcb \
        --name o3de-playground-run \
        "${IMAGE_NAME}:${TAG}" \
        $cmd
}

TAG="latest"
GPU=""
COMMAND="shell"

while [[ $# -gt 0 ]]; do
    case "$1" in
        shell|editor|game|nav)
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
        --tag)
            TAG="$2"
            shift 2
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

case "$COMMAND" in
    shell)
        CMD="/bin/bash"
        ;;
    editor)
        CMD="/data/workspace/Project/build/linux/bin/profile/Editor"
        ;;
    game)
        CMD="/data/workspace/Project/build/linux/bin/profile/Playground.GameLauncher"
        ;;
    nav)
        CMD="bash -c 'source /opt/ros/jazzy/setup.bash && source /data/workspace/ros2_ws/install/setup.bash && ros2 launch playground_nav navigation.launch.py'"
        ;;
esac

echo -e "${GREEN}Running: $CMD${NC}"
echo "GPU mode: $GPU"

case "$GPU" in
    nvidia)
        run_nvidia "$CMD"
        ;;
    amd)
        run_amd "$CMD"
        ;;
    none)
        echo -e "${RED}No GPU detected. Running without GPU support (will likely fail for Editor).${NC}"
        docker run -it --rm "${IMAGE_NAME}:${TAG}" $CMD
        ;;
esac
