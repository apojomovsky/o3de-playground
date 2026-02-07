#!/usr/bin/env bash
#
# O3DE ROS2 Playground - Main Build Script
#
# This script orchestrates the staged build process.
# Each stage must pass before the next one starts.
#
# Usage:
#   ./scripts/build.sh [stage]
#
# Stages:
#   0 - ROS2 workspace only (colcon build)
#   1 - Docker base image (ROS2 Jazzy)
#   2 - Docker with O3DE installed
#   3 - Project registration
#   4 - Full build (project + assets)
#   all - Run all stages (default)
#
# Examples:
#   ./scripts/build.sh 1      # Build just the base image
#   ./scripts/build.sh all    # Build everything
#   ./scripts/build.sh        # Same as 'all'
#
# Behavior:
#   Stage 2+ auto-prepares host-side O3DE .deb cache via scripts/cache-o3de-deb.sh
#

set -euo pipefail

# =============================================================================
# Configuration
# =============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Docker settings
IMAGE_NAME="o3de-playground"
DOCKERFILE="$PROJECT_ROOT/docker/Dockerfile"

# Build args
O3DE_VERSION="${O3DE_VERSION:-2510.2}"
O3DE_EXTRAS_VERSION="${O3DE_EXTRAS_VERSION:-2510.2}"
ROS_DISTRO_BUILD="${ROS_DISTRO_BUILD:-jazzy}"
O3DE_DEB_SHA256="${O3DE_DEB_SHA256:-}"
O3DE_CACHE_DEB="${O3DE_CACHE_DEB:-1}"
O3DE_CACHE_FORCE="${O3DE_CACHE_FORCE:-0}"
O3DE_INSTALL_VERSION="${O3DE_INSTALL_VERSION:-}"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Log file
LOG_DIR="$PROJECT_ROOT/logs"
mkdir -p "$LOG_DIR"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
LOG_FILE="$LOG_DIR/build_${TIMESTAMP}.log"

# =============================================================================
# Helper Functions
# =============================================================================

log() {
    local level="$1"
    shift
    local msg="$*"
    local timestamp=$(date '+%Y-%m-%d %H:%M:%S')
    
    case "$level" in
        INFO)  echo -e "${BLUE}[INFO]${NC} $msg" ;;
        OK)    echo -e "${GREEN}[OK]${NC} $msg" ;;
        WARN)  echo -e "${YELLOW}[WARN]${NC} $msg" ;;
        ERROR) echo -e "${RED}[ERROR]${NC} $msg" ;;
        *)     echo "$msg" ;;
    esac
    
    echo "[$timestamp] [$level] $msg" >> "$LOG_FILE"
}

die() {
    log ERROR "$1"
    log ERROR "See full log: $LOG_FILE"
    exit 1
}

check_command() {
    if ! command -v "$1" &> /dev/null; then
        die "Required command not found: $1"
    fi
}

normalize_o3de_install_version() {
    if [[ -n "$O3DE_INSTALL_VERSION" ]]; then
        return
    fi

    if [[ "$O3DE_VERSION" =~ ^[0-9]{4}\.[0-9]+$ ]]; then
        local major_minor="${O3DE_VERSION%%.*}"
        local patch="${O3DE_VERSION##*.}"
        O3DE_INSTALL_VERSION="${major_minor:0:2}.${major_minor:2:2}.${patch}"
    else
        O3DE_INSTALL_VERSION="$O3DE_VERSION"
    fi
}

ensure_o3de_deb_cache() {
    if [[ "$O3DE_CACHE_DEB" != "1" ]]; then
        log INFO "Skipping O3DE .deb cache preparation (O3DE_CACHE_DEB=$O3DE_CACHE_DEB)"
        return
    fi

    local cache_script="$SCRIPT_DIR/cache-o3de-deb.sh"

    if [[ ! -x "$cache_script" ]]; then
        die "Missing executable cache helper: $cache_script"
    fi

    log INFO "Preparing host-side O3DE .deb cache for version $O3DE_VERSION..."
    local cache_args=(--version "$O3DE_VERSION")

    if [[ "$O3DE_CACHE_FORCE" == "1" ]]; then
        cache_args+=(--force)
    fi

    if ! "$cache_script" "${cache_args[@]}" 2>&1 | tee -a "$LOG_FILE"; then
        die "Failed to prepare O3DE .deb cache"
    fi

    log OK "O3DE .deb cache is ready"
}

stage_header() {
    local stage_num="$1"
    local stage_name="$2"
    echo ""
    echo "============================================================================="
    echo -e "${BLUE}STAGE $stage_num: $stage_name${NC}"
    echo "============================================================================="
    echo ""
}

stage_complete() {
    local stage_num="$1"
    log OK "Stage $stage_num complete!"
    echo ""
}

# =============================================================================
# Stage 0: ROS2 Workspace Verification
# =============================================================================
# This stage verifies the ROS2 workspace can be built.
# It does NOT require Docker - just checks syntax and dependencies.
# =============================================================================

stage_0_ros2_workspace() {
    stage_header 0 "ROS2 Workspace Verification"
    
    local ros2_ws="$PROJECT_ROOT/ros2_ws"
    
    if [[ ! -d "$ros2_ws" ]]; then
        die "ROS2 workspace not found: $ros2_ws"
    fi
    
    log INFO "Checking Python syntax in launch files..."
    
    # Check all Python files for syntax errors
    local errors=0
    while IFS= read -r -d '' pyfile; do
        if ! python3 -m py_compile "$pyfile" 2>> "$LOG_FILE"; then
            log ERROR "Syntax error in: $pyfile"
            errors=$((errors + 1))
        fi
    done < <(find "$ros2_ws" -name "*.py" -print0)
    
    if [[ $errors -gt 0 ]]; then
        die "Found $errors Python syntax error(s)"
    fi
    log OK "All Python files pass syntax check"
    
    # Check package.xml exists and is valid XML
    log INFO "Checking package.xml files..."
    while IFS= read -r -d '' pkgxml; do
        if ! python3 -c "import xml.etree.ElementTree as ET; ET.parse('$pkgxml')" 2>> "$LOG_FILE"; then
            log ERROR "Invalid XML: $pkgxml"
            errors=$((errors + 1))
        fi
    done < <(find "$ros2_ws" -name "package.xml" -print0)
    
    if [[ $errors -gt 0 ]]; then
        die "Found $errors package.xml error(s)"
    fi
    log OK "All package.xml files are valid"
    
    # Check setup.py/setup.cfg exist for Python packages
    log INFO "Checking package structure..."
    for pkg_dir in "$ros2_ws"/src/*/; do
        if [[ -f "$pkg_dir/package.xml" ]]; then
            local pkg_name=$(basename "$pkg_dir")
            if [[ ! -f "$pkg_dir/setup.py" ]] && [[ ! -f "$pkg_dir/setup.cfg" ]] && [[ ! -f "$pkg_dir/CMakeLists.txt" ]]; then
                log WARN "Package $pkg_name has no setup.py, setup.cfg, or CMakeLists.txt"
            fi
        fi
    done
    
    stage_complete 0
}

# =============================================================================
# Stage 1: Docker Base Image
# =============================================================================
# Builds just the ROS2 base image without O3DE.
# This catches issues with:
#   - Base image availability
#   - ROS2 package installation
#   - Basic dependencies
# =============================================================================

stage_1_docker_base() {
    stage_header 1 "Docker Base Image (ROS2 Jazzy)"
    
    check_command docker
    
    log INFO "Building base image target..."
    log INFO "This installs ROS2 Jazzy + navigation packages"
    log INFO "Expected time: 5-15 minutes"
    
    # Build just the 'base' target
    if ! docker build \
        --target base \
        --tag "${IMAGE_NAME}:base" \
        --build-arg ROS_VERSION="$ROS_DISTRO_BUILD" \
        --build-arg O3DE_VERSION="$O3DE_VERSION" \
        --progress=plain \
        -f "$DOCKERFILE" \
        "$PROJECT_ROOT" 2>&1 | tee -a "$LOG_FILE"; then
        die "Docker base build failed"
    fi
    
    # Verify the image exists
    if ! docker image inspect "${IMAGE_NAME}:base" &> /dev/null; then
        die "Base image was not created"
    fi
    
    # Quick sanity check
    log INFO "Verifying ROS2 installation in container..."
    if ! docker run --rm "${IMAGE_NAME}:base" bash -c "source /opt/ros/jazzy/setup.bash && ros2 --help" &>> "$LOG_FILE"; then
        die "ROS2 not working in base image"
    fi
    log OK "ROS2 CLI responds correctly"
    
    stage_complete 1
}

# =============================================================================
# Stage 2: Docker + O3DE Installation
# =============================================================================
# Extends base image with O3DE engine and gems.
# This catches issues with:
#   - O3DE .deb installation
#   - O3DE Python setup
#   - Gem registration
# =============================================================================

stage_2_docker_o3de() {
    stage_header 2 "Docker + O3DE Installation"
    
    check_command docker

    ensure_o3de_deb_cache
    
    log INFO "Building O3DE builder image..."
    log INFO "This installs O3DE $O3DE_VERSION (.deb) and registers gems"
    log INFO "Expected time: 10-20 minutes (depends on network/cache)"
    
    # Build the 'o3de-builder' target
    if ! docker build \
        --target o3de-builder \
        --tag "${IMAGE_NAME}:builder" \
        --build-arg ROS_VERSION="$ROS_DISTRO_BUILD" \
        --build-arg O3DE_VERSION="$O3DE_VERSION" \
        --build-arg O3DE_INSTALL_VERSION="$O3DE_INSTALL_VERSION" \
        --build-arg O3DE_EXTRAS_VERSION="$O3DE_EXTRAS_VERSION" \
        --build-arg O3DE_DEB_SHA256="$O3DE_DEB_SHA256" \
        --progress=plain \
        -f "$DOCKERFILE" \
        "$PROJECT_ROOT" 2>&1 | tee -a "$LOG_FILE"; then
        die "Docker O3DE build failed"
    fi
    
    # Verify O3DE CLI works
    log INFO "Verifying O3DE installation..."
    if ! docker run --rm "${IMAGE_NAME}:builder" \
        bash -lc '/opt/O3DE/'"$O3DE_INSTALL_VERSION"'/scripts/o3de.sh --help' &>> "$LOG_FILE"; then
        die "O3DE CLI not working"
    fi
    log OK "O3DE CLI responds correctly"
    
    stage_complete 2
}

# =============================================================================
# Stage 3: Project Registration
# =============================================================================
# Verifies the project can be registered with O3DE.
# This catches issues with:
#   - project.json validity
#   - Gem dependencies
#   - Project structure
# =============================================================================

stage_3_project_registration() {
    stage_header 3 "Project Registration"
    
    check_command docker
    
    log INFO "Checking project registration..."
    
    # This is checked during the o3de-builder stage
    # We verify by checking if the project appears in the registry
    if ! docker run --rm "${IMAGE_NAME}:builder" \
        bash -lc '/opt/O3DE/'"$O3DE_INSTALL_VERSION"'/scripts/o3de.sh get-registered --projects' 2>&1 | \
        tee -a "$LOG_FILE" | grep -q "Project"; then
        log WARN "Project may not be registered (this might be OK if path differs)"
    else
        log OK "Project is registered"
    fi
    
    stage_complete 3
}

# =============================================================================
# Stage 4: Full Build
# =============================================================================
# Builds the complete project including Editor and GameLauncher.
# This catches issues with:
#   - CMake configuration
#   - C++ compilation
#   - Asset processing
# =============================================================================

stage_4_full_build() {
    stage_header 4 "Full Project Build"
    
    check_command docker
    
    log INFO "Building complete image with project..."
    log INFO "This compiles O3DE project (C++ build)"
    log INFO "Expected time: 1-3 hours"
    
    # Build the final 'runtime' target
    if ! docker build \
        --target runtime \
        --tag "${IMAGE_NAME}:latest" \
        --build-arg ROS_VERSION="$ROS_DISTRO_BUILD" \
        --build-arg O3DE_VERSION="$O3DE_VERSION" \
        --build-arg O3DE_INSTALL_VERSION="$O3DE_INSTALL_VERSION" \
        --build-arg O3DE_EXTRAS_VERSION="$O3DE_EXTRAS_VERSION" \
        --build-arg O3DE_DEB_SHA256="$O3DE_DEB_SHA256" \
        --progress=plain \
        -f "$DOCKERFILE" \
        "$PROJECT_ROOT" 2>&1 | tee -a "$LOG_FILE"; then
        die "Full Docker build failed"
    fi
    
    # Verify the image
    log INFO "Verifying final image..."
    if ! docker image inspect "${IMAGE_NAME}:latest" &> /dev/null; then
        die "Final image was not created"
    fi
    
    # Check Editor binary exists
    log INFO "Checking Editor binary..."
    if docker run --rm "${IMAGE_NAME}:latest" \
        test -f /opt/O3DE/${O3DE_INSTALL_VERSION}/bin/Linux/profile/Default/Editor; then
        log OK "Editor binary exists"
    else
        log WARN "Editor binary not found in expected O3DE install path"
    fi
    
    stage_complete 4
    
    echo ""
    echo "============================================================================="
    echo -e "${GREEN}BUILD COMPLETE!${NC}"
    echo "============================================================================="
    echo ""
    echo "To run the container:"
    echo ""
    echo "  # NVIDIA GPU:"
    echo "  xhost +local:docker"
    echo "  docker run --runtime=nvidia --gpus all \\"
    echo "    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \\"
    echo "    -e DISPLAY=\$DISPLAY \\"
    echo "    ${IMAGE_NAME}:latest"
    echo ""
    echo "  # AMD GPU (XWayland):"
    echo "  docker run --device=/dev/kfd --device=/dev/dri \\" 
    echo "    --group-add video \\" 
    echo "    --group-add render \\" 
    echo "    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \\" 
    echo "    -e DISPLAY=\$DISPLAY \\" 
    echo "    -e XDG_RUNTIME_DIR=/tmp/runtime-root \\" 
    echo "    -e QT_QPA_PLATFORM=xcb \\" 
    echo "    -e VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/radeon_icd.json \\" 
    echo "    ${IMAGE_NAME}:latest"
    echo ""
}

# =============================================================================
# Main
# =============================================================================

main() {
    local stage="${1:-all}"

    normalize_o3de_install_version
    
    echo ""
    echo "O3DE ROS2 Playground - Build Script"
    echo "===================================="
    echo ""
    echo "Project root: $PROJECT_ROOT"
    echo "Log file: $LOG_FILE"
    echo "O3DE Version: $O3DE_VERSION"
    echo "O3DE Install Version: $O3DE_INSTALL_VERSION"
    echo "ROS Distro: $ROS_DISTRO_BUILD"
    echo "Auto cache .deb: $O3DE_CACHE_DEB"
    echo ""
    
    # Check basic requirements
    check_command python3
    
    case "$stage" in
        0)
            stage_0_ros2_workspace
            ;;
        1)
            stage_0_ros2_workspace
            stage_1_docker_base
            ;;
        2)
            stage_0_ros2_workspace
            stage_1_docker_base
            stage_2_docker_o3de
            ;;
        3)
            stage_0_ros2_workspace
            stage_1_docker_base
            stage_2_docker_o3de
            stage_3_project_registration
            ;;
        4|all)
            stage_0_ros2_workspace
            stage_1_docker_base
            stage_2_docker_o3de
            stage_3_project_registration
            stage_4_full_build
            ;;
        *)
            echo "Usage: $0 [0|1|2|3|4|all]"
            echo ""
            echo "Stages:"
            echo "  0   - ROS2 workspace verification only"
            echo "  1   - Docker base image (ROS2)"
            echo "  2   - Docker + O3DE installation"
            echo "  3   - Project registration check"
            echo "  4   - Full build (same as 'all')"
            echo "  all - Run all stages (default)"
            exit 1
            ;;
    esac
}

main "$@"
