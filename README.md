# O3DE ROS2 Playground

A Docker-based development environment for mobile robotics simulation using **O3DE 2510.2** and **ROS2 Jazzy**.

Perfect for experimenting with navigation, SLAM, sensor simulation, and robotics algorithms without manual dependency management.

## Features

- 🐳 **Fully Dockerized** - One command to build, zero manual setup
- 🤖 **ROS2 Jazzy** with Nav2 navigation stack and SLAM Toolbox
- 🎮 **O3DE 2510.2** with PhysX5 physics and Atom renderer
- 🔧 **Helper Scripts** - Simple commands for build, run, and development
- 🎨 **Hot Reload** - Edit levels and assets without rebuilding containers
- 🖥️ **GPU Support** - NVIDIA and AMD hardware acceleration

## Quick Start

### Prerequisites

**Hardware:**
- GPU: NVIDIA (recommended) or AMD
- RAM: 16GB minimum, 32GB recommended  
- Storage: 60GB+ free space

**Software:**
- Ubuntu 24.04 (Noble)
- Docker 24.0+
- For NVIDIA: [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)
- For AMD: ROCm drivers

### 1. Build the Environment

```bash
# Clone the repository
git clone <repository-url>
cd o3de-playground

# Build everything (first build takes ~30-45 minutes)
./scripts/build.sh
```

The build script automatically:
- Downloads and caches O3DE 2510.2 installer
- Installs ROS2 dependencies via rosdep
- Compiles the O3DE project
- Builds the ROS2 workspace
- Seeds host-side asset cache for fast iteration

### 2. Launch the Editor

```bash
# Auto-detects GPU (NVIDIA/AMD)
./scripts/run.sh editor

# Or specify GPU explicitly
./scripts/run.sh editor --nvidia
./scripts/run.sh editor --amd
```

The editor opens with the Project Manager where you can select and open projects.

### 3. Run Simulations

```bash
# Launch the game
./scripts/run.sh game --amd

# In another terminal: Launch ROS2 navigation
./scripts/run.sh nav --amd
```

## Usage Guide

### Helper Scripts

The `scripts/` directory contains three main tools:

#### `build.sh` - Build the Docker Environment

```bash
# Full build (all stages)
./scripts/build.sh

# Build specific stages
./scripts/build.sh 1    # Base ROS2 image only
./scripts/build.sh 2    # + O3DE installation
./scripts/build.sh 3    # + Project registration  
./scripts/build.sh 4    # + Full project build

# Seed host cache (for fast asset iteration)
./scripts/build.sh seed-cache
```

**Stages:**
- **Stage 0**: ROS2 workspace verification
- **Stage 1**: Docker base image (ROS2 Jazzy)
- **Stage 2**: O3DE installation and gem registration
- **Stage 3**: Project registration
- **Stage 4**: Full build (project + assets + ROS2 workspace)
- **Stage 5**: Seed host asset cache

#### `run.sh` - Launch Containers

```bash
# Interactive shell (starts container if needed, attaches if running)
./scripts/run.sh shell [--nvidia|--amd]

# O3DE Editor
./scripts/run.sh editor [--nvidia|--amd]

# Game launcher
./scripts/run.sh game [--nvidia|--amd] [--process-assets|--skip-process-assets]

# ROS2 navigation stack
./scripts/run.sh nav [--nvidia|--amd]
```

**Smart Attach:**
The `shell` command is smart - if a container is already running (e.g. you launched Editor), it will attach to it instead of failing or starting a new one. This allows you to have multiple terminals working in the same environment.

**Options:**
- `--nvidia` - Use NVIDIA GPU (auto-detected if nvidia-smi found)
- `--amd` - Use AMD GPU (auto-detected if /dev/kfd exists)
- `--audio` - Enable host audio passthrough
- `--no-mounts` - Disable project bind mounts (use container copy)
- `--process-assets` - Force asset processing before game launch
- `--skip-process-assets` - Skip asset processing entirely

**Tip:** You can open multiple shells in the running container:

```bash
# Terminal 1: Start editor
./scripts/run.sh editor --amd

# Terminal 2: Open shell in same container
./scripts/run.sh shell --amd
# Now you can run ROS2 commands, check logs, etc.
```

#### `cache-o3de-deb.sh` - Download O3DE Installer

```bash
# Download and cache O3DE installer (automatic via build.sh)
./scripts/cache-o3de-deb.sh --version 2510.2

# Force re-download
./scripts/cache-o3de-deb.sh --version 2510.2 --force
```

### Development Workflow

The container automatically bind-mounts key directories for fast iteration:

```
Host                          → Container
Project/Levels/              → /home/o3de/workspace/Project/Levels/
Project/Registry/            → /home/o3de/workspace/Project/Registry/
Project/Assets/              → /home/o3de/workspace/Project/Assets/
Project/Scripts/             → /home/o3de/workspace/Project/Scripts/
Project/Cache/               → /home/o3de/workspace/Project/Cache/
ros2_ws/                     → /home/o3de/workspace/ros2_ws/
```

**Fast Iteration Loop:**

1. Edit level/asset on host using Editor
2. Save changes
3. Relaunch game - **no container rebuild needed!**

```bash
# Edit in Editor
./scripts/run.sh editor --amd

# Test changes immediately
./scripts/run.sh game --amd
```

### Working with ROS2

```bash
# Enter shell with ROS2 environment
./scripts/run.sh shell --amd

# Inside container - ROS2 is pre-configured
ros2 topic list
ros2 node list

# Launch navigation
ros2 launch playground_nav navigation.launch.py

# Monitor topics
ros2 topic echo /scan
ros2 topic echo /odom

# View TF tree
ros2 run tf2_tools view_frames
```

### Asset Processing Modes

When launching the game, asset processing behavior:

```bash
# Auto mode (default) - uses cached assets, runs AP if needed
./scripts/run.sh game --amd

# Force mode - always runs AssetProcessorBatch
./scripts/run.sh game --amd --process-assets

# Skip mode - never runs AssetProcessorBatch (fastest)
./scripts/run.sh game --amd --skip-process-assets
```

**First run**: Use `--process-assets` or let auto mode seed the cache.  
**Subsequent runs**: Use auto mode or skip for fastest launch.

## Project Structure

```
o3de-playground/
├── scripts/
│   ├── build.sh              # Main build orchestrator
│   ├── run.sh                # Container launch helper
│   └── cache-o3de-deb.sh     # O3DE installer downloader
├── docker/
│   ├── Dockerfile            # Multi-stage build definition
│   ├── cache/                # Cached O3DE .deb files
│   └── .dockerignore         # Build context exclusions
├── Project/                  # O3DE project
│   ├── project.json          # Project config (gems, version)
│   ├── CMakeLists.txt        # Build configuration
│   ├── Gem/                  # Project-specific code
│   ├── Assets/               # Robot models, materials, prefabs
│   ├── Levels/               # Simulation environments
│   ├── Registry/             # Asset configurations
│   └── Cache/                # Asset cache (gitignored)
├── ros2_ws/                  # ROS2 colcon workspace
│   └── src/
│       ├── playground_nav/   # Nav2 launch configuration
│       └── turtlebot_3/      # TurtleBot3 packages (git submodules)
└── README.md                 # This file
```

## Configuration

### ROS2 Workspace

Edit ROS2 package dependencies:

```bash
# Dependencies auto-installed via rosdep
vim ros2_ws/src/playground_nav/package.xml

# Rebuild ROS2 workspace
./scripts/run.sh shell
colcon build --symlink-install
```

### Navigation Parameters

```bash
# Nav2 configuration
vim ros2_ws/src/playground_nav/launch/config/navigation_params.yaml

# SLAM configuration  
vim ros2_ws/src/playground_nav/launch/config/slam_params.yaml
```

### O3DE Project

```bash
# Add/remove gems
vim Project/project.json

# Rebuild project
./scripts/build.sh 4
```

## Troubleshooting

### Build Issues

**Problem**: Docker build fails at rosdep stage

```bash
# Solution: Missing ROS2 packages unavailable in Jazzy are auto-skipped
# Check logs for actual errors:
tail -100 logs/build_*.log
```

**Problem**: Out of disk space

```bash
# Check Docker disk usage
docker system df

# Clean up old images/containers
docker system prune -a
```

### Runtime Issues

**Problem**: GPU not detected in container

```bash
# NVIDIA: Verify toolkit installation
docker run --rm --gpus all nvidia/cuda:12.0-base nvidia-smi

# AMD: Check device access
ls -l /dev/kfd /dev/dri
```

**Problem**: X11 display not working

```bash
# Allow Docker X11 access
xhost +local:docker

# Verify DISPLAY is set
echo $DISPLAY
```

**Problem**: File permission errors

The container runs as your host user (not root), so file ownership matches automatically. If you see permission errors:

```bash
# Check ownership
ls -la Project/

# Fix if needed (should rarely be necessary)
sudo chown -R $(whoami):$(whoami) Project/
```

### ROS2 Issues

**Problem**: No topics visible

```bash
# Verify ROS2 environment
./scripts/run.sh shell
echo $ROS_DISTRO          # Should show: jazzy
echo $RMW_IMPLEMENTATION  # Should show: rmw_cyclonedds_cpp

# Check if workspace is sourced
env | grep AMENT_PREFIX_PATH
```

**Problem**: Nav2 won't launch

```bash
# Verify dependencies installed
./scripts/run.sh shell
ros2 pkg list | grep nav2
```

## Advanced Usage

### Custom Docker Tags

```bash
# Build with custom tag
docker build -t o3de-playground:custom-tag -f docker/Dockerfile .

# Run custom tag
./scripts/run.sh editor --tag custom-tag
```

### Direct Docker Commands

If you need finer control, bypass helper scripts:

```bash
# Manual build
docker build -t o3de-playground:latest -f docker/Dockerfile .

# Manual run with NVIDIA
docker run -it --rm \
  --runtime=nvidia --gpus all \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -e DISPLAY=$DISPLAY \
  o3de-playground:latest bash

# Manual run with AMD  
docker run -it --rm \
  --device=/dev/kfd --device=/dev/dri \
  --group-add video --group-add render \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -e DISPLAY=$DISPLAY \
  -e XDG_RUNTIME_DIR=/tmp/runtime-root \
  o3de-playground:latest bash
```

### Incremental Builds

The Dockerfile uses multi-stage builds for efficient layer caching:

```
Stage 1: base          → O3DE build tools + ROS2 base
Stage 2: ros-deps      → rosdep-installed dependencies
Stage 3: o3de-deps     → O3DE + gems + 3rdParty packages
Stage 4: o3de-builder  → Project build
Stage 5: runtime       → Final image with user/GID
```

**Rebuild strategies:**

```bash
# Only runtime environment changed (PATH, user config)
# → Rebuilds stage 5 only (~10 seconds)
docker build --target runtime ...

# Project code changed (Project/, ros2_ws/)
# → Rebuilds stages 4-5 only (no dependency re-download)
docker build ...

# ROS2 workspace dependencies changed (package.xml)
# → Rebuilds stages 2-5 (rosdep re-runs)
docker build ...
```

## References

- [O3DE Documentation](https://docs.o3de.org/)
- [O3DE ROS2 Gem Guide](https://docs.o3de.org/docs/user-guide/gems/reference/robotics/ros2/)
- [Nav2 Documentation](https://navigation.ros.org/)
- [ROS2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [ROSConDemo](https://github.com/o3de/ROSConDemo) - Reference O3DE+ROS2 project
- [RobotVacuumSample](https://github.com/o3de/RobotVacuumSample) - Example navigation implementation

## Contributing

Contributions welcome! Please:
1. Fork the repository
2. Create a feature branch
3. Test your changes with `./scripts/build.sh`
4. Submit a pull request

## License

Apache-2.0

---

**Need help?** Open an issue with:
- Build logs from `logs/build_*.log`
- Output of `docker --version` and `nvidia-smi` or `rocm-smi`
- Your GPU model and driver version
