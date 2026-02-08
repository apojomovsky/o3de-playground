# O3DE ROS2 Playground Docker Setup

This Docker setup provides a complete containerized environment for running O3DE with ROS2 Jazzy integration on Ubuntu 24.04 (Noble).

## Prerequisites

### Hardware Requirements
- **Minimum 60 GB** of free disk space (O3DE + ROS2 + dependencies)
- **GPU Support**:
  - NVIDIA GPU (RTX 4050 or compatible) with NVIDIA Container Toolkit
  - AMD GPU (780M or compatible) with ROCm support

### Software Requirements
- Docker installed and configured (without requiring `sudo`)
  - See [Docker Engine post-installation steps](https://docs.docker.com/engine/install/linux-postinstall/)
- For NVIDIA GPU: [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker)
- For AMD GPU: [ROCm support](https://rocmdocs.amd.com/en/docs-5.7.1/deploy/linux/index.html)

## Building the Docker Image

### Build Command

From the repository root, run:

```bash
./scripts/cache-o3de-deb.sh --version 2510.2

docker build -t o3de-playground:latest \
  --build-arg O3DE_VERSION=2510.2 \
  --build-arg O3DE_EXTRAS_VERSION=2510.2 \
  -f docker/Dockerfile .

# Optional: seed host Project/Cache from the built image
./scripts/build.sh seed-cache
```

### Build Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `ROS_VERSION` | `jazzy` | ROS2 distribution (only Jazzy supported) |
| `UBUNTU_VERSION` | `noble` | Ubuntu version (24.04 = noble) |
| `O3DE_VERSION` | `2510.2` | O3DE engine version tag |
| `O3DE_EXTRAS_VERSION` | `2510.2` | O3DE extras (ROS2 Gem) version tag |
| `O3DE_DEB_SHA256` | empty | Optional SHA256 to verify downloaded/cached O3DE .deb |

### Build Time

The build process typically takes **1-3 hours** depending on hardware and network connectivity.
Installing O3DE from a cached host-side `.deb` removes most engine download overhead.

### Host Cache Behavior

- If `docker/cache/o3de_<version>.deb` exists, Docker installs that package first.
- Docker verifies the package using `https://o3debinaries.org/main/Latest/Linux/o3de_<version>.deb.sha256`.

## Running the Docker Container

### NVIDIA GPU (X11 Display)

Enable X11 access for the Docker daemon:

```bash
xhost +local:docker
```

Run the container with GPU support and X11 forwarding:

```bash
docker run --runtime=nvidia --gpus all \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -e DISPLAY=$DISPLAY \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  --name o3de-playground \
  o3de-playground:latest
```

**Inside the container**, launch the O3DE Editor:

```bash
/opt/O3DE/25.10.2/bin/Linux/profile/Default/Editor --project-path /data/workspace/Project
```

Or launch the game:

```bash
/data/workspace/Project/build/linux/bin/profile/Playground.GameLauncher
```

### AMD GPU (Wayland Host with XWayland)

Run the container with AMD GPU support using XWayland for display:

```bash
docker run --device=/dev/kfd --device=/dev/dri \
  --group-add video \
  --group-add render \
  --security-opt seccomp=unconfined \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -e DISPLAY=$DISPLAY \
  -e XDG_RUNTIME_DIR=/tmp/runtime-root \
  -e QT_QPA_PLATFORM=xcb \
  -e VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/radeon_icd.json \
  --name o3de-playground \
  o3de-playground:latest
```

**Inside the container**, launch the O3DE Editor:

```bash
/opt/O3DE/25.10.2/bin/Linux/profile/Default/Editor --project-path /data/workspace/Project
```

**Headless Fallback**: If display issues occur, use headless mode:

```bash
docker run --device=/dev/kfd --device=/dev/dri \
  --group-add video \
  --group-add render \
  --security-opt seccomp=unconfined \
  -e XDG_RUNTIME_DIR=/tmp/runtime-root \
  -e VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/radeon_icd.json \
  --name o3de-playground \
  o3de-playground:latest \
  /data/workspace/Project/build/linux/bin/profile/Playground.GameLauncher
```

### Interactive Shell

To get an interactive bash shell:

```bash
docker run -it \
  --runtime=nvidia --gpus all \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -e DISPLAY=$DISPLAY \
  o3de-playground:latest \
  /bin/bash
```

### Fast No-Rebuild Loop

Use `scripts/run.sh` for day-to-day level/asset iteration. It bind-mounts host project content and seeds host
`Project/Cache` from the image on first run. If seeding is unavailable, it falls back to `AssetProcessorBatch`.

```bash
./scripts/run.sh game --amd
```

Force or skip asset processing:

```bash
./scripts/run.sh game --amd --process-assets
./scripts/run.sh game --amd --skip-process-assets
```

Disable bind mounts (fallback behavior):

```bash
./scripts/run.sh game --amd --no-mounts
```

## Container Environment

### Workspace Structure

Inside the container at `/data/workspace/`:

```
/data/workspace/
├── o3de/                    # O3DE engine
├── o3de-extras/             # ROS2 Gem and additional packages
├── Project/                 # O3DE Playground project
│   └── build/linux/bin/profile/
│       ├── Editor           # O3DE Editor executable
│       ├── Playground.GameLauncher  # Game launcher
│       └── AssetProcessor   # Asset processor
└── ros2_ws/                 # ROS2 colcon workspace (if present)
    └── install/
```

### Environment Variables

The container automatically sets:

```bash
WORKSPACE=/data/workspace
O3DE_ROOT=/data/workspace/o3de
O3DE_EXTRAS_ROOT=/data/workspace/o3de-extras
PROJECT_ROOT=/data/workspace/Project
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### ROS2 Setup

ROS2 Jazzy environment is automatically sourced. The entrypoint script:
1. Sources `/opt/ros/jazzy/setup.bash`
2. Sources `/data/workspace/ros2_ws/install/setup.bash` (if available)
3. Sets `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`

## Common Tasks

### Launch Navigation Stack

Inside the container:

```bash
# Source ROS2 environment
source /opt/ros/jazzy/setup.bash
source /data/workspace/ros2_ws/install/setup.bash

# Launch navigation
ros2 launch o3de_kraken_nav navigation_multi.launch.py namespace:=robot_1 rviz:=True
```

### Run Asset Processor

```bash
/data/workspace/Project/build/linux/bin/profile/AssetProcessor
```

### Access ROS2 Topics

```bash
# List all topics
ros2 topic list

# Echo a topic
ros2 topic echo /tf

# Publish to a topic
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 1.0}}'
```

### Build Custom Packages

Inside the container:

```bash
cd /data/workspace/ros2_ws
colcon build --packages-select my_package
```

## Troubleshooting

### X11 Display Issues

If you get "Cannot connect to X server":

```bash
# Ensure X11 forwarding is enabled
xhost +local:docker

# Check DISPLAY variable
echo $DISPLAY

# Verify X11 socket is accessible
ls -la /tmp/.X11-unix/
```

### GPU Not Detected

**NVIDIA:**
```bash
# Inside container, verify GPU access
nvidia-smi
```

**AMD:**
```bash
# Inside container, verify Vulkan detects AMD (not llvmpipe)
vulkaninfo --summary
```

### Out of Memory

If the build fails with OOM errors:
- Increase Docker's memory limit
- Use `--memory=16g` flag when running the container
- Reduce parallel build jobs with `-j2` in cmake

### ROS2 Middleware Issues

If ROS2 nodes don't communicate:

```bash
# Verify middleware is set correctly
echo $RMW_IMPLEMENTATION
# Should output: rmw_cyclonedds_cpp

# Check ROS2 domain ID
echo $ROS_DOMAIN_ID
# Default is 0, ensure all nodes use the same domain
```

## Advanced Usage

### Mount Local Workspace

To develop locally and test in the container:

```bash
docker run -it \
  --runtime=nvidia --gpus all \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -e DISPLAY=$DISPLAY \
  -v /path/to/local/workspace:/data/workspace/ros2_ws \
  o3de-playground:latest
```

### Custom Build Configuration

To modify the build, edit the Dockerfile and rebuild:

```bash
docker build -t o3de-playground:custom \
  --build-arg O3DE_VERSION=2510.1 \
  -f docker/Dockerfile .
```

### Multi-Stage Build Optimization

The Dockerfile uses multi-stage builds to reduce final image size:
- **Stage 1 (base)**: ROS2 + dependencies
- **Stage 2 (o3de-builder)**: O3DE compilation
- **Stage 3 (runtime)**: Final optimized image

## Performance Tips

1. **Use SSD storage** for Docker images and containers
2. **Allocate sufficient CPU cores**: `--cpus=8` or more
3. **Increase memory**: `--memory=16g` for smooth operation
4. **Use volume mounts** for large datasets instead of copying
5. **Enable BuildKit** for faster builds: `DOCKER_BUILDKIT=1 docker build ...`

## Cleanup

### Remove Container

```bash
docker rm o3de-playground
```

### Remove Image

```bash
docker rmi o3de-playground:latest
```

### Prune Unused Resources

```bash
docker system prune -a
```

## Support

For issues related to:
- **O3DE**: See [O3DE Documentation](https://www.o3de.org/docs/)
- **ROS2**: See [ROS2 Documentation](https://docs.ros.org/)
- **Docker**: See [Docker Documentation](https://docs.docker.com/)

## License

This Docker setup is provided under the same license as O3DE (Apache 2.0 OR MIT).
