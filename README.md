# O3DE ROS2 Playground

A containerized O3DE + ROS2 Jazzy playground for mobile robotics simulation.

## Overview

This project provides a ready-to-use environment for:
- **Experimentation** - Quick setup/teardown for testing robots and sensors
- **Learning** - Understanding ROS2/O3DE integration patterns
- **Development** - Platform for navigation, SLAM, and robotics algorithms
- **Future extensibility** - Foundation for manipulation, multi-robot, and more

## Requirements

### Hardware
- GPU: NVIDIA (recommended) or AMD
- RAM: 16GB minimum, 32GB recommended
- Storage: 60GB+ free space

### Software
- Ubuntu 24.04 (Noble)
- Docker with GPU support
- For NVIDIA: [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)
- For AMD: ROCm drivers

## Quick Start

### Option 1: Docker (Recommended)

```bash
# Build the Docker image (2-4 hours first time)
cd docker
docker build -t o3de-playground:latest .

# Run with NVIDIA GPU (X11)
xhost +local:docker
docker run --runtime=nvidia --gpus all \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -e DISPLAY=$DISPLAY \
  o3de-playground:latest

# Run with AMD GPU (Wayland host using XWayland)
docker run --device=/dev/kfd --device=/dev/dri \
  --group-add video \
  --security-opt seccomp=unconfined \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -e DISPLAY=$DISPLAY \
  -e QT_QPA_PLATFORM=xcb \
  o3de-playground:latest

# Inside container: Launch Editor
/data/workspace/Project/build/linux/bin/profile/Editor

# In another terminal: Launch Nav2
docker exec -it <container_id> bash
source /data/workspace/ros2_ws/install/setup.bash
ros2 launch playground_nav navigation.launch.py
```

### Option 2: Native Build

```bash
# Prerequisites
sudo apt install ros-jazzy-desktop ros-jazzy-navigation2 ros-jazzy-slam-toolbox

# Source ROS2
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Install O3DE (Debian package)
wget https://o3debinaries.org/download/o3de_2510_2.deb
sudo apt install ./o3de_2510_2.deb

# Clone o3de-extras for ROS2 Gem
git clone --branch 2510.2 https://github.com/o3de/o3de-extras.git
o3de register --gem-path o3de-extras/Gems/ROS2

# Register and build this project
o3de register -pp Project
cd Project
cmake -B build/linux -G "Ninja Multi-Config" -DLY_DISABLE_TEST_MODULES=ON
cmake --build build/linux --config profile --target Playground Editor AssetProcessor

# Build ROS2 workspace
cd ../ros2_ws
colcon build --symlink-install
```

## Project Structure

```
o3de-playground/
├── Project/                    # O3DE project (CMake-based)
│   ├── project.json            # Project config, gem dependencies
│   ├── CMakeLists.txt          # Build configuration
│   ├── Gem/                    # Internal gem for project code
│   ├── Assets/                 # Robot models, prefabs
│   ├── Levels/                 # Simulation scenes
│   └── build/                  # Build artifacts (gitignored)
├── ros2_ws/                    # ROS2 colcon workspace
│   └── src/
│       └── playground_nav/     # Nav2 launch package
├── docker/                     # Docker configuration
│   ├── Dockerfile              # Multi-stage build
│   └── README.md               # Docker-specific docs
├── REQUIREMENTS.md             # Detailed requirements document
└── README.md                   # This file
```

## Key Features

### ROS2 Integration
- **ROS2 Jazzy** on Ubuntu 24.04
- **CycloneDDS** middleware (recommended for Nav2)
- **Nav2** navigation stack integration
- **SLAM Toolbox** for mapping

### O3DE Features
- **O3DE 2510.2** (latest stable)
- **ROS2 Gem** ≥3.3.0 for native integration
- **PhysX5** physics simulation
- **Atom** renderer

### Sensors (Planned)
- Lidar (2D/3D point clouds)
- Camera (RGB + Depth)
- IMU
- GNSS
- Odometry
- Contact sensors

## Usage

### Spawning Robots

```bash
# Spawn a robot at a named spawn point
ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity \
  '{name: "robot", xml: "spawn_point_1"}'

# Spawn at specific pose
ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity \
  '{name: "robot", initial_pose: {position: {x: 0, y: 0, z: 0.2}}}'
```

### Navigation

```bash
# Launch full navigation stack
ros2 launch playground_nav navigation.launch.py

# Use RViz2 "2D Goal Pose" tool to set navigation goals
```

### Visualization

```bash
# Check available topics
ros2 topic list

# View TF tree
ros2 run tf2_tools view_frames

# Monitor sensor data
ros2 topic echo /scan
ros2 topic echo /odom
```

## Configuration

### Nav2 Parameters
Edit `ros2_ws/src/playground_nav/launch/config/navigation_params.yaml`:
- Controller frequency
- Costmap sizes and resolution
- Planner settings
- Recovery behaviors

### SLAM Parameters
Edit `ros2_ws/src/playground_nav/launch/config/slam_params.yaml`:
- Update rate
- Resolution
- Scan matching parameters

## Troubleshooting

### No ROS2 Topics
```bash
# Verify ROS2 is sourced
echo $ROS_DISTRO  # Should show: jazzy

# Check RMW implementation
echo $RMW_IMPLEMENTATION  # Should show: rmw_cyclonedds_cpp
```

### GPU Issues in Docker
```bash
# NVIDIA: Verify GPU access
nvidia-smi

# Check container toolkit
docker run --rm --gpus all nvidia/cuda:12.0-base nvidia-smi
```

### O3DE Build Errors
```bash
# Ensure ROS2 is sourced BEFORE cmake
source /opt/ros/jazzy/setup.bash
cmake -B build/linux ...
```

## References

- [O3DE Documentation](https://docs.o3de.org/)
- [O3DE ROS2 Gem](https://docs.o3de.org/docs/user-guide/gems/reference/robotics/ros2/)
- [Nav2 Documentation](https://navigation.ros.org/)
- [ROSConDemo](https://github.com/o3de/ROSConDemo) - Reference project
- [RobotVacuumSample](https://github.com/o3de/RobotVacuumSample) - Reference project

## License

Apache-2.0
