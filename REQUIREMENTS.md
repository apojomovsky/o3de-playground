# O3DE ROS2 Playground - Requirements

## Project Vision

A containerized O3DE + ROS2 playground for mobile robotics, serving as:
- **Experimentation sandbox** - Quick setup/teardown for testing robots/sensors
- **Learning environment** - Understanding ROS2/O3DE integration patterns
- **Development testbed** - Platform for algorithm development (navigation, SLAM, etc.)
- **Future extensibility** - Foundation for more advanced scenarios (manipulation, multi-robot, etc.)

---

## Platform & Environment

### Target Platform
- **OS**: Ubuntu 24.04 (host)
- **ROS2 Distribution**: Jazzy
- **Deployment**: Fully containerized with Docker
- **RMW**: CycloneDDS (recommended for Nav2)

### Containerization Strategy
- Docker-based development environment
- Reproducible builds
- Isolated from host system
- Multi-GPU support: AMD 780M (Wayland) + NVIDIA RTX 4050 (X11)

**Decision**: Single fat container with multi-stage build for development
- Reference pattern: ROSConDemo's multi-Dockerfile approach
- Base: `ros:jazzy-ros-base-noble`
- Three variants: Full (Editor+Sim+Nav), Simulation-only, Nav-only
- Multi-stage for AMD and NVIDIA GPU support

---

## Robot Focus

### Current Scope
- **Mobile robots only** (wheeled navigation platforms)
- Navigation stack (Nav2) integration
- SLAM capabilities

### Sensors (All Required)
- **Lidar** - 2D/3D point cloud generation
- **Cameras** - RGB + Depth
- **IMU** - Orientation and acceleration
- **GNSS** - GPS positioning simulation
- **Odometry** - Wheel encoder simulation
- **Contact** - Collision detection

### Future Extensibility
- Potential for manipulation (arms/grippers)
- Multi-robot coordination
- Custom algorithms beyond Nav2

---

## Project Structure

### Workspace Architecture

**Decision**: Hybrid - Separate build systems (NOT unified colcon workspace)

**Confirmed Pattern** (from ROSConDemo and RobotVacuumSample):
```
o3de-playground/
├── Project/                    # O3DE project (CMake-based)
│   ├── project.json            # Declares ROS2 gem dependency (≥3.3.0)
│   ├── CMakeLists.txt          # O3DE build configuration
│   ├── Gem/                    # Custom gem for project-specific code
│   │   ├── gem.json
│   │   ├── CMakeLists.txt      # target_depends_on_ros2_packages()
│   │   └── Source/
│   ├── Assets/
│   ├── Levels/
│   └── build/                  # O3DE build artifacts
├── ros2_ws/                    # ROS2 colcon workspace (separate)
│   ├── src/
│   │   └── playground_nav/     # Nav2 launch files, configs
│   ├── build/
│   ├── install/
│   └── log/
├── reference-samples/          # For learning only (not dependencies)
│   ├── ROSConDemo/
│   └── RobotVacuumSample/
└── docker/
    └── Dockerfile

```

**Build Commands** (separate, sequential):
```bash
# 1. Build O3DE project
cd Project
source /opt/ros/jazzy/setup.bash
cmake -B build/linux -G"Ninja Multi-Config" -DLY_DISABLE_TEST_MODULES=ON -DLY_STRIP_DEBUG_SYMBOLS=ON
cmake --build build/linux --config profile --target Playground Editor AssetProcessor

# 2. Build ROS2 workspace
cd ../ros2_ws
colcon build --symlink-install

# 3. Run (source ROS2 workspace first)
source ros2_ws/install/setup.bash
Project/build/linux/bin/profile/Editor
```

**Why separate builds**:
- O3DE uses custom CMake with EngineFinder.cmake pattern
- Colcon wrapping O3DE would cause version conflicts
- Both systems communicate via ROS2 middleware (topics/services)
- Reference samples all use this pattern

### Reference Samples

**Approach**: Clone samples locally for reference, but DO NOT make them dependencies.

**Samples to Reference**:
- [ROSConDemo](https://github.com/o3de/ROSConDemo) - Apple Kraken navigation demo
- [RobotVacuumSample](https://github.com/o3de/RobotVacuumSample) - Nav2 integration example

**Usage**: Extract patterns, component usage, configuration approaches - then implement from scratch.

---

## Technical Requirements

### ROS2 Integration
- Direct ROS2 node integration (no bridge)
- Standard ROS2 topics/services/actions
- Nav2 stack compatibility
- Standard sensor message types:
  - `sensor_msgs/PointCloud2` (Lidar)
  - `sensor_msgs/Image` (Camera)
  - `sensor_msgs/Imu` (IMU)
  - `sensor_msgs/NavSatFix` (GNSS)
  - `nav_msgs/Odometry` (Odometry)

### O3DE Components
- **Core Gems Required**:
  - ROS2 Gem (core integration)
  - ROS2 Sensors Gem (sensor simulation)
  - ROS2 Controllers Gem (differential drive, ackermann)
  - Robot Importer Gem (URDF support for future robots)

### Build System
- Docker-based builds
- Reproducible configuration
- Hot-reload capability (for rapid iteration)

**Confirmed Approach**:
- **O3DE**: Standard CMake with Ninja Multi-Config generator
- **ROS2**: Colcon with symlink-install for fast iteration
- **Docker**: Multi-stage build referencing ROSConDemo pattern
  - Build time: ~2+ hours first build (includes O3DE compilation)
  - Optimization flags: `-DLY_STRIP_DEBUG_SYMBOLS=ON`, `-DLY_DISABLE_TEST_MODULES=ON`

**Asset Pipeline in Container**:
- AssetProcessor runs inside container
- Assets built during Docker image creation
- Final step: `.Assets` target (~1 hour+)

---

## Development Workflow

### Desired Workflow
1. Clone repository
2. Build Docker image (installs O3DE 2510.2, ROS2 Jazzy, dependencies)
3. Launch container with GPU support (AMD ROCm or NVIDIA)
4. Inside container:
   ```bash
   # O3DE project already built in Docker image
   # Build ROS2 workspace (if modified)
   cd /data/workspace/ros2_ws
   colcon build --symlink-install
   
   # Source and launch
   source install/setup.bash
   /data/workspace/Project/build/linux/bin/profile/Editor
   ```
5. Launch Nav2 stack (separate terminal in same container)
6. Iterate on algorithms/robots

**Build Sequence** (documented, not questioned):
```bash
# In Dockerfile (build-time)
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Register O3DE
./o3de/scripts/o3de.sh register --this-engine
./o3de/scripts/o3de.sh register --gem-path o3de-extras/Gems/ROS2
./o3de/scripts/o3de.sh register -pp /data/workspace/Project

# Build O3DE
cmake -B build/linux -G"Ninja Multi-Config" -DLY_DISABLE_TEST_MODULES=ON
cmake --build build/linux --config profile --target Playground Editor AssetProcessor Playground.Assets

# Build Nav2 workspace
cd ros2_ws && colcon build --symlink-install
```

**Sourcing Strategy**:
- ROS2 sourced in container entrypoint: `source /opt/ros/jazzy/setup.bash`
- Nav workspace sourced before launch: `source ros2_ws/install/setup.bash`
- RMW set globally: `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`

### Iteration Speed
- Fast rebuilds for code changes
- Quick sensor/robot parameter tuning
- Easy level/scenario switching

---

## Minimal Setup Requirements

### What "Minimal" Means
- **One mobile robot** (differential drive or ackermann)
  - **Decision**: Use ROSBot XL URDF as reference (well-documented, Nav2 examples exist)
- **Essential sensors** (Lidar + Camera minimum, all 6 sensors nice-to-have)
- **Basic world** (ROS logo obstacle course - 9 cylinders in 3×3 grid)
  - **Programmatic creation**: Lua script spawner component
  - Cylinder specs: radius=0.15m, height=0.5m, grid spacing=1.1m
- **Nav2 integration** (working navigation stack)
  - Reference: RobotVacuumSample's navigation.launch.py pattern
- **Spawn/despawn** capability via ROS2 service
  - gazebo_msgs/srv/SpawnEntity service

### What "Informed and Idiomatic" Means
- Follow O3DE component architecture patterns
- Use ROS2Frame components properly (REP-103 compliance)
- Proper coordinate frame setup (`/tf`, `/tf_static`)
- Standard ROS2 naming conventions
- QoS policies matching Nav2 expectations
- Component composition (focused, single-purpose components)

---

## Answered Questions (Research Complete)

### Q1: O3DE + Colcon Workspace Integration ✅

**Answer**: Separate build systems, NOT integrated.

- O3DE project and ROS2 workspace are sibling directories
- O3DE uses CMake directly, ROS2 uses colcon
- They communicate via ROS2 middleware (topics/services)
- Both ROSConDemo and RobotVacuumSample use this pattern
- O3DE gems do NOT integrate with colcon packages

**Evidence**: ROSConDemo has `/Project/` (O3DE) and `/kraken_nav/` (colcon) as separate directories with independent build systems.

---

### Q2: Docker GPU/X11 Strategy ✅

**Answer**: Multi-GPU support with different strategies per platform.

**AMD 780M (Wayland host with XWayland)**:
```bash
docker run \
  --device=/dev/kfd --device=/dev/dri \
  --group-add video \
  --security-opt seccomp=unconfined \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -e DISPLAY=$DISPLAY \
  -e QT_QPA_PLATFORM=xcb \
  <image>
```
**Note**: Uses XWayland compatibility layer for X11 applications on Wayland host.
**Fallback**: Headless mode available via `Playground.GameLauncher` if display issues occur.

**NVIDIA RTX 4050 (X11 host)**:
```bash
xhost +local:docker
docker run \
  --runtime=nvidia --gpus all \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -e DISPLAY=$DISPLAY \
  <image>
```

**References**:
- [husarion/o3de-docker](https://github.com/husarion/o3de-docker) - Production X11+NVIDIA setup
- [ROCm container toolkit](https://github.com/ROCm/container-toolkit) - AMD GPU passthrough

**Performance**: ROSConDemo Dockerfile uses `-DLY_STRIP_DEBUG_SYMBOLS=ON` for optimization.

---

### Q3: O3DE Project Structure ✅

**Minimal required files** (from ROSConDemo analysis):

```
Project/
├── project.json              # REQUIRED - Project metadata, gem declarations
├── CMakeLists.txt            # REQUIRED - Root build config
├── game.cfg                  # Config file
├── Gem/                      # REQUIRED - Internal gem
│   ├── gem.json              # Gem metadata
│   ├── CMakeLists.txt        # Gem build config
│   │   # Must include: target_depends_on_ros2_packages(...)
│   ├── enabled_gems.cmake    # List of gems to enable (ROS2, PhysX, Atom, etc.)
│   ├── Source/
│   │   ├── *SystemComponent.cpp/h  # System component implementation
│   │   └── *Module.cpp             # Module entry point
│   ├── Include/              # Public headers
│   └── Platform/Linux/       # Platform abstraction layer
├── Assets/                   # Game assets (models, textures)
├── Levels/                   # Scene/level files (.prefab)
├── Registry/                 # O3DE settings (.setreg files)
└── cmake/
    └── EngineFinder.cmake    # Locates O3DE engine
```

**project.json structure**:
```json
{
  "project_name": "Playground",
  "project_id": "{unique-guid}",
  "engine": "o3de",
  "external_subdirectories": ["Gem"],
  "gem_names": [
    "ROS2>=3.3.0",
    "PhysX5",
    "Atom",
    "CameraFramework",
    "PrimitiveAssets"
  ],
  "engine_version": "2510.2"
}
```

---

### Q4: Nav2 Integration Points ✅

**From RobotVacuumSample analysis**:

**Required Topics**:
- `/scan` - sensor_msgs/LaserScan (lidar input)
- `/odom` - nav_msgs/Odometry (odometry)
- `/map` - nav_msgs/OccupancyGrid (SLAM output)
- `/cmd_vel` - geometry_msgs/Twist (velocity commands)
- `/tf`, `/tf_static` - TF transforms

**Required Frames**:
```
map (global frame)
  └── odom (odometry frame)
      └── base_link (robot base)
```

**QoS Policies**:
- Map: Reliable, Transient Local, Keep Last
- LaserScan: Best Effort, Volatile, Keep Last
- Odometry: Reliable, Volatile, Keep Last

**Nav2 Configuration** (navigation_params.yaml - 268 lines):
- Controller frequency: 20 Hz
- Local costmap: 5m×5m, 0.02m resolution, rolling window
- Global costmap: static map, 0.02m resolution
- Robot radius: 0.25m
- Inflation radius: 0.55m (local), 0.4m (global)
- Planner: NavfnPlanner (A*)
- Transform timeout: 0.2s
- TF publish period: 0.02s (50 Hz)

**Map Server**:
- Static maps: `/map` topic subscription
- SLAM: slam_toolbox publishes `/map` and map→odom transform

**Launch Pattern** (from RobotVacuumSample):
```python
# 1. SLAM launch (includes pointcloud_to_laserscan conversion)
# 2. Nav2 bringup with custom params
# 3. RViz2 for visualization
```

---

### Q5: Reference Sample Analysis ✅

**ROSConDemo Patterns to Adopt**:
- ✅ Hybrid build system (O3DE + ROS2 separate)
- ✅ Gem-based architecture with component composition
- ✅ Platform Abstraction Layer (PAL) for multi-platform
- ✅ Docker multi-image strategy (Full, Simulation, NavStack)
- ✅ ROS2 package structure (ament_python with entry points)
- ✅ Configuration management (.setreg, .yaml, .launch.py)

**RobotVacuumSample Patterns to Adopt**:
- ✅ Nav2 launch orchestration (SLAM → Nav2 → RViz2)
- ✅ SLAM Toolbox integration with pointcloud conversion
- ✅ Nav2 parameter structure (268-line YAML)
- ✅ RViz configuration with TF tree visualization
- ✅ Coordinate frame setup (map→odom→base_link)

**Critical Files to Replicate**:

| File | Purpose | Source |
|------|---------|--------|
| `Project/project.json` | O3DE project config, ROS2 gem dependency | ROSConDemo |
| `Project/Gem/CMakeLists.txt` | Gem build with `target_depends_on_ros2_packages()` | ROSConDemo |
| `Project/Gem/enabled_gems.cmake` | Enable ROS2, PhysX, Atom gems | ROSConDemo |
| `nav_workspace/src/*/launch/navigation.launch.py` | Nav2 orchestration | RobotVacuumSample |
| `nav_workspace/src/*/launch/config/navigation_params.yaml` | Nav2 configuration | RobotVacuumSample |
| `nav_workspace/src/*/launch/config/slam_params.yaml` | SLAM Toolbox config | RobotVacuumSample |
| `Docker/Dockerfile` | Container build | ROSConDemo |

**Component Architecture**:
- System component for gem initialization
- Feature components (ApplePicker, FruitStorage in ROSConDemo)
- Bus interfaces for inter-component communication
- Separate concerns (navigation, manipulation, sensors)

---

### Q6: Multi-Robot Future-Proofing ✅

**Namespace Strategy** (from ROSConDemo):
- Use `ROS2FrameComponent` for namespace management
- Namespace validation automatic via `ROS2Names::GetNamespacedName()`
- Multi-robot spawning via `/spawn_entity` service
- Index auto-appended: `apple_kraken_rusty` → `apple_kraken_rusty_1`

**Example**:
```bash
ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity \
  '{name: "robot", xml: "spawn_point_1"}'
# Creates robot with namespace /robot_1
```

---

### Q7: Performance Tuning ✅

**Sensor Update Rates** (from RobotVacuumSample):
- Joint states: 50 Hz
- SLAM TF publish: 50 Hz (0.02s period)
- Controller: 20 Hz
- Local costmap update: 5 Hz, publish: 2 Hz
- Global costmap update: 1 Hz, publish: 1 Hz

**Physics vs ROS2**:
- Physics: 1000 Hz update (0.001s step)
- ROS2: Sensors publish at component-defined rates
- No tight coupling required

**CycloneDDS Tuning**:
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# Use CycloneDDS for Nav2 (FastDDS has known issues with navigation)
```

**Docker Performance**:
- Strip debug symbols: `-DLY_STRIP_DEBUG_SYMBOLS=ON`
- Disable test modules: `-DLY_DISABLE_TEST_MODULES=ON`
- Use Ninja Multi-Config for parallel builds
- Asset processing: Final, slowest step (~1 hour)

---

## Environment Creation: ROS Logo Obstacle Course

**Decision**: Programmatic creation via Lua script spawner

### Specifications (from Gazebo TurtleBot3 World)

**9 Cylinders in 3×3 Grid**:
- **Radius**: 0.15m (15cm)
- **Height**: 0.5m (50cm)
- **Grid spacing**: 1.1m between centers
- **Total grid size**: 2.2m × 2.2m
- **Color**: White
- **Z-position**: 0.25m (centered vertically)

**Grid Positions**:
| Row | X | Y Positions |
|-----|---|-------------|
| 1 | -1.1 | -1.1, 0, 1.1 |
| 2 | 0 | -1.1, 0, 1.1 |
| 3 | 1.1 | -1.1, 0, 1.1 |

### O3DE Implementation Approaches

**Option A: Lua Script Component** (Recommended for runtime spawning):
```lua
local NavTestSpawner = {
    Properties = {
        CylinderPrefab = { default=SpawnableScriptAssetRef(), 
                          description="Cylinder prefab to spawn" },
    }
}

function NavTestSpawner:OnActivate()
    self.spawnableMediator = SpawnableScriptMediator()
    self.ticket = self.spawnableMediator:CreateSpawnTicket(
        self.Properties.CylinderPrefab)
    
    -- Spawn 9 cylinders in 3x3 grid
    for i = 0, 8 do
        local x = (i % 3 - 1) * 1.1  -- -1.1, 0, 1.1
        local y = (math.floor(i / 3) - 1) * 1.1
        local pos = Vector3(x, y, 0.25)
        
        self.spawnableMediator:SpawnAndParentAndTransform(
            self.ticket, self.entityId, pos, 
            Quaternion.CreateIdentity(), Vector3(1,1,1))
    end
end

return NavTestSpawner
```

**Option B: Prefab JSON** (Configuration-first, no runtime spawning):
- Create `cylinder.prefab` with Cylinder Shape component (radius=0.15, height=0.5)
- Create parent prefab with 9 child entities at grid positions
- Instantiate in level

**Option C: Python Editor Automation** (Batch generation):
```python
import azlmbr.entity

for i in range(9):
    x = (i % 3 - 1) * 1.1
    y = (i // 3 - 1) * 1.1
    create_cylinder_entity(position=(x, y, 0.25), 
                          radius=0.15, height=0.5)
```

**Recommended**: Option A (Lua) for flexibility and runtime control.

**References**:
- [O3DE Procedural Prefabs](https://docs.o3de.org/docs/user-guide/assets/scene-pipeline/procedural_prefab/)
- [O3DE Spawnable System](https://github.com/o3de/o3de/tree/development/AutomatedTesting/LuaScripts/Spawnables)
- [TurtleBot3 World SDF](https://github.com/mlherd/Dataset-of-Gazebo-Worlds-Models-and-Maps/blob/main/worlds/turtlebot3_world/model.sdf)

---

## Open Questions Requiring Research (NONE - All Answered)

---

## Success Criteria (Revisited)

### Minimum Viable Playground

A containerized environment where:
1. ✅ **Build**: `docker build -t o3de-playground .` completes successfully
2. ✅ **Launch**: Container runs with GPU support (NVIDIA or AMD)
3. ✅ **Editor**: O3DE Editor launches with NavTest level (9-cylinder obstacle course)
4. ✅ **Spawn Robot**: `ros2 service call /spawn_entity` successfully spawns mobile robot
5. ✅ **Sensors**: Robot publishes `/scan` (Lidar) and `/camera/image_raw` (Camera)
6. ✅ **TF Tree**: Robot publishes `map→odom→base_link` transforms
7. ✅ **Nav2**: Navigation stack localizes robot and plans paths around obstacles
8. ✅ **RViz2**: Visualization shows TF tree, costmaps, sensor data, and navigation goal

### Stretch Goals
- Multiple robot spawning (test namespace isolation)
- All 6 sensor types publishing correct message types
- SLAM integration (slam_toolbox builds map from `/scan`)
- Docker image published to Docker Hub or GitHub Container Registry
- Comprehensive documentation (README, DOCKER, QUICKSTART guides)

---

## Design Decisions (Finalized)

| Decision | Selected Option | Rationale | Status |
|----------|----------------|-----------|--------|
| **Workspace structure** | **B) Separate parallel builds** | ROSConDemo/RobotVacuumSample pattern, avoids version conflicts | ✅ **FINAL** |
| **Docker strategy** | **A) Single fat container** (with multi-stage for variants) | Simplicity for dev, can split later for deployment | ✅ **FINAL** |
| **Reference robot** | **A) Import existing (ROSBot XL)** | Well-documented, Nav2 examples exist, realistic | ✅ **FINAL** |
| **World complexity** | **B) Simple obstacles** (ROS logo 9-cylinder grid) | Good for Nav2 testing, programmatically creatable | ✅ **FINAL** |
| **Display strategy** | **Hybrid**: X11 (NVIDIA machine), Headless (AMD machine) | Supports both GPU types, Wayland not mature for O3DE | ✅ **FINAL** |
| **Build commands** | **Separate**: CMake for O3DE, colcon for ROS2 | Industry standard, documented in both samples | ✅ **FINAL** |

---

## Key Technologies \u0026 Versions

| Component | Version | Notes |
|-----------|---------|-------|
| **Ubuntu** | 24.04 (Noble) | Host and container base |
| **ROS2** | Jazzy | Tested with O3DE ROS2 Gem ≥3.3.0 |
| **O3DE** | 2510.2 (Jan 2026) | Latest stable, Debian package available |
| **O3DE ROS2 Gem** | ≥3.3.0 | From o3de-extras repository |
| **RMW** | CycloneDDS (`rmw_cyclonedds_cpp`) | Required for Nav2 (FastDDS has issues) |
| **Nav2** | Latest (Jazzy) | From `ros-jazzy-navigation2` package |
| **SLAM** | slam_toolbox | From `ros-jazzy-slam-toolbox` package |
| **Physics** | PhysX5 | O3DE Gem |
| **Rendering** | Atom | O3DE Gem |

### O3DE Installation Method

**Decision**: Use **Debian package** for simplicity.

**Rationale**:
- Faster setup (no 2+ hour compilation)
- ROS2 Gem works with Debian-installed O3DE (officially confirmed)
- Ubuntu 24.04 + ROS2 Jazzy is an explicitly tested combination
- No engine source modifications planned

**Installation**:
```bash
# Download from o3debinaries.org
wget https://o3debinaries.org/download/o3de_2510_2.deb
sudo apt install ./o3de_2510_2.deb

# Fix apt permission issue (known bug)
sudo chown -Rv _apt:root /var/cache/apt/archives/partial/
sudo chmod -Rv 700 /var/cache/apt/archives/partial/
```

**Alternative (Source Build)**: Use if engine modifications needed.
```bash
git clone --branch 2510.2 https://github.com/o3de/o3de.git
git clone --branch 2510.2 https://github.com/o3de/o3de-extras.git
```

---

## Repository Structure (Final)

```
o3de-playground/
├── Project/                           # O3DE project (CMake-based)
│   ├── project.json                   # Declares ROS2 gem ≥3.3.0
│   ├── CMakeLists.txt
│   ├── game.cfg
│   ├── Gem/                           # Internal gem for project
│   │   ├── gem.json
│   │   ├── CMakeLists.txt             # target_depends_on_ros2_packages()
│   │   ├── enabled_gems.cmake         # ROS2, PhysX5, Atom, etc.
│   │   ├── Source/
│   │   │   ├── PlaygroundSystemComponent.cpp/h
│   │   │   └── PlaygroundModule.cpp
│   │   ├── Include/Playground/
│   │   └── Platform/Linux/
│   ├── Assets/
│   │   ├── Robots/                    # Robot URDF files
│   │   └── Obstacles/                 # Cylinder prefab
│   ├── Levels/
│   │   └── NavTest/                   # Test level with spawner
│   ├── Scripts/
│   │   └── NavTestSpawner.lua         # 9-cylinder spawner
│   ├── Registry/                      # O3DE settings (.setreg)
│   ├── cmake/
│   │   └── EngineFinder.cmake
│   └── build/                         # O3DE build artifacts (gitignored)
├── nav_workspace/                     # ROS2 colcon workspace
│   ├── src/
│   │   └── playground_nav/            # Nav2 launch package
│   │       ├── package.xml
│   │       ├── setup.py
│   │       ├── launch/
│   │       │   ├── navigation.launch.py
│   │       │   └── config/
│   │       │       ├── navigation_params.yaml
│   │       │       ├── slam_params.yaml
│   │       │       └── config.rviz
│   │       └── resource/
│   ├── build/                         # Colcon build (gitignored)
│   ├── install/                       # Colcon install (gitignored)
│   └── log/                           # Colcon logs (gitignored)
├── reference-samples/                 # Reference only (gitignored)
│   ├── ROSConDemo/
│   └── RobotVacuumSample/
├── Docker/
│   ├── Dockerfile                     # Multi-stage (AMD + NVIDIA)
│   ├── README.md                      # Build/run instructions
│   └── .dockerignore
├── REQUIREMENTS.md                    # This file
├── README.md                          # Project overview
├── DOCKER.md                          # Docker-specific docs
├── .gitignore                         # Build artifacts, references
└── .gitattributes                     # Git LFS configuration (if needed)
```

---

## References \u0026 Resources

### Official Documentation
- [O3DE Documentation](https://docs.o3de.org/)
- [O3DE ROS2 Gem](https://docs.o3de.org/docs/user-guide/gems/reference/robotics/ros2/)
- [O3DE Robotics Overview](https://docs.o3de.org/docs/user-guide/interactivity/robotics/overview/)
- [O3DE Prefab System](https://docs.o3de.org/docs/engine-dev/architecture/prefabs/)
- [ROS2 Project Configuration](https://docs.o3de.org/docs/user-guide/interactivity/robotics/project-configuration/)

### Reference Projects
- [o3de/ROSConDemo](https://github.com/o3de/ROSConDemo) - Apple Kraken navigation demo
- [o3de/RobotVacuumSample](https://github.com/o3de/RobotVacuumSample) - Nav2 integration example
- [husarion/o3de-docker](https://github.com/husarion/o3de-docker) - Production Docker setup

### ROS Logo Environment
- [TurtleBot3 World SDF](https://github.com/mlherd/Dataset-of-Gazebo-Worlds-Models-and-Maps/blob/main/worlds/turtlebot3_world/model.sdf) - 9-cylinder grid specs

### Docker \u0026 GPU
- [ROCm Container Toolkit](https://github.com/ROCm/container-toolkit) - AMD GPU passthrough
- [O3DE Docker Blog](https://o3de.org/running-robotic-simulations-using-docker-images/) - Official guidance

### Nav2 Resources
- [Nav2 Documentation](https://navigation.ros.org/)
- [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) - SLAM implementation

---

## Glossary

- **O3DE**: Open 3D Engine - Open-source game/simulation engine
- **Gem**: O3DE plugin/module system (similar to ROS packages)
- **Prefab**: O3DE entity template (similar to URDF models)
- **Component**: Modular behavior attached to entities (ECS pattern)
- **EBus**: Event bus system for inter-component communication
- **PAL**: Platform Abstraction Layer - Multi-platform support
- **Nav2**: ROS2 Navigation stack
- **SLAM**: Simultaneous Localization and Mapping
- **TF**: Transform tree (coordinate frame relationships)
- **QoS**: Quality of Service policies for ROS2 topics
- **RMW**: ROS Middleware (DDS implementation)
- **CycloneDDS**: Eclipse Cyclone DDS - Recommended for Nav2
- **Ackermann**: Steering geometry for car-like robots
- **Differential Drive**: Two-wheel drive for mobile robots

---

## Next Steps - Implementation Plan

### Phase 1: Project Skeleton

1. **Create O3DE Project Structure**:
   ```bash
   mkdir -p Project/{Gem/Source,Assets,Levels,Registry,cmake}
   ```
   Files to create:
   - `Project/project.json` - Based on ROSConDemo pattern
   - `Project/CMakeLists.txt` - O3DE root build config
   - `Project/Gem/gem.json` - Gem metadata
   - `Project/Gem/CMakeLists.txt` - With `target_depends_on_ros2_packages()`
   - `Project/Gem/enabled_gems.cmake` - Enable ROS2, PhysX5, Atom
   - `Project/Gem/Source/PlaygroundSystemComponent.cpp/h` - System component
   - `Project/cmake/EngineFinder.cmake` - O3DE engine locator

2. **Create ROS2 Workspace**:
   ```bash
   mkdir -p nav_workspace/src/playground_nav/{launch/config,resource}
   ```
   Files to create:
   - `nav_workspace/src/playground_nav/package.xml` - ROS2 package metadata
   - `nav_workspace/src/playground_nav/setup.py` - ament_python setup
   - `nav_workspace/src/playground_nav/launch/navigation.launch.py` - Based on RobotVacuumSample
   - `nav_workspace/src/playground_nav/launch/config/navigation_params.yaml` - Nav2 config (adapt from RobotVacuumSample)
   - `nav_workspace/src/playground_nav/launch/config/slam_params.yaml` - SLAM config

3. **Create Docker Setup**:
   ```bash
   mkdir -p Docker
   ```
   Files to create:
   - `Docker/Dockerfile` - Multi-stage build (AMD + NVIDIA variants)
   - `Docker/README.md` - Build and run instructions
   - `Docker/.dockerignore` - Exclude build artifacts

4. **Create Documentation**:
   - `README.md` - Project overview, build instructions
   - `DOCKER.md` - GPU-specific run commands
   - `.gitignore` - O3DE and ROS2 build artifacts

### Phase 2: Minimal World

1. **Create NavTest Level**:
   - `Project/Levels/NavTest/NavTest.prefab` - Empty level with ground plane
   
2. **Create Cylinder Prefab**:
   - `Project/Assets/Obstacles/cylinder.prefab` - Cylinder with PhysX collider
   - Components: Cylinder Shape (r=0.15, h=0.5), PhysX Static, Mesh

3. **Create Lua Spawner**:
   - `Project/Scripts/NavTestSpawner.lua` - Spawns 9-cylinder grid
   - Attach to spawner entity in NavTest level

### Phase 3: Robot Integration

1. **Import Robot URDF** (ROSBot XL or similar):
   - Place URDF in `Project/Assets/Robots/`
   - Use O3DE Robot Importer tool

2. **Configure Sensors**:
   - Add Lidar component (publishes `/scan`)
   - Add Camera component (publishes `/camera/image_raw`)
   - Configure ROS2Frame components for TF tree

3. **Test Spawning**:
   - Implement `/spawn_entity` service in system component
   - Test with: `ros2 service call /spawn_entity ...`

### Phase 4: Nav2 Integration

1. **Launch File Testing**:
   - Verify SLAM launch works with O3DE lidar data
   - Verify Nav2 costmaps use `/scan` correctly
   - Test RViz2 visualization

2. **Parameter Tuning**:
   - Adjust costmap sizes for test environment
   - Tune controller parameters for robot dynamics
   - Set appropriate velocity limits

3. **Full Integration Test**:
   - Launch Editor with NavTest level
   - Spawn robot
   - Launch Nav2 stack
   - Send navigation goal in RViz2
   - Verify robot navigates around cylinders

### Phase 5: Docker Deployment

1. **Build Docker Image**:
   ```bash
   cd Docker
   docker build -t o3de-playground:latest .
   ```

2. **Test on NVIDIA (X11)**:
   ```bash
   xhost +local:docker
   docker run --runtime=nvidia --gpus all \
     -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
     -e DISPLAY=$DISPLAY \
     o3de-playground:latest
   ```

3. **Test on AMD (Headless)**:
   ```bash
   docker run --device=/dev/kfd --device=/dev/dri \
     --group-add video \
     o3de-playground:latest \
     /data/workspace/Playground/build/linux/bin/profile/Playground.GameLauncher
   ```

---

## Success Criteria (Revisited)
