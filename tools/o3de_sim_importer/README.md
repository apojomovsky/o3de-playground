# o3de_sim_importer

Utility scripts to preprocess Gazebo/ROS robot and world assets into a shape that is easier to import into O3DE.

Current scope:

- Robot preprocessing (`prepare_robot.py`)
- World asset extraction manifest (`prepare_world.py`)
- Prefab scaffold generation (`generate_prefab.py`)
- Validation (`validate_import.py`)

## Why this exists

O3DE can import robots via Robot Importer, but common ROS/Gazebo references (`package://`, `model://`) and external mesh layouts usually require preprocessing. These scripts automate that repetitive work and generate manifests for traceability.

## Quick start

From repository root:

```bash
python3 tools/o3de_sim_importer/prepare_robot.py \
  --urdf /home/alexis/o3de-playground/turtlebot_3/turtlebot3_simulations/turtlebot3_gazebo/urdf/turtlebot3_burger.urdf \
  --workspace-root /home/alexis/o3de-playground/turtlebot_3/turtlebot3 \
  --workspace-root /home/alexis/o3de-playground/turtlebot_3/turtlebot3_simulations \
  --output-root /home/alexis/o3de-playground/Project/Assets/ImportedRobots \
  --robot-name turtlebot3_burger

python3 tools/o3de_sim_importer/validate_import.py \
  --manifest /home/alexis/o3de-playground/Project/Assets/ImportedRobots/turtlebot3_burger/import_manifest.json
```

World extraction (manifest-first):

```bash
python3 tools/o3de_sim_importer/prepare_world.py \
  --world-file /home/alexis/o3de-playground/turtlebot_3/turtlebot3_simulations/turtlebot3_gazebo/worlds/turtlebot3_world.world \
  --models-root /home/alexis/o3de-playground/turtlebot_3/turtlebot3_simulations/turtlebot3_gazebo/models \
  --gazebo-models-root /usr/share/gazebo-11/models \
  --output-root /home/alexis/o3de-playground/Project/Assets/ImportedWorlds \
  --world-name turtlebot3_world

python3 tools/o3de_sim_importer/validate_import.py \
  --manifest /home/alexis/o3de-playground/Project/Assets/ImportedWorlds/turtlebot3_world/world_manifest.json

python3 tools/o3de_sim_importer/generate_prefab.py \
  --scene-manifest /home/alexis/o3de-playground/Project/Assets/ImportedWorlds/turtlebot3_world/o3de_scene_manifest.json

# Optional flags:
# --project-root /home/alexis/o3de-playground
# --output-prefab /tmp/turtlebot3_world_scaffold.prefab
# --max-entities 50
# --no-lowercase-hints
```

## Outputs

### prepare_robot.py

- `.../<robot_name>/urdf/<robot_name>_import_ready.urdf`
- `.../<robot_name>/assets/...` copied mesh files
- `.../<robot_name>/import_manifest.json`

### prepare_world.py

- `.../<world_name>/assets/...` copied mesh files from included model URIs
- `.../<world_name>/world_manifest.json`
- `.../<world_name>/o3de_scene_manifest.json` first-pass entity placement data
- `model://ground_plane` and `model://sun` are intentionally treated as built-in O3DE level replacements (not copied as imported assets)

### generate_prefab.py

- `.../<world_name>/<world_name>_scaffold.prefab` first-pass O3DE prefab scaffold
- Applies translation and rotation from `world_pose_additive` (`radians -> degrees` for rotation)
- Resolves `assetHint` from `world_manifest.json` copied assets and optionally from `Project/Cache/linux`
- Supports `--project-root`, `--output-prefab`, `--max-entities`, and `--no-lowercase-hints`

## Limitations

- Does not convert Gazebo material scripts to O3DE materials.
- Generates a first-pass prefab scaffold, not a final production-ready level.
- `world_pose_additive` in scene manifest uses simple xyz+rpy addition (good first pass, refine in editor).
- `assetHint` mapping is best effort; confirm final model bindings in O3DE Asset Browser if meshes are missing.
