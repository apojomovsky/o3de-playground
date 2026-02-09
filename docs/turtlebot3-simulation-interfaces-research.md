# TurtleBot3 + O3DE: `simulation_interfaces` Research Notes

Date: 2026-02-08

## Scope

This note captures findings about whether the ROS 2 `simulation_interfaces` standard is useful for our TurtleBot3 migration work (robot + environment) in O3DE.

## Short Answer

- `simulation_interfaces` is real, released, and adopted in multiple simulators.
- It is useful for **runtime control and automation**.
- It does **not** solve robot/world asset ingestion by itself.

For our goals, it is worth adopting **after** the import/conversion pipeline is working.

## What the Standard Is

Authoritative package:

- Repo: `ros-simulation/simulation_interfaces`
- URL: <https://github.com/ros-simulation/simulation_interfaces>
- ROS Index: <https://index.ros.org/p/simulation_interfaces/>

The package defines ROS 2 messages/services/actions for simulator control, including:

- Entity lifecycle: `SpawnEntity`, `DeleteEntity`
- Simulation control: `Get/SetSimulationState`, `ResetSimulation`, `StepSimulation`
- Entity queries/state: `GetEntities`, `GetEntityState`, `SetEntityState`
- Capability discovery: `GetSimulatorFeatures`
- World management (newer versions): `LoadWorld`, `UnloadWorld`, `GetAvailableWorlds`, `GetCurrentWorld`

## O3DE-Specific Status

O3DE includes a dedicated SimulationInterfaces Gem:

- Docs: <https://docs.o3de.org/docs/user-guide/interactivity/robotics/simulation-interfaces>

Key point:

- This Gem integrates O3DE with ROS 2 `simulation_interfaces` for runtime communication/control.
- It does not replace robot/world import tooling.

## Adoption in Other Simulators (Evidence)

- Gazebo support/tutorial:
  - <https://gazebosim.org/docs/latest/ros2_sim_interfaces/>
- Gazebo implementation code (`ros_gz_sim`):
  - <https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_sim/src/gz_simulation_interfaces>
- Isaac Sim tutorial/docs:
  - <https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_simulation_control.html>
- Isaac Sim implementation source:
  - <https://github.com/isaac-sim/IsaacSim/blob/main/source/extensions/isaacsim.ros2.sim_control/python/impl/simulation_control.py>

## What We Gain (for TurtleBot3)

If we adopt this after import:

- One ROS 2 control surface for simulation orchestration across simulators
- Better CI/test automation (spawn/reset/step/state checks)
- Less simulator-specific orchestration code

Examples of practical use:

- Spawn TurtleBot3 in test scenes at known poses
- Reset simulation between test cases
- Step simulation deterministically for reproducible checks
- Query ground-truth state directly for assertions

## What It Does NOT Give Us

- No direct import of Gazebo `.world` files into O3DE
- No automatic URDF/SDF conversion by itself
- No automatic transfer of Gazebo materials/plugins/world semantics

So for TurtleBot3 migration we still need:

1. Robot import path (URDF/Xacro -> O3DE assets/prefab/spawnable)
2. World conversion/rebuild path (Gazebo SDF/world -> O3DE level/assets)

## Recommendation for Our Project

Pursue in this order:

1. Complete TurtleBot3 robot import and world conversion in O3DE.
2. Add SimulationInterfaces Gem and validate base services in GameLauncher.
3. Build a small ROS 2 orchestration layer around `GetSimulatorFeatures`, `SpawnEntity`, `ResetSimulation`, `StepSimulation`, and `GetEntityState`.

This gives immediate practical value without blocking current migration work.

## Original O3DE 25.05 Claim Context

Release post quote source:

- <https://o3de.org/o3de-25-05-0-release-june-18-2025/>

Based on available sources, the claim is directionally accurate: O3DE ships support and this standard is being used across major simulators, but adoption depth should always be validated per simulator version via `GetSimulatorFeatures`.
