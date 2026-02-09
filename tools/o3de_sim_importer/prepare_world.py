#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

from common import copy_asset, parse_package_map, write_json


DEFAULT_O3DE_REPLACEMENT_MODELS = {
    "ground_plane": "Use project default level ground plane",
    "sun": "Use project default level primary light",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Extract Gazebo world assets into O3DE-friendly folder."
    )
    parser.add_argument(
        "--world-file", required=True, help="Path to input Gazebo .world file"
    )
    parser.add_argument(
        "--output-root",
        default="Project/Assets/ImportedWorlds",
        help="Output root for extracted world assets",
    )
    parser.add_argument("--world-name", default="", help="Output world folder name")
    parser.add_argument(
        "--models-root",
        action="append",
        default=[],
        help="Directories containing Gazebo model folders (repeatable)",
    )
    parser.add_argument(
        "--gazebo-models-root",
        action="append",
        default=[],
        help="Additional Gazebo model directories (repeatable)",
    )
    parser.add_argument(
        "--package-map",
        action="append",
        default=[],
        help="Reserved for future package:// world references (name=/path)",
    )
    return parser.parse_args()


def parse_pose(text: str | None) -> dict[str, float]:
    if not text:
        return {"x": 0.0, "y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}
    parts = [p for p in text.split() if p]
    vals = [float(p) for p in parts]
    while len(vals) < 6:
        vals.append(0.0)
    return {
        "x": vals[0],
        "y": vals[1],
        "z": vals[2],
        "roll": vals[3],
        "pitch": vals[4],
        "yaw": vals[5],
    }


def add_pose(a: dict[str, float], b: dict[str, float]) -> dict[str, float]:
    return {
        "x": a["x"] + b["x"],
        "y": a["y"] + b["y"],
        "z": a["z"] + b["z"],
        "roll": a["roll"] + b["roll"],
        "pitch": a["pitch"] + b["pitch"],
        "yaw": a["yaw"] + b["yaw"],
    }


def discover_default_model_roots(world_file: Path) -> list[Path]:
    roots: list[Path] = []
    roots.append(world_file.parent.parent / "models")
    roots.append(Path.home() / ".gazebo" / "models")

    for pattern in [
        "/usr/share/gazebo-*/models",
        "/usr/share/gz-*/models",
        "/usr/share/gz/models",
    ]:
        for path in Path("/").glob(pattern.lstrip("/")):
            roots.append(path)

    for env_name in [
        "GAZEBO_MODEL_PATH",
        "IGN_GAZEBO_RESOURCE_PATH",
        "GZ_SIM_RESOURCE_PATH",
    ]:
        raw = os.environ.get(env_name, "")
        for entry in raw.split(":"):
            if entry:
                roots.append(Path(entry).expanduser())

    unique: list[Path] = []
    seen = set()
    for root in roots:
        resolved = root.expanduser().resolve()
        if resolved in seen:
            continue
        seen.add(resolved)
        unique.append(resolved)
    return unique


def find_model_dir(model_name: str, roots: list[Path]) -> Path | None:
    for root in roots:
        candidate = root / model_name
        if (candidate / "model.sdf").exists():
            return candidate
    return None


def resolve_model_uri(
    uri: str, model_dir: Path, model_roots: list[Path]
) -> Path | None:
    if uri.startswith("model://"):
        payload = uri[len("model://") :]
        parts = payload.split("/", 1)
        model_name = parts[0]
        rel = Path(parts[1]) if len(parts) == 2 else Path("")
        target_model_dir = find_model_dir(model_name, model_roots)
        if target_model_dir is None:
            return None
        return (target_model_dir / rel).resolve()
    if uri.startswith("file://"):
        return Path(uri[len("file://") :]).expanduser().resolve()
    return (model_dir / uri).resolve()


def parse_model_visuals(model_sdf: Path) -> list[dict]:
    tree = ET.parse(model_sdf)
    root = tree.getroot()
    model = root.find(".//model")
    if model is None:
        return []

    visuals: list[dict] = []
    for link in model.findall("link"):
        link_name = link.get("name", "")
        link_pose = parse_pose(link.findtext("pose", default=""))

        for visual in link.findall("visual"):
            visual_name = visual.get("name", "")
            visual_pose = parse_pose(visual.findtext("pose", default=""))
            mesh_uri = visual.findtext("geometry/mesh/uri", default="").strip()
            if not mesh_uri:
                continue
            scale_raw = visual.findtext("geometry/mesh/scale", default="1 1 1")
            scale_parts = [p for p in scale_raw.split() if p]
            while len(scale_parts) < 3:
                scale_parts.append("1")
            scale = {
                "x": float(scale_parts[0]),
                "y": float(scale_parts[1]),
                "z": float(scale_parts[2]),
            }

            visuals.append(
                {
                    "link_name": link_name,
                    "visual_name": visual_name,
                    "mesh_uri": mesh_uri,
                    "link_pose": link_pose,
                    "visual_pose": visual_pose,
                    "scale": scale,
                }
            )
    return visuals


def main() -> int:
    args = parse_args()

    world_file = Path(args.world_file).expanduser().resolve()
    if not world_file.exists():
        print(f"[ERROR] World file not found: {world_file}", file=sys.stderr)
        return 1

    world_name = args.world_name or world_file.stem
    output_root = Path(args.output_root).expanduser().resolve()
    world_root = output_root / world_name
    assets_root = world_root / "assets"
    manifest_path = world_root / "world_manifest.json"
    scene_manifest_path = world_root / "o3de_scene_manifest.json"

    _package_map = parse_package_map(args.package_map)

    model_roots = [Path(x).expanduser().resolve() for x in args.models_root]
    model_roots.extend(Path(x).expanduser().resolve() for x in args.gazebo_models_root)
    model_roots.extend(discover_default_model_roots(world_file))

    # De-dup roots while preserving order.
    deduped_roots: list[Path] = []
    seen_roots = set()
    for root in model_roots:
        if root in seen_roots:
            continue
        seen_roots.add(root)
        deduped_roots.append(root)
    model_roots = deduped_roots

    world_tree = ET.parse(world_file)
    world_xml = world_tree.getroot()

    include_records: list[dict] = []
    for include in world_xml.findall(".//include"):
        uri_node = include.find("uri")
        if uri_node is None or not uri_node.text:
            continue
        include_records.append(
            {
                "uri": uri_node.text.strip(),
                "name": include.findtext("name", default="").strip(),
                "pose": parse_pose(include.findtext("pose", default="")),
            }
        )

    copied: list[dict[str, str]] = []
    unresolved: list[dict[str, str]] = []
    discovered_models: list[str] = []
    scene_entities: list[dict] = []

    for include_record in include_records:
        include_uri = include_record["uri"]
        include_pose = include_record["pose"]
        include_name = include_record["name"]

        if not include_uri.startswith("model://"):
            unresolved.append(
                {"include": include_uri, "reason": "unsupported include URI"}
            )
            continue

        model_name = include_uri[len("model://") :].strip("/")
        if model_name in DEFAULT_O3DE_REPLACEMENT_MODELS:
            discovered_models.append(model_name)
            continue

        model_dir = find_model_dir(model_name, model_roots)
        if model_dir is None:
            unresolved.append(
                {"include": include_uri, "reason": "model directory not found"}
            )
            continue

        discovered_models.append(model_name)
        model_sdf = model_dir / "model.sdf"
        if not model_sdf.exists():
            unresolved.append({"include": include_uri, "reason": "model.sdf missing"})
            continue

        visuals = parse_model_visuals(model_sdf)
        seen_mesh_uris: set[str] = set()
        for visual in visuals:
            mesh_uri = visual["mesh_uri"]
            if mesh_uri in seen_mesh_uris:
                pass
            else:
                source = resolve_model_uri(mesh_uri, model_dir, model_roots)
                if source is None:
                    unresolved.append(
                        {"mesh_uri": mesh_uri, "reason": "model:// root not found"}
                    )
                elif not source.exists():
                    unresolved.append(
                        {"mesh_uri": mesh_uri, "reason": "source mesh not found"}
                    )
                else:
                    destination = assets_root / model_name / source.name
                    copy_asset(source, destination)
                    copied.append(
                        {"from": str(source), "to": str(destination), "uri": mesh_uri}
                    )
                seen_mesh_uris.add(mesh_uri)

            world_pose = add_pose(
                include_pose, add_pose(visual["link_pose"], visual["visual_pose"])
            )
            scene_entities.append(
                {
                    "entity_name": include_name
                    or f"{model_name}_{visual['link_name']}_{visual['visual_name']}",
                    "model_name": model_name,
                    "mesh_uri": mesh_uri,
                    "link_name": visual["link_name"],
                    "visual_name": visual["visual_name"],
                    "include_pose": include_pose,
                    "link_pose": visual["link_pose"],
                    "visual_pose": visual["visual_pose"],
                    "world_pose_additive": world_pose,
                    "scale": visual["scale"],
                }
            )

    manifest = {
        "input_world": str(world_file),
        "output_root": str(world_root),
        "models_roots": [str(x) for x in model_roots],
        "included_models": sorted(set(discovered_models)),
        "replaced_models": DEFAULT_O3DE_REPLACEMENT_MODELS,
        "includes": include_records,
        "copied_assets": copied,
        "unresolved": unresolved,
        "next_step": "Use o3de_scene_manifest.json for first-pass entity placement in O3DE.",
    }
    write_json(manifest_path, manifest)

    scene_manifest = {
        "input_world": str(world_file),
        "world_name": world_name,
        "entity_candidates": scene_entities,
        "note": "world_pose_additive is simple xyz+rpy addition, good for first pass; refine in O3DE Editor.",
    }
    write_json(scene_manifest_path, scene_manifest)

    print(f"[OK] World extraction output: {world_root}")
    print(f"[OK] Included models discovered: {len(set(discovered_models))}")
    print(f"[OK] Copied assets: {len(copied)}")
    print(f"[OK] Scene entity candidates: {len(scene_entities)}")
    if unresolved:
        print(f"[WARN] Unresolved references: {len(unresolved)}")
    print(f"[OK] Manifest: {manifest_path}")
    print(f"[OK] Scene manifest: {scene_manifest_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
