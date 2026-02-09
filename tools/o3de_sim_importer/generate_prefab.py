#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate an O3DE prefab scaffold from scene manifest."
    )
    parser.add_argument(
        "--scene-manifest", required=True, help="Path to o3de_scene_manifest.json"
    )
    parser.add_argument(
        "--output-prefab",
        default="",
        help="Output prefab path. Defaults to next to scene manifest as <world_name>_scaffold.prefab",
    )
    parser.add_argument(
        "--max-entities",
        type=int,
        default=0,
        help="Optional cap on number of entities to emit (0 means all)",
    )
    parser.add_argument(
        "--project-root",
        default="",
        help="Project root path containing Project/Assets and Project/Cache (auto-detected if omitted)",
    )
    parser.add_argument(
        "--no-lowercase-hints",
        action="store_true",
        help="Keep generated assetHint case (default lowercases for cache compatibility)",
    )
    return parser.parse_args()


def load_json(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def infer_project_root(scene_manifest_path: Path, explicit: str) -> Path:
    if explicit:
        return Path(explicit).expanduser().resolve()
    for parent in scene_manifest_path.parents:
        if parent.name == "Project":
            return parent.parent
    # Fallback for unexpected layouts.
    return (
        scene_manifest_path.parents[4]
        if len(scene_manifest_path.parents) > 4
        else scene_manifest_path.parent
    )


def normalize_asset_hint(mesh_uri: str, world_name: str) -> str:
    if mesh_uri.startswith("model://"):
        payload = mesh_uri[len("model://") :]
        parts = payload.split("/", 1)
        model_name = parts[0]
        rel = parts[1] if len(parts) == 2 else ""
        filename = Path(rel).name
        stem = Path(filename).stem
        return f"importedworlds/{world_name}/assets/{model_name}/{stem}.azmodel"
    filename = Path(mesh_uri).name
    return f"importedworlds/{world_name}/assets/external/{Path(filename).stem}.azmodel"


def build_uri_hint_map(
    world_manifest: dict,
    world_name: str,
    project_root: Path,
    lowercase_hints: bool,
) -> dict[str, str]:
    uri_to_hint: dict[str, str] = {}
    copied_assets = world_manifest.get("copied_assets", [])
    assets_prefix = (project_root / "Project" / "Assets").resolve()
    cache_root = (project_root / "Project" / "Cache" / "linux").resolve()

    for record in copied_assets:
        uri = record.get("uri", "")
        to_path = Path(record.get("to", ""))
        hint = normalize_asset_hint(uri, world_name)

        try:
            rel = to_path.resolve().relative_to(assets_prefix)
            hint = f"{rel.as_posix().rsplit('.', 1)[0]}.azmodel"
        except Exception:
            pass

        stem = to_path.stem
        if cache_root.exists() and stem:
            matches = list(cache_root.rglob(f"{stem}.azmodel"))
            if len(matches) == 1:
                hint = matches[0].relative_to(cache_root).as_posix()

        if lowercase_hints:
            hint = hint.lower()
        uri_to_hint[uri] = hint

    return uri_to_hint


def make_container_components(child_ids: list[str]) -> dict:
    return {
        "EditorDisabledCompositionComponent": {
            "$type": "EditorDisabledCompositionComponent",
            "Id": 9100000000000000001,
        },
        "EditorEntityIconComponent": {
            "$type": "EditorEntityIconComponent",
            "Id": 9100000000000000002,
        },
        "EditorEntitySortComponent": {
            "$type": "EditorEntitySortComponent",
            "Id": 9100000000000000003,
            "Child Entity Order": child_ids,
        },
        "EditorInspectorComponent": {
            "$type": "EditorInspectorComponent",
            "Id": 9100000000000000004,
        },
        "EditorLockComponent": {
            "$type": "EditorLockComponent",
            "Id": 9100000000000000005,
        },
        "EditorOnlyEntityComponent": {
            "$type": "EditorOnlyEntityComponent",
            "Id": 9100000000000000006,
        },
        "EditorPendingCompositionComponent": {
            "$type": "EditorPendingCompositionComponent",
            "Id": 9100000000000000007,
        },
        "EditorPrefabComponent": {
            "$type": "EditorPrefabComponent",
            "Id": 9100000000000000008,
        },
        "EditorVisibilityComponent": {
            "$type": "EditorVisibilityComponent",
            "Id": 9100000000000000009,
        },
        "TransformComponent": {
            "$type": "{27F1E1A1-8D9D-4C3B-BD3A-AFB9762449C0} TransformComponent",
            "Id": 9100000000000000010,
            "Parent Entity": "",
            "Transform Data": {"Translate": [0.0, 0.0, 0.0]},
        },
    }


def make_entity(
    index: int,
    candidate: dict,
    world_name: str,
    uri_to_hint: dict[str, str],
    lowercase_hints: bool,
) -> tuple[str, dict]:
    entity_id = f"Entity_[950000000000{index:04d}]"
    pose = candidate.get("world_pose_additive", {})
    tx = float(pose.get("x", 0.0))
    ty = float(pose.get("y", 0.0))
    tz = float(pose.get("z", 0.0))
    roll_deg = math.degrees(float(pose.get("roll", 0.0)))
    pitch_deg = math.degrees(float(pose.get("pitch", 0.0)))
    yaw_deg = math.degrees(float(pose.get("yaw", 0.0)))

    mesh_uri = candidate.get("mesh_uri", "")
    asset_hint = uri_to_hint.get(mesh_uri, normalize_asset_hint(mesh_uri, world_name))
    if lowercase_hints:
        asset_hint = asset_hint.lower()

    base = 9200000000000000000 + index * 100
    entity = {
        "Id": entity_id,
        "Name": candidate.get("entity_name", f"Imported_{index}"),
        "Components": {
            f"Component_[{base + 1}]": {
                "$type": "{27F1E1A1-8D9D-4C3B-BD3A-AFB9762449C0} TransformComponent",
                "Id": base + 1,
                "Parent Entity": "ContainerEntity",
                "Transform Data": {
                    "Translate": [tx, ty, tz],
                    "Rotate": [roll_deg, pitch_deg, yaw_deg],
                },
            },
            f"Component_[{base + 2}]": {
                "$type": "AZ::Render::EditorMeshComponent",
                "Id": base + 2,
                "Controller": {
                    "Configuration": {
                        "ModelAsset": {
                            "assetId": {
                                "guid": "{00000000-0000-0000-0000-000000000000}",
                                "subId": 0,
                            },
                            "assetHint": asset_hint,
                        }
                    }
                },
            },
            f"Component_[{base + 3}]": {
                "$type": "EditorInspectorComponent",
                "Id": base + 3,
            },
            f"Component_[{base + 4}]": {
                "$type": "EditorEntityIconComponent",
                "Id": base + 4,
            },
            f"Component_[{base + 5}]": {
                "$type": "EditorDisabledCompositionComponent",
                "Id": base + 5,
            },
            f"Component_[{base + 6}]": {
                "$type": "EditorOnlyEntityComponent",
                "Id": base + 6,
            },
            f"Component_[{base + 7}]": {"$type": "EditorLockComponent", "Id": base + 7},
            f"Component_[{base + 8}]": {
                "$type": "EditorVisibilityComponent",
                "Id": base + 8,
            },
            f"Component_[{base + 9}]": {
                "$type": "EditorPendingCompositionComponent",
                "Id": base + 9,
            },
        },
    }
    return entity_id, entity


def main() -> int:
    args = parse_args()
    scene_manifest_path = Path(args.scene_manifest).expanduser().resolve()
    scene_manifest = load_json(scene_manifest_path)
    world_manifest_path = scene_manifest_path.parent / "world_manifest.json"
    world_manifest = (
        load_json(world_manifest_path) if world_manifest_path.exists() else {}
    )

    world_name = scene_manifest.get("world_name", "imported_world")
    lowercase_hints = not args.no_lowercase_hints
    project_root = infer_project_root(scene_manifest_path, args.project_root)

    if args.output_prefab:
        output_prefab = Path(args.output_prefab).expanduser().resolve()
    else:
        output_prefab = scene_manifest_path.parent / f"{world_name}_scaffold.prefab"

    candidates: list[dict] = scene_manifest.get("entity_candidates", [])
    if args.max_entities > 0:
        candidates = candidates[: args.max_entities]

    uri_to_hint = build_uri_hint_map(
        world_manifest, world_name, project_root, lowercase_hints
    )

    entities = {}
    child_ids: list[str] = []
    for i, candidate in enumerate(candidates, start=1):
        eid, entity = make_entity(
            i, candidate, world_name, uri_to_hint, lowercase_hints
        )
        child_ids.append(eid)
        entities[eid] = entity

    prefab = {
        "ContainerEntity": {
            "Id": "ContainerEntity",
            "Name": f"{world_name}_scaffold",
            "Components": make_container_components(child_ids),
        },
        "Entities": entities,
    }

    output_prefab.parent.mkdir(parents=True, exist_ok=True)
    output_prefab.write_text(json.dumps(prefab, indent=4) + "\n", encoding="utf-8")

    print(f"[OK] Prefab scaffold generated: {output_prefab}")
    print(f"[OK] Entities emitted: {len(candidates)}")
    print("[OK] Rotation applied from scene manifest (radians -> degrees).")
    print(
        "[WARN] Verify assetHint matches AP output in O3DE Asset Browser if any mesh is missing."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
