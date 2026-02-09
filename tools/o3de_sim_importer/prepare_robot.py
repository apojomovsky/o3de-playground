#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

from common import (
    copy_asset,
    make_relative,
    parse_package_map,
    resolve_uri_to_source_path,
    write_json,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Prepare URDF assets for O3DE Robot Importer."
    )
    parser.add_argument("--urdf", required=True, help="Path to input URDF file")
    parser.add_argument(
        "--output-root",
        default="Project/Assets/ImportedRobots",
        help="Root output directory for prepared assets",
    )
    parser.add_argument("--robot-name", default="", help="Output robot folder name")
    parser.add_argument(
        "--workspace-root",
        action="append",
        default=[],
        help="Workspace roots to search package.xml in (repeatable)",
    )
    parser.add_argument(
        "--package-map",
        action="append",
        default=[],
        help="Explicit package mapping name=/path (repeatable)",
    )
    return parser.parse_args()


def normalize_robot_name(urdf_path: Path, override: str) -> str:
    if override:
        return override
    stem = urdf_path.stem
    return stem.replace(".", "_")


def main() -> int:
    args = parse_args()

    urdf_path = Path(args.urdf).expanduser().resolve()
    if not urdf_path.exists():
        print(f"[ERROR] URDF file not found: {urdf_path}", file=sys.stderr)
        return 1

    robot_name = normalize_robot_name(urdf_path, args.robot_name)
    output_root = Path(args.output_root).expanduser().resolve()
    robot_root = output_root / robot_name
    out_urdf_dir = robot_root / "urdf"
    out_assets_dir = robot_root / "assets"
    out_urdf_path = out_urdf_dir / f"{robot_name}_import_ready.urdf"
    out_manifest_path = robot_root / "import_manifest.json"

    workspace_roots = [Path(x).expanduser().resolve() for x in args.workspace_root]
    package_map = parse_package_map(args.package_map)
    package_cache: dict[str, Path | None] = {}

    tree = ET.parse(urdf_path)
    root = tree.getroot()

    rewritten = 0
    unresolved: list[dict[str, str]] = []
    copied: list[dict[str, str]] = []

    for mesh in root.iter("mesh"):
        filename = mesh.get("filename", "").strip()
        if not filename:
            continue

        source = resolve_uri_to_source_path(
            filename,
            urdf_path.parent,
            package_map,
            workspace_roots,
            package_cache,
        )
        if source is None or not source.exists():
            unresolved.append({"uri": filename})
            continue

        if filename.startswith("package://"):
            payload = filename[len("package://") :]
            parts = payload.split("/", 1)
            package_name = parts[0]
            rel = Path(parts[1]) if len(parts) == 2 else Path(source.name)
            destination = out_assets_dir / package_name / rel
        else:
            destination = out_assets_dir / "external" / source.name

        copy_asset(source, destination)
        rel_for_urdf = make_relative(destination, out_urdf_dir)
        mesh.set("filename", rel_for_urdf)
        rewritten += 1
        copied.append({"from": str(source), "to": str(destination), "uri": filename})

    out_urdf_dir.mkdir(parents=True, exist_ok=True)
    tree.write(out_urdf_path, encoding="utf-8", xml_declaration=True)

    manifest = {
        "input_urdf": str(urdf_path),
        "output_urdf": str(out_urdf_path),
        "rewritten_mesh_uris": rewritten,
        "unresolved_mesh_uris": unresolved,
        "copied_assets": copied,
        "workspace_roots": [str(x) for x in workspace_roots],
        "package_map": {k: str(v) for k, v in package_map.items()},
    }
    write_json(out_manifest_path, manifest)

    print(f"[OK] Prepared robot assets under: {robot_root}")
    print(f"[OK] Import-ready URDF: {out_urdf_path}")
    print(f"[OK] Rewritten mesh URIs: {rewritten}")
    if unresolved:
        print(f"[WARN] Unresolved mesh URIs: {len(unresolved)}")
    print(f"[OK] Manifest: {out_manifest_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
