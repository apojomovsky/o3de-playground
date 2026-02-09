#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate generated importer manifests and URDF references."
    )
    parser.add_argument(
        "--manifest", required=True, help="Path to import manifest JSON"
    )
    parser.add_argument(
        "--strict", action="store_true", help="Fail when unresolved references exist"
    )
    return parser.parse_args()


def load_manifest(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def validate_robot_manifest(manifest: dict, strict: bool) -> int:
    errors = 0
    unresolved = manifest.get("unresolved_mesh_uris", [])
    copied_assets = manifest.get("copied_assets", [])
    output_urdf = Path(manifest["output_urdf"])

    if not output_urdf.exists():
        print(f"[ERROR] Output URDF missing: {output_urdf}")
        return 1

    for record in copied_assets:
        dst = Path(record["to"])
        if not dst.exists():
            print(f"[ERROR] Copied asset missing: {dst}")
            errors += 1

    try:
        tree = ET.parse(output_urdf)
    except ET.ParseError as exc:
        print(f"[ERROR] Output URDF XML parse failed: {exc}")
        return 1

    for mesh in tree.getroot().iter("mesh"):
        filename = (mesh.get("filename") or "").strip()
        if not filename:
            continue
        candidate = (output_urdf.parent / filename).resolve()
        if not candidate.exists():
            print(f"[ERROR] URDF mesh reference not found: {filename}")
            errors += 1

    if unresolved:
        print(f"[WARN] Unresolved mesh URIs: {len(unresolved)}")
        if strict:
            errors += len(unresolved)

    if errors == 0:
        print("[OK] Robot manifest validation passed")
    return 0 if errors == 0 else 1


def validate_world_manifest(manifest: dict, strict: bool) -> int:
    errors = 0
    copied_assets = manifest.get("copied_assets", [])
    unresolved = manifest.get("unresolved", [])

    for record in copied_assets:
        dst = Path(record["to"])
        if not dst.exists():
            print(f"[ERROR] Copied world asset missing: {dst}")
            errors += 1

    if unresolved:
        print(f"[WARN] Unresolved world references: {len(unresolved)}")
        if strict:
            errors += len(unresolved)

    if errors == 0:
        print("[OK] World manifest validation passed")
    return 0 if errors == 0 else 1


def main() -> int:
    args = parse_args()
    manifest_path = Path(args.manifest).expanduser().resolve()
    if not manifest_path.exists():
        print(f"[ERROR] Manifest not found: {manifest_path}", file=sys.stderr)
        return 1

    manifest = load_manifest(manifest_path)
    if "output_urdf" in manifest:
        return validate_robot_manifest(manifest, args.strict)
    if "input_world" in manifest:
        return validate_world_manifest(manifest, args.strict)

    print("[ERROR] Unknown manifest type", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
