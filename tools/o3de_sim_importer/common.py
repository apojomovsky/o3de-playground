from __future__ import annotations

import json
import os
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path


def parse_package_map(entries: list[str]) -> dict[str, Path]:
    result: dict[str, Path] = {}
    for entry in entries:
        if "=" not in entry:
            raise ValueError(f"Invalid --package-map '{entry}', expected name=/path")
        name, raw_path = entry.split("=", 1)
        path = Path(raw_path).expanduser().resolve()
        result[name.strip()] = path
    return result


def find_package_path(
    package_name: str,
    package_map: dict[str, Path],
    workspace_roots: list[Path],
    cache: dict[str, Path | None],
) -> Path | None:
    if package_name in cache:
        return cache[package_name]

    mapped = package_map.get(package_name)
    if mapped and mapped.exists():
        cache[package_name] = mapped
        return mapped

    for root in workspace_roots:
        candidate = root / package_name
        if (candidate / "package.xml").exists():
            cache[package_name] = candidate
            return candidate

    for root in workspace_roots:
        for package_xml in root.rglob("package.xml"):
            try:
                tree = ET.parse(package_xml)
                pkg_name = tree.getroot().findtext("name", default="").strip()
            except ET.ParseError:
                continue
            if pkg_name == package_name:
                cache[package_name] = package_xml.parent
                return package_xml.parent

    cache[package_name] = None
    return None


def resolve_uri_to_source_path(
    uri: str,
    base_dir: Path,
    package_map: dict[str, Path],
    workspace_roots: list[Path],
    package_cache: dict[str, Path | None],
) -> Path | None:
    if uri.startswith("package://"):
        payload = uri[len("package://") :]
        parts = payload.split("/", 1)
        package_name = parts[0]
        rel = parts[1] if len(parts) == 2 else ""
        package_path = find_package_path(
            package_name, package_map, workspace_roots, package_cache
        )
        if package_path is None:
            return None
        return (package_path / rel).resolve()

    if uri.startswith("file://"):
        return Path(uri[len("file://") :]).expanduser().resolve()

    source = Path(uri)
    if source.is_absolute():
        return source.resolve()

    return (base_dir / source).resolve()


def copy_asset(source_path: Path, destination_path: Path) -> None:
    destination_path.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source_path, destination_path)


def write_json(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def make_relative(target: Path, origin_dir: Path) -> str:
    return os.path.relpath(target, origin_dir).replace("\\", "/")
