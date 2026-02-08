#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

O3DE_VERSION="${O3DE_VERSION:-2510.2}"
CACHE_DIR="${CACHE_DIR:-$PROJECT_ROOT/docker/cache}"
FORCE_DOWNLOAD=0

usage() {
    cat <<EOF
Usage: $0 [--version X.Y.Z] [--cache-dir PATH] [--force]

Downloads O3DE Debian package to a host cache directory so Docker builds can
reuse it without repeated network downloads.

Options:
  --version X.Y.Z   O3DE version (default: $O3DE_VERSION)
  --cache-dir PATH  Cache directory (default: $CACHE_DIR)
  --force           Re-download even if file exists
  -h, --help        Show this help
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --version)
            O3DE_VERSION="$2"
            shift 2
            ;;
        --cache-dir)
            CACHE_DIR="$2"
            shift 2
            ;;
        --force)
            FORCE_DOWNLOAD=1
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown argument: $1" >&2
            usage
            exit 1
            ;;
    esac
done

mkdir -p "$CACHE_DIR"

IFS='.' read -r -a version_parts <<< "$O3DE_VERSION"
if [[ "${#version_parts[@]}" -eq 3 ]]; then
    deb_suffix="${version_parts[0]}${version_parts[1]}_${version_parts[2]}"
elif [[ "${#version_parts[@]}" -eq 2 ]]; then
    deb_suffix="${version_parts[0]}_${version_parts[1]}"
else
    deb_suffix="${O3DE_VERSION//./_}"
fi

deb_file="o3de_${deb_suffix}.deb"
deb_path="$CACHE_DIR/$deb_file"
sha_path="$deb_path.sha256"
deb_url="https://o3debinaries.org/main/Latest/Linux/$deb_file"
sha_url="${deb_url}.sha256"

if [[ -f "$deb_path" && "$FORCE_DOWNLOAD" -eq 0 ]]; then
    echo "Using existing cached package: $deb_path"
else
    echo "Downloading: $deb_url"
    curl -fL "$deb_url" -o "$deb_path"
fi

echo "Downloading checksum: $sha_url"
curl -fL "$sha_url" -o "$sha_path"

(
    cd "$CACHE_DIR"
    expected_sha="$(awk '{print $1}' "$(basename "$sha_path")")"
    echo "$expected_sha  $(basename "$deb_path")" | sha256sum -c -
)
echo "Checksum verified: $sha_path"

echo ""
echo "Cache ready: $deb_path"
echo "You can now run: docker build -t o3de-playground:latest -f docker/Dockerfile ."
