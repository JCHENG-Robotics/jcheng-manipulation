#!/usr/bin/env python3
"""Pin the private vendor workspace without publishing its source.

The vendor workspace is gitignored and may not be redistributed. This tool
records a SHA-256 tree digest plus per-package metadata so a machine that has
the private workspace (or its archived tarball) can prove it is the same
baseline used for the hardware evidence.

Usage:
    python3 tools/vendor_baseline_manifest.py --write
    python3 tools/vendor_baseline_manifest.py --check
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE = REPO_ROOT / "armpi_ultra_ws_clean" / "src"
DEFAULT_ARCHIVE = (
    Path("~/backups/2026-11-16/armpi_ultra_ws_clean_20261115.tar.gz")
    .expanduser()
)
DEFAULT_MANIFEST = (
    REPO_ROOT / "docs" / "hardware" / "vendor_baseline_manifest.json"
)
EXCLUDED_DIRS = {
    ".git",
    "__pycache__",
    ".pytest_cache",
    "build",
    "install",
    "log",
}


def _iter_files(root: Path):
    for path in sorted(root.rglob("*")):
        if not path.is_file():
            continue
        relative = path.relative_to(root)
        if any(part in EXCLUDED_DIRS for part in relative.parts):
            continue
        yield path, relative


def tree_digest(root: Path) -> tuple[str, int, int]:
    """Return (digest, file count, byte count) for the source tree."""
    digest = hashlib.sha256()
    files = 0
    total_bytes = 0
    for path, relative in _iter_files(root):
        data = path.read_bytes()
        digest.update(relative.as_posix().encode("utf-8"))
        digest.update(b"\0")
        digest.update(hashlib.sha256(data).digest())
        files += 1
        total_bytes += len(data)
    return digest.hexdigest(), files, total_bytes


def _package_records(root: Path) -> list[dict[str, str]]:
    records = []
    for path in sorted(root.rglob("package.xml")):
        relative = path.relative_to(root)
        if any(part in EXCLUDED_DIRS for part in relative.parts):
            continue
        tree = ET.parse(path)
        package = tree.getroot()
        records.append(
            {
                "path": relative.as_posix(),
                "name": package.findtext("name", default=""),
                "version": package.findtext("version", default=""),
                "package_xml_sha256": hashlib.sha256(
                    path.read_bytes()
                ).hexdigest(),
            }
        )
    return records


def build_manifest(source: Path, archive: Path) -> dict:
    if not source.is_dir():
        raise FileNotFoundError(f"vendor source not found: {source}")
    digest, files, total_bytes = tree_digest(source)
    manifest = {
        "schema_version": 1,
        "source_root": "armpi_ultra_ws_clean/src",
        "source_tree": {
            "sha256": digest,
            "files": files,
            "bytes": total_bytes,
        },
        "packages": _package_records(source),
        "archive": {
            "path": str(archive),
            "sha256": (
                hashlib.sha256(archive.read_bytes()).hexdigest()
                if archive.is_file()
                else None
            ),
        },
        "note": (
            "Hashes only; the vendor source is private and stays out of the "
            "public repository."
        ),
    }
    return manifest


def check_manifest(manifest_path: Path, source: Path, archive: Path) -> int:
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    expected = manifest["source_tree"]
    problems = []
    if source.is_dir():
        digest, files, total_bytes = tree_digest(source)
        if (
            digest != expected["sha256"]
            or files != expected["files"]
            or total_bytes != expected["bytes"]
        ):
            problems.append(
                "vendor source tree differs: "
                f"sha256 {digest} files {files} bytes {total_bytes}; "
                f"expected {expected['sha256']} "
                f"{expected['files']} {expected['bytes']}"
            )
        else:
            print(f"vendor source OK: {digest} ({files} files)")
    else:
        print(f"vendor source not present, skipped: {source}")

    expected_archive = manifest.get("archive") or {}
    archive_sha = expected_archive.get("sha256")
    if archive.is_file() and archive_sha:
        actual = hashlib.sha256(archive.read_bytes()).hexdigest()
        if actual != archive_sha:
            problems.append(
                f"vendor archive differs: {actual}; expected {archive_sha}"
            )
        else:
            print(f"vendor archive OK: {actual}")
    else:
        print(f"vendor archive not present, skipped: {archive}")

    for problem in problems:
        print(f"ERROR: {problem}", file=sys.stderr)
    return 1 if problems else 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--write", action="store_true", help="write the manifest"
    )
    parser.add_argument(
        "--check", action="store_true", help="verify the manifest"
    )
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--archive", type=Path, default=DEFAULT_ARCHIVE)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    args = parser.parse_args(argv)
    if args.write == args.check:
        parser.error("pass exactly one of --write or --check")
    if args.write:
        manifest = build_manifest(args.source, args.archive)
        args.manifest.parent.mkdir(parents=True, exist_ok=True)
        args.manifest.write_text(
            json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
        )
        print(f"wrote {args.manifest}")
        return 0
    return check_manifest(args.manifest, args.source, args.archive)


if __name__ == "__main__":
    raise SystemExit(main())
