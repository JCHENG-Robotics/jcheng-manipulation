#!/usr/bin/env python3
"""Check that the release manifest's evidence paths exist and have a README.

The manifest is the public index of what was actually run. A missing folder
or README means a released claim has no traceable artifact.
"""

from __future__ import annotations

import json
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
MANIFEST = REPO_ROOT / "release" / "manifest.json"


def _check_path(relative: str) -> list[str]:
    path = REPO_ROOT / relative
    problems = []
    if not path.is_dir():
        problems.append(f"missing evidence directory: {relative}")
    elif not (path / "README.md").is_file():
        problems.append(f"missing evidence README: {relative}/README.md")
    return problems


def main() -> int:
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    paths = list(manifest.get("hardware_evidence", []))
    paths.extend(manifest.get("build_evidence", []))
    paths.extend(manifest.get("ops_evidence", []))
    acceptance = manifest.get("post_rc1_acceptance", {})
    if acceptance.get("evidence"):
        paths.append(acceptance["evidence"])
    build = acceptance.get("reproducible_build", {})
    if build.get("evidence"):
        paths.append(build["evidence"])
    operations = acceptance.get("operations", {})
    if operations.get("evidence"):
        paths.append(operations["evidence"])

    problems = []
    for relative in sorted(set(paths)):
        problems.extend(_check_path(relative))

    if problems:
        for problem in problems:
            print(f"ERROR: {problem}")
        return 1
    print(f"EVIDENCE OK: {len(set(paths))} manifest paths")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
