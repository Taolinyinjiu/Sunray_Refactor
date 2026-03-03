#!/usr/bin/env python3
"""Merge multiple compile_commands.json files into one.

Usage:
  1) Auto-scan build subdirectories (default):
     ./tools/code_intel/merge_compile_commands.py

  2) Merge specific build subdirectories or json files:
     ./tools/code_intel/merge_compile_commands.py uav_control px4_bridge
     ./tools/code_intel/merge_compile_commands.py build/uav_control/compile_commands.json

Options:
  -o, --output <path>    Output file path (default: ./compile_commands.json)
  -b, --build-dir <path> Build directory root for subdir auto-discovery
                         (default: ./build)
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Iterable, List


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Merge compile_commands.json files")
    parser.add_argument(
        "inputs",
        nargs="*",
        help=(
            "Build subdirectory name (e.g. uav_control), a directory path, "
            "or a compile_commands.json path. If empty, auto-scan build dir."
        ),
    )
    parser.add_argument(
        "-o",
        "--output",
        default="compile_commands.json",
        help="Output path for merged compile_commands.json",
    )
    parser.add_argument(
        "-b",
        "--build-dir",
        default="build",
        help="Build root dir for default scan and subdir-name resolution",
    )
    return parser.parse_args()


def _discover_default(build_dir: Path) -> List[Path]:
    if not build_dir.exists():
        return []
    return sorted(p for p in build_dir.glob("*/compile_commands.json") if p.is_file())


def _resolve_inputs(raw_inputs: Iterable[str], build_dir: Path) -> List[Path]:
    resolved: List[Path] = []
    for item in raw_inputs:
        p = Path(item)
        candidates: List[Path] = []

        if p.is_file():
            candidates.append(p)
        elif p.is_dir():
            candidates.append(p / "compile_commands.json")
        else:
            # Treat as build subdir name under build/
            candidates.append(build_dir / item / "compile_commands.json")
            # Also support explicit path-like input that may not exist yet in cwd
            candidates.append(p / "compile_commands.json")

        found = next((c for c in candidates if c.is_file()), None)
        if found is None:
            raise FileNotFoundError(
                f"Cannot resolve compile_commands.json from input '{item}'"
            )
        resolved.append(found)

    # Stable de-dup while preserving order
    unique: List[Path] = []
    seen = set()
    for p in resolved:
        rp = p.resolve()
        if rp not in seen:
            unique.append(rp)
            seen.add(rp)
    return unique


def _load_entries(path: Path) -> List[dict]:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise ValueError(f"Invalid JSON in '{path}': {exc}") from exc
    if not isinstance(data, list):
        raise ValueError(f"Expected a JSON array in '{path}'")
    out = []
    for idx, entry in enumerate(data):
        if not isinstance(entry, dict):
            raise ValueError(f"Entry #{idx} in '{path}' is not a JSON object")
        out.append(entry)
    return out


def main() -> int:
    args = _parse_args()
    build_dir = Path(args.build_dir).resolve()
    output = Path(args.output).resolve()

    if args.inputs:
        inputs = _resolve_inputs(args.inputs, build_dir)
    else:
        inputs = _discover_default(build_dir)

    if not inputs:
        print(
            f"No compile_commands.json found. "
            f"Build dir checked: {build_dir}",
            file=sys.stderr,
        )
        return 1

    merged: List[dict] = []
    for path in inputs:
        merged.extend(_load_entries(path))

    # De-dup by (file, directory) so the latest source wins if repeated.
    deduped = {}
    for entry in merged:
        key = (entry.get("file"), entry.get("directory"))
        deduped[key] = entry
    result = list(deduped.values())

    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    print(f"Merged {len(inputs)} files -> {output}")
    for p in inputs:
        print(f"  - {p}")
    print(f"Total entries: {len(result)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
