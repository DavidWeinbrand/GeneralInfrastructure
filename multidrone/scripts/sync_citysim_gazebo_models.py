#!/usr/bin/env python3
"""Copy osrf/gazebo_models entries required by citysim (including nested refs)."""
from __future__ import annotations

import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
VENDOR = ROOT / "vendor" / "gazebo_models"
WORLD = ROOT / "vendor" / "citysim" / "worlds" / "simple_city.world"
UPSTREAM = "https://github.com/osrf/gazebo_models.git"

# Nested models referenced from other models (not always in the world file).
EXTRA = ("stop_light", "prius_hybrid")


def model_names_in_world(text: str) -> set[str]:
    return set(re.findall(r"model://([A-Za-z0-9_]+)", text))


def nested_models(model_dir: Path) -> set[str]:
    found: set[str] = set()
    for sdf in model_dir.glob("**/*.sdf"):
        try:
            text = sdf.read_text(encoding="utf-8", errors="ignore")
        except OSError:
            continue
        found.update(re.findall(r"model://([A-Za-z0-9_]+)", text))
    return found


def main() -> int:
    if not WORLD.is_file():
        print(f"missing {WORLD}", file=sys.stderr)
        return 1
    needed = model_names_in_world(WORLD.read_text(encoding="utf-8"))
    for name in list(needed):
        local = VENDOR / name
        if local.is_dir():
            needed.update(nested_models(local))
    needed.update(EXTRA)

    missing = sorted(n for n in needed if not (VENDOR / n).is_dir())
    if not missing:
        print("citysim gazebo_models: all dependencies present")
        return 0

    tmp = Path(tempfile.mkdtemp(prefix="gazebo_models_"))
    try:
        subprocess.run(
            ["git", "clone", "--depth", "1", UPSTREAM, str(tmp / "repo")],
            check=True,
            capture_output=True,
        )
        src_root = tmp / "repo"
        VENDOR.mkdir(parents=True, exist_ok=True)
        for name in missing:
            src = src_root / name
            if not src.is_dir():
                print(f"warning: upstream has no model {name}", file=sys.stderr)
                continue
            dest = VENDOR / name
            if dest.exists():
                shutil.rmtree(dest)
            shutil.copytree(src, dest)
            print(f"copied {name}")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
