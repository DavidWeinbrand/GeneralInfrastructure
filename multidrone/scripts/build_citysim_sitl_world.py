#!/usr/bin/env python3
"""Build a PX4/Gazebo-Classic SITL world from osrf simple_city.world.

Strips GUI-only and citysim-built plugins that are not in the Docker image.
Keeps all city model includes.
"""
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "vendor" / "citysim" / "worlds" / "simple_city.world"
OUT = ROOT / "worlds" / "citysim" / "citysim_sitl.world"

DROP_PLUGINS = (
    "libKeyboardGUIPlugin.so",
    "libTrafficLightsGUIPlugin.so",
    "libBloomVisualPlugin.so",
    "libLensFlareVisualPlugin.so",
    "libPriusHybridPlugin.so",
    "libJoyPlugin.so",
)


def _plugin_lib(line: str) -> str | None:
    for lib in DROP_PLUGINS:
        if f'filename="{lib}"' in line or f"filename='{lib}'" in line:
            return lib
    return None


def patch_physics_for_px4(text: str) -> str:
    """PX4 gazebo_mavlink_interface aborts when real_time_update_rate is 0."""
    return text.replace(
        "<real_time_update_rate>0</real_time_update_rate>",
        "<real_time_update_rate>500</real_time_update_rate>",
    )


def strip_plugins(text: str) -> str:
    lines = text.splitlines(keepends=True)
    out: list[str] = []
    skip_depth = 0
    for line in lines:
        if skip_depth > 0:
            skip_depth += line.count("<plugin")
            skip_depth -= line.count("</plugin>")
            continue
        if "<plugin" in line and _plugin_lib(line) is not None:
            if "/>" in line:
                continue
            skip_depth = 1
            continue
        out.append(line)
    return "".join(out)


def main() -> int:
    if not SRC.is_file():
        print(f"missing upstream world: {SRC}", file=sys.stderr)
        return 1
    OUT.parent.mkdir(parents=True, exist_ok=True)
    stripped = patch_physics_for_px4(strip_plugins(SRC.read_text(encoding="utf-8")))
    OUT.write_text(stripped, encoding="utf-8")
    print(f"wrote {OUT} ({OUT.stat().st_size} bytes)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
