#!/usr/bin/env python3
"""Scatter static photo-textured cars across the Gazebo field.

Meshes are the Open Source Robotics Foundation hatchback, pickup, and
ambulance models (model:// under Dockerfiles/models). Gazebo x is north and
y is east. Pads on the spawn line (x=0, y=0/3/6/9) are left clear.

Replaces the earlier box-and-pole stand-ins. Deletes those if they are still
in the running world.
"""
from __future__ import annotations

import math
import random
import subprocess
from pathlib import Path

PADS_XY = ((0.0, 0.0), (0.0, 3.0), (0.0, 6.0), (0.0, 9.0))
PAD_CLEAR_M = 4.0
KINDS = ("hatchback", "pickup", "ambulance")
OLD_MODELS = ("area_scan_cars", "car_under_cam")
OVERHEAD_SUN = "overhead_sun"


def car_spawns(count: int = 24, span: float = 30.0) -> list[dict]:
    """Deterministic poses covering the field the drones actually fly."""
    rng = random.Random(11)
    poses: list[dict] = []
    guard = 0
    while len(poses) < count and guard < 8000:
        guard += 1
        x = rng.uniform(-span, span)
        y = rng.uniform(-span, span)
        if any(math.hypot(x - px, y - py) < PAD_CLEAR_M for px, py in PADS_XY):
            continue
        poses.append(
            {
                "name": f"scan_car_{len(poses)}",
                "kind": KINDS[len(poses) % len(KINDS)],
                "x": x,
                "y": y,
                "yaw": rng.uniform(-math.pi, math.pi),
            }
        )
    if len(poses) < count:
        raise SystemExit(f"spawn_scan_cars: only placed {len(poses)} of {count}")
    return poses


def spawn_sdf(name: str, kind: str) -> str:
    return (
        "<?xml version='1.0'?>\n"
        "<sdf version='1.6'>\n"
        f"  <model name='{name}'>\n"
        "    <static>true</static>\n"
        "    <include>\n"
        f"      <uri>model://{kind}</uri>\n"
        "    </include>\n"
        "  </model>\n"
        "</sdf>\n"
    )


def _gz(args: list[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ["gz", "model", *args],
        check=False,
        capture_output=True,
        text=True,
    )


def overhead_sun_sdf() -> str:
    """Bright sun from above. Not exactly (0, 0, -1): Gazebo's shadow camera
    crashes when a directional light is parallel to world up."""
    return """<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='overhead_sun'>
    <static>true</static>
    <pose>0 0 40 0 0 0</pose>
    <link name='link'>
      <visual name='keep_bounds'>
        <geometry><sphere><radius>0.05</radius></sphere></geometry>
      </visual>
      <light name='overhead' type='directional'>
        <cast_shadows>false</cast_shadows>
        <diffuse>1.15 1.12 1.02 1</diffuse>
        <specular>0.55 0.55 0.5 1</specular>
        <direction>0.12 0.05 -1</direction>
        <attenuation>
          <range>2000</range>
          <constant>0.15</constant>
          <linear>0</linear>
          <quadratic>0</quadratic>
        </attenuation>
      </light>
    </link>
  </model>
</sdf>
"""


def delete_model(name: str) -> None:
    _gz(["--model-name", name, "--delete"])


def spawn_overhead_sun() -> None:
    delete_model(OVERHEAD_SUN)
    path = Path("/tmp/scan_cars") / "overhead_sun.sdf"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(overhead_sun_sdf(), encoding="utf-8")
    proc = _gz(
        [
            "--spawn-file",
            str(path),
            "--model-name",
            OVERHEAD_SUN,
            "--pose-x",
            "0",
            "--pose-y",
            "0",
            "--pose-z",
            "40",
        ]
    )
    detail = (proc.stderr or proc.stdout or "").strip()
    if proc.returncode != 0:
        print(f"spawn_scan_cars: overhead sun failed: {detail}", flush=True)
    else:
        print("spawn_scan_cars: overhead sun shining straight down", flush=True)


def main() -> int:
    spawn_overhead_sun()
    for name in OLD_MODELS:
        delete_model(name)
    poses = car_spawns()
    for pose in poses:
        delete_model(pose["name"])
    tmp = Path("/tmp/scan_cars")
    tmp.mkdir(parents=True, exist_ok=True)
    failed = 0
    for pose in poses:
        path = tmp / f"{pose['name']}.sdf"
        path.write_text(spawn_sdf(pose["name"], pose["kind"]), encoding="utf-8")
        proc = _gz(
            [
                "--spawn-file",
                str(path),
                "--model-name",
                pose["name"],
                "--pose-x",
                f"{pose['x']:.3f}",
                "--pose-y",
                f"{pose['y']:.3f}",
                "--pose-z",
                "0",
                "--pose-Y",
                f"{pose['yaw']:.4f}",
            ]
        )
        if proc.returncode != 0:
            failed += 1
            detail = (proc.stderr or proc.stdout or "").strip()
            print(f"spawn_scan_cars: {pose['name']} failed: {detail}", flush=True)
        else:
            print(
                f"spawn_scan_cars: {pose['name']} {pose['kind']} "
                f"at x={pose['x']:.1f} y={pose['y']:.1f}",
                flush=True,
            )
    print(f"spawn_scan_cars: {len(poses) - failed} textured cars in the world", flush=True)
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
