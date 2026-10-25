#!/usr/bin/env python3
"""Optional: spawn the area-scan training polygon as a flat outline at the Gazebo origin.

Not run by default; set ``CATSWARM_SPAWN_SCAN_POLYGON=1`` in ``sitl_multiple_run.sh``.
The ring is one episode from ver_11_8 ``make_env_cfg`` (seed 11): 12 corners,
about 900 m², centroid on the world origin. Gazebo +x is north and +y is east,
so a vertex (east, north) is spawned at (x=north, y=east). The four default
pads (north 0, east 0/3/6/9) lie inside this ring.
"""
from __future__ import annotations

import math
import subprocess
from pathlib import Path

# East, north metres. Centroid is the Gazebo / PX4 home.
TRAINED_POLYGON_EN: tuple[tuple[float, float], ...] = (
    (-20.2325, -14.1099),
    (16.3363, -19.1193),
    (16.3363, -5.9278),
    (11.3269, -5.7608),
    (11.4939, -2.4212),
    (16.3363, -2.4212),
    (16.3363, 10.7703),
    (-8.5439, 5.7608),
    (-8.7108, 14.1099),
    (-15.2231, 14.1099),
    (-15.2231, 2.5882),
    (-20.2325, 2.4212),
)

MODEL_NAME = "area_scan_polygon"
EDGE_WIDTH_M = 0.25
EDGE_HEIGHT_M = 0.05


def edge_gazebo_pose(
    p0: tuple[float, float], p1: tuple[float, float]
) -> tuple[float, float, float, float]:
    """Return gazebo (x, y, yaw, length). x is north, y is east."""
    e0, n0 = p0
    e1, n1 = p1
    de = e1 - e0
    dn = n1 - n0
    length = math.hypot(de, dn)
    x = (n0 + n1) / 2.0
    y = (e0 + e1) / 2.0
    yaw = math.atan2(de, dn)
    return x, y, yaw, length


def point_in_polygon(east: float, north: float) -> bool:
    inside = False
    verts = TRAINED_POLYGON_EN
    j = len(verts) - 1
    for i in range(len(verts)):
        xi, yi = verts[i]
        xj, yj = verts[j]
        intersects = ((yi > north) != (yj > north)) and (
            east < (xj - xi) * (north - yi) / (yj - yi + 1e-12) + xi
        )
        if intersects:
            inside = not inside
        j = i
    return inside


def polygon_sdf() -> str:
    links = []
    for i, p0 in enumerate(TRAINED_POLYGON_EN):
        p1 = TRAINED_POLYGON_EN[(i + 1) % len(TRAINED_POLYGON_EN)]
        x, y, yaw, length = edge_gazebo_pose(p0, p1)
        links.append(
            "    <link name='edge{i}'>\n"
            "      <pose>{x:.4f} {y:.4f} 0 {yaw:.6f} 0 0</pose>\n"
            "      <visual name='visual'>\n"
            "        <geometry><box><size>{length:.4f} {w} {h}</size></box></geometry>\n"
            "        <material>\n"
            "          <ambient>0.22 0.74 0.95 1</ambient>\n"
            "          <diffuse>0.22 0.74 0.95 1</diffuse>\n"
            "        </material>\n"
            "      </visual>\n"
            "    </link>\n".format(
                i=i, x=x, y=y, yaw=yaw, length=max(length, 0.05),
                w=EDGE_WIDTH_M, h=EDGE_HEIGHT_M,
            )
        )
    return (
        "<?xml version='1.0'?>\n"
        "<sdf version='1.6'>\n"
        f"  <model name='{MODEL_NAME}'>\n"
        "    <static>true</static>\n"
        "    <pose>0 0 0.03 0 0 0</pose>\n"
        + "".join(links)
        + "  </model>\n"
        "</sdf>\n"
    )


def main() -> int:
    path = Path("/tmp") / f"{MODEL_NAME}.sdf"
    path.write_text(polygon_sdf(), encoding="utf-8")
    print(f"spawn_scan_polygon: {len(TRAINED_POLYGON_EN)} corners at the Gazebo origin")
    subprocess.run(
        [
            "gz", "model",
            f"--spawn-file={path}",
            f"--model-name={MODEL_NAME}",
            "-x", "0", "-y", "0", "-z", "0",
        ],
        check=False,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
