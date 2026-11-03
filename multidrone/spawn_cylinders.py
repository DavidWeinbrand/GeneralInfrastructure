#!/usr/bin/env python3
"""Sample visual-only colored cylinders in a disk and build Gazebo spawn argv."""
from __future__ import annotations

import math
from pathlib import Path

CYLINDER_COLORS: tuple[str, ...] = ("blue", "red", "green", "yellow")
CYLINDER_DIAMETER_M: float = 0.50
CYLINDER_HEIGHT_M: float = 1.0
DEFAULT_RADIUS_M: float = 10.0
COLOR_RGB: dict[str, tuple[float, float, float]] = {
    "blue": (0, 0, 1),
    "red": (1, 0, 0),
    "green": (0, 1, 0),
    "yellow": (1, 1, 0),
}

_EMPTY_WORLDS = frozenset({"empty", "cylinders", "static_cars"})


def is_procedural_world(name: str) -> bool:
    return name in ("cylinders", "static_cars")


def gz_world_name(world: str) -> str:
    return "empty" if world in _EMPTY_WORLDS else world


def parse_cylinder_radius(raw: object) -> float:
    if raw is None or raw == "":
        return DEFAULT_RADIUS_M
    try:
        value = float(raw)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"invalid cylinder radius: {raw!r}") from exc
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f"cylinder radius must be finite > 0, got {value!r}")
    return value


def parse_origin_xy(positions_text: str) -> tuple[float, float]:
    for raw_line in positions_text.splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        if len(parts) < 2:
            raise ValueError(f"expected x y [z], got {line!r}")
        try:
            return float(parts[0]), float(parts[1])
        except ValueError as exc:
            raise ValueError(f"invalid origin xy: {line!r}") from exc
    raise ValueError("no origin xy in positions text")


def sample_disk_xy(
    origin_xy: tuple[float, float], radius_m: float, rng
) -> tuple[float, float]:
    r = radius_m * math.sqrt(rng.random())
    theta = 2 * math.pi * rng.random()
    ox, oy = origin_xy
    return ox + r * math.cos(theta), oy + r * math.sin(theta)


def place_cylinders(origin_xy, radius_m, rng) -> list[dict]:
    placed = []
    z = 0.0
    for color in CYLINDER_COLORS:
        _rgb = COLOR_RGB[color]
        x, y = sample_disk_xy(origin_xy, radius_m, rng)
        placed.append(
            {
                "name": f"cyl_{color}",
                "color": color,
                "x": x,
                "y": y,
                "z": z,
            }
        )
    return placed


def cylinder_sdf(color: str, rgb: tuple[float, float, float]) -> str:
    radius = CYLINDER_DIAMETER_M / 2.0
    length = CYLINDER_HEIGHT_M
    pose_z = length / 2.0
    r, g, b = rgb
    rgba = f"{r} {g} {b} 1"
    return (
        "<?xml version='1.0'?>\n"
        "<sdf version='1.6'>\n"
        f"  <model name='cyl_{color}'>\n"
        "    <static>true</static>\n"
        "    <link name='link'>\n"
        f"      <pose>0 0 {pose_z} 0 0 0</pose>\n"
        "      <visual name='visual'>\n"
        "        <geometry>\n"
        "          <cylinder>\n"
        f"            <radius>{radius}</radius>\n"
        f"            <length>{length}</length>\n"
        "          </cylinder>\n"
        "        </geometry>\n"
        "        <material>\n"
        f"          <ambient>{rgba}</ambient>\n"
        f"          <diffuse>{rgba}</diffuse>\n"
        "        </material>\n"
        "      </visual>\n"
        "    </link>\n"
        "  </model>\n"
        "</sdf>\n"
    )


def gz_spawn_argv(
    model_name: str, sdf_path: str, x: float, y: float, z: float
) -> list[str]:
    return [
        "gz",
        "model",
        f"--spawn-file={sdf_path}",
        f"--model-name={model_name}",
        "-x",
        str(x),
        "-y",
        str(y),
        "-z",
        str(z),
    ]


def spawn_all(origin_xy, radius_m, rng, run_gz, write_sdf, sdf_dir: Path) -> None:
    for p in place_cylinders(origin_xy, radius_m, rng):
        path = Path(sdf_dir) / f"{p['name']}.sdf"
        write_sdf(path, cylinder_sdf(p["color"], COLOR_RGB[p["color"]]))
        run_gz(gz_spawn_argv(p["name"], str(path), p["x"], p["y"], p["z"]))


def main(argv=None) -> int:
    import argparse, subprocess, sys, random
    parser = argparse.ArgumentParser(prog="spawn_cylinders")
    parser.add_argument("--positions", required=True)
    parser.add_argument("--radius", default=None)
    ns = parser.parse_args(argv)
    text = Path(ns.positions).read_text(encoding="utf-8")
    origin = parse_origin_xy(text)
    radius = parse_cylinder_radius(ns.radius)
    seed = random.randrange(0, 2**31)
    rng = random.Random(seed)
    print(f"spawn_cylinders: origin={origin} radius={radius} seed={seed}")
    spawn_all(
        origin, radius, rng,
        run_gz=lambda argv: subprocess.run(argv, check=False),
        write_sdf=lambda path, xml: path.write_text(xml, encoding="utf-8"),
        sdf_dir=Path("/tmp"),
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
