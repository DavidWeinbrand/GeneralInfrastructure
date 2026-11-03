from pathlib import Path
import math
import random
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from spawn_cylinders import (  # noqa: E402
    COLOR_RGB,
    CYLINDER_COLORS,
    CYLINDER_DIAMETER_M,
    CYLINDER_HEIGHT_M,
    DEFAULT_RADIUS_M,
    cylinder_sdf,
    gz_spawn_argv,
    gz_world_name,
    is_procedural_world,
    parse_cylinder_radius,
    parse_origin_xy,
    place_cylinders,
    sample_disk_xy,
)


def test_world_helpers():
    assert is_procedural_world("cylinders") is True
    assert is_procedural_world("empty") is False
    assert is_procedural_world("teradyon") is False
    assert gz_world_name("cylinders") == "empty"
    assert gz_world_name("static_cars") == "empty"
    assert gz_world_name("empty") == "empty"
    assert gz_world_name("teradyon") == "teradyon"


def test_parse_radius_and_origin():
    assert parse_cylinder_radius(None) == DEFAULT_RADIUS_M
    assert parse_cylinder_radius("") == DEFAULT_RADIUS_M
    assert parse_cylinder_radius(10) == 10.0
    assert parse_cylinder_radius("7.5") == 7.5
    with pytest.raises(ValueError):
        parse_cylinder_radius(0)
    with pytest.raises(ValueError):
        parse_cylinder_radius(-1)
    with pytest.raises(ValueError):
        parse_cylinder_radius("x")
    assert parse_origin_xy("0 0\n0 3\n") == (0.0, 0.0)
    assert parse_origin_xy("# c\n1.5 2.25 0.83\n") == (1.5, 2.25)
    with pytest.raises(ValueError):
        parse_origin_xy("")


def test_disk_and_place_in_radius():
    rng = random.Random(1)
    x, y = sample_disk_xy((10.0, -4.0), 10.0, rng)
    assert math.hypot(x - 10.0, y + 4.0) <= 10.0 + 1e-9
    rng = random.Random(2)
    placed = place_cylinders((0.0, 0.0), 10.0, rng)
    assert [p["color"] for p in placed] == list(CYLINDER_COLORS)
    assert [p["name"] for p in placed] == ["cyl_blue", "cyl_red", "cyl_green", "cyl_yellow"]
    for p in placed:
        assert math.hypot(p["x"], p["y"]) <= 10.0 + 1e-9
        assert p["z"] == pytest.approx(0.0)
    rng_a = random.Random(3)
    rng_b = random.Random(3)
    assert place_cylinders((0.0, 0.0), 10.0, rng_a) == place_cylinders((0.0, 0.0), 10.0, rng_b)


def test_sdf_visual_only_and_gz_argv():
    assert CYLINDER_DIAMETER_M == 0.50
    assert CYLINDER_HEIGHT_M == 1.0
    xml = cylinder_sdf("blue", COLOR_RGB["blue"])
    assert "<collision>" not in xml
    assert "<radius>0.25</radius>" in xml
    assert "<length>1.0</length>" in xml
    assert "0 0 1 1" in xml
    argv = gz_spawn_argv("cyl_blue", "/tmp/cyl_blue.sdf", 1.0, 2.0, 0.5)
    assert argv[:2] == ["gz", "model"]
    assert "--spawn-file=/tmp/cyl_blue.sdf" in argv
    assert "--model-name=cyl_blue" in argv
    assert argv[argv.index("-x") + 1] == "1.0"
    assert argv[argv.index("-y") + 1] == "2.0"
    assert argv[argv.index("-z") + 1] == "0.5"


def test_spawn_all_calls_gz_four_times(tmp_path):
    from spawn_cylinders import spawn_all
    called = []

    def run_gz(argv):
        called.append(list(argv))

    def write_sdf(path, xml):
        path.write_text(xml)

    spawn_all(
        origin_xy=(0.0, 0.0),
        radius_m=10.0,
        rng=random.Random(0),
        run_gz=run_gz,
        write_sdf=write_sdf,
        sdf_dir=tmp_path,
    )
    assert len(called) == 4
    names = []
    for c in called:
        names.extend([a.split("=", 1)[1] for a in c if a.startswith("--model-name=")])
        assert float(c[c.index("-z") + 1]) == pytest.approx(0.0)
    assert names == ["cyl_blue", "cyl_red", "cyl_green", "cyl_yellow"]
