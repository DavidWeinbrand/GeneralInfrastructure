"""The Gazebo outline is the 12-corner ring the scan policy was trained on."""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from spawn_scan_polygon import (  # noqa: E402
    TRAINED_POLYGON_EN,
    edge_gazebo_pose,
    point_in_polygon,
    polygon_sdf,
)


def test_ring_has_twelve_corners_and_contains_the_pads():
    assert len(TRAINED_POLYGON_EN) == 12
    for east in (0.0, 3.0, 6.0, 9.0):
        assert point_in_polygon(east, 0.0)


def test_edges_use_gazebo_north_east_axes():
    x, y, _yaw, length = edge_gazebo_pose((0.0, 0.0), (0.0, 10.0))
    assert x == 5.0
    assert y == 0.0
    assert length == 10.0
    sdf = polygon_sdf()
    assert sdf.count("<link name=") == 12
    assert "area_scan_polygon" in sdf
