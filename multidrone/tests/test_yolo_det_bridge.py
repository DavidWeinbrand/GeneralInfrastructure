"""YOLO bridge payload shape. Does not load a model or ROS."""
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from yolo_det_bridge import (  # noqa: E402
    DRONE1_DET_PORT,
    DRONE1_IMAGE_TOPIC,
    DRONE1_JPEG_URL,
    PREVIEW_HZ,
    WINDOW_NAME,
    YOLO_IMGSZ,
    YOLO_WEIGHTS,
    boxes_from_rows,
    frame_payload,
)
from yolo_config import HF_YOLO_REPO_ID  # noqa: E402


def test_project_default_yolo():
    assert HF_YOLO_REPO_ID == "dronefreak/visdrone-yolo11s"
    assert YOLO_WEIGHTS.endswith("visdrone_yolo11s.pt")
    assert DRONE1_IMAGE_TOPIC == "/iris_1/vio_cam/image_raw"
    assert DRONE1_JPEG_URL == "http://127.0.0.1:5700"
    assert DRONE1_DET_PORT == 7753
    assert WINDOW_NAME == "Drone 1 camera"
    assert YOLO_IMGSZ == 640
    assert PREVIEW_HZ == 30


def test_frame_payload_is_detection_frame_v1():
    boxes = boxes_from_rows(
        xyxy=[[10.0, 20.0, 80.0, 100.0]],
        conf=[0.81],
        cls=[2],
        ids=[7],
        names={2: "car"},
    )
    payload = frame_payload(3, 1.5, boxes)
    assert payload["schema"] == "DetectionFrame_v1"
    assert payload["source"] == "yolo"
    assert payload["frame_id"] == 3
    box = payload["boxes"][0]
    assert box["class_id"] == "car"
    assert box["score"] == 0.81
    assert box["track_id"] == 7
    assert box["x1"] == 10.0
    assert box["y2"] == 100.0


def test_overhead_sun_points_straight_down():
    from spawn_scan_cars import overhead_sun_sdf

    text = overhead_sun_sdf()
    assert "type='directional'" in text
    assert "<direction>0 0 -1</direction>" not in text
    assert "<direction>0.12 0.05 -1</direction>" in text


def test_static_cars_world_fills_the_old_field():
    import math

    from spawn_scan_cars import STATIC_CAR_COUNT, STATIC_CAR_SEP_M, STATIC_CAR_SPAN_M, car_spawns

    cars = car_spawns(count=STATIC_CAR_COUNT, span=STATIC_CAR_SPAN_M, min_sep=STATIC_CAR_SEP_M)
    assert len(cars) == 12
    assert "ambulance" not in {c["kind"] for c in cars}
    assert max(c["x"] for c in cars) - min(c["x"] for c in cars) > 60
    for i, car in enumerate(cars):
        for other in cars[i + 1 :]:
            assert math.hypot(car["x"] - other["x"], car["y"] - other["y"]) >= STATIC_CAR_SEP_M


def test_default_field_has_no_parked_cars():
    from spawn_scan_cars import car_spawns

    assert car_spawns() == []


def test_requested_cars_miss_pads_and_the_loop():
    import math

    from spawn_scan_cars import (
        CIRCLE_CENTER_X,
        CIRCLE_CENTER_Y,
        CIRCLE_CLEAR_M,
        CIRCLE_RADIUS_M,
        PAD_CLEAR_M,
        PADS_XY,
        car_spawns,
    )

    cars = car_spawns(count=8)
    assert len(cars) == 8
    assert {c["kind"] for c in cars} == {"hatchback", "pickup"}
    for car in cars:
        for px, py in PADS_XY:
            assert math.hypot(car["x"] - px, car["y"] - py) >= PAD_CLEAR_M
        radial = math.hypot(car["x"] - CIRCLE_CENTER_X, car["y"] - CIRCLE_CENTER_Y)
        assert abs(radial - CIRCLE_RADIUS_M) >= CIRCLE_CLEAR_M


def test_circle_car_follows_the_loop():
    import math

    from spawn_scan_cars import (
        CIRCLE_CENTER_X,
        CIRCLE_CENTER_Y,
        CIRCLE_RADIUS_M,
        CIRCLE_SPEED_MPS,
        circle_pose,
    )

    x0, y0, _yaw0 = circle_pose(0.0)
    x1, y1, yaw1 = circle_pose(1.0)
    assert abs(math.hypot(x0 - CIRCLE_CENTER_X, y0 - CIRCLE_CENTER_Y) - CIRCLE_RADIUS_M) < 1e-6
    assert abs(math.hypot(x1 - CIRCLE_CENTER_X, y1 - CIRCLE_CENTER_Y) - CIRCLE_RADIUS_M) < 1e-6
    assert abs(math.hypot(x1 - x0, y1 - y0) - CIRCLE_SPEED_MPS) < 0.05
    assert abs(yaw1 - math.pi / 2) < 0.2
    from spawn_scan_cars import CIRCLE_LOOPS, circle_pose_of

    assert len(CIRCLE_LOOPS) == 3
    for loop in CIRCLE_LOOPS:
        x, y, _yaw = circle_pose_of(loop, 0.0)
        assert abs(math.hypot(x - loop["cx"], y - loop["cy"]) - loop["r"]) < 1e-6
        for px, py in ((0.0, 0.0), (0.0, 3.0), (0.0, 6.0), (0.0, 9.0)):
            assert math.hypot(loop["cx"] - px, loop["cy"] - py) > loop["r"] + 4.0


def test_missing_track_id_is_omitted():
    boxes = boxes_from_rows(
        xyxy=[[0, 0, 4, 4]],
        conf=[0.5],
        cls=[0],
        ids=None,
        names={0: "person"},
    )
    assert "track_id" not in boxes[0]
    assert boxes[0]["class_id"] == "person"
