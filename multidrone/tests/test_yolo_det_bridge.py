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
    CAMERA_HZ,
    WINDOW_NAME,
    YOLO_IMGSZ,
    YOLO_WEIGHTS,
    boxes_from_rows,
    frame_payload,
)


def test_one_drone_constants():
    assert DRONE1_IMAGE_TOPIC == "/iris_1/vio_cam/image_raw"
    assert DRONE1_JPEG_URL == "http://127.0.0.1:5700"
    assert DRONE1_DET_PORT == 7753
    assert WINDOW_NAME == "Drone 1 camera"
    assert YOLO_WEIGHTS.endswith("visdrone_yolov8m.pt")
    assert YOLO_IMGSZ == 640
    assert CAMERA_HZ == 15


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


def test_photo_cars_cover_the_field_and_miss_the_pads():
    import math

    from spawn_scan_cars import PAD_CLEAR_M, PADS_XY, car_spawns

    cars = car_spawns()
    assert len(cars) == 24
    assert {c["kind"] for c in cars} == {"hatchback", "pickup", "ambulance"}
    assert min(c["x"] for c in cars) < -15
    assert max(c["x"] for c in cars) > 15
    assert min(c["y"] for c in cars) < -15
    assert max(c["y"] for c in cars) > 15
    for car in cars:
        for px, py in PADS_XY:
            assert math.hypot(car["x"] - px, car["y"] - py) >= PAD_CLEAR_M


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
