#!/usr/bin/env python3
"""One YOLO process for drone 1.

Reads the drone 1 camera preview and publishes DetectionFrame_v1 on port 7753
(drone 1, zmqCommPort 7701 + 52). Weights: project default visdrone-yolo11s
(see ``yolo_config.py``).
"""
from __future__ import annotations

import json
from pathlib import Path

from yolo_config import (
    DEFAULT_YOLO_WEIGHTS,
    YOLO_IMGSZ,
    resolve_yolo_weights,
)

DRONE1_IMAGE_TOPIC = "/iris_1/vio_cam/image_raw"
DRONE1_JPEG_URL = "http://127.0.0.1:5700"
DRONE1_DET_PORT = 7753
DETECTION_TOPIC = b"DETECTION_FRAME"
WINDOW_NAME = "Drone 1 camera"
YOLO_WEIGHTS = str(DEFAULT_YOLO_WEIGHTS)
PREVIEW_HZ = 30


def boxes_from_rows(xyxy, conf, cls, ids, names) -> list[dict]:
    """Turn one detector frame into DetectionFrame_v1 boxes."""
    boxes = []
    n = len(xyxy)
    id_list = list(ids) if ids is not None else [None] * n
    for i in range(n):
        x1, y1, x2, y2 = (float(v) for v in xyxy[i])
        class_i = int(cls[i])
        if isinstance(names, dict):
            name = names.get(class_i, str(class_i))
        else:
            name = names[class_i]
        box = {
            "class_id": str(name),
            "score": float(conf[i]),
            "x1": x1,
            "y1": y1,
            "x2": x2,
            "y2": y2,
        }
        if i < len(id_list) and id_list[i] is not None:
            box["track_id"] = int(id_list[i])
        boxes.append(box)
    return boxes


def frame_payload(frame_id: int, t_mono: float, boxes: list[dict]) -> dict:
    return {
        "schema": "DetectionFrame_v1",
        "frame_id": int(frame_id),
        "t_mono": float(t_mono),
        "source": "yolo",
        "boxes": boxes,
    }


def boxes_from_result(result) -> list[dict]:
    boxes = getattr(result, "boxes", None)
    if boxes is None or boxes.xyxy is None or len(boxes.xyxy) == 0:
        return []
    xyxy = boxes.xyxy.detach().cpu().tolist()
    conf = boxes.conf.detach().cpu().tolist()
    cls = boxes.cls.detach().cpu().tolist()
    ids = None if boxes.id is None else boxes.id.detach().cpu().tolist()
    return boxes_from_rows(xyxy, conf, cls, ids, result.names)


def draw_boxes(image, boxes: list[dict]):
    """Green rectangle and label on a BGR image. Returns a copy."""
    import cv2

    out = image.copy()
    for box in boxes:
        x1 = int(round(box["x1"]))
        y1 = int(round(box["y1"]))
        x2 = int(round(box["x2"]))
        y2 = int(round(box["y2"]))
        cv2.rectangle(out, (x1, y1), (x2, y2), (0, 220, 0), 2)
        label = f"{box['class_id']} {float(box['score']):.2f}"
        cv2.putText(
            out, label, (x1, max(12, y1 - 4)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 220, 0), 1, cv2.LINE_AA,
        )
    return out


def fetch_jpeg(url: str):
    """One camera JPEG from vio_cam_tcp. None when the sim camera is not up yet."""
    import urllib.error
    import urllib.request

    import cv2
    import numpy as np

    try:
        with urllib.request.urlopen(url, timeout=2) as resp:
            data = resp.read()
    except (urllib.error.URLError, TimeoutError, OSError):
        return None
    frame = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        return None
    return frame


def show_frame(image) -> bool:
    """Open the desktop window. False when this OpenCV build cannot show one."""
    import cv2

    cv2.imshow(WINDOW_NAME, image)
    cv2.waitKey(1)
    return True


def ros_image_to_bgr(msg):
    import numpy as np

    height = int(msg.height)
    width = int(msg.width)
    encoding = getattr(msg, "encoding", "") or ""
    frame = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if encoding == "bgr8":
        return frame.reshape((height, width, 3)).copy()
    if encoding == "rgb8":
        import cv2

        rgb = frame.reshape((height, width, 3))
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    raise RuntimeError(f"yolo_det_bridge: unsupported image encoding {encoding!r}")


def _detect(model, image) -> list[dict]:
    try:
        results = model.track(image, persist=True, verbose=False, imgsz=YOLO_IMGSZ, conf=0.25)
        return boxes_from_result(results[0]) if results else []
    except Exception as exc:
        print(f"yolo_det_bridge: track failed: {exc}", flush=True)
        return []


def _publish_boxes(sock, frame_id: int, boxes: list[dict]) -> int:
    import time

    frame_id += 1
    payload = frame_payload(frame_id, time.monotonic(), boxes)
    sock.send_multipart([DETECTION_TOPIC, json.dumps(payload).encode("utf-8")])
    if boxes:
        names = ",".join(f"{b['class_id']}:{b['score']:.2f}" for b in boxes)
        print(f"yolo_det_bridge: frame {frame_id} {names}", flush=True)
    return frame_id


def _show(image, boxes: list[dict]) -> None:
    try:
        show_frame(draw_boxes(image, boxes))
    except Exception as exc:
        print(f"yolo_det_bridge: camera window failed: {exc}", flush=True)


def _open_pub():
    import zmq

    ctx = zmq.Context()
    sock = ctx.socket(zmq.PUB)
    sock.bind(f"tcp://127.0.0.1:{DRONE1_DET_PORT}")
    return sock


def run_jpeg(url: str, weights_path: str) -> None:
    import threading
    import time

    from ultralytics import YOLO

    print(
        f"yolo_det_bridge: waiting for drone 1 camera at {url}",
        flush=True,
    )
    image = None
    deadline = time.time() + 180
    while time.time() < deadline and image is None:
        image = fetch_jpeg(url)
        if image is None:
            time.sleep(0.5)
    if image is None:
        raise SystemExit(f"yolo_det_bridge: no camera JPEG from {url}")
    model = YOLO(weights_path)
    sock = _open_pub()
    print(
        f"yolo_det_bridge: window '{WINDOW_NAME}', weights={weights_path} "
        f"preview {PREVIEW_HZ} fps, "
        f"PUB {DETECTION_TOPIC.decode()} :{DRONE1_DET_PORT}",
        flush=True,
    )
    lock = threading.Lock()
    state = {"image": image, "seq": 1, "boxes": []}
    stop = threading.Event()

    def detect_loop() -> None:
        frame_id = 0
        seen = 0
        while not stop.is_set():
            with lock:
                seq = state["seq"]
                snap = None if seq == seen else state["image"].copy()
            if snap is None:
                time.sleep(0.005)
                continue
            seen = seq
            boxes = _detect(model, snap)
            frame_id = _publish_boxes(sock, frame_id, boxes)
            with lock:
                state["boxes"] = boxes

    threading.Thread(target=detect_loop, name="yolo_det", daemon=True).start()
    try:
        while True:
            nxt = fetch_jpeg(url)
            if nxt is None:
                time.sleep(0.02)
                continue
            with lock:
                state["image"] = nxt
                state["seq"] += 1
                boxes = list(state["boxes"])
            _show(nxt, boxes)
    finally:
        stop.set()


def run_ros(weights_path: str) -> None:
    import time

    import rospy
    from sensor_msgs.msg import Image
    from ultralytics import YOLO

    rospy.init_node("yolo_det_bridge_drone1", anonymous=True)
    model = YOLO(weights_path)
    latest: list = []

    def on_img(msg) -> None:
        try:
            latest[:] = [ros_image_to_bgr(msg)]
        except Exception as exc:
            rospy.logwarn_throttle(2.0, f"yolo_det_bridge: image convert failed: {exc}")

    rospy.Subscriber(DRONE1_IMAGE_TOPIC, Image, on_img, queue_size=1)
    sock = _open_pub()
    print(
        f"yolo_det_bridge: window '{WINDOW_NAME}', weights={weights_path} "
        f"preview {PREVIEW_HZ} fps, "
        f"SUB {DRONE1_IMAGE_TOPIC} PUB {DETECTION_TOPIC.decode()} :{DRONE1_DET_PORT}",
        flush=True,
    )
    import threading

    lock = threading.Lock()
    state = {"image": None, "seq": 0, "boxes": []}
    stop = threading.Event()

    def detect_loop() -> None:
        frame_id = 0
        seen = 0
        while not stop.is_set() and not rospy.is_shutdown():
            with lock:
                seq = state["seq"]
                snap = None if seq == 0 or seq == seen or state["image"] is None else state["image"].copy()
            if snap is None:
                time.sleep(0.005)
                continue
            seen = seq
            boxes = _detect(model, snap)
            frame_id = _publish_boxes(sock, frame_id, boxes)
            with lock:
                state["boxes"] = boxes

    threading.Thread(target=detect_loop, name="yolo_det", daemon=True).start()
    period = 1.0 / PREVIEW_HZ
    try:
        while not rospy.is_shutdown():
            if not latest:
                time.sleep(period)
                continue
            image = latest[0]
            with lock:
                state["image"] = image
                state["seq"] += 1
                boxes = list(state["boxes"])
            _show(image, boxes)
            time.sleep(period)
    finally:
        stop.set()


def main() -> None:
    import argparse
    import sys

    parser = argparse.ArgumentParser(prog="yolo_det_bridge")
    parser.add_argument(
        "--jpeg-url",
        default=DRONE1_JPEG_URL,
        help="Drone 1 color preview (vio_cam_tcp). This is the host-side default.",
    )
    parser.add_argument(
        "--weights",
        default=None,
        help=f"Ultralytics .pt (default: {DEFAULT_YOLO_WEIGHTS.name}, HF dronefreak/visdrone-yolo11s).",
    )
    parser.add_argument(
        "--ros",
        action="store_true",
        help="Subscribe to /iris_1/vio_cam/image_raw instead of the JPEG preview.",
    )
    args = parser.parse_args()
    try:
        import cv2  # noqa: F401
        from ultralytics import YOLO  # noqa: F401
    except ImportError as exc:
        print(
            "yolo_det_bridge: cannot start "
            f"({exc}). Install opencv-python and ultralytics where torch already is "
            "(the host, not the PX4 container).",
            file=sys.stderr,
        )
        raise SystemExit(1) from exc
    weights_path = resolve_yolo_weights(args.weights)
    if args.ros:
        run_ros(weights_path)
    else:
        run_jpeg(args.jpeg_url, weights_path)


if __name__ == "__main__":
    main()
