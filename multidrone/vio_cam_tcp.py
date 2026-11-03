#!/usr/bin/env python3
"""Republish Gazebo VIO camera images as SVOF grayscale TCP frames. No rospy at import."""
from __future__ import annotations

import argparse
import logging
import socket
import struct
import threading

# Gazebo vio_cam update_rate is 30. The operator JPEG follows that.
# The gray VIO stream stays slower: its Python conversion is the heavy copy.
CAMERA_HZ = 15
PREVIEW_HZ = 30
CAMERA_TCP_BASE = 5600
JPEG_PREVIEW_BASE = 5700
_SVOF_HDR = struct.Struct("<4sHHQI")
_COLOR_ENCODINGS = ("rgb8", "bgr8", "rgba8", "bgra8")


def camera_tcp_port(drone_id: int) -> int:
    return CAMERA_TCP_BASE + (int(drone_id) - 1)


def jpeg_preview_port(drone_id: int) -> int:
    return JPEG_PREVIEW_BASE + (int(drone_id) - 1)


def rgb_array_from_msg(msg):
    """RGB uint8 array, or None when numpy cannot view this encoding."""
    try:
        import numpy as np
    except ImportError:
        return None
    encoding = getattr(msg, "encoding", "") or ""
    if encoding not in ("rgb8", "bgr8"):
        return None
    width = int(msg.width)
    height = int(msg.height)
    step = int(getattr(msg, "step", 0) or width * 3)
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if step == width * 3:
        arr = raw.reshape((height, width, 3))
    else:
        rows = [raw[y * step : y * step + width * 3].reshape((width, 3)) for y in range(height)]
        arr = np.stack(rows, axis=0)
    if encoding == "bgr8":
        arr = arr[:, :, ::-1]
    return np.ascontiguousarray(arr)


def jpeg_bytes_from_rgb_array(arr) -> bytes:
    import cv2

    bgr = arr[:, :, ::-1]
    ok, buf = cv2.imencode(".jpg", bgr)
    if not ok:
        raise RuntimeError("no JPEG encoder")
    return buf.tobytes()


def rgb_to_gray(rgb: bytes, width: int, height: int) -> bytes:
    n = int(width) * int(height)
    out = bytearray(n)
    for i in range(n):
        o = i * 3
        r, g, b = rgb[o], rgb[o + 1], rgb[o + 2]
        out[i] = int(0.299 * r + 0.587 * g + 0.114 * b)
    return bytes(out)


def _default_jpeg_encode():
    try:
        import cv2
        import numpy as np
    except ImportError:
        cv2 = None
        np = None
    if cv2 is not None and np is not None:
        def encode(rgb, width, height):
            arr = np.frombuffer(rgb, dtype=np.uint8).reshape((int(height), int(width), 3))
            bgr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
            ok, buf = cv2.imencode(".jpg", bgr)
            if not ok:
                raise RuntimeError("no JPEG encoder")
            return buf.tobytes()

        return encode
    try:
        from PIL import Image
        import io
    except ImportError:
        raise RuntimeError("no JPEG encoder")

    def encode(rgb, width, height):
        img = Image.frombytes("RGB", (int(width), int(height)), rgb)
        out = io.BytesIO()
        img.save(out, format="JPEG")
        return out.getvalue()

    return encode


def rgb_jpeg_bytes(rgb: bytes, width: int, height: int, encode=None) -> bytes:
    if encode is None:
        encode = _default_jpeg_encode()
    return encode(bytes(rgb), int(width), int(height))


def pack_svof(w, h, t_ns, gray: bytes) -> bytes:
    return _SVOF_HDR.pack(b"SVOF", int(w), int(h), int(t_ns), len(gray)) + gray


def _close_quiet(obj) -> None:
    closer = getattr(obj, "close", None)
    if closer is not None:
        try:
            closer()
        except Exception:
            pass


def serve_frames(drone_id: int, *, bind_fn, gray_iter) -> None:
    port = camera_tcp_port(drone_id)
    srv = bind_fn(("0.0.0.0", port))
    pending = iter(gray_iter)
    current = None
    try:
        while True:
            try:
                conn, _addr = srv.accept()
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError):
                continue
            try:
                while True:
                    if current is None:
                        try:
                            current = next(pending)
                        except StopIteration:
                            return
                    w, h, t_ns, gray = current
                    conn.sendall(pack_svof(w, h, t_ns, gray))
                    current = None
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError):
                continue
            finally:
                _close_quiet(conn)
    finally:
        _close_quiet(srv)


def serve_latest_jpeg(drone_id: int, *, bind_fn, jpeg_iter) -> None:
    port = jpeg_preview_port(drone_id)
    srv = bind_fn(("0.0.0.0", port))
    pending = iter(jpeg_iter)
    current = None
    try:
        while True:
            try:
                conn, _addr = srv.accept()
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError):
                continue
            try:
                try:
                    conn.recv(65536)
                except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError):
                    continue
                if current is None:
                    try:
                        current = next(pending)
                    except StopIteration:
                        return
                body = current
                header = (
                    b"HTTP/1.0 200 OK\r\n"
                    b"Content-Type: image/jpeg\r\n"
                    b"Content-Length: "
                    + str(len(body)).encode("ascii")
                    + b"\r\n"
                    b"Connection: close\r\n"
                    b"\r\n"
                )
                conn.sendall(header + body)
                current = None
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError):
                continue
            finally:
                _close_quiet(conn)
    finally:
        _close_quiet(srv)


def _default_bind(addr):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.settimeout(None)
    srv.bind(addr)
    srv.listen(1)
    return srv


def _image_to_rgb(msg) -> bytes:
    encoding = getattr(msg, "encoding", "") or ""
    raw = bytes(msg.data)
    width = int(msg.width)
    height = int(msg.height)
    if encoding in _COLOR_ENCODINGS:
        bpp = 4 if "a" in encoding else 3
        row_step = int(getattr(msg, "step", 0) or width * bpp)
        rgb = bytearray()
        bgr = encoding.startswith("bgr")
        for y in range(height):
            row = raw[y * row_step : y * row_step + width * bpp]
            for i in range(0, len(row), bpp):
                if bgr:
                    rgb.extend((row[i + 2], row[i + 1], row[i]))
                else:
                    rgb.extend(row[i : i + 3])
        return bytes(rgb)
    row_step = int(getattr(msg, "step", 0) or width)
    rgb = bytearray()
    for y in range(height):
        row = raw[y * row_step : y * row_step + width]
        for v in row:
            rgb.extend((v, v, v))
    return bytes(rgb)


def _image_to_gray(msg) -> bytes:
    encoding = getattr(msg, "encoding", "") or ""
    width = int(msg.width)
    height = int(msg.height)
    if encoding in _COLOR_ENCODINGS:
        return rgb_to_gray(_image_to_rgb(msg), width, height)
    raw = bytes(msg.data)
    row_step = int(getattr(msg, "step", 0) or width)
    gray = bytearray()
    for y in range(height):
        gray.extend(raw[y * row_step : y * row_step + width])
    return bytes(gray)


if __name__ == "__main__":
    import rospy
    from sensor_msgs.msg import Image

    parser = argparse.ArgumentParser(prog="vio_cam_tcp")
    parser.add_argument("--num", type=int, required=True)
    ns = parser.parse_args()

    rospy.init_node("vio_cam_tcp", anonymous=True)

    def run_one(drone_id: int) -> None:
        latest: list = []
        latest_rgb: list = []
        try:
            encode_fn = _default_jpeg_encode()
        except RuntimeError:
            encode_fn = None

        def on_img(msg):
            stamp = msg.header.stamp
            t_ns = int(stamp.to_nsec()) if stamp else 0
            width = int(msg.width)
            height = int(msg.height)
            arr = rgb_array_from_msg(msg)
            if arr is not None:
                import numpy as np

                gray = (
                    0.299 * arr[:, :, 0] + 0.587 * arr[:, :, 1] + 0.114 * arr[:, :, 2]
                ).astype(np.uint8)
                latest[:] = [(width, height, t_ns, gray.tobytes())]
                latest_rgb[:] = [(arr, width, height, t_ns)]
                return
            rgb = _image_to_rgb(msg)
            latest[:] = [(width, height, t_ns, rgb_to_gray(rgb, width, height))]
            latest_rgb[:] = [(rgb, width, height, t_ns)]

        rospy.Subscriber(
            f"/iris_{drone_id}/vio_cam/image_raw",
            Image,
            on_img,
            queue_size=1,
        )

        def gray_iter():
            rate = rospy.Rate(CAMERA_HZ)
            while not rospy.is_shutdown():
                if latest:
                    yield latest.pop()
                rate.sleep()

        latest_jpeg = {"body": None, "t": None}

        def encode_preview():
            import time
            period = 1.0 / PREVIEW_HZ
            last_t = None
            while not rospy.is_shutdown():
                started = time.monotonic()
                frame = latest_rgb[0] if latest_rgb else None
                if frame is not None:
                    rgb, width, height, t_ns = frame
                    if t_ns != last_t:
                        if not isinstance(rgb, (bytes, bytearray)):
                            body = jpeg_bytes_from_rgb_array(rgb)
                        else:
                            body = rgb_jpeg_bytes(rgb, width, height, encode=encode_fn)
                        latest_jpeg["body"] = body
                        latest_jpeg["t"] = t_ns
                        last_t = t_ns
                remain = period - (time.monotonic() - started)
                if remain > 0:
                    time.sleep(remain)

        def jpeg_iter():
            import time
            last = None
            while not rospy.is_shutdown():
                body = latest_jpeg["body"]
                stamp = latest_jpeg["t"]
                if body is None or stamp == last:
                    time.sleep(0.002)
                    continue
                last = stamp
                yield body

        def jpeg_thread_fn():
            if encode_fn is None:
                logging.warning("no JPEG encoder; preview disabled")
                return
            threading.Thread(target=encode_preview, daemon=True).start()
            serve_latest_jpeg(drone_id, bind_fn=_default_bind, jpeg_iter=jpeg_iter())

        threading.Thread(target=jpeg_thread_fn, daemon=True).start()
        serve_frames(drone_id, bind_fn=_default_bind, gray_iter=gray_iter())

    threads = []
    for i in range(1, int(ns.num) + 1):
        t = threading.Thread(target=run_one, args=(i,), daemon=True)
        t.start()
        threads.append(t)
    rospy.spin()
