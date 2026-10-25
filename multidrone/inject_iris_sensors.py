#!/usr/bin/env python3
"""Inject a downward rangefinder + optical flow source into a generated iris SDF (multi-SITL).

Two optical-flow sources exist in PX4's gazebo-classic plugins; both publish on the
topic `gazebo_mavlink_interface` subscribes to (`~/<model>/px4flow/link/opticalFlow`):

* ``mockup`` (default) — ``libgazebo_opticalflow_mockup_plugin.so``, a model plugin that
  derives flow from body velocity + the lidar range. No rendering, so it works headless
  (Xvfb / no GPU), which is where the camera plugin silently never publishes.
* ``camera`` — nested ``model://px4flow``, whose OpenCV-based ``libgazebo_opticalflow_plugin.so``
  needs a working GL render pipeline.

Pick with ``CATSWARM_OF_MODE=mockup|camera|both`` (``both`` publishes twice on the same
topic when rendering works — for A/B checks only).

The mockup plugin reads range from ``~/<model>/link/lidar``, published by the
inline CatSwarm lidar (2 cm min, matching real air) or legacy ``model://lidar``.
"""
from __future__ import annotations

import math
import os
import sys
from pathlib import Path

OF_MODES = ("mockup", "camera", "both")
DEFAULT_OF_MODE = "mockup"

MOCKUP_PLUGIN_SO = "libgazebo_opticalflow_mockup_plugin.so"

# Inline lidar (not stock model://lidar): PX4 stock clamps min_distance at 0.2 m;
# real CatSwarm air uses ranger/OF from ~2 cm — match that in SITL.
LIDAR_SNIPPET = """
    <!-- CatSwarm: downward rangefinder (topic ~/link/lidar -> DISTANCE_SENSOR; min 2 cm) -->
    <model name="lidar">
      <pose>0 0 -0.05 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>0.01</mass>
          <inertia>
            <ixx>2.1733e-6</ixx><ixy>0</ixy><ixz>0</ixz>
            <iyy>2.1733e-6</iyy><iyz>0</iyz><izz>1.8e-7</izz>
          </inertia>
        </inertial>
        <visual name="visual">
          <geometry><cylinder><radius>0.006</radius><length>0.05</length></cylinder></geometry>
        </visual>
        <sensor name="laser" type="ray">
          <pose>0 0 0 0 1.57079633 0</pose>
          <ray>
            <scan><horizontal>
              <samples>1</samples><resolution>1</resolution>
              <min_angle>0</min_angle><max_angle>0</max_angle>
            </horizontal></scan>
            <range><min>0.02</min><max>35</max><resolution>0.01</resolution></range>
            <noise><type>gaussian</type><mean>0.0</mean><stddev>0.01</stddev></noise>
          </ray>
          <plugin name="LaserPlugin" filename="libgazebo_lidar_plugin.so">
            <robotNamespace></robotNamespace>
            <min_distance>0.02</min_distance>
            <max_distance>50.0</max_distance>
          </plugin>
          <always_on>1</always_on>
          <update_rate>20</update_rate>
        </sensor>
      </link>
    </model>
    <joint name="lidar_joint" type="fixed">
      <parent>base_link</parent>
      <child>lidar::link</child>
    </joint>
"""

PX4FLOW_SNIPPET = """
    <!-- CatSwarm: camera optical flow (nested model://px4flow, needs GL rendering) -->
    <include>
      <uri>model://px4flow</uri>
      <pose>0.05 0 -0.05 0 0 0</pose>
      <name>px4flow</name>
    </include>
    <joint name="px4flow_joint" type="revolute">
      <parent>base_link</parent>
      <child>px4flow::link</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <upper>0</upper>
          <lower>0</lower>
        </limit>
      </axis>
    </joint>
"""

MOCKUP_SNIPPET = f"""
    <!-- CatSwarm: headless-safe optical flow (no rendering; range from the lidar include) -->
    <plugin name="opticalflow_mockup_plugin" filename="{MOCKUP_PLUGIN_SO}">
      <robotNamespace></robotNamespace>
      <pubRate>20</pubRate>
    </plugin>
"""

def vio_cam_pose_xyzrpy(pitch_deg: float) -> str:
    """Nested vio_cam pose: 10 cm body-forward, Gazebo Z-up pitch = −FRD deg."""
    rad = math.radians(-float(pitch_deg))
    if abs(rad) < 1e-12:
        rad = 0.0
    return f"0.10 0 0 0 {rad:.6f} 0"


def resolve_vio_pitch(value: str | None = None) -> float:
    raw = value if value is not None else os.environ.get("CATSWARM_VIO_PITCH", "-90")
    if str(raw).strip() == "":
        return -90.0
    try:
        v = float(raw)
    except (TypeError, ValueError) as exc:
        raise SystemExit(
            f"inject_iris_sensors: CATSWARM_VIO_PITCH {raw!r} must be in [-90, 0]"
        ) from exc
    if v < -90.0 or v > 0.0:
        raise SystemExit(
            f"inject_iris_sensors: CATSWARM_VIO_PITCH {v} must be in [-90, 0]"
        )
    return v


def vio_cam_snippet(pitch_deg: float | None = None) -> str:
    pose = vio_cam_pose_xyzrpy(resolve_vio_pitch() if pitch_deg is None else float(pitch_deg))
    return f"""
    <!-- CatSwarm: VIO mono camera -->
    <model name="vio_cam">
      <pose>{pose}</pose>
      <link name="link">
        <gravity>false</gravity>
        <inertial>
          <mass>0.01</mass>
          <inertia>
            <ixx>1e-6</ixx><ixy>0</ixy><ixz>0</ixz>
            <iyy>1e-6</iyy><iyz>0</iyz><izz>1e-6</izz>
          </inertia>
        </inertial>
        <sensor name="camera" type="camera">
          <update_rate>15</update_rate>
          <camera>
            <horizontal_fov>1.04719755</horizontal_fov>
            <image>
              <width>640</width>
              <height>640</height>
              <format>R8G8B8</format>
            </image>
          </camera>
          <plugin name="vio_cam_controller" filename="libgazebo_ros_camera.so">
            <robotNamespace></robotNamespace>
            <cameraName>vio_cam</cameraName>
            <imageTopicName>image_raw</imageTopicName>
          </plugin>
          <always_on>1</always_on>
        </sensor>
      </link>
    </model>
    <joint name="vio_cam_pitch" type="fixed">
      <parent>base_link</parent>
      <child>vio_cam::link</child>
    </joint>
"""


def resolve_vio_cam(value: str | None = None) -> bool:
    raw = value if value is not None else os.environ.get("CATSWARM_VIO_CAM", "1")
    return str(raw).strip().lower() not in ("0", "false", "off", "no")


def yolo_det_requested() -> bool:
    raw = os.environ.get("CATSWARM_YOLO_DET", "0").strip().lower()
    return raw in ("1", "true", "yes", "on")


def vio_cam_enabled_for_instance(instance_n: int | None) -> bool:
    """Rendered camera. With CATSWARM_YOLO_DET, only iris 1 gets one."""
    if not resolve_vio_cam():
        return False
    if yolo_det_requested() and instance_n is not None and int(instance_n) != 1:
        return False
    return True


def resolve_of_mode(of_mode: str | None = None) -> str:
    mode = (of_mode or os.environ.get("CATSWARM_OF_MODE") or DEFAULT_OF_MODE).strip().lower()
    if mode not in OF_MODES:
        raise SystemExit(
            f"inject_iris_sensors: invalid CATSWARM_OF_MODE {mode!r}; want one of {OF_MODES}"
        )
    return mode


def inject(sdf_path: Path, of_mode: str | None = None, instance_n: int | None = None) -> None:
    """Add the missing sensor blocks to ``sdf_path`` in place; idempotent per block."""
    mode = resolve_of_mode(of_mode)
    text = sdf_path.read_text(encoding="utf-8")

    additions = []
    # Prefer CatSwarm inline lidar (2 cm min). Skip if already injected or legacy include present.
    if 'name="lidar_joint"' not in text and "model://lidar" not in text:
        additions.append(LIDAR_SNIPPET)
    if mode in ("camera", "both") and "model://px4flow" not in text:
        additions.append(PX4FLOW_SNIPPET)
    if mode in ("mockup", "both") and MOCKUP_PLUGIN_SO not in text:
        additions.append(MOCKUP_SNIPPET)
    if vio_cam_enabled_for_instance(instance_n) and 'name="vio_cam_pitch"' not in text:
        additions.append(vio_cam_snippet())
    if not additions:
        return

    idx = text.rfind("</model>")
    if idx < 0:
        raise SystemExit(f"inject_iris_sensors: no </model> in {sdf_path}")
    sdf_path.write_text(text[:idx] + "".join(additions) + "\n" + text[idx:], encoding="utf-8")
    print(f"inject_iris_sensors: of_mode={mode} added {len(additions)} block(s) to {sdf_path}")


def main() -> None:
    if len(sys.argv) not in (2, 3):
        raise SystemExit(f"usage: {sys.argv[0]} <iris.sdf> [instance_n]")
    instance_n = int(sys.argv[2]) if len(sys.argv) == 3 else None
    inject(Path(sys.argv[1]), instance_n=instance_n)


if __name__ == "__main__":
    main()
