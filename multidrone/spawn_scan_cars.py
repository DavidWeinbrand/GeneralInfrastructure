#!/usr/bin/env python3
"""Scatter static photo-textured cars across the Gazebo field.

Meshes are the Open Source Robotics Foundation hatchback and pickup
models (model:// under Dockerfiles/models). Gazebo x is north and
y is east. Pads on the spawn line (x=0, y=0/3/6/9) are left clear.

Replaces the earlier box-and-pole stand-ins. Deletes those if they are still
in the running world.
"""
from __future__ import annotations

import math
import os
import random
import subprocess
from pathlib import Path

PADS_XY = ((0.0, 0.0), (0.0, 3.0), (0.0, 6.0), (0.0, 9.0))
PAD_CLEAR_M = 4.0
KINDS = ("hatchback", "pickup")
OLD_MODELS = ("area_scan_cars", "car_under_cam")
OVERHEAD_SUN = "overhead_sun"
CAR_COUNT = 0
# The Static cars world uses the empty field, half the old crowd, spread wide.
STATIC_CAR_COUNT = 12
STATIC_CAR_SPAN_M = 55.0
STATIC_CAR_SEP_M = 14.0
# Earlier launches placed 24. Drop the extra names when this script runs again.
PREVIOUS_CAR_COUNT = 24
# Three hatchbacks and pickups, each on its own loop. Clear of the pads at x=0.
CIRCLE_LOOPS = (
    {"name": "scan_car_circle", "kind": "hatchback", "cx": 12.0, "cy": -14.0, "r": 7.0, "speed": 0.6},
    {"name": "scan_car_circle_b", "kind": "pickup", "cx": -18.0, "cy": 16.0, "r": 6.0, "speed": 0.45},
    {"name": "scan_car_circle_c", "kind": "hatchback", "cx": -16.0, "cy": -18.0, "r": 5.0, "speed": 0.8},
)
CIRCLE_NAME = CIRCLE_LOOPS[0]["name"]
CIRCLE_KIND = CIRCLE_LOOPS[0]["kind"]
CIRCLE_CENTER_X = CIRCLE_LOOPS[0]["cx"]
CIRCLE_CENTER_Y = CIRCLE_LOOPS[0]["cy"]
CIRCLE_RADIUS_M = CIRCLE_LOOPS[0]["r"]
CIRCLE_SPEED_MPS = CIRCLE_LOOPS[0]["speed"]
# Parked cars whose centre is this close to the first loop would overlap it.
CIRCLE_CLEAR_M = 7.0


def car_spawns(count: int = CAR_COUNT, span: float = 30.0, min_sep: float = 0.0) -> list[dict]:
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
        radial = math.hypot(x - CIRCLE_CENTER_X, y - CIRCLE_CENTER_Y)
        if abs(radial - CIRCLE_RADIUS_M) < CIRCLE_CLEAR_M:
            continue
        if min_sep > 0.0 and any(
            math.hypot(x - other["x"], y - other["y"]) < min_sep for other in poses
        ):
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


def circle_pose_of(loop: dict, t_s: float) -> tuple[float, float, float]:
    """North, east, and yaw tangent to one loop at time t_s."""
    omega = loop["speed"] / loop["r"]
    ang = omega * t_s
    x = loop["cx"] + loop["r"] * math.cos(ang)
    y = loop["cy"] + loop["r"] * math.sin(ang)
    yaw = math.atan2(math.cos(ang), -math.sin(ang))
    return x, y, yaw


def circle_pose(t_s: float) -> tuple[float, float, float]:
    """Pose of the original loop. Kept for the existing tests."""
    return circle_pose_of(CIRCLE_LOOPS[0], t_s)


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


def spawn_circle_car() -> bool:
    """Place each circling car at the start of its own loop."""
    tmp = Path("/tmp/scan_cars")
    tmp.mkdir(parents=True, exist_ok=True)
    ok = True
    for loop in CIRCLE_LOOPS:
        delete_model(loop["name"])
        x, y, yaw = circle_pose_of(loop, 0.0)
        path = tmp / f"{loop['name']}.sdf"
        path.write_text(spawn_sdf(loop["name"], loop["kind"]), encoding="utf-8")
        proc = _gz(
            [
                "--spawn-file",
                str(path),
                "--model-name",
                loop["name"],
                "--pose-x",
                f"{x:.3f}",
                "--pose-y",
                f"{y:.3f}",
                "--pose-z",
                "0",
                "--pose-Y",
                f"{yaw:.4f}",
            ]
        )
        if proc.returncode != 0:
            ok = False
            detail = (proc.stderr or proc.stdout or "").strip()
            print(f"spawn_scan_cars: {loop['name']} failed: {detail}", flush=True)
        else:
            print(
                f"spawn_scan_cars: {loop['name']} {loop['kind']} circling at "
                f"{loop['speed']:.2f} m/s, radius {loop['r']:.0f} m",
                flush=True,
            )
    return ok


def drive_circle() -> None:
    """Move scan_car_circle on the wall clock.

    rospy.Rate follows sim time. When that clock pauses, the car sits still and
    the next pose is a jump. A fixed real-time step keeps the motion even.
    """
    import time

    import rospy
    from gazebo_msgs.msg import ModelState
    from gazebo_msgs.srv import SetModelState

    rospy.init_node("scan_car_circle", anonymous=True, disable_signals=False)
    rospy.wait_for_service("/gazebo/set_model_state", timeout=60.0)
    setter = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    period = 0.025
    t0 = time.monotonic()
    while not rospy.is_shutdown():
        tick = time.monotonic()
        elapsed = tick - t0
        for loop in CIRCLE_LOOPS:
            x, y, yaw = circle_pose_of(loop, elapsed)
            msg = ModelState()
            msg.model_name = loop["name"]
            msg.reference_frame = "world"
            msg.pose.position.x = x
            msg.pose.position.y = y
            msg.pose.position.z = 0.0
            msg.pose.orientation.z = math.sin(yaw * 0.5)
            msg.pose.orientation.w = math.cos(yaw * 0.5)
            try:
                setter(msg)
            except rospy.ServiceException as exc:
                print(f"spawn_scan_cars: {loop['name']} step failed: {exc}", flush=True)
        remain = period - (time.monotonic() - tick)
        if remain > 0.0:
            time.sleep(remain)


def main() -> int:
    spawn_overhead_sun()
    for name in OLD_MODELS:
        delete_model(name)
    world = os.environ.get("CATSWARM_WORLD", "")
    parked = STATIC_CAR_COUNT if world == "static_cars" else CAR_COUNT
    span = STATIC_CAR_SPAN_M if world == "static_cars" else 30.0
    sep = STATIC_CAR_SEP_M if world == "static_cars" else 0.0
    poses = car_spawns(count=parked, span=span, min_sep=sep)
    for pose in poses:
        delete_model(pose["name"])
    for index in range(len(poses), PREVIOUS_CAR_COUNT):
        delete_model(f"scan_car_{index}")
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
    # CitySim and the static-car field do not get the circling hatchback.
    if world not in ("citysim", "static_cars"):
        if not spawn_circle_car():
            failed += 1
    print(f"spawn_scan_cars: {len(poses) - failed} textured cars in the world", flush=True)
    return 1 if failed else 0


if __name__ == "__main__":
    import sys

    if "--drive-circle" in sys.argv:
        drive_circle()
        raise SystemExit(0)
    raise SystemExit(main())
