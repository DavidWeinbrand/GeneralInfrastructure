from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from inject_iris_sensors import (  # noqa: E402
    DEFAULT_OF_MODE,
    MOCKUP_PLUGIN_SO,
    inject,
    resolve_of_mode,
    resolve_vio_pitch,
    vio_cam_pose_xyzrpy,
)

BARE_IRIS = """<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='iris_1'>
    <link name='base_link'/>
  </model>
</sdf>
"""

LIDAR_ONLY_IRIS = """<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='iris_1'>
    <link name='base_link'/>
    <include>
      <uri>model://lidar</uri>
      <name>lidar</name>
    </include>
    <joint name="lidar_joint" type="fixed">
      <parent>base_link</parent>
      <child>lidar::link</child>
    </joint>
  </model>
</sdf>
"""


def _write(tmp_path: Path, text: str) -> Path:
    sdf = tmp_path / "iris_1.sdf"
    sdf.write_text(text, encoding="utf-8")
    return sdf


def test_default_mode_is_mockup_plus_lidar(tmp_path: Path):
    sdf = _write(tmp_path, BARE_IRIS)
    inject(sdf)
    text = sdf.read_text(encoding="utf-8")
    assert DEFAULT_OF_MODE == "mockup"
    assert MOCKUP_PLUGIN_SO in text
    assert 'name="lidar_joint"' in text
    assert "<min_distance>0.02</min_distance>" in text
    # Headless default must not pull in the render-dependent camera plugin.
    assert 'model://px4flow' not in text


def test_camera_mode_adds_px4flow_and_lidar(tmp_path: Path):
    sdf = _write(tmp_path, BARE_IRIS)
    inject(sdf, of_mode="camera")
    text = sdf.read_text(encoding="utf-8")
    assert 'model://px4flow' in text
    assert 'name="px4flow_joint"' in text
    assert 'name="lidar_joint"' in text
    assert MOCKUP_PLUGIN_SO not in text


def test_both_mode_adds_every_source(tmp_path: Path):
    sdf = _write(tmp_path, BARE_IRIS)
    inject(sdf, of_mode="both")
    text = sdf.read_text(encoding="utf-8")
    assert 'model://px4flow' in text
    assert 'name="lidar_joint"' in text
    assert MOCKUP_PLUGIN_SO in text


@pytest.mark.parametrize("mode", ["mockup", "camera", "both"])
def test_inject_is_idempotent(tmp_path: Path, mode: str):
    sdf = _write(tmp_path, BARE_IRIS)
    inject(sdf, of_mode=mode)
    once = sdf.read_text(encoding="utf-8")
    inject(sdf, of_mode=mode)
    assert sdf.read_text(encoding="utf-8") == once


@pytest.mark.parametrize("mode", ["mockup", "camera"])
def test_lidar_only_sdf_is_upgraded_without_duplicate_lidar(tmp_path: Path, mode: str):
    sdf = _write(tmp_path, LIDAR_ONLY_IRIS)
    inject(sdf, of_mode=mode)
    text = sdf.read_text(encoding="utf-8")
    # Legacy model://lidar counts as already present — do not double-inject.
    assert text.count("model://lidar") == 1
    assert text.count('name="lidar_joint"') == 1
    if mode == "mockup":
        assert MOCKUP_PLUGIN_SO in text
    else:
        assert 'model://px4flow' in text


def test_blocks_land_inside_the_model_element(tmp_path: Path):
    sdf = _write(tmp_path, BARE_IRIS)
    inject(sdf, of_mode="both")
    text = sdf.read_text(encoding="utf-8")
    assert text.rfind(MOCKUP_PLUGIN_SO) < text.rfind("</model>")
    assert text.rfind("model://px4flow") < text.rfind("</model>")


def test_env_override(monkeypatch):
    monkeypatch.setenv("CATSWARM_OF_MODE", "camera")
    assert resolve_of_mode() == "camera"
    monkeypatch.delenv("CATSWARM_OF_MODE")
    assert resolve_of_mode() == DEFAULT_OF_MODE


def test_invalid_mode_rejected():
    with pytest.raises(SystemExit):
        resolve_of_mode("bogus")


def test_yolo_det_renders_camera_on_drone_1_only(tmp_path: Path, monkeypatch):
    monkeypatch.setenv("CATSWARM_YOLO_DET", "1")
    drone1 = _write(tmp_path, BARE_IRIS)
    inject(drone1, instance_n=1)
    assert 'name="vio_cam_pitch"' in drone1.read_text(encoding="utf-8")
    drone2 = tmp_path / "iris_2.sdf"
    drone2.write_text(BARE_IRIS, encoding="utf-8")
    inject(drone2, instance_n=2)
    text2 = drone2.read_text(encoding="utf-8")
    assert 'name="vio_cam_pitch"' not in text2
    assert 'name="lidar_joint"' in text2
    assert MOCKUP_PLUGIN_SO in text2


def test_camera_stays_on_other_iris_without_yolo_det(tmp_path: Path, monkeypatch):
    monkeypatch.delenv("CATSWARM_YOLO_DET", raising=False)
    sdf = _write(tmp_path, BARE_IRIS)
    inject(sdf, instance_n=2)
    assert 'name="vio_cam_pitch"' in sdf.read_text(encoding="utf-8")


def test_inject_adds_vio_camera_and_keeps_of_lidar(tmp_path: Path):
    sdf = _write(tmp_path, BARE_IRIS)
    inject(sdf)
    text = sdf.read_text(encoding="utf-8")
    assert 'name="vio_cam_pitch"' in text
    assert "horizontal_fov>1.04719755" in text.replace(" ", "") or "1.04719755" in text
    assert "<width>640</width>" in text and "<height>640</height>" in text
    assert "<update_rate>15</update_rate>" in text
    assert 'name="lidar_joint"' in text
    assert MOCKUP_PLUGIN_SO in text
    inject(sdf)
    assert sdf.read_text(encoding="utf-8").count('name="vio_cam_pitch"') == 1


def test_vio_cam_spawn_pose_looks_down_by_default():
    import math

    assert resolve_vio_pitch() == -90.0
    pose = vio_cam_pose_xyzrpy(-90)
    assert pose.startswith("0.10 0 0 0 ")
    assert abs(float(pose.split()[4]) - math.pi / 2) < 1e-6


def test_vio_cam_spawn_pose_forward_at_zero_pitch():
    assert vio_cam_pose_xyzrpy(0) == "0.10 0 0 0 0.000000 0"


def test_inject_bakes_look_down_pose_into_sdf(tmp_path: Path):
    sdf = _write(tmp_path, BARE_IRIS)
    inject(sdf)
    text = sdf.read_text(encoding="utf-8")
    assert vio_cam_pose_xyzrpy(-90) in text
    assert "<gravity>false</gravity>" in text
    assert "<pose>0.10 0 0 0 0 0</pose>" not in text


def test_vio_cam_pitch_joint_is_fixed(tmp_path: Path):
    """revolute + gz joint --pos-t applies unlimited PID and collapses iris to origin."""
    sdf = _write(tmp_path, BARE_IRIS)
    inject(sdf)
    text = sdf.read_text(encoding="utf-8")
    assert '<joint name="vio_cam_pitch" type="fixed">' in text
    assert 'type="revolute"' not in text.split('name="vio_cam_pitch"')[1].split("</joint>")[0]


def test_inject_zero_pitch_is_body_forward(tmp_path: Path, monkeypatch):
    monkeypatch.setenv("CATSWARM_VIO_PITCH", "0")
    sdf = _write(tmp_path, BARE_IRIS)
    inject(sdf)
    text = sdf.read_text(encoding="utf-8")
    assert vio_cam_pose_xyzrpy(0) in text


def test_vio_cam_off_skips_camera_keeps_of_lidar(tmp_path: Path, monkeypatch):
    monkeypatch.setenv("CATSWARM_VIO_CAM", "0")
    sdf = _write(tmp_path, BARE_IRIS)
    inject(sdf)
    text = sdf.read_text(encoding="utf-8")
    assert 'name="vio_cam_pitch"' not in text
    assert 'name="vio_cam"' not in text
    assert 'name="lidar_joint"' in text
    assert MOCKUP_PLUGIN_SO in text

