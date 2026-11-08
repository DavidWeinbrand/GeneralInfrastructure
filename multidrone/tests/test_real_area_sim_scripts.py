from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SITL = ROOT / "multidrone" / "sitl_multiple_run.sh"
RUN = ROOT / "runSimNoeticMulti.sh"


def test_sitl_appends_catswarm_host_after_setup():
    text = SITL.read_text()
    i_src = text.index("setup_gazebo.bash")
    i_host = text.index("models/catswarm_host")
    assert i_src < i_host
    assert "GAZEBO_MODEL_PATH" in text


def test_runsim_help_and_world_flag():
    import subprocess
    out = subprocess.check_output(["bash", str(RUN), "--help"], text=True)
    assert "--world" in out
    assert "--vio-cam" in out
    assert "--vio-pitch" in out
    assert "CATSWARM_VIO_PITCH" in RUN.read_text()


def test_runsim_missing_world_exits_before_docker(tmp_path):
    import subprocess
    proc = subprocess.run(
        ["bash", str(RUN), "--world", "definitely_missing_world_xyz", "--num", "1"],
        cwd=str(ROOT), capture_output=True, text=True, timeout=15,
    )
    assert proc.returncode != 0
    assert "generate_real_area.py" in (proc.stdout + proc.stderr)


def test_runsim_origin_gate_skips_cylinders():
    text = RUN.read_text()
    # The required-origin block must not treat cylinders as a host map.
    gate = text.split("if [[ \"$WORLD\" != \"empty\"")[1].split("fi")[0]
    assert "cylinders" in gate


def test_runsim_cylinders_is_empty_like_and_has_radius_flag():
    import subprocess
    out = subprocess.check_output(["bash", str(RUN), "--help"], text=True)
    assert "--cylinder-radius" in out
    text = RUN.read_text()
    assert "CATSWARM_CYLINDER_RADIUS" in text
    assert "CATSWARM_WORLD" in text
    # cylinders must not share the origin.json required-file path with teradyon
    assert 'WORLD" != "cylinders"' in text or "cylinders" in text
    sitl = SITL.read_text()
    assert "spawn_cylinders" in sitl
    assert "CATSWARM_WORLD" in sitl
    assert '!= "citysim"' in sitl
    city = ROOT / "multidrone" / "worlds" / "citysim"
    assert (city / "citysim.world").is_file()
    assert (city / "citysim_sitl.world").is_file()
    sitl_world = (city / "citysim_sitl.world").read_text()
    assert "model://city_terrain" in sitl_world
    assert "<real_time_update_rate>500</real_time_update_rate>" in sitl_world
    assert "<real_time_update_rate>0</real_time_update_rate>" not in sitl_world
    assert (ROOT / "multidrone" / "vendor" / "citysim" / "models" / "city_terrain" / "model.config").is_file()
    assert (ROOT / "multidrone" / "vendor" / "gazebo_models" / "house_1" / "model.config").is_file()
    assert '"lat"' in (city / "origin.json").read_text()
