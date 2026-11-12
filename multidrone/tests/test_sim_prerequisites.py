import os
from pathlib import Path
import subprocess


SCRIPT = Path(__file__).resolve().parents[2] / "runSimNoeticMulti.sh"


def test_missing_image_fails_before_cleanup(tmp_path):
    calls = tmp_path / "calls"
    docker = tmp_path / "docker"
    docker.write_text(
        "#!/bin/sh\n"
        'printf "%s\\n" "$*" >> "$TEST_CALLS"\n'
        '[ "$1" = info ] && exit 0\n'
        "exit 1\n"
    )
    docker.chmod(0o755)
    env = {**os.environ, "PATH": f"{tmp_path}:{os.environ['PATH']}", "TEST_CALLS": str(calls)}
    result = subprocess.run(["bash", str(SCRIPT)], env=env, capture_output=True, text=True)
    assert result.returncode == 1
    assert "not installed" in result.stderr
    assert calls.read_text().splitlines() == ["info", "image inspect px4-noetic-sim-ros:latest"]
    assert "Cleaning up" not in result.stdout
