"""Project default YOLO for area-scan sim detection.

Model: https://huggingface.co/dronefreak/visdrone-yolo11s (VisDrone YOLO11s, ``best.pt``).
"""
from __future__ import annotations

import os
import shutil
from pathlib import Path

MULTIDRONE_DIR = Path(__file__).resolve().parent
HF_YOLO_REPO_ID = "dronefreak/visdrone-yolo11s"
HF_YOLO_FILENAME = "best.pt"
HF_YOLO_URL = f"https://huggingface.co/{HF_YOLO_REPO_ID}"
DEFAULT_YOLO_WEIGHTS = MULTIDRONE_DIR / "weights" / "visdrone_yolo11s.pt"
YOLO_IMGSZ = 640
ENV_YOLO_WEIGHTS = "CATSWARM_YOLO_WEIGHTS"


def default_yolo_weights_path() -> Path:
    return DEFAULT_YOLO_WEIGHTS


def download_default_weights(dest: Path | None = None) -> Path:
    target = (dest or DEFAULT_YOLO_WEIGHTS).expanduser()
    try:
        from huggingface_hub import hf_hub_download
    except ImportError as exc:
        raise SystemExit(
            f"Missing weights at {target}. Install huggingface_hub and re-run, or:\n"
            f"  huggingface-cli download {HF_YOLO_REPO_ID} {HF_YOLO_FILENAME} "
            f"-O {target.parent} && mv {target.parent / HF_YOLO_FILENAME} {target}\n"
            f"See {HF_YOLO_URL}"
        ) from exc
    print(
        f"yolo: downloading {HF_YOLO_REPO_ID}/{HF_YOLO_FILENAME} → {target}",
        flush=True,
    )
    cached = hf_hub_download(repo_id=HF_YOLO_REPO_ID, filename=HF_YOLO_FILENAME)
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(cached, target)
    return target.resolve()


def resolve_yolo_weights(explicit: str | None = None, *, allow_download: bool = True) -> str:
    raw = (explicit or os.environ.get(ENV_YOLO_WEIGHTS) or "").strip()
    path = Path(raw).expanduser() if raw else DEFAULT_YOLO_WEIGHTS
    if path.is_file():
        return str(path.resolve())
    if allow_download and not raw and path.resolve() == DEFAULT_YOLO_WEIGHTS.resolve():
        return str(download_default_weights(path))
    raise SystemExit(
        f"yolo: weights not found: {path} (project default: {DEFAULT_YOLO_WEIGHTS}, HF {HF_YOLO_REPO_ID})"
    )
