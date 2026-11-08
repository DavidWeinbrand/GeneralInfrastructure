# YOLO weights (not in git)

Area-scan sim detection uses **[dronefreak/visdrone-yolo11s](https://huggingface.co/dronefreak/visdrone-yolo11s)** (VisDrone, YOLO11s).

Place the checkpoint here as **`visdrone_yolo11s.pt`** (~19 MB), or let `yolo_det_bridge.py` download `best.pt` from Hugging Face on first run (needs `huggingface_hub`).

Manual download:

```bash
huggingface-cli download dronefreak/visdrone-yolo11s best.pt \
  --local-dir "$(dirname "$0")" \
  && mv best.pt visdrone_yolo11s.pt
```

Override path: `CATSWARM_YOLO_WEIGHTS=/path/to/model.pt`.

Legacy **`visdrone_yolov8m.pt`** is not used by the bridge anymore.
