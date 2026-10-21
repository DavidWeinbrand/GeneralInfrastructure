# CatSwarm General Infrastructure

## Idea
Host-side PX4 SITL launch, iris sensor inject, and Raspberry Pi companion process orchestration. Flight control and MAVLink codecs live in other trees; this repo starts those processes and shapes the simulated vehicle.

## Architecture

### Simulation
Images live in `Dockerfiles/` (nested `PX4dockerfiles`, PX4 **v1.17.0**):

- **Noetic** `px4-noetic-sim-ros` — Gazebo Classic multi-iris. Entry: `./runSimNoeticMulti.sh`.
- **Noble** `px4-noble-sim-ros` — Gazebo Jetty + FlightGear 2024.1.6. `./runSimNoble.sh` is a single `gz_x500`. Rascal is `make px4_sitl_nolockstep flightgear_rascal` inside the container. The image prebuilds `flightgear_bridge` only (`make … flightgear_rascal` always launches `fgfs`, so it is not a Docker build step). This tree has no `fixedwing/` host runner.

`runSimNoeticMulti.sh` (`--num`, `--file`, `--world`, `--cylinder-radius`, `--vio-cam`, `--vio-pitch`, `--kill`) bind-mounts the inject scripts and `multidrone/airframes/10015_gazebo-classic_iris.post`, then runs `multidrone/sitl_multiple_run.sh`. Gazebo is pinned to loopback (`GAZEBO_IP=127.0.0.1`, `GAZEBO_MASTER_URI=http://127.0.0.1:11345`). A missing non-empty world exits before the cleanup trap (a running sim is left alone).

By default **`CATSWARM_SIM_GPU=auto`** adds Docker `--gpus all` and NVIDIA GL env when `nvidia-smi` and the Docker nvidia runtime are present (typical laptop dGPU). Set `CATSWARM_SIM_GPU=off` to force software/Intel GL via X11 only, or `on` to require GPU passthrough.

Iris spawn order: jinja → `inject_iris_sensors.py` → `inject_iris_colors.py` → `gz model --spawn-file` → `bin/px4`. PX4 attaches `OPTICAL_FLOW_RAD` only when flow is already advertised inside `Sensors::init()`.

`CATSWARM_OF_MODE` default `mockup`: inline lidar (min 0.02 m) plus a headless optical-flow plugin (body velocity + lidar). `camera` or `both` also nest `model://px4flow` (needs GL). `CATSWARM_VIO_CAM=1` (default) adds fixed-joint `vio_cam` (320×240, 10 cm forward). Look direction is the SDF pose `CATSWARM_VIO_PITCH` (default −90 FRD, Gazebo +π/2). `sim_vio.sh pitch` is a no-op: `gz joint` on `vio_cam_pitch` collapses the iris to the origin.

`inject_iris_colors.py` paints `iris_N` with the ObservationBoard palette (`web/frontend/src/utils/droneColors.ts`, drone id `N`).

Boot params in `10015_gazebo-classic_iris.post`: `DISTANCE_SENSOR` 20 Hz and `OPTICAL_FLOW_RAD` 50 Hz on both the GCS and offboard mavlink ports; `EKF2_GPS_CTRL 7` and `SYS_HAS_GPS 1` (GPS fusion on; `SYS_HAS_GPS` applies only from this boot); `EKF2_OF_CTRL 0` (stream flow, do not fuse it with GPS); `SENS_FLOW_ROT 0` (mockup is body-frame; yaw 270 is the real px4flow camera); `SENS_FLOW_MINHGT 0.02`. A separate onboard link carries VIO IMU: PX4 local `14680+px4_instance`, remote `14640+px4_instance`, `HIGHRES_IMU` 100 Hz. It does not share the hardware-adapter port `14540`.

After spawn, `multidrone/vio_cam_tcp.py` (skipped when `--vio-cam=0`) republishes `/iris_{id}/vio_cam/image_raw`: SVOF grayscale TCP on `5600+(id-1)`, JPEG HTTP on `5700+(id-1)`. The listen socket is blocking so rospy’s process-wide socket timeout cannot kill `accept`.

Worlds (`--world`, default `empty`):

- `empty` — PX4 empty world.
- `cylinders` — empty world plus four visual cylinders (`spawn_cylinders.py`): diameter 0.50 m, height 1.0 m, spawn `-z 0`, link pose at `height/2` so the base sits on z=0. Disk radius `--cylinder-radius` (default 10) around the first `positions.txt` XY. Colors blue, red, green, yellow.
- any other `NAME` — host `Dockerfiles/models/NAME/` must contain `NAME.world` and `origin.json`. The models tree is mounted at `.../models/catswarm_host` (PX4 `iris` is not overlaid). `PX4_HOME_*` is taken from `origin.json`.

Generate a map on the host, from the CatSwarm root (not inside the image, and the GUI does not call it):

```bash
python3 general_infrastructure/Dockerfiles/scripts/generate_real_area.py \
  --lat LAT --lon LON --radius-m METERS \
  --out general_infrastructure/Dockerfiles/models/NAME
```

The GUI lists `empty`, `cylinders`, and host directories that already have `<name>.world`.

`multidrone/positions.txt` is one `x y [z]` per drone (z defaults to 0.83). Instance numbers start at 1.

Desktop MAVLink↔ZMQ binaries are built in **hardware_adapter**. `multidrone/run_multidrone_bridges.sh` starts C `mavlink_to_ZMQ` and `zmq_commands_mavlink` from `../hardware_adapter/bin` (one tmux window per positions line): telemetry UDP `14540+id` / ZMQ `9900+id`, commands UDP `14030+id` / ZMQ `7700+id`. Do not pass `--zmqFlightData` to the C command bridge (unknown flag, exit 1). That relative path is `general_infrastructure/hardware_adapter`, which this checkout does not contain; the sibling tree is `CatSwarm/hardware_adapter`.

Manual checks: `multidrone/mavlinkTakeoffLandAlt.py` (`--takeoff` / `--land`, `--udp` default 14541) and `multidrone/zmqTakeoffLandAlt.py` (command ZMQ default 7700). The GUI Start Sim / takeoff / mode paths call `runSimNoeticMulti.sh` and these scripts.

### Companion (Raspberry Pi 5)
Sources: `deploymentScript/startup_scripts/` and `deploymentScript/deploy_pi5/`. On the Pi they are `~/RL/startup_scripts` and `~/deploy_pi5`. `CATSWARM_ROOT` is the parent of `startup_scripts` (`~/RL`) and must contain `hardware_adapter/`, `system_manager/`, and `GPS_RTK/`.

`start_companion_drone_tmux.sh <id>` (default `--version=CPP`) opens tmux `catswarm_sim` with `TMUX_TMPDIR=/tmp` (so an SSH session finds the systemd tmux server):

- Window `hardware_adapter_<id>` via `hardware_adapter/hardware_adapter_multi.sh`. The bottom pane is then `system_manager/SystemManagerMain`, or `system_manager/system_managerPY/system_manager.py` when `--version=PY`. The drone index matches `system_manager/MultiInput/multiSetup.list`.
- GPS window: `GPS_RTK/startRtkCommPI.sh` (`rover_zmq` + `emulate_gps_to_px4`), or `GPS_RTK/startRtcmToMavlinkPI.sh` when the RTK sink is `mavlink_rtcm`.
- Window `vio_<id>`: SchurVINS `svo_pi` on the CSI camera. A missing camera or binary parks the window and does not fail companion start. Use the conda interpreter when it can `import picamera2`; otherwise `/usr/bin/python3`. `SCHURVINS_ROOT` is `${CATSWARM_ROOT}/SchurVINS` when that directory exists, otherwise the parent directory’s `SchurVINS`.

Fleet MAVLink is a single port, not the desktop per-id offsets: telemetry `127.0.0.1:14540`, commands `udpout:127.0.0.1:14580`, target system 1. `mavlink-server`’s UDP server must include `address = "0.0.0.0"` on port 14580; without it the RTCM inject has no listener.

UART: `/dev/ttyAMA0` NMEA to PX4 (GPIO14 TX, header pin 8); `/dev/ttyAMA2` ground-station radio; `/dev/ttyAMA3` PX4 telemetry (`mavlink-server`); `/dev/ttyAMA4` optional DA rover. EA LC29H default `/dev/ttyUSB0` at 460800 / 10 Hz. F9P is ACM at 115200 or `ttyUSB*` at 230400, 1 Hz, `ROVER_WIRE=ubx`. Boot keeps `~/.config/companion-gps` when that tty exists; otherwise it sniffs and persists.

Operator switches restart one pane and do not clobber the other config file:

- `switch_rtk_WIFI_RF.sh` — `~/.config/companion-rtk`, WiFi vs serial RF, sink `rover_uart` or `mavlink_rtcm`.
- `switch_comm_WIFI_RF.sh` — `~/.config/companion-comm`, fabric `rf` (default) or `wifi`. WiFi adds `--wifi-comm-uplink=tcp://<gs>:18811` and `--wifi-comm-downlink=tcp://<gs>:18812` and omits `--serial-comm-tx`.

Both relaunch `ZMQ_to_comm` with flight-data `21700+id`, comm pub `7800+id`, neighbour sub `22700+id`, mavlink fallback `9900+id`. ObservationBoard Apply RTK / Apply COMM SSH these scripts.

Host SITL VIO (separate from the Pi CSI window) is `deploymentScript/startup_scripts/util/sim_vio.sh`: `svo_pi --source gazebo` on `udp:127.0.0.1:$((14640+id))`, with host SchurVINS library paths. A skip does not fail the swarm. `pitch` exits 0 and does not call `gz`.

After the windows exist, pane stdout is pipe-paned to `~/RL/logs/tmux/<UTC>_<session_window_pane>.log`. `install-companion-boot.sh` sets journald `Storage=persistent` and enables `companion-drone.service`.

`util/qgc_mavlink_streams.py` requests `HIGHRES_IMU` (msgid 105, 100 Hz) and `OPTICAL_FLOW_RAD` (msgid 106, 50 Hz) on `tcp:127.0.0.1:5760`, refreshed every 10 s. A global `--hz` overrides both rates; leave it unset so HIGHRES stays at 100 Hz.

`report_version.sh` POSTs the component git commit to the GUI (`CATSWARM_GUI_URL`, default `http://localhost:3001`, path `/api/drone/<id>/version`). Failure does not block startup.

### Offline policy code
`rl_policy.py`, `rlPolicyCPP/`, `model_v1`–`model_v3`, `rl_cnn_fov_forward_31_5/`, and `pth2json.py` train or run policies offline. The sim and companion launchers do not start them. `runOnBoardCPP.sh`, `runOnBoardPY.sh`, and `hardware_adapter.sh` are older single-drone tmux helpers; the Pi entry point is `start_companion_drone_tmux.sh`.

`compile.sh` builds `$repo/hardware_adapter` and `$repo/system_managerCPP`. Those directories are not in this checkout. The sibling projects are `hardware_adapter/` and `system_manager/system_managerCPP`.

## Reading order for agents
1. Read this `README.md` (mandatory if present).
2. Read `UPDATES.md` (mandatory) for the change history and current state before working.
