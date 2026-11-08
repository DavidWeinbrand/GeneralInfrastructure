# CitySim world (Gazebo Classic)

The sim loads **`citysim_sitl.world`** (osrf `simple_city.world` with Docker-only plugins removed). Regenerate after syncing upstream:

```bash
multidrone/scripts/sync_osrf_citysim.sh
```

- `citysim.world` → `citysim_sitl.world`
- Prop models: `multidrone/vendor/gazebo_models` (run `sync_citysim_gazebo_models.py` if lights/cars fail to load)
- Terrain, ocean, and walking actors come from `vendor/citysim/models/`.

Start:

```bash
./runSimNoeticMulti.sh --world citysim --num 4
```

Drone pads use Z=8 m in `sitl_multiple_run.sh` so irises spawn on the road surface.
