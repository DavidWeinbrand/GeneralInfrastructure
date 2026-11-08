#!/usr/bin/env bash
# Refresh multidrone/vendor/citysim from https://github.com/osrf/citysim
# and point multidrone/worlds/citysim/citysim.world at simple_city.world.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENDOR="${ROOT}/vendor/citysim"
WORLD_DIR="${ROOT}/worlds/citysim"
UPSTREAM="${UPSTREAM:-https://github.com/osrf/citysim.git}"
REF="${REF:-master}"
TMP="$(mktemp -d)"

cleanup() { rm -rf "${TMP}"; }
trap cleanup EXIT

git clone --depth 1 --branch "${REF}" "${UPSTREAM}" "${TMP}/citysim"
COMMIT="$(git -C "${TMP}/citysim" rev-parse HEAD)"

mkdir -p "${VENDOR}"
rsync -a --delete \
  --exclude '.git' \
  "${TMP}/citysim/" "${VENDOR}/"

mkdir -p "${WORLD_DIR}"
python3 "${ROOT}/scripts/build_citysim_sitl_world.py"
python3 "${ROOT}/scripts/sync_citysim_gazebo_models.py"
WORLD_LINK="${WORLD_DIR}/citysim.world"
rm -f "${WORLD_LINK}"
ln -sf "citysim_sitl.world" "${WORLD_LINK}"

cat > "${VENDOR}/CATSWARM_SOURCE.txt" <<EOF
Upstream: ${UPSTREAM}
Ref: ${REF}
Commit: ${COMMIT}
Synced: $(date -u +"%Y-%m-%dT%H:%M:%SZ")
World used in sim: worlds/citysim/citysim_sitl.world (plugins stripped for Docker SITL)
EOF

echo "Synced osrf/citysim @ ${COMMIT:0:12} into ${VENDOR}"
echo "citysim.world -> simple_city.world"
