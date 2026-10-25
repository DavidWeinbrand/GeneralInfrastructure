# shellcheck shell=bash
# Sourced by runSimNoeticMulti.sh — Docker flags for Gazebo OpenGL (gzclient/gzserver).
#
# CATSWARM_SIM_GPU: auto (default) | on | off
#   auto — --gpus all when nvidia-smi and Docker nvidia runtime are available
#   on   — always --gpus all (exit with error if toolkit/driver missing)
#   off  — no GPU flags (software/Intel GL via X11 only)

CATSWARM_DOCKER_GPU_RUN_ARGS=()
CATSWARM_DOCKER_GPU_ENV=()

catswarm_sim_gpu_mode() {
    case "${CATSWARM_SIM_GPU:-auto}" in
        0|off|false|no|disable) echo off ;;
        1|on|true|yes|nvidia|gpu) echo on ;;
        auto|*) echo auto ;;
    esac
}

catswarm_docker_has_nvidia_runtime() {
    docker info 2>/dev/null | grep -qiE '(^|[[:space:]])nvidia'
}

catswarm_host_has_nvidia_gpu() {
    command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi -L >/dev/null 2>&1
}

catswarm_append_docker_gpu() {
    CATSWARM_DOCKER_GPU_RUN_ARGS=()
    CATSWARM_DOCKER_GPU_ENV=()

    local mode use_gpu=0
    mode="$(catswarm_sim_gpu_mode)"

    case "$mode" in
        off)
            use_gpu=0
            ;;
        on)
            if ! catswarm_host_has_nvidia_gpu; then
                echo "ERROR: CATSWARM_SIM_GPU=on but nvidia-smi cannot talk to the driver." >&2
                exit 1
            fi
            if ! catswarm_docker_has_nvidia_runtime; then
                echo "ERROR: CATSWARM_SIM_GPU=on but Docker has no NVIDIA runtime (install nvidia-container-toolkit)." >&2
                exit 1
            fi
            use_gpu=1
            ;;
        auto|*)
            if catswarm_host_has_nvidia_gpu && catswarm_docker_has_nvidia_runtime; then
                use_gpu=1
            fi
            ;;
    esac

    if [ "$use_gpu" -eq 1 ]; then
        CATSWARM_DOCKER_GPU_RUN_ARGS=(--gpus all)
        CATSWARM_DOCKER_GPU_ENV=(
            --env=NVIDIA_VISIBLE_DEVICES=all
            --env=NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute
            --env=__GLX_VENDOR_LIBRARY_NAME=nvidia
            --env=__NV_PRIME_RENDER_OFFLOAD=1
            --env=__VK_LAYER_NV_optimus=NVIDIA_only
        )
        echo "CatSwarm sim: NVIDIA GPU enabled for Gazebo (CATSWARM_SIM_GPU=${CATSWARM_SIM_GPU:-auto})."
    else
        echo "CatSwarm sim: Gazebo without GPU passthrough (CATSWARM_SIM_GPU=${CATSWARM_SIM_GPU:-auto})."
    fi
}
