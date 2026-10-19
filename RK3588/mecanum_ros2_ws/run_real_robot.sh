#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$(readlink -f "$0")")"
launch_args=()
for arg in "$@"; do
    case "$arg" in
        --local-display)
            export DISPLAY="${MECANUM_LOCAL_DISPLAY:-:0}"
            for auth_file in "/run/user/$(id -u)"/.mutter-Xwaylandauth.*; do
                if [[ -r "$auth_file" ]]; then
                    export XAUTHORITY="$auth_file"
                    break
                fi
            done
            ;;
        --help|-h)
            echo 'Usage: ./run_real_robot.sh [--local-display] [rviz:=false] [v_max:=0.3] [a_max:=0.5]'
            echo 'Run with the CAN bridge in the same ROS_DOMAIN_ID (defaults to 0).'
            exit 0
            ;;
        *) launch_args+=("$arg") ;;
    esac
done
source /opt/ros/jazzy/setup.bash
if [[ ! -f install/setup.bash ]]; then
    echo 'Please run ./build.sh first.' >&2
    exit 1
fi
source install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export PYTHONPATH="$PWD/.venv/lib/python3.12/site-packages:${PYTHONPATH:-}"
export ROS_LOG_DIR="$PWD/.ros/log"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
export QT_QPA_PLATFORM=xcb
if [[ -z ${DISPLAY:-} && " ${launch_args[*]} " != *' rviz:=false '* ]]; then
    echo 'DISPLAY is unset; run from a desktop/X11-forwarded terminal or use --local-display.' >&2
    exit 1
fi
exec ros2 launch fast_fly_ros2 real_robot.launch.py "${launch_args[@]}"
