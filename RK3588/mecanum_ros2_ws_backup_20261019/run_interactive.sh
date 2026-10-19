#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$(readlink -f "$0")")"
launch_args=()
for arg in "$@"; do
    case "$arg" in
        --local-display)
            # Explicitly select the board's desktop, not the SSH client's screen.
            export DISPLAY="${MECANUM_LOCAL_DISPLAY:-:0}"
            for auth_file in "/run/user/$(id -u)"/.mutter-Xwaylandauth.*; do
                if [[ -r "$auth_file" ]]; then
                    export XAUTHORITY="$auth_file"
                    break
                fi
            done
            ;;
        --help|-h)
            echo '用法：./run_interactive.sh [--local-display] [gui:=true] [rviz:=false]'
            echo '显示到电脑：从电脑用 ssh -Y pi@192.168.1.88 连接后运行。'
            echo '显示到板子本机桌面：./run_interactive.sh --local-display'
            exit 0
            ;;
        *) launch_args+=("$arg") ;;
    esac
done
source /opt/ros/jazzy/setup.bash
if [[ ! -f install/setup.bash ]]; then
    echo '请先执行 ./build.sh' >&2
    exit 1
fi
source install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-72}"
export PYTHONPATH="$PWD/.venv/lib/python3.12/site-packages:${PYTHONPATH:-}"
export ROS_LOG_DIR="$PWD/.ros/log"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
export QT_QPA_PLATFORM=xcb
# Preserve SSH-forwarded DISPLAY and its Xauthority; never silently switch desktops.
if [[ -z ${DISPLAY:-} && " $* " != *' rviz:=false '* ]]; then
    echo '未设置 DISPLAY。请在桌面终端运行，或使用 ssh -Y 连接后运行。' >&2
    echo '显示到板子本机桌面：./run_interactive.sh --local-display' >&2
    echo '显示到电脑：请在电脑端用 ssh -Y pi@192.168.1.88 重新连接。' >&2
    exit 1
fi
exec ros2 launch fast_fly_ros2 interactive_sim.launch.py "${launch_args[@]}"
