#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$(readlink -f "$0")")"
source /opt/ros/jazzy/setup.bash
export ROS_LOG_DIR="$PWD/.ros/log"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
if [[ ! -x .venv/bin/python ]]; then
    /usr/bin/python3 -m venv --system-site-packages .venv
fi
.venv/bin/python -m pip install --disable-pip-version-check --no-cache-dir -r requirements.txt
colcon build --symlink-install --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DPython3_EXECUTABLE=/usr/bin/python3
