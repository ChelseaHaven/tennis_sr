#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
BRIDGE_SCRIPT="${SCRIPT_DIR}/../can_ros2_bridge/run.sh"
ROBOT_SCRIPT="${SCRIPT_DIR}/run_real_robot.sh"

if [[ ! -x "${BRIDGE_SCRIPT}" ]]; then
  echo "CAN bridge launcher is missing or not executable: ${BRIDGE_SCRIPT}" >&2
  exit 1
fi
if [[ ! -x "${ROBOT_SCRIPT}" ]]; then
  echo "Real robot launcher is missing or not executable: ${ROBOT_SCRIPT}" >&2
  exit 1
fi
if [[ ! -f "${SCRIPT_DIR}/install/setup.bash" ]]; then
  echo "Mecanum workspace is not built. Run ./RK3588/mecanum_ros2_ws/build.sh first." >&2
  exit 1
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
echo "Starting CAN bridge and real robot planner (ROS_DOMAIN_ID=${ROS_DOMAIN_ID})."
echo "Chassis enable is intentionally left disabled; publish it manually when safe."

child_pids=()
cleanup()
{
  trap - EXIT INT TERM
  for pid in "${child_pids[@]}"; do
    kill -INT "${pid}" 2>/dev/null || true
  done
  for pid in "${child_pids[@]}"; do
    wait "${pid}" 2>/dev/null || true
  done
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

"${BRIDGE_SCRIPT}" -p "log_can_frames:=${CAN_BRIDGE_LOG_FRAMES:-0}" &
bridge_pid=$!
child_pids+=("${bridge_pid}")

"${ROBOT_SCRIPT}" "$@" &
robot_pid=$!
child_pids+=("${robot_pid}")

# If either long-running component exits unexpectedly, stop the other as well.
set +e
wait -n "${bridge_pid}" "${robot_pid}"
status=$?
set -e
exit "${status}"
