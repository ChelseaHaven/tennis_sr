#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
CAN_INTERFACE="${CAN_INTERFACE:-can0}"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
NODE_EXECUTABLE="${WORKSPACE_DIR}/install/can_ros2_bridge/lib/can_ros2_bridge/can_receiver_node"

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "找不到 ROS 2 环境：${ROS_SETUP}" >&2
  echo "请确认已安装 ROS 2 ${ROS_DISTRO}，或设置 ROS_DISTRO。" >&2
  exit 1
fi

source "${ROS_SETUP}"

if [[ ! -x "${NODE_EXECUTABLE}" ]]; then
  echo "尚未构建 can_ros2_bridge，开始构建..."
  cd "${WORKSPACE_DIR}"
  colcon build --packages-select can_ros2_bridge
fi

if [[ ! -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
  echo "找不到工作区环境：${WORKSPACE_DIR}/install/setup.bash" >&2
  exit 1
fi

source "${WORKSPACE_DIR}/install/setup.bash"

if [[ ! -e "/sys/class/net/${CAN_INTERFACE}" ]]; then
  echo "找不到 CAN 网卡 '${CAN_INTERFACE}'。可用 CAN_INTERFACE=can1 $0 指定接口。" >&2
  exit 1
fi

exec ros2 run can_ros2_bridge can_receiver_node \
  --ros-args -p "interface:=${CAN_INTERFACE}" "$@"
