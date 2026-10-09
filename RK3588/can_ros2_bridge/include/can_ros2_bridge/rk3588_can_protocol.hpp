#ifndef CAN_ROS2_BRIDGE__RK3588_CAN_PROTOCOL_HPP_
#define CAN_ROS2_BRIDGE__RK3588_CAN_PROTOCOL_HPP_

#include <cstdint>

#include "can_ros2_bridge/can_frame.hpp"

namespace can_ros2_bridge
{

class Rk3588Protocol
{
public:
  static constexpr std::uint32_t kNodeId = 16U;
  static constexpr std::uint32_t kFeedbackIdOffset = 0x100U;
  static constexpr std::uint32_t kFeedbackId = kNodeId + kFeedbackIdOffset;
  static constexpr std::uint8_t kCommandChassisEnable = 0x00U;

  static bool is_heartbeat(const CanFrame & frame) noexcept;
  static CanFrame make_chassis_enable_frame(bool enabled) noexcept;
};

}  // namespace can_ros2_bridge

#endif  // CAN_ROS2_BRIDGE__RK3588_CAN_PROTOCOL_HPP_
