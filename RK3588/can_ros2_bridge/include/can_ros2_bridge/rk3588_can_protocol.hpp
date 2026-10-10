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
  static constexpr std::uint8_t kCommandGlobalEnable = 0x00U;
  static constexpr std::uint8_t kCommandManualEnable = 0x01U;
  static constexpr std::uint32_t kManualEnableChassisBit = 1U << 0U;
  static constexpr std::uint32_t kManualEnableLauncherBit = 1U << 1U;

  static bool is_heartbeat(const CanFrame & frame) noexcept;
  static CanFrame make_global_enable_frame(bool enabled) noexcept;
  static CanFrame make_manual_enable_frame(std::uint32_t control_bits) noexcept;
};

}  // namespace can_ros2_bridge

#endif  // CAN_ROS2_BRIDGE__RK3588_CAN_PROTOCOL_HPP_
