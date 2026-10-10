#include "can_ros2_bridge/rk3588_can_protocol.hpp"

namespace can_ros2_bridge
{

bool Rk3588Protocol::is_heartbeat(const CanFrame & frame) noexcept
{
  return !frame.is_extended && frame.id == kNodeId &&
         frame.length == 1U && frame.data[0] == 0x00U;
}

CanFrame Rk3588Protocol::make_global_enable_frame(const bool enabled) noexcept
{
  CanFrame frame;
  frame.id = kFeedbackId;
  frame.length = 2U;
  frame.data[0] = kCommandGlobalEnable;
  frame.data[1] = static_cast<std::uint8_t>(enabled ? 1U : 0U);
  return frame;
}

CanFrame Rk3588Protocol::make_manual_enable_frame(const std::uint32_t control_bits) noexcept
{
  CanFrame frame;
  frame.id = kFeedbackId;
  frame.length = 5U;
  frame.data[0] = kCommandManualEnable;
  frame.data[1] = static_cast<std::uint8_t>(control_bits & 0xFFU);
  frame.data[2] = static_cast<std::uint8_t>((control_bits >> 8U) & 0xFFU);
  frame.data[3] = static_cast<std::uint8_t>((control_bits >> 16U) & 0xFFU);
  frame.data[4] = static_cast<std::uint8_t>((control_bits >> 24U) & 0xFFU);
  return frame;
}

}  // namespace can_ros2_bridge
