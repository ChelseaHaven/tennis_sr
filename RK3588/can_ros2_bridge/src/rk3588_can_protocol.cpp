#include "can_ros2_bridge/rk3588_can_protocol.hpp"

namespace can_ros2_bridge
{

bool Rk3588Protocol::is_heartbeat(const CanFrame & frame) noexcept
{
  return !frame.is_extended && frame.id == kNodeId &&
         frame.length == 1U && frame.data[0] == 0x00U;
}

CanFrame Rk3588Protocol::make_chassis_enable_frame(const bool enabled) noexcept
{
  CanFrame frame;
  frame.id = kFeedbackId;
  frame.length = 2U;
  frame.data[0] = kCommandChassisEnable;
  frame.data[1] = static_cast<std::uint8_t>(enabled ? 1U : 0U);
  return frame;
}

}  // namespace can_ros2_bridge
