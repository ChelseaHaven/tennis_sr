#include "can_ros2_bridge/rk3588_protocol.hpp"

namespace can_ros2_bridge
{

bool Rk3588Protocol::is_heartbeat(const CanFrame & frame) noexcept
{
  return !frame.is_extended && frame.id == kNodeId &&
         frame.length == 1U && frame.data[0] == kCommandHeartbeatFromMcu;
}

bool Rk3588Protocol::parse_int16x3(
  const CanFrame & frame, const std::uint8_t command,
  std::array<std::int16_t, 3U> & values) noexcept
{
  if (frame.is_extended || frame.id != kNodeId || frame.length != 7U ||
    frame.data[0] != command)
  {
    return false;
  }

  for (std::size_t i = 0U; i < values.size(); ++i) {
    const auto offset = 1U + (2U * i);
    const auto raw = static_cast<std::uint16_t>(
      (static_cast<std::uint16_t>(frame.data[offset]) << 8U) |
      static_cast<std::uint16_t>(frame.data[offset + 1U]));
    const auto signed_value = raw <= 0x7FFFU ?
      static_cast<std::int32_t>(raw) : static_cast<std::int32_t>(raw) - 0x10000;
    values[i] = static_cast<std::int16_t>(signed_value);
  }

  return true;
}

CanFrame Rk3588Protocol::make_heartbeat_frame() noexcept
{
  CanFrame frame;
  frame.id = kFeedbackId;
  frame.length = 1U;
  frame.data[0] = kCommandHeartbeatToMcu;
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

CanFrame Rk3588Protocol::make_int16x3_frame(
  const std::uint8_t command, const std::int16_t value0,
  const std::int16_t value1, const std::int16_t value2) noexcept
{
  CanFrame frame;
  const std::array<std::int16_t, 3U> values{value0, value1, value2};
  frame.id = kFeedbackId;
  frame.length = 7U;
  frame.data[0] = command;
  for (std::size_t i = 0U; i < values.size(); ++i) {
    const auto bits = static_cast<std::uint16_t>(values[i]);
    frame.data[1U + (2U * i)] = static_cast<std::uint8_t>(bits >> 8U);
    frame.data[2U + (2U * i)] = static_cast<std::uint8_t>(bits & 0xFFU);
  }
  return frame;
}

}  // namespace can_ros2_bridge
