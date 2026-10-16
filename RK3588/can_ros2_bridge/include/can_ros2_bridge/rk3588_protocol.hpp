#ifndef CAN_ROS2_BRIDGE__RK3588_PROTOCOL_HPP_
#define CAN_ROS2_BRIDGE__RK3588_PROTOCOL_HPP_

#include <array>
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

  // Commands received from the MCU on kNodeId.
  static constexpr std::uint8_t kCommandHeartbeat = 0x00U;
  static constexpr std::uint8_t kCommandImuGyro = 0x01U;
  static constexpr std::uint8_t kCommandImuAccel = 0x02U;
  static constexpr std::uint8_t kCommandImuAttitude = 0x03U;
  static constexpr std::uint8_t kCommandChassisVelocity = 0x04U;
  static constexpr std::uint8_t kCommandChassisPosition = 0x05U;

  // Commands sent to the MCU on kFeedbackId.
  static constexpr std::uint8_t kCommandGlobalEnable = 0x00U;
  static constexpr std::uint8_t kCommandManualEnable = 0x01U;
  static constexpr std::uint8_t kCommandTargetVelocity = 0x02U;
  static constexpr std::uint32_t kManualEnableChassisBit = 1U << 0U;
  static constexpr std::uint32_t kManualEnableLauncherBit = 1U << 1U;

  static bool is_heartbeat(const CanFrame & frame) noexcept;
  static bool parse_int16x3(
    const CanFrame & frame, std::uint8_t command,
    std::array<std::int16_t, 3U> & values) noexcept;
  static CanFrame make_global_enable_frame(bool enabled) noexcept;
  static CanFrame make_manual_enable_frame(std::uint32_t control_bits) noexcept;
  static CanFrame make_int16x3_frame(
    std::uint8_t command, std::int16_t value0,
    std::int16_t value1, std::int16_t value2) noexcept;
};

}  // namespace can_ros2_bridge

#endif  // CAN_ROS2_BRIDGE__RK3588_PROTOCOL_HPP_
