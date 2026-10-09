#ifndef CAN_ROS2_BRIDGE__CAN_FRAME_HPP_
#define CAN_ROS2_BRIDGE__CAN_FRAME_HPP_

#include <array>
#include <cstddef>
#include <cstdint>

namespace can_ros2_bridge
{

struct CanFrame
{
  std::uint32_t id{0U};
  std::array<std::uint8_t, 8U> data{};
  std::size_t length{0U};
  bool is_extended{false};
};

}  // namespace can_ros2_bridge

#endif  // CAN_ROS2_BRIDGE__CAN_FRAME_HPP_
