#ifndef CAN_ROS2_BRIDGE__MANUAL_ENABLE_HPP_
#define CAN_ROS2_BRIDGE__MANUAL_ENABLE_HPP_

#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace can_ros2_bridge
{

class ManualEnableController
{
public:
  using SendCommand = std::function<bool(std::uint32_t)>;

  ManualEnableController(rclcpp::Node & node, SendCommand send_command);
  void clear_state();
  void resend_current_state();

private:
  void handle_enable_bit(
    std::uint32_t bit_mask, const char * control_name, std::uint8_t value);

  rclcpp::Logger logger_;
  SendCommand send_command_;
  std::vector<rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr> subscriptions_;
  std::mutex mutex_;
  std::optional<std::uint32_t> control_bits_;
  std::optional<std::uint32_t> last_sent_control_bits_;
};

}  // namespace can_ros2_bridge

#endif  // CAN_ROS2_BRIDGE__MANUAL_ENABLE_HPP_
