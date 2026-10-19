#ifndef CAN_ROS2_BRIDGE__CAN_COMMAND_CONTROLLER_HPP_
#define CAN_ROS2_BRIDGE__CAN_COMMAND_CONTROLLER_HPP_

#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <vector>

#include "can_ros2_bridge/can_frame.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace can_ros2_bridge
{

class CanCommandController
{
public:
  using SendFrame = std::function<bool(const CanFrame &)>;

  CanCommandController(
    rclcpp::Node & node, SendFrame send_frame,
    std::chrono::milliseconds heartbeat_period);

  // Cached control state -> CAN commands when F407 reconnects or times out.
  void clear_state();
  void resend_current_state();

private:
  void handle_enable_bit(
    std::uint32_t bit_mask, const char * control_name, std::uint8_t value);
  void handle_target_velocity(const geometry_msgs::msg::Twist & message);
  bool send_manual_enable(std::uint32_t control_bits);
  bool send_int16x3(
    std::uint8_t command, std::int16_t value0,
    std::int16_t value1, std::int16_t value2);
  void send_heartbeat();

  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  SendFrame send_frame_;
  std::vector<rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr> enable_subscriptions_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr target_velocity_subscription_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  std::mutex mutex_;
  std::optional<std::uint32_t> control_bits_;
  std::optional<std::uint32_t> last_sent_control_bits_;
};

}  // namespace can_ros2_bridge

#endif  // CAN_ROS2_BRIDGE__CAN_COMMAND_CONTROLLER_HPP_
