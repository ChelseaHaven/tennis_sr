#include "can_ros2_bridge/manual_enable.hpp"

#include <utility>

#include "can_ros2_bridge/rk3588_protocol.hpp"

namespace can_ros2_bridge
{

ManualEnableController::ManualEnableController(
  rclcpp::Node & node, SendCommand send_command)
: logger_(node.get_logger()), send_command_(std::move(send_command))
{
  subscriptions_.push_back(
    node.create_subscription<std_msgs::msg::UInt8>(
      "/manual_enable/chassis", 10,
      [this](const std_msgs::msg::UInt8::SharedPtr message) {
        handle_enable_bit(
          Rk3588Protocol::kManualEnableChassisBit, "chassis", message->data);
      }));
  subscriptions_.push_back(
    node.create_subscription<std_msgs::msg::UInt8>(
      "/manual_enable/launcher", 10,
      [this](const std_msgs::msg::UInt8::SharedPtr message) {
        handle_enable_bit(
          Rk3588Protocol::kManualEnableLauncherBit, "launcher", message->data);
      }));

  RCLCPP_INFO(
    logger_, "Manual enable topics: /manual_enable/chassis and "
    "/manual_enable/launcher (std_msgs/UInt8, data must be 0 or 1)");
}

void ManualEnableController::handle_enable_bit(
  const std::uint32_t bit_mask, const char * control_name, const std::uint8_t value)
{
  if (value > 1U) {
    RCLCPP_WARN(
      logger_, "Ignoring invalid value %u for %s enable; expected 0 or 1",
      static_cast<unsigned int>(value), control_name);
    return;
  }

  const std::lock_guard<std::mutex> lock(mutex_);
  const std::uint32_t current_bits = control_bits_.value_or(0U);
  const std::uint32_t updated_bits = value == 1U ? current_bits | bit_mask :
    current_bits & ~bit_mask;
  control_bits_ = updated_bits;

  if (last_sent_control_bits_ == control_bits_) {
    return;
  }

  if (send_command_(updated_bits)) {
    last_sent_control_bits_ = updated_bits;
  }
}

void ManualEnableController::resend_current_state()
{
  const std::lock_guard<std::mutex> lock(mutex_);
  if (!control_bits_.has_value()) {
    return;
  }

  if (send_command_(control_bits_.value())) {
    last_sent_control_bits_ = control_bits_;
  }
}

void ManualEnableController::clear_state()
{
  const std::lock_guard<std::mutex> lock(mutex_);
  control_bits_ = 0U;

  if (last_sent_control_bits_ == control_bits_) {
    return;
  }

  if (send_command_(0U)) {
    last_sent_control_bits_ = 0U;
  }
}

}  // namespace can_ros2_bridge

