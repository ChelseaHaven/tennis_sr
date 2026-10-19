#include "can_ros2_bridge/can_command_controller.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

#include "can_ros2_bridge/rk3588_protocol.hpp"

namespace can_ros2_bridge
{

namespace
{
std::int16_t to_int16_saturated(const double value)
{
  if (value >= 32767.0) {
    return 32767;
  }
  if (value <= -32768.0) {
    return -32768;
  }
  return static_cast<std::int16_t>(std::lround(value));
}
}  // namespace

CanCommandController::CanCommandController(
  rclcpp::Node & node, SendFrame send_frame,
  const std::chrono::milliseconds heartbeat_period)
: logger_(node.get_logger()), clock_(node.get_clock()), send_frame_(std::move(send_frame))
{
  if (!send_frame_) {
    throw std::invalid_argument("CAN frame sender must be set");
  }
  if (heartbeat_period <= std::chrono::milliseconds::zero()) {
    throw std::invalid_argument("Downlink heartbeat period must be positive");
  }

  const auto target_velocity_topic = node.declare_parameter<std::string>(
    "target_velocity_topic", "/cmd_vel");
  enable_subscriptions_.push_back(
    node.create_subscription<std_msgs::msg::UInt8>(
      "/manual_enable/chassis", 10,
      [this](const std_msgs::msg::UInt8::SharedPtr message) {
        handle_enable_bit(
          Rk3588Protocol::kManualEnableChassisBit, "chassis", message->data);
      }));
  enable_subscriptions_.push_back(
    node.create_subscription<std_msgs::msg::UInt8>(
      "/manual_enable/launcher", 10,
      [this](const std_msgs::msg::UInt8::SharedPtr message) {
        handle_enable_bit(
          Rk3588Protocol::kManualEnableLauncherBit, "launcher", message->data);
      }));
  target_velocity_subscription_ = node.create_subscription<geometry_msgs::msg::Twist>(
    target_velocity_topic, rclcpp::SensorDataQoS(),
    [this](const geometry_msgs::msg::Twist::SharedPtr message) {
      handle_target_velocity(*message);
    });

  heartbeat_timer_ = node.create_wall_timer(
    heartbeat_period, [this]() {send_heartbeat();});
  RCLCPP_INFO(
    logger_, "Manual enable topics: /manual_enable/chassis and "
    "/manual_enable/launcher (std_msgs/UInt8, data must be 0 or 1)");
  RCLCPP_INFO(logger_, "Sending target chassis velocity from '%s'", target_velocity_topic.c_str());
}

void CanCommandController::handle_enable_bit(
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

  if (send_manual_enable(updated_bits)) {
    last_sent_control_bits_ = updated_bits;
  }
}

void CanCommandController::handle_target_velocity(const geometry_msgs::msg::Twist & message)
{
  const double vx_mm_s = message.linear.x * 1000.0;
  const double vy_mm_s = message.linear.y * 1000.0;
  const double wz_mrad_s = message.angular.z * 1000.0;
  if (!std::isfinite(vx_mm_s) || !std::isfinite(vy_mm_s) || !std::isfinite(wz_mrad_s)) {
    RCLCPP_WARN(logger_, "Ignoring non-finite /cmd_vel chassis velocity");
    return;
  }

  const auto vx = to_int16_saturated(vx_mm_s);
  const auto vy = to_int16_saturated(vy_mm_s);
  const auto wz = to_int16_saturated(wz_mrad_s);
  if (!send_int16x3(Rk3588Protocol::kCommandTargetVelocity, vx, vy, wz)) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000, "Failed to send target chassis velocity over CAN");
  }
}

bool CanCommandController::send_manual_enable(const std::uint32_t control_bits)
{
  const auto frame = Rk3588Protocol::make_manual_enable_frame(control_bits);
  const bool sent = send_frame_(frame);
  if (sent) {
    RCLCPP_INFO(
      logger_, "Sent manual enable command: control_bits=0x%08X CAN ID=0x%X",
      static_cast<unsigned int>(control_bits), frame.id);
  }
  return sent;
}

bool CanCommandController::send_int16x3(
  const std::uint8_t command, const std::int16_t value0,
  const std::int16_t value1, const std::int16_t value2)
{
  const auto frame = Rk3588Protocol::make_int16x3_frame(command, value0, value1, value2);
  const bool sent = send_frame_(frame);
  if (sent) {
    RCLCPP_DEBUG(
      logger_, "Sent int16x3 command 0x%02X: values=[%d, %d, %d]",
      command, static_cast<int>(value0), static_cast<int>(value1), static_cast<int>(value2));
  }
  return sent;
}

void CanCommandController::send_heartbeat()
{
  const auto frame = Rk3588Protocol::make_heartbeat_frame();
  if (send_frame_(frame)) {
    RCLCPP_DEBUG(logger_, "Sent F407 heartbeat: CAN ID=0x%X", frame.id);
  }
}

void CanCommandController::resend_current_state()
{
  const std::lock_guard<std::mutex> lock(mutex_);
  if (!control_bits_.has_value()) {
    return;
  }

  if (send_manual_enable(control_bits_.value())) {
    last_sent_control_bits_ = control_bits_;
  }
}

void CanCommandController::clear_state()
{
  const std::lock_guard<std::mutex> lock(mutex_);
  control_bits_ = 0U;

  if (last_sent_control_bits_ == control_bits_) {
    return;
  }

  if (send_manual_enable(0U)) {
    last_sent_control_bits_ = 0U;
  }
}

}  // namespace can_ros2_bridge
