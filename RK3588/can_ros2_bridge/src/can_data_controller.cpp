#include "can_ros2_bridge/can_data_controller.hpp"

#include <array>
#include <cmath>
#include <cstdint>
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

CanDataController::CanDataController(
  rclcpp::Node & node, SendCommand send_command, SendInt16x3 send_int16x3)
: logger_(node.get_logger()), send_command_(std::move(send_command)),
  send_int16x3_(std::move(send_int16x3))
{
  clock_ = node.get_clock();
  const auto gyro_topic = node.declare_parameter<std::string>("gyro_topic", "/imu/gyro");
  const auto accel_topic = node.declare_parameter<std::string>("accel_topic", "/imu/accel");
  const auto attitude_topic = node.declare_parameter<std::string>("attitude_topic", "/imu/euler");
  const auto chassis_velocity_topic = node.declare_parameter<std::string>(
    "chassis_velocity_topic", "/chassis/velocity");
  const auto chassis_position_topic = node.declare_parameter<std::string>(
    "chassis_position_topic", "/chassis/position");
  const auto target_velocity_topic = node.declare_parameter<std::string>(
    "target_velocity_topic", "/cmd_vel");
  imu_frame_id_ = node.declare_parameter<std::string>("imu_frame_id", "imu_link");
  chassis_frame_id_ = node.declare_parameter<std::string>("chassis_frame_id", "base_link");
  odom_frame_id_ = node.declare_parameter<std::string>("odom_frame_id", "odom");
  gyro_publisher_ = node.create_publisher<geometry_msgs::msg::Vector3Stamped>(
    gyro_topic, rclcpp::SensorDataQoS());
  accel_publisher_ = node.create_publisher<geometry_msgs::msg::Vector3Stamped>(
    accel_topic, rclcpp::SensorDataQoS());
  attitude_publisher_ = node.create_publisher<geometry_msgs::msg::Vector3Stamped>(
    attitude_topic, rclcpp::SensorDataQoS());
  chassis_velocity_publisher_ = node.create_publisher<geometry_msgs::msg::Vector3Stamped>(
    chassis_velocity_topic, rclcpp::SensorDataQoS());
  chassis_position_publisher_ = node.create_publisher<geometry_msgs::msg::Vector3Stamped>(
    chassis_position_topic, rclcpp::SensorDataQoS());

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
  target_velocity_subscription_ = node.create_subscription<geometry_msgs::msg::Twist>(
    target_velocity_topic, rclcpp::SensorDataQoS(),
    [this](const geometry_msgs::msg::Twist::SharedPtr message) {
      handle_target_velocity(*message);
    });

  RCLCPP_INFO(
    logger_, "Manual enable topics: /manual_enable/chassis and "
    "/manual_enable/launcher (std_msgs/UInt8, data must be 0 or 1)");
  RCLCPP_INFO(
    logger_, "Publishing gyro='%s', accel='%s', pitch/roll/yaw='%s' (frame_id='%s')",
    gyro_topic.c_str(), accel_topic.c_str(), attitude_topic.c_str(), imu_frame_id_.c_str());
  RCLCPP_INFO(
    logger_, "Publishing chassis velocity='%s' and position='%s'",
    chassis_velocity_topic.c_str(), chassis_position_topic.c_str());
  RCLCPP_INFO(logger_, "Sending target chassis velocity from '%s'", target_velocity_topic.c_str());
}

void CanDataController::handle_can_frame(const CanFrame & frame)
{
  std::array<std::int16_t, 3U> values{};
  using Protocol = Rk3588Protocol;

  if (Protocol::parse_int16x3(frame, Protocol::kCommandChassisVelocity, values)) {
    chassis_velocity_raw_ = values;
    chassis_velocity_pending_ = true;
  } else if (Protocol::parse_int16x3(frame, Protocol::kCommandChassisPosition, values)) {
    chassis_position_raw_ = values;
    chassis_position_pending_ = true;
  } else if (Protocol::parse_int16x3(frame, Protocol::kCommandImuGyro, values)) {
    gyro_raw_ = values;
    gyro_pending_ = true;
  } else if (Protocol::parse_int16x3(frame, Protocol::kCommandImuAccel, values)) {
    accel_raw_ = values;
    accel_pending_ = true;
  } else if (Protocol::parse_int16x3(frame, Protocol::kCommandImuAttitude, values)) {
    attitude_raw_ = values;
    attitude_pending_ = true;
  } else {
    return;
  }

  publish_pending_data();
}

void CanDataController::publish_pending_data()
{
  constexpr double kGyroRawToDegreesPerSec = 2000.0 / 32768.0;
  constexpr double kAccelRawToG = 16.0 / 32768.0;
  constexpr double kAngleRawToDegrees = 180.0 / 32768.0;
  constexpr double kChassisLinearVelocityRawToMetersPerSec = 0.001;
  constexpr double kChassisAngularVelocityRawToRadiansPerSec = 0.001;
  constexpr double kChassisPositionRawToMeters = 0.01;
  constexpr double kChassisYawRawToRadians = 0.001;
  const auto stamp = clock_->now();

  if (chassis_velocity_pending_) {
    geometry_msgs::msg::Vector3Stamped message;
    message.header.stamp = stamp;
    message.header.frame_id = chassis_frame_id_;
    message.vector.x = chassis_velocity_raw_[0] * kChassisLinearVelocityRawToMetersPerSec;
    message.vector.y = chassis_velocity_raw_[1] * kChassisLinearVelocityRawToMetersPerSec;
    message.vector.z = chassis_velocity_raw_[2] * kChassisAngularVelocityRawToRadiansPerSec;
    chassis_velocity_publisher_->publish(message);
    chassis_velocity_pending_ = false;
  }

  if (chassis_position_pending_) {
    geometry_msgs::msg::Vector3Stamped message;
    message.header.stamp = stamp;
    message.header.frame_id = odom_frame_id_;
    message.vector.x = chassis_position_raw_[0] * kChassisPositionRawToMeters;
    message.vector.y = chassis_position_raw_[1] * kChassisPositionRawToMeters;
    message.vector.z = chassis_position_raw_[2] * kChassisYawRawToRadians;
    chassis_position_publisher_->publish(message);
    chassis_position_pending_ = false;
  }

  if (gyro_pending_) {
    geometry_msgs::msg::Vector3Stamped message;
    message.header.stamp = stamp;
    message.header.frame_id = imu_frame_id_;
    message.vector.x = gyro_raw_[0] * kGyroRawToDegreesPerSec;
    message.vector.y = gyro_raw_[1] * kGyroRawToDegreesPerSec;
    message.vector.z = gyro_raw_[2] * kGyroRawToDegreesPerSec;
    gyro_publisher_->publish(message);
    gyro_pending_ = false;
  }

  if (accel_pending_) {
    geometry_msgs::msg::Vector3Stamped message;
    message.header.stamp = stamp;
    message.header.frame_id = imu_frame_id_;
    message.vector.x = accel_raw_[0] * kAccelRawToG;
    message.vector.y = accel_raw_[1] * kAccelRawToG;
    message.vector.z = accel_raw_[2] * kAccelRawToG;
    accel_publisher_->publish(message);
    accel_pending_ = false;
  }

  if (attitude_pending_) {
    geometry_msgs::msg::Vector3Stamped message;
    message.header.stamp = stamp;
    message.header.frame_id = imu_frame_id_;
    message.vector.x = attitude_raw_[0] * kAngleRawToDegrees;
    message.vector.y = attitude_raw_[1] * kAngleRawToDegrees;
    message.vector.z = attitude_raw_[2] * kAngleRawToDegrees;
    attitude_publisher_->publish(message);
    attitude_pending_ = false;
  }
}

void CanDataController::handle_enable_bit(
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

void CanDataController::handle_target_velocity(const geometry_msgs::msg::Twist & message)
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
  if (!send_int16x3_(Rk3588Protocol::kCommandTargetVelocity, vx, vy, wz)) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000, "Failed to send target chassis velocity over CAN");
  }
}

void CanDataController::resend_current_state()
{
  const std::lock_guard<std::mutex> lock(mutex_);
  if (!control_bits_.has_value()) {
    return;
  }

  if (send_command_(control_bits_.value())) {
    last_sent_control_bits_ = control_bits_;
  }
}

void CanDataController::clear_state()
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

