#include "can_ros2_bridge/f407_uplink_controller.hpp"

#include <stdexcept>
#include <utility>

#include "can_ros2_bridge/rk3588_protocol.hpp"

namespace can_ros2_bridge
{

F407UplinkController::F407UplinkController(
  rclcpp::Node & node, const std::chrono::milliseconds heartbeat_timeout,
  StateChangeCallback on_connection_change)
: logger_(node.get_logger()), ros_clock_(node.get_clock()),
  on_connection_change_(std::move(on_connection_change)),
  heartbeat_timeout_(heartbeat_timeout)
{
  if (heartbeat_timeout_ <= std::chrono::milliseconds::zero()) {
    throw std::invalid_argument("Heartbeat timeout must be positive");
  }
  if (!on_connection_change_) {
    throw std::invalid_argument("F407 connection callback must be set");
  }

  const auto gyro_topic = node.declare_parameter<std::string>("gyro_topic", "/imu/gyro");
  const auto accel_topic = node.declare_parameter<std::string>("accel_topic", "/imu/accel");
  const auto attitude_topic = node.declare_parameter<std::string>("attitude_topic", "/imu/euler");
  const auto chassis_velocity_topic = node.declare_parameter<std::string>(
    "chassis_velocity_topic", "/chassis/velocity");
  const auto chassis_position_topic = node.declare_parameter<std::string>(
    "chassis_position_topic", "/chassis/position");
  const auto f407_status_topic = node.declare_parameter<std::string>(
    "f407_status_topic", "/f407/connected");
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
  f407_status_publisher_ = node.create_publisher<std_msgs::msg::UInt8>(
    f407_status_topic, rclcpp::QoS(1).transient_local());

  RCLCPP_INFO(
    logger_, "Publishing gyro='%s', accel='%s', pitch/roll/yaw='%s' (frame_id='%s')",
    gyro_topic.c_str(), accel_topic.c_str(), attitude_topic.c_str(), imu_frame_id_.c_str());
  RCLCPP_INFO(
    logger_, "Publishing chassis velocity='%s' and position='%s'",
    chassis_velocity_topic.c_str(), chassis_position_topic.c_str());
  RCLCPP_INFO(
    logger_, "Publishing F407 connection status='%s' every 100 ms",
    f407_status_topic.c_str());

  status_timer_ = node.create_wall_timer(
    std::chrono::milliseconds(100), [this]() {publish_connection_status();});
}

void F407UplinkController::handle_can_frame(const CanFrame & frame)
{
  if (Rk3588Protocol::is_heartbeat(frame)) {
    last_heartbeat_time_ = Clock::now();
    update_connection_state(true);
    return;
  }

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

void F407UplinkController::check_heartbeat_timeout(const Clock::time_point now)
{
  if (f407_connected_.load() && last_heartbeat_time_.has_value() &&
    now - last_heartbeat_time_.value() > heartbeat_timeout_)
  {
    update_connection_state(false);
  }
}

void F407UplinkController::update_connection_state(const bool connected)
{
  if (f407_connected_.exchange(connected) == connected) {
    return;
  }

  RCLCPP_INFO(logger_, "F407 heartbeat %s", connected ? "detected" : "timed out");
  on_connection_change_(connected);
}

void F407UplinkController::publish_connection_status()
{
  std_msgs::msg::UInt8 message;
  message.data = static_cast<std::uint8_t>(f407_connected_.load() ? 1U : 0U);
  f407_status_publisher_->publish(message);
}

void F407UplinkController::publish_pending_data()
{
  constexpr double kGyroRawToDegreesPerSec = 2000.0 / 32768.0;
  constexpr double kAccelRawToG = 16.0 / 32768.0;
  constexpr double kAngleRawToDegrees = 180.0 / 32768.0;
  constexpr double kChassisLinearVelocityRawToMetersPerSec = 0.001;
  constexpr double kChassisAngularVelocityRawToRadiansPerSec = 0.001;
  constexpr double kChassisPositionRawToMeters = 0.01;
  constexpr double kChassisYawRawToRadians = 0.001;
  const auto stamp = ros_clock_->now();

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

}  // namespace can_ros2_bridge
