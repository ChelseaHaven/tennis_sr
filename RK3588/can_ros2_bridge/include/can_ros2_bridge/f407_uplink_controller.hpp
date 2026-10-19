#ifndef CAN_ROS2_BRIDGE__F407_UPLINK_CONTROLLER_HPP_
#define CAN_ROS2_BRIDGE__F407_UPLINK_CONTROLLER_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>

#include "can_ros2_bridge/can_frame.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace can_ros2_bridge
{

class F407UplinkController
{
public:
  using Clock = std::chrono::steady_clock;
  using StateChangeCallback = std::function<void(bool)>;

  F407UplinkController(
    rclcpp::Node & node, std::chrono::milliseconds heartbeat_timeout,
    StateChangeCallback on_connection_change);

  // Incoming F407 CAN frames: heartbeat and telemetry.
  void handle_can_frame(const CanFrame & frame);
  void check_heartbeat_timeout(Clock::time_point now = Clock::now());

private:
  void update_connection_state(bool connected);
  void publish_connection_status();
  void publish_pending_data();

  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr ros_clock_;
  StateChangeCallback on_connection_change_;
  std::chrono::milliseconds heartbeat_timeout_;
  std::optional<Clock::time_point> last_heartbeat_time_;
  std::atomic_bool f407_connected_{false};
  rclcpp::TimerBase::SharedPtr status_timer_;

  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr gyro_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr accel_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr attitude_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
    chassis_velocity_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
    chassis_position_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr f407_status_publisher_;

  std::string imu_frame_id_;
  std::string chassis_frame_id_;
  std::string odom_frame_id_;
  std::array<std::int16_t, 3U> gyro_raw_{};
  std::array<std::int16_t, 3U> accel_raw_{};
  std::array<std::int16_t, 3U> attitude_raw_{};
  std::array<std::int16_t, 3U> chassis_velocity_raw_{};
  std::array<std::int16_t, 3U> chassis_position_raw_{};
  bool gyro_pending_{false};
  bool accel_pending_{false};
  bool attitude_pending_{false};
  bool chassis_velocity_pending_{false};
  bool chassis_position_pending_{false};
};

}  // namespace can_ros2_bridge

#endif  // CAN_ROS2_BRIDGE__F407_UPLINK_CONTROLLER_HPP_
