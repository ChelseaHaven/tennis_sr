#ifndef CAN_ROS2_BRIDGE__CAN_DATA_CONTROLLER_HPP_
#define CAN_ROS2_BRIDGE__CAN_DATA_CONTROLLER_HPP_

#include <array>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "can_ros2_bridge/can_frame.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace can_ros2_bridge
{

class CanDataController
{
public:
  using SendCommand = std::function<bool(std::uint32_t)>;
  using SendInt16x3 = std::function<bool(
    std::uint8_t, std::int16_t, std::int16_t, std::int16_t)>;

  CanDataController(
    rclcpp::Node & node, SendCommand send_command, SendInt16x3 send_int16x3);
  void handle_can_frame(const CanFrame & frame);
  void clear_state();
  void resend_current_state();

private:
  void handle_enable_bit(
    std::uint32_t bit_mask, const char * control_name, std::uint8_t value);
  void handle_target_velocity(const geometry_msgs::msg::Twist & message);
  void publish_pending_data();

  rclcpp::Logger logger_;
  SendCommand send_command_;
  SendInt16x3 send_int16x3_;
  rclcpp::Clock::SharedPtr clock_;
  std::vector<rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr> subscriptions_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr target_velocity_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr gyro_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr accel_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr attitude_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr chassis_velocity_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr chassis_position_publisher_;
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
  std::mutex mutex_;
  std::optional<std::uint32_t> control_bits_;
  std::optional<std::uint32_t> last_sent_control_bits_;
};

}  // namespace can_ros2_bridge

#endif  // CAN_ROS2_BRIDGE__CAN_DATA_CONTROLLER_HPP_
