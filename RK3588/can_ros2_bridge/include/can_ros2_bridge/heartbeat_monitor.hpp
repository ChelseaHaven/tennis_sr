#ifndef CAN_ROS2_BRIDGE__HEARTBEAT_MONITOR_HPP_
#define CAN_ROS2_BRIDGE__HEARTBEAT_MONITOR_HPP_

#include <chrono>
#include <optional>

namespace can_ros2_bridge
{

class HeartbeatWatchdog
{
public:
  using Clock = std::chrono::steady_clock;

  explicit HeartbeatWatchdog(std::chrono::milliseconds timeout);

  std::optional<bool> on_heartbeat(Clock::time_point now = Clock::now());
  std::optional<bool> check_timeout(Clock::time_point now = Clock::now());

private:
  std::chrono::milliseconds timeout_;
  std::optional<Clock::time_point> last_heartbeat_time_;
  bool enabled_{false};
};

}  // namespace can_ros2_bridge

#endif  // CAN_ROS2_BRIDGE__HEARTBEAT_MONITOR_HPP_
