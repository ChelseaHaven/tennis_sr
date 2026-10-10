#ifndef CAN_ROS2_BRIDGE__HEARTBEAT_HPP_
#define CAN_ROS2_BRIDGE__HEARTBEAT_HPP_

#include <chrono>
#include <functional>
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

class HeartbeatController
{
public:
  using Clock = HeartbeatWatchdog::Clock;
  using StateChangeCallback = std::function<void(bool)>;

  HeartbeatController(
    std::chrono::milliseconds timeout, StateChangeCallback on_state_change);

  void on_heartbeat(Clock::time_point now = Clock::now());
  void check_timeout(Clock::time_point now = Clock::now());

private:
  void handle_state_change(const std::optional<bool> & enabled);

  HeartbeatWatchdog watchdog_;
  StateChangeCallback on_state_change_;
};

}  // namespace can_ros2_bridge

#endif  // CAN_ROS2_BRIDGE__HEARTBEAT_HPP_
