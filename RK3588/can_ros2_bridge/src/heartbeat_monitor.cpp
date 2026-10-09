#include "can_ros2_bridge/heartbeat_monitor.hpp"

#include <stdexcept>

namespace can_ros2_bridge
{

HeartbeatWatchdog::HeartbeatWatchdog(const std::chrono::milliseconds timeout)
: timeout_(timeout)
{
  if (timeout_ <= std::chrono::milliseconds::zero()) {
    throw std::invalid_argument("Heartbeat timeout must be positive");
  }
}

std::optional<bool> HeartbeatWatchdog::on_heartbeat(const Clock::time_point now)
{
  std::optional<bool> state_change;
  if (last_heartbeat_time_.has_value()) {
    const bool heartbeat_is_fresh = now - last_heartbeat_time_.value() <= timeout_;
    if (enabled_ != heartbeat_is_fresh) {
      enabled_ = heartbeat_is_fresh;
      state_change = enabled_;
    }
  }
  last_heartbeat_time_ = now;
  return state_change;
}

std::optional<bool> HeartbeatWatchdog::check_timeout(const Clock::time_point now)
{
  if (enabled_ && last_heartbeat_time_.has_value() &&
    now - last_heartbeat_time_.value() > timeout_)
  {
    enabled_ = false;
    return false;
  }
  return std::nullopt;
}

}  // namespace can_ros2_bridge
