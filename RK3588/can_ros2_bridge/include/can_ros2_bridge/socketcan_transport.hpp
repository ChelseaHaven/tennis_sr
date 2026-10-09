#ifndef CAN_ROS2_BRIDGE__SOCKETCAN_TRANSPORT_HPP_
#define CAN_ROS2_BRIDGE__SOCKETCAN_TRANSPORT_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "can_ros2_bridge/can_frame.hpp"
#include "ros2_socketcan/socket_can_receiver.hpp"
#include "ros2_socketcan/socket_can_sender.hpp"

namespace can_ros2_bridge
{

class CanTransport
{
public:
  explicit CanTransport(const std::string & interface);

  CanFrame receive(
    std::chrono::nanoseconds timeout = std::chrono::nanoseconds::zero());
  void send(
    const CanFrame & frame,
    std::chrono::nanoseconds timeout = std::chrono::nanoseconds::zero());

private:
  std::unique_ptr<drivers::socketcan::SocketCanReceiver> receiver_;
  std::unique_ptr<drivers::socketcan::SocketCanSender> sender_;
};

}  // namespace can_ros2_bridge

#endif  // CAN_ROS2_BRIDGE__SOCKETCAN_TRANSPORT_HPP_
