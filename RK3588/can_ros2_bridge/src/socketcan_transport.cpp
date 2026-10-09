#include "can_ros2_bridge/socketcan_transport.hpp"

#include <stdexcept>

#include "ros2_socketcan/socket_can_id.hpp"

namespace can_ros2_bridge
{

CanTransport::CanTransport(const std::string & interface)
{
  receiver_ = std::make_unique<drivers::socketcan::SocketCanReceiver>(interface, false);
  sender_ = std::make_unique<drivers::socketcan::SocketCanSender>(interface, false);
}

CanFrame CanTransport::receive(const std::chrono::nanoseconds timeout)
{
  CanFrame frame;
  const auto can_id = receiver_->receive(static_cast<void *>(frame.data.data()), timeout);
  frame.id = can_id.identifier();
  frame.is_extended = can_id.is_extended();
  frame.length = can_id.length();
  return frame;
}

void CanTransport::send(const CanFrame & frame, const std::chrono::nanoseconds timeout)
{
  const auto max_id = frame.is_extended ? 0x1FFFFFFFU : 0x7FFU;
  if (frame.id > max_id) {
    throw std::invalid_argument("CAN ID exceeds the selected frame format");
  }
  if (frame.length > 8U) {
    throw std::invalid_argument("Classic CAN payload exceeds 8 bytes");
  }

  const auto id = frame.is_extended ?
    drivers::socketcan::CanId(
    frame.id, 0U, drivers::socketcan::FrameType::DATA,
    drivers::socketcan::ExtendedFrame) :
    drivers::socketcan::CanId(
    frame.id, 0U, drivers::socketcan::FrameType::DATA,
    drivers::socketcan::StandardFrame);

  sender_->send(frame.data.data(), frame.length, id, timeout);
}

}  // namespace can_ros2_bridge
