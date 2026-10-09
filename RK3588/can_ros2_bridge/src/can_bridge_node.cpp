#include <atomic>
#include <chrono>
#include <cstdio>
#include <exception>
#include <memory>
#include <optional>
#include <string>
#include <thread>

#include "can_ros2_bridge/socketcan_transport.hpp"
#include "can_ros2_bridge/heartbeat_monitor.hpp"
#include "can_ros2_bridge/rk3588_can_protocol.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_socketcan/socket_can_id.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr auto kHeartbeatTimeout = 200ms;
constexpr auto kReceivePollInterval = 10ms;
}

class CanBridgeNode : public rclcpp::Node
{
public:
  CanBridgeNode()
  : Node("can_receiver_node"), watchdog_(kHeartbeatTimeout), running_(true)
  {
    const auto interface = declare_parameter<std::string>("interface", "can0");
    transport_ = std::make_unique<can_ros2_bridge::CanTransport>(interface);

    RCLCPP_INFO(get_logger(), "Listening on SocketCAN interface '%s'", interface.c_str());
    RCLCPP_INFO(
      get_logger(), "Heartbeat ID=0x%X timeout=%ld ms; chassis command ID=0x%X",
      can_ros2_bridge::Rk3588Protocol::kNodeId,
      static_cast<long>(kHeartbeatTimeout.count()),
      can_ros2_bridge::Rk3588Protocol::kFeedbackId);

    send_chassis_enable(false);
    receive_thread_ = std::thread(&CanBridgeNode::receive_loop, this);
  }

  ~CanBridgeNode() override
  {
    running_.store(false);
    if (receive_thread_.joinable()) {
      receive_thread_.join();
    }
  }

private:
  void send_chassis_enable(const bool enabled)
  {
    const auto frame = can_ros2_bridge::Rk3588Protocol::make_chassis_enable_frame(enabled);
    try {
      transport_->send(frame, 100ms);
      RCLCPP_INFO(
        get_logger(), "Sent chassis %s: CAN ID=0x%X data=[0x%02X, 0x%02X]",
        enabled ? "enable" : "disable", frame.id, frame.data[0], frame.data[1]);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Failed to send chassis command: %s", error.what());
    }
  }

  void send_manual_enable(const std::uint32_t control_bits)
  {
    const auto frame = can_ros2_bridge::Rk3588Protocol::make_manual_enable_frame(
      control_bits);
    try {
      transport_->send(frame, 100ms);
      RCLCPP_INFO(
        get_logger(),
        "Sent manual enable command: control_bits=0x%08X CAN ID=0x%X "
        "data=[0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X]",
        static_cast<unsigned int>(control_bits), frame.id, frame.data[0], frame.data[1],
        frame.data[2], frame.data[3], frame.data[4]);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Failed to send manual enable command: %s", error.what());
    }
  }

  void handle_state_change(const std::optional<bool> & enabled)
  {
    if (enabled.has_value()) {
      send_chassis_enable(enabled.value());
    }
  }

  void log_frame(const can_ros2_bridge::CanFrame & frame)
  {
    std::string bytes;
    for (std::size_t i = 0; i < frame.length; ++i) {
      if (!bytes.empty()) {
        bytes += ' ';
      }
      char byte[4];
      std::snprintf(byte, sizeof(byte), "%02X", frame.data[i]);
      bytes += byte;
    }
    RCLCPP_INFO(
      get_logger(), "CAN ID=0x%X len=%zu data=[%s]",
      frame.id, frame.length, bytes.c_str());
  }

  void receive_loop()
  {
    while (running_.load() && rclcpp::ok()) {
      try {
        const auto frame = transport_->receive(kReceivePollInterval);
        log_frame(frame);
        if (can_ros2_bridge::Rk3588Protocol::is_heartbeat(frame)) {
          handle_state_change(watchdog_.on_heartbeat());
        }
      } catch (const drivers::socketcan::SocketCanTimeout &) {  
        // Poll timeout is expected; check for a lost heartbeat below.
      } catch (const std::exception & error) {
        RCLCPP_ERROR(get_logger(), "CAN receive failed: %s", error.what());
        break;
      }

      handle_state_change(watchdog_.check_timeout());
    }
  }

  std::unique_ptr<can_ros2_bridge::CanTransport> transport_;
  can_ros2_bridge::HeartbeatWatchdog watchdog_;
  std::atomic_bool running_;
  std::thread receive_thread_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
