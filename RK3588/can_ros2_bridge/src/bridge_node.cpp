#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdint>
#include <exception>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "can_ros2_bridge/can_transport.hpp"
#include "can_ros2_bridge/heartbeat.hpp"
#include "can_ros2_bridge/manual_enable.hpp"
#include "can_ros2_bridge/rk3588_protocol.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_socketcan/socket_can_id.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr auto kHeartbeatTimeout = 200ms;
constexpr auto kReceivePollInterval = 10ms;
}  // namespace

class CanBridgeNode : public rclcpp::Node
{
public:
  CanBridgeNode()
  : Node("can_receiver_node"),
    heartbeat_controller_(kHeartbeatTimeout, [this](const bool enabled) {
      send_global_enable(enabled);
      if (manual_enable_) {
        if (enabled) {
          manual_enable_->resend_current_state();
        } else {
          manual_enable_->clear_state();
        }
      }
    }),
    running_(true)
  {
    const auto interface = declare_parameter<std::string>("interface", "can0");
    transport_ = std::make_unique<can_ros2_bridge::CanTransport>(interface);

    RCLCPP_INFO(get_logger(), "Listening on SocketCAN interface '%s'", interface.c_str());
    RCLCPP_INFO(
      get_logger(), "Heartbeat ID=0x%X timeout=%ld ms; global command ID=0x%X",
      can_ros2_bridge::Rk3588Protocol::kNodeId,
      static_cast<long>(kHeartbeatTimeout.count()),
      can_ros2_bridge::Rk3588Protocol::kFeedbackId);

    manual_enable_ =
      std::make_unique<can_ros2_bridge::ManualEnableController>(
      *this, [this](const std::uint32_t control_bits) {
        return send_manual_enable(control_bits);
      });

    send_global_enable(false);
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
  void send_global_enable(const bool enabled)
  {
    const auto frame = can_ros2_bridge::Rk3588Protocol::make_global_enable_frame(enabled);
    try {
      const std::lock_guard<std::mutex> lock(send_mutex_);
      transport_->send(frame, 100ms);
      RCLCPP_INFO(
        get_logger(), "Sent global %s: CAN ID=0x%X data=[0x%02X, 0x%02X]",
        enabled ? "enable" : "disable", frame.id, frame.data[0], frame.data[1]);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Failed to send chassis command: %s", error.what());
    }
  }

  bool send_manual_enable(const std::uint32_t control_bits)
  {
    const auto frame = can_ros2_bridge::Rk3588Protocol::make_manual_enable_frame(
      control_bits);
    try {
      const std::lock_guard<std::mutex> lock(send_mutex_);
      transport_->send(frame, 100ms);
      RCLCPP_INFO(
        get_logger(),
        "Sent manual enable command: control_bits=0x%08X CAN ID=0x%X "
        "data=[0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X]",
        static_cast<unsigned int>(control_bits), frame.id, frame.data[0], frame.data[1],
        frame.data[2], frame.data[3], frame.data[4]);
      return true;
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Failed to send manual enable command: %s", error.what());
      return false;
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
          heartbeat_controller_.on_heartbeat();
        }
      } catch (const drivers::socketcan::SocketCanTimeout &) {  
        // Poll timeout is expected; check for a lost heartbeat below.
      } catch (const std::exception & error) {
        RCLCPP_ERROR(get_logger(), "CAN receive failed: %s", error.what());
        break;
      }

      heartbeat_controller_.check_timeout();
    }
  }

  std::unique_ptr<can_ros2_bridge::CanTransport> transport_;
  can_ros2_bridge::HeartbeatController heartbeat_controller_;
  std::unique_ptr<can_ros2_bridge::ManualEnableController> manual_enable_;
  std::mutex send_mutex_;
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
