#include <atomic>
#include <chrono>
#include <cstdio>
#include <exception>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "can_ros2_bridge/can_command_controller.hpp"
#include "can_ros2_bridge/can_transport.hpp"
#include "can_ros2_bridge/f407_uplink_controller.hpp"
#include "can_ros2_bridge/rk3588_protocol.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_socketcan/socket_can_id.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr auto kHeartbeatTimeout = 200ms;
constexpr auto kReceivePollInterval = 10ms;
constexpr auto kDownlinkHeartbeatPeriod = 100ms;
}  // namespace

class CanBridgeNode : public rclcpp::Node
{
public:
  CanBridgeNode()
  : Node("can_receiver_node"), running_(true)
  {
    const auto interface = declare_parameter<std::string>("interface", "can0");
    log_can_frames_ = declare_parameter<int>("log_can_frames", 0) == 1;
    transport_ = std::make_unique<can_ros2_bridge::CanTransport>(interface);

    uplink_controller_ = std::make_unique<can_ros2_bridge::F407UplinkController>(
      *this, kHeartbeatTimeout, [this](const bool connected) {
        if (!downlink_controller_) {
          return;
        }
        if (connected) {
          downlink_controller_->resend_current_state();
        } else {
          downlink_controller_->clear_state();
        }
      });
    downlink_controller_ = std::make_unique<can_ros2_bridge::CanCommandController>(
      *this, [this](const can_ros2_bridge::CanFrame & frame) {return send_frame(frame);},
      kDownlinkHeartbeatPeriod);

    RCLCPP_INFO(get_logger(), "Listening on SocketCAN interface '%s'", interface.c_str());
    RCLCPP_INFO(
      get_logger(),
      "F407 heartbeat ID=0x%X timeout=%ld ms; downlink heartbeat ID=0x%X period=%ld ms",
      can_ros2_bridge::Rk3588Protocol::kNodeId,
      static_cast<long>(kHeartbeatTimeout.count()),
      can_ros2_bridge::Rk3588Protocol::kFeedbackId,
      static_cast<long>(kDownlinkHeartbeatPeriod.count()));

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
  bool send_frame(const can_ros2_bridge::CanFrame & frame)
  {
    try {
      const std::lock_guard<std::mutex> lock(send_mutex_);
      transport_->send(frame, 100ms);
      RCLCPP_DEBUG(
        get_logger(), "Sent CAN frame: ID=0x%X len=%zu", frame.id, frame.length);
      return true;
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(), "Failed to send CAN frame ID=0x%X: %s", frame.id, error.what());
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
        if (log_can_frames_) {
          log_frame(frame);
        }
        uplink_controller_->handle_can_frame(frame);
      } catch (const drivers::socketcan::SocketCanTimeout &) {
        // Poll timeout is expected; check F407 heartbeat freshness below.
      } catch (const std::exception & error) {
        RCLCPP_ERROR(get_logger(), "CAN receive failed: %s", error.what());
        break;
      }

      uplink_controller_->check_heartbeat_timeout();
    }
  }

  std::unique_ptr<can_ros2_bridge::CanTransport> transport_;
  std::unique_ptr<can_ros2_bridge::F407UplinkController> uplink_controller_;
  std::unique_ptr<can_ros2_bridge::CanCommandController> downlink_controller_;
  std::mutex send_mutex_;
  std::atomic_bool running_;
  bool log_can_frames_{false};
  std::thread receive_thread_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
