#include "mavros_msgs/Mavlink.h"
#include "mavros_msgs/mavlink_convert.h"
#include "ros/ros.h"

#include <mavlink/v2.0/common/common.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <mutex>
#include <regex>
#include <stdexcept>
#include <string>

class NshTerminalBridge {
public:
  explicit NshTerminalBridge(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    pnh.param<std::string>("ns", ns_, "/uav1");
    pnh.param<double>("cmd_timeout_sec", cmd_timeout_sec_, 8.0);
    pub_ = nh.advertise<mavros_msgs::Mavlink>(ns_ + "/mavlink/to", 10);
    sub_ = nh.subscribe(ns_ + "/mavlink/from", 100, &NshTerminalBridge::rxCb, this);
  }

  double cmdTimeoutSec() const { return cmd_timeout_sec_; }

  std::string runCmd(const std::string& cmd, double timeout_sec) {
    {
      std::lock_guard<std::mutex> lk(mtx_);
      buf_.clear();
    }
    sendNsh(cmd);

    const auto deadline =
        std::chrono::steady_clock::now() +
        std::chrono::milliseconds(static_cast<int>(timeout_sec * 1000.0));
    std::unique_lock<std::mutex> lk(mtx_);
    while (std::chrono::steady_clock::now() < deadline) {
      if (buf_.find("nsh>") != std::string::npos ||
          buf_.find("pxh>") != std::string::npos) {
        std::string out = buf_;
        buf_.clear();
        return out;
      }
      cv_.wait_until(lk, deadline);
    }

    const std::string out = buf_;
    buf_.clear();
    throw std::runtime_error("timeout waiting shell prompt");
  }

private:
  void sendNsh(const std::string& cmd_in) {
    std::string cmd = cmd_in;
    if (cmd.empty() || cmd.back() != '\n') {
      cmd.push_back('\n');
    }

    size_t offset = 0;
    while (offset < cmd.size()) {
      const size_t chunk_len = std::min<size_t>(70, cmd.size() - offset);
      sendChunk(cmd.data() + offset, chunk_len);
      offset += chunk_len;
    }
  }

  void sendChunk(const char* data_ptr, size_t len) {
    std::array<uint8_t, 70> data{};
    std::memcpy(data.data(), data_ptr, len);

    mavlink::mavlink_message_t mmsg{};
    mavlink::MsgMap map(mmsg);
    mavlink::common::msg::SERIAL_CONTROL serial{};
    serial.device =
        static_cast<uint8_t>(mavlink::common::SERIAL_CONTROL_DEV::SHELL);
    serial.flags =
        static_cast<uint8_t>(mavlink::common::SERIAL_CONTROL_FLAG::RESPOND) |
        static_cast<uint8_t>(mavlink::common::SERIAL_CONTROL_FLAG::MULTI);
    serial.timeout = 0;
    serial.baudrate = 0;
    serial.count = static_cast<uint8_t>(len);
    serial.data = data;
    serial.target_system = 0;
    serial.target_component = 0;
    serial.serialize(map);
    mavlink::mavlink_finalize_message(&mmsg, 255, 190, serial.MIN_LENGTH,
                                      serial.LENGTH, serial.CRC_EXTRA);

    mavros_msgs::Mavlink ros_msg;
    mavros_msgs::mavlink::convert(mmsg, ros_msg);
    pub_.publish(ros_msg);
  }

  static std::string sanitize(const std::string& text) {
    static const std::regex ansi_re("\\x1b\\[[0-9;?]*[A-Za-z]");
    static const std::regex ctrl_re("[^\\x09\\x0a\\x0d\\x20-\\x7e]");
    std::string out = std::regex_replace(text, ansi_re, "");
    out = std::regex_replace(out, ctrl_re, "");
    out.erase(std::remove(out.begin(), out.end(), '\r'), out.end());
    return out;
  }

  void rxCb(const mavros_msgs::Mavlink::ConstPtr& ros_msg) {
    mavlink::mavlink_message_t mmsg{};
    if (!mavros_msgs::mavlink::convert(*ros_msg, mmsg)) {
      return;
    }
    if (mmsg.msgid != mavlink::common::msg::SERIAL_CONTROL::MSG_ID) {
      return;
    }

    mavlink::MsgMap map(mmsg);
    mavlink::common::msg::SERIAL_CONTROL serial{};
    serial.deserialize(map);
    if (serial.device !=
        static_cast<uint8_t>(mavlink::common::SERIAL_CONTROL_DEV::SHELL)) {
      return;
    }

    const std::string raw(reinterpret_cast<const char*>(serial.data.data()),
                          static_cast<size_t>(serial.count));
    const std::string out = sanitize(raw);
    if (out.empty()) {
      return;
    }

    {
      std::lock_guard<std::mutex> lk(mtx_);
      buf_ += out;
    }
    cv_.notify_all();
  }

private:
  ros::Publisher pub_;
  ros::Subscriber sub_;
  std::string ns_;
  double cmd_timeout_sec_{8.0};

  std::mutex mtx_;
  std::condition_variable cv_;
  std::string buf_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "nsh_proxy_terminal_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ros::AsyncSpinner spinner(2);
  spinner.start();

  NshTerminalBridge bridge(nh, pnh);

  std::cout << "NSH terminal proxy started. Type commands and press Enter.\n";
  std::cout << "Local exit command: :quit\n";

  try {
    const std::string initial = bridge.runCmd("", 3.0);
    if (!initial.empty()) {
      std::cout << initial;
      if (initial.back() != '\n') {
        std::cout << "\n";
      }
    }
  } catch (const std::exception&) {
    std::cout << "(No initial prompt yet, continue anyway)\n";
  }

  while (ros::ok()) {
    std::cout << "nsh-proxy> " << std::flush;
    std::string line;
    if (!std::getline(std::cin, line)) {
      std::cout << "\nstdin closed, exit.\n";
      break;
    }

    if (line == ":quit") {
      std::cout << "exit.\n";
      break;
    }

    try {
      const std::string out = bridge.runCmd(line, bridge.cmdTimeoutSec());
      std::cout << out;
      if (!out.empty() && out.back() != '\n') {
        std::cout << "\n";
      }
    } catch (const std::exception& e) {
      std::cout << "[ERROR] " << e.what() << "\n";
    }
  }

  return 0;
}
