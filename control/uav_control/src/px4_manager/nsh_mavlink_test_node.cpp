#include "mavros_msgs/EstimatorStatus.h"
#include "mavros_msgs/Mavlink.h"
#include "mavros_msgs/mavlink_convert.h"
#include "ros/ros.h"

#include <mavlink/v2.0/common/common.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <exception>
#include <mutex>
#include <regex>
#include <stdexcept>
#include <string>

class NshBridge {
public:
  explicit NshBridge(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    pnh.param<std::string>("ns", ns_, "/uav1");
    pnh.param<bool>("verbose_nsh", verbose_nsh_, false);
    pub_ = nh.advertise<mavros_msgs::Mavlink>(ns_ + "/mavlink/to", 10);
    sub_ = nh.subscribe(ns_ + "/mavlink/from", 100, &NshBridge::rxCb, this);
    est_sub_ = nh.subscribe(ns_ + "/mavros/estimator_status", 20,
                            &NshBridge::estimatorCb, this);
  }

  void sendNsh(const std::string& cmd_in) {
    std::string cmd = cmd_in;
    if (cmd.empty() || cmd.back() != '\n') {
      cmd.push_back('\n');
    }
    if (cmd.size() > 70U) {
      throw std::invalid_argument("NSH command must be <= 70 bytes");
    }

    std::array<uint8_t, 70> data{};
    std::memcpy(data.data(), cmd.data(), cmd.size());

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
    serial.count = static_cast<uint8_t>(cmd.size());
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

  std::string runCmd(const std::string& cmd, double timeout_sec = 8.0) {
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
    throw std::runtime_error("timeout waiting shell prompt after cmd: " + cmd +
                             " output: " + out);
  }

  bool waitEstimatorUpdate(int prev_count, double timeout_sec = 4.0) {
    const auto deadline =
        std::chrono::steady_clock::now() +
        std::chrono::milliseconds(static_cast<int>(timeout_sec * 1000.0));
    std::unique_lock<std::mutex> lk(mtx_);
    while (std::chrono::steady_clock::now() < deadline) {
      if (est_updates_ > prev_count) {
        return true;
      }
      cv_.wait_until(lk, deadline);
    }
    return false;
  }

  int estimatorCount() {
    std::lock_guard<std::mutex> lk(mtx_);
    return est_updates_;
  }

private:
  void estimatorCb(const mavros_msgs::EstimatorStatus::ConstPtr&) {
    {
      std::lock_guard<std::mutex> lk(mtx_);
      ++est_updates_;
    }
    cv_.notify_all();
  }

  std::string sanitize(const std::string& text) {
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
      ROS_WARN("Mavlink convert failed");
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

    if (verbose_nsh_) {
      ROS_INFO_STREAM("NSH: " << out);
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
  ros::Subscriber est_sub_;
  std::string ns_;
  bool verbose_nsh_{false};

  std::mutex mtx_;
  std::condition_variable cv_;
  std::string buf_;
  int est_updates_{0};
};

bool ekf2Running(const std::string& status_output) {
  std::string s = status_output;
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (s.find("not running") != std::string::npos) {
    return false;
  }
  if (s.find("unknown command") != std::string::npos ||
      s.find("invalid command") != std::string::npos) {
    return false;
  }
  return (s.find("available instances:") != std::string::npos) ||
         (s.find("ekf2:0 ekf dt:") != std::string::npos) ||
         (s.find("ekf2:1 ekf dt:") != std::string::npos) ||
         (s.find("ekf2:2 ekf dt:") != std::string::npos);
}

bool waitNoEstimatorUpdate(NshBridge& bridge, double quiet_window_sec = 1.2,
                           double timeout_sec = 4.0) {
  const auto start = std::chrono::steady_clock::now();
  int last = bridge.estimatorCount();
  auto last_change = start;
  ros::Rate rate(10.0);
  while (ros::ok()) {
    const auto now = std::chrono::steady_clock::now();
    const double elapsed =
        std::chrono::duration_cast<std::chrono::duration<double>>(now - start)
            .count();
    if (elapsed >= timeout_sec) {
      return false;
    }

    const int curr = bridge.estimatorCount();
    if (curr != last) {
      last = curr;
      last_change = now;
    }
    const double quiet_time =
        std::chrono::duration_cast<std::chrono::duration<double>>(now -
                                                                   last_change)
            .count();
    if (quiet_time >= quiet_window_sec) {
      return true;
    }
    rate.sleep();
  }
  return false;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "ekf2_restart_test_cpp");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  bool verbose_status = false;
  pnh.param<bool>("verbose_status", verbose_status, false);

  ros::AsyncSpinner spinner(2);
  spinner.start();

  NshBridge bridge(nh, pnh);
  ros::Duration(1.5).sleep();

  bool success = true;
  try {
    ROS_INFO("Step 1/4: baseline check");
    const std::string status_before = bridge.runCmd("ekf2 status");
    if (verbose_status) {
      ROS_INFO_STREAM("ekf2 status(before):\n" << status_before);
    }
    if (!ekf2Running(status_before)) {
      ROS_ERROR("FAIL: ekf2 is not running before restart");
      success = false;
    }

    ROS_INFO("Step 2/4: stop ekf2");
    bridge.runCmd("ekf2 stop");
    ros::Duration(0.8).sleep();

    ROS_INFO("Step 3/4: verify ekf2 stopped");
    const std::string status_stopped = bridge.runCmd("ekf2 status");
    if (verbose_status) {
      ROS_INFO_STREAM("ekf2 status(stopped):\n" << status_stopped);
    }
    if (ekf2Running(status_stopped)) {
      ROS_ERROR("FAIL: ekf2 still running after stop");
      success = false;
    }

    if (!waitNoEstimatorUpdate(bridge, 1.0, 3.0)) {
      ROS_WARN(
          "WARN: estimator_status did not become quiet after ekf2 stop "
          "(continuing)");
    }

    ROS_INFO("Step 4/4: start ekf2");
    const int est_before_start = bridge.estimatorCount();
    bridge.runCmd("ekf2 start");
    ros::Duration(1.5).sleep();

    const std::string status_after = bridge.runCmd("ekf2 status");
    if (verbose_status) {
      ROS_INFO_STREAM("ekf2 status(after):\n" << status_after);
    }
    if (!ekf2Running(status_after)) {
      ROS_ERROR("FAIL: ekf2 not running after start");
      success = false;
    }

    if (!bridge.waitEstimatorUpdate(est_before_start, 5.0)) {
      ROS_ERROR("FAIL: /mavros/estimator_status not updated after ekf2 start");
      success = false;
    }
  } catch (const std::exception& e) {
    ROS_ERROR("FAIL: %s", e.what());
    success = false;
  }

  if (success) {
    ROS_INFO("PASS: EKF2 restart test completed successfully");
    return 0;
  }

  ROS_ERROR("FAIL: EKF2 restart test failed");
  return 1;
}
