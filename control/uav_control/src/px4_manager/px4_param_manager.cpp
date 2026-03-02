#include "px4_manager/px4_param_manager.h"

#include "mavros_msgs/ParamSet.h"
#include "mavros_msgs/mavlink_convert.h"
#include "ros/ros.h"

#include <mavlink/v2.0/common/common.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <exception>
#include <stdexcept>
#include <string>

PX4_ParamManager::PX4_ParamManager(ros::NodeHandle nh)
    : ansi_re_("\\x1b\\[[0-9;?]*[A-Za-z]"),
      ctrl_re_("[^\\x09\\x0a\\x0d\\x20-\\x7e]") {
  // 读取目标 UAV 标识。缺失则保持未初始化状态，后续调用会被 ensure_client_ready 拦截。
  if (!nh.getParam("uav_id", uav_id_)) {
    ROS_WARN("PX4_ParamManager: missing param uav_id");
    return;
  }
  if (!nh.getParam("uav_name", uav_name_)) {
    ROS_WARN("PX4_ParamManager: missing param uav_name");
    return;
  }

  const std::string uav_ns = "/" + uav_name_ + std::to_string(uav_id_);
  mavros_ns_ = uav_ns + "/mavros";

  // 参数写入 service：/uavX/mavros/param/set
  param_set_client_ =
      nh.serviceClient<mavros_msgs::ParamSet>(mavros_ns_ + "/param/set");

  // NSH 控制通道（注意这里不是 /mavros/mavlink/*，而是 /uavX/mavlink/*）。
  mavlink_to_pub_ =
      nh.advertise<mavros_msgs::Mavlink>(uav_ns + "/mavlink/to", 10);
  mavlink_from_sub_ = nh.subscribe(uav_ns + "/mavlink/from", 100,
                                   &PX4_ParamManager::rx_cb, this);

  // 估计器状态订阅，用于在 ekf2 start 后确认数据链路恢复。
  estimator_sub_ = nh.subscribe(mavros_ns_ + "/estimator_status", 20,
                                &PX4_ParamManager::estimator_cb, this);

  initialized_ = true;
}

// EKF2 参数设置入口：
// - 每个字段先判断是否需要设置（-1 表示跳过）；
// - 任一写入失败立即返回 false；
// - 全部写入完成后执行 boot_ekf2()，保证参数生效。
bool PX4_ParamManager::set_param_ekf2(px4_data::Ekf2Params ekf2_param_) {
  bool set_flag = false;
  std::string param_name = "null";

  // 设置 EKF2_EV_CTRL（int）
  if (ekf2_param_.ev_ctrl != -1) {
    param_name = "EKF2_EV_CTRL";
    set_flag = setParamInt(param_name, ekf2_param_.ev_ctrl);
    if (set_flag == false)
      return set_flag;
  }

  // 设置 EKF2_HGT_REF（int）
  if (ekf2_param_.hgt_ref != -1) {
    param_name = "EKF2_HGT_REF";
    set_flag = setParamInt(param_name, ekf2_param_.hgt_ref);
    if (set_flag == false)
      return set_flag;
  }

  // 设置 EKF2_EV_DELAY（float）
  if (ekf2_param_.ev_delay != -1) {
    param_name = "EKF2_EV_DELAY";
    set_flag = setParamFloat(param_name, ekf2_param_.ev_delay);
    if (set_flag == false)
      return set_flag;
  }

  // 参数写入完成后重启 ekf2，确保新值生效。
  set_flag = boot_ekf2();
  return set_flag;
}

bool PX4_ParamManager::ensure_client_ready(void) {
  // 初始化失败时，拒绝任何对外操作，避免出现“半初始化对象”行为不确定的问题。
  if (!initialized_) {
    ROS_WARN("PX4_ParamManager not initialized");
    return false;
  }

  // 这里仅做可观测性告警，不强制失败（保留当前行为）。
  // 真正调用时若服务不可达，会在 call() 返回 false。
  if (!param_set_client_.exists()) {
    ROS_WARN("PX4 param/set service not available: %s/param/set",
             mavros_ns_.c_str());
  }
  return true;
}

bool PX4_ParamManager::setParamInt(const std::string &name, int value) {
  if (!ensure_client_ready())
    return false;

  // MAVROS ParamSet：整型参数写入路径
  mavros_msgs::ParamSet srv;
  srv.request.param_id = name;
  srv.request.value.integer = value;
  if (!param_set_client_.call(srv)) {
    ROS_WARN("Failed to call param/set for %s", name.c_str());
    return false;
  }
  return srv.response.success;
}

bool PX4_ParamManager::setParamFloat(const std::string &name, float value) {
  if (!ensure_client_ready())
    return false;

  // MAVROS ParamSet：浮点参数写入路径
  mavros_msgs::ParamSet srv;
  srv.request.param_id = name;
  srv.request.value.real = value;
  if (!param_set_client_.call(srv)) {
    ROS_WARN("Failed to call param/set for %s", name.c_str());
    return false;
  }
  return srv.response.success;
}

void PX4_ParamManager::send_nsh(const std::string &cmd_in) {
  // 统一保证 NSH 命令以 '\n' 结尾，符合 shell 执行习惯。
  std::string cmd = cmd_in;
  if (cmd.empty() || cmd.back() != '\n') {
    cmd.push_back('\n');
  }

  // SERIAL_CONTROL data 长度固定 70 字节，本实现采用“单条命令不超过 70”策略。
  // 超长命令直接抛异常，避免截断导致不可预期行为。
  if (cmd.size() > 70U) {
    throw std::invalid_argument("NSH command must be <= 70 bytes");
  }

  std::array<uint8_t, 70> data{};
  std::memcpy(data.data(), cmd.data(), cmd.size());

  mavlink::mavlink_message_t mmsg{};
  mavlink::MsgMap map(mmsg);
  mavlink::common::msg::SERIAL_CONTROL serial{};

  // device=SHELL：告诉 PX4 将数据投递到 NSH。
  serial.device =
      static_cast<uint8_t>(mavlink::common::SERIAL_CONTROL_DEV::SHELL);

  // RESPOND|MULTI：
  // - RESPOND 要求对端回包
  // - MULTI   允许对端多帧回包直到输出清空
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
  mavlink_to_pub_.publish(ros_msg);
}

std::string PX4_ParamManager::run_nsh_cmd(const std::string &cmd,
                                          double timeout_sec) {
  // 清空旧缓存，避免上条命令残留影响本次 prompt 判定。
  {
    std::lock_guard<std::mutex> lk(mtx_);
    nsh_buf_.clear();
  }
  send_nsh(cmd);

  const auto deadline =
      std::chrono::steady_clock::now() +
      std::chrono::milliseconds(static_cast<int>(timeout_sec * 1000.0));
  std::unique_lock<std::mutex> lk(mtx_);
  while (std::chrono::steady_clock::now() < deadline) {
    // 一旦看到 shell prompt，认为命令执行已结束。
    if (nsh_buf_.find("nsh>") != std::string::npos ||
        nsh_buf_.find("pxh>") != std::string::npos) {
      std::string out = nsh_buf_;
      nsh_buf_.clear();
      return out;
    }

    // 主动 spinOnce：兼容“外部调用者没有 spin”的场景。
    lk.unlock();
    ros::spinOnce();
    lk.lock();
    cv_.wait_for(lk, std::chrono::milliseconds(20));
  }

  const std::string out = nsh_buf_;
  nsh_buf_.clear();
  throw std::runtime_error("timeout waiting shell prompt");
}

std::string PX4_ParamManager::sanitize_text(const std::string &text) const {
  // 去 ANSI 颜色/光标控制序列，减少日志污染。
  std::string out = std::regex_replace(text, ansi_re_, "");
  // 去不可见控制字符（保留常见换行与制表符）。
  out = std::regex_replace(out, ctrl_re_, "");
  // 去 CR，统一为 LF 风格。
  out.erase(std::remove(out.begin(), out.end(), '\r'), out.end());
  return out;
}

void PX4_ParamManager::rx_cb(const mavros_msgs::Mavlink::ConstPtr &ros_msg) {
  // ROS msg -> mavlink_message_t
  mavlink::mavlink_message_t mmsg{};
  if (!mavros_msgs::mavlink::convert(*ros_msg, mmsg)) {
    return;
  }

  // 只处理 SERIAL_CONTROL，其他 MAVLink 消息忽略。
  if (mmsg.msgid != mavlink::common::msg::SERIAL_CONTROL::MSG_ID) {
    return;
  }

  mavlink::MsgMap map(mmsg);
  mavlink::common::msg::SERIAL_CONTROL serial{};
  serial.deserialize(map);

  // 只接受来自 SHELL 设备的输出，避免混入其他串口设备数据。
  if (serial.device !=
      static_cast<uint8_t>(mavlink::common::SERIAL_CONTROL_DEV::SHELL)) {
    return;
  }

  const std::string raw(reinterpret_cast<const char *>(serial.data.data()),
                        static_cast<size_t>(serial.count));
  const std::string out = sanitize_text(raw);
  if (out.empty()) {
    return;
  }

  // 追加到共享缓存，并唤醒等待线程。
  std::lock_guard<std::mutex> lk(mtx_);
  nsh_buf_ += out;
  cv_.notify_all();
}

void PX4_ParamManager::estimator_cb(
    const mavros_msgs::EstimatorStatus::ConstPtr &) {
  // 仅做计数，不解析内容。用于判断“start 后是否有新数据”。
  std::lock_guard<std::mutex> lk(mtx_);
  ++estimator_updates_;
  cv_.notify_all();
}

int PX4_ParamManager::estimator_count() const {
  std::lock_guard<std::mutex> lk(mtx_);
  return estimator_updates_;
}

bool PX4_ParamManager::wait_estimator_update(int prev_count,
                                             double timeout_sec) {
  // 等待计数从 prev_count 向上增长。
  const auto deadline =
      std::chrono::steady_clock::now() +
      std::chrono::milliseconds(static_cast<int>(timeout_sec * 1000.0));
  std::unique_lock<std::mutex> lk(mtx_);
  while (std::chrono::steady_clock::now() < deadline) {
    if (estimator_updates_ > prev_count) {
      return true;
    }
    lk.unlock();
    ros::spinOnce();
    lk.lock();
    cv_.wait_for(lk, std::chrono::milliseconds(20));
  }
  return false;
}

namespace {

bool ekf2_running(const std::string &status_output) {
  // 文本规则判定：这不是 PX4 官方 API，而是面向当前 NSH 输出格式的启发式实现。
  std::string s = status_output;
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });

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

} // namespace

bool PX4_ParamManager::boot_ekf2(void) {
  if (!ensure_client_ready()) {
    return false;
  }

  try {
    const std::string uav_ns = "/" + uav_name_ + std::to_string(uav_id_);

    // 先确认 /uavX/mavlink/to 有订阅者（通常是 mavros 的 mavlink 插件）。
    // 若没有订阅者，发送命令不会到达飞控，提前失败比盲等超时更清晰。
    const auto sub_deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (ros::ok() && mavlink_to_pub_.getNumSubscribers() == 0 &&
           std::chrono::steady_clock::now() < sub_deadline) {
      ros::spinOnce();
      ros::Duration(0.05).sleep();
    }
    if (mavlink_to_pub_.getNumSubscribers() == 0) {
      ROS_ERROR("boot_ekf2: no subscriber on %s/mavlink/to", uav_ns.c_str());
      return false;
    }

    // Step 1: 基线检查，启动前应为运行态。
    ROS_INFO("boot_ekf2: check baseline");
    const std::string status_before = run_nsh_cmd("ekf2 status", 8.0);
    if (!ekf2_running(status_before)) {
      ROS_ERROR("boot_ekf2: ekf2 is not running before restart");
      return false;
    }

    // Step 2: 停止 EKF2。
    ROS_INFO("boot_ekf2: stop ekf2");
    run_nsh_cmd("ekf2 stop", 8.0);
    ros::Duration(0.8).sleep();

    // Step 3: 校验已停止。
    const std::string status_stopped = run_nsh_cmd("ekf2 status", 8.0);
    if (ekf2_running(status_stopped)) {
      ROS_ERROR("boot_ekf2: ekf2 still running after stop");
      return false;
    }

    // Step 4: 启动 EKF2。
    ROS_INFO("boot_ekf2: start ekf2");
    const int est_before_start = estimator_count();
    run_nsh_cmd("ekf2 start", 8.0);
    ros::Duration(1.5).sleep();

    // Step 5: 启动后运行态检查。
    const std::string status_after = run_nsh_cmd("ekf2 status", 8.0);
    if (!ekf2_running(status_after)) {
      ROS_ERROR("boot_ekf2: ekf2 not running after start");
      return false;
    }

    // Step 6: 估计器话题更新检查，防止“进程起来但未正常输出数据”的假阳性。
    if (!wait_estimator_update(est_before_start, 5.0)) {
      ROS_ERROR("boot_ekf2: estimator_status not updated after start");
      return false;
    }
  } catch (const std::exception &e) {
    ROS_ERROR("boot_ekf2: exception: %s", e.what());
    return false;
  }

  ROS_INFO("boot_ekf2: success");
  return true;
}
