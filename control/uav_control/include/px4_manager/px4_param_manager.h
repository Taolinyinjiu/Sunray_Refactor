/**
 * @file px4_param_manager.h
 * @brief PX4 参数管理器（含 EKF2 参数设置与模块重启）
 *
 * @details
 * 这个类是“参数写入 + 必要的模块重启”的一体化封装，核心目标是：
 * 1. 通过 MAVROS `/param/set` 统一写入 PX4 参数；
 * 2. 对“写入后需重启模块才生效”的参数（如 EKF2 一部分参数）做自动化处理；
 * 3. 把 NSH-MAVLink 的细节隐藏起来，降低上层调用复杂度。
 *
 * 架构上分成两条通道：
 * A) 参数通道（service）：
 *    - 使用 `/uavX/mavros/param/set` 写参数。
 * B) 模块控制通道（topic）：
 *    - 使用 `/uavX/mavlink/to` 与 `/uavX/mavlink/from` 发送/接收
 *      MAVLink SERIAL_CONTROL，以执行 NSH 命令（ekf2 stop/start/status）。
 *
 * 注意事项：
 * - 该类依赖参数 `uav_id` / `uav_name`，用于拼接命名空间。
 * - `set_param_ekf2()` 中目前使用 -1 作为“该字段不修改”的哨兵值。
 * - `boot_ekf2()` 会等待 shell prompt 与 estimator 更新，属于阻塞调用。
 */

#pragma once

#include "mavros_msgs/EstimatorStatus.h"
#include "mavros_msgs/Mavlink.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/service_client.h"
#include "ros/subscriber.h"

#include <condition_variable>
#include <mutex>
#include <regex>
#include <string>

#include "px4_manager/px4_data_types.h"

class PX4_ParamManager {
public:
  /**
   * @brief 构造函数：读取 UAV 标识并创建必要通信对象。
   *
   * 初始化完成后，类会持有：
   * - param_set_client_：参数设置 service client；
   * - mavlink_to_pub_：发 NSH 命令；
   * - mavlink_from_sub_：收 NSH 回显；
   * - estimator_sub_：监听 estimator_status 变化用于重启后健康确认。
   *
   * @param nh ROS NodeHandle（需要能访问参数服务器）
   */
  explicit PX4_ParamManager(ros::NodeHandle nh);

  ~PX4_ParamManager() = default;

  /**
   * @brief 设置 EKF2 相关参数，并在写入成功后重启 EKF2 模块。
   *
   * 执行顺序：
   * 1. 根据字段是否为 -1 决定是否写入对应参数；
   * 2. 逐项写入（任何一步失败立刻返回 false）；
   * 3. 调用 boot_ekf2() 执行 stop/start 与健康检查；
   * 4. 全部成功返回 true。
   *
   * @param ekf2_param_ 目标参数集合
   * @return true 成功，false 失败
   */
  bool set_param_ekf2(px4_data::Ekf2Params ekf2_param_);

private:
  // ===== UAV 标识信息 =====
  // 由参数服务器读取：uav_id + uav_name。
  // 示例：uav_name_="uav", uav_id_=1 -> /uav1。
  int uav_id_{0};
  std::string uav_name_{"null"};

  // MAVROS 命名空间（例如 /uav1/mavros）。
  std::string mavros_ns_;

  // 构造阶段是否成功初始化。
  bool initialized_{false};

  // ===== 参数 service 通道 =====
  // 对应 /uavX/mavros/param/set。
  ros::ServiceClient param_set_client_;

  // ===== NSH-MAVLink 通道 =====
  // mavlink_to_pub_   -> /uavX/mavlink/to
  // mavlink_from_sub_ -> /uavX/mavlink/from
  // estimator_sub_    -> /uavX/mavros/estimator_status
  ros::Publisher mavlink_to_pub_;
  ros::Subscriber mavlink_from_sub_;
  ros::Subscriber estimator_sub_;

  // ===== 同步与缓存 =====
  // 用于跨回调线程和主逻辑线程同步 shell 输出与 estimator 计数。
  mutable std::mutex mtx_;
  std::condition_variable cv_;
  std::string nsh_buf_;
  int estimator_updates_{0};

  // 文本清洗规则：去 ANSI 控制序列、过滤不可见控制字符。
  std::regex ansi_re_;
  std::regex ctrl_re_;

  // 统一前置检查：对象是否初始化完成、service 是否存在。
  bool ensure_client_ready(void);

  // 基础参数写入函数（对 MAVROS ParamSet 的轻量封装）。
  bool setParamInt(const std::string& name, int value);
  bool setParamFloat(const std::string& name, float value);

  /**
   * @brief 重启 EKF2 并验证重启成功。
   *
   * 验证链路：
   * - status 前检查（应在运行）
   * - stop 后检查（应停止）
   * - start 后检查（应恢复运行）
   * - estimator_status 更新检查（应有新数据）
   */
  bool boot_ekf2(void);

  // ===== 以下为 boot_ekf2 的支撑函数 =====
  void rx_cb(const mavros_msgs::Mavlink::ConstPtr& ros_msg);
  void estimator_cb(const mavros_msgs::EstimatorStatus::ConstPtr& msg);
  void send_nsh(const std::string& cmd);
  std::string run_nsh_cmd(const std::string& cmd, double timeout_sec);
  std::string sanitize_text(const std::string& text) const;
  int estimator_count() const;
  bool wait_estimator_update(int prev_count, double timeout_sec);
};
