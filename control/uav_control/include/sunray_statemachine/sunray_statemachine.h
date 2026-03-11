#pragma once

#include <memory>
#include <string>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>

#include <px4_bridge/px4_data_reader.h>
#include <px4_bridge/px4_param_manager.h>

#include "sunray_statemachine_datatypes.h"

#include "controller/base_controller/base_controller.hpp"
#include "sunray_control_arbiter/sunray_control_arbiter.h"

namespace sunray_fsm {

/**
 * @class Sunray_StateMachine
 * @brief Sunray 飞控任务状态机。
 *
 * 主要职责：
 * - 维护当前主状态；
 * - 接收事件并执行状态转移；
 * - 管理单一控制器实例，保证各状态控制链路一致性；
 * - 在 OFF 阶段为起飞做前置条件检查。
 * -
 * 安全检查分为两部分，起飞前和起飞后，起飞前需要结合里程计来源进行PX4参数审查，起飞后主要观察里程计稳定性
 */
class Sunray_StateMachine {
  /** -----------------公开函数------外部可以调用使用------------------ */
public:
  /**
   * @brief 构造状态机。
   * @param nh ROS 节点句柄。
   */
  explicit Sunray_StateMachine(ros::NodeHandle &nh);

  /**
   * @brief 状态机事件处理入口
   * @param event 输入事件。
   * @return true 发生有效转移；false 事件被忽略或不满足转移条件。
   */
  bool handle_event(SunrayEvent event);

  /**
   * @brief 状态机周期更新函数。
   * @note 按当前状态调用控制器的 takeoff/land/emergency_land/update。
   */
  void update();

  /**
   * @brief 获取当前状态。
   * @return 当前状态枚举值。
   */
  SunrayState get_current_state() const;

  /**
   * @brief 状态枚举转字符串。
   * @param state 状态值。
   * @return 对应字符串常量。
   */
  static const char *to_string(SunrayState state);

  /**
   * @brief 事件枚举转字符串。
   * @param event 事件值。
   * @return 对应字符串常量。
   */
  static const char *to_string(SunrayEvent event);

  /** -----------------私有函数/变量------状态机内部使用------------------ */
private:
  /**
   * @brief 根据参数选择控制器类型，注册到状态机中
   * @param controller_types 控制器类型
   * @return true 注册成功；false 注册失败。
   * @note 状态机所有状态共享同一个控制器对象，以避免多控制器状态不一致问题。
   */
  bool register_controller(int controller_types);

  void controller_update_timer_cb(const ros::TimerEvent &);

  /**
   * @brief 解析 UAV 命名空间（uav_ns 或 uav_name+uav_id）。
   * @return 例如 "uav1"，失败返回空字符串。
   */
  std::string resolve_uav_namespace() const;

  /**
   * @brief MAVROS 状态回调。
   */
  void mavros_state_callback(const mavros_msgs::StateConstPtr &msg);

  /**
   * @brief 在飞行相关状态确保 OFFBOARD + ARM。
   * @return true 当前已满足；false 尚未满足。
   */
  bool ensure_offboard_and_arm();

  /**
   * @brief 当前状态是否需要飞控处于 OFFBOARD/ARM。
   */
  bool requires_offboard() const;

  /**
   * @brief 起飞/解锁前 安全检查
   * @return true 可解锁/起飞；false 禁止解锁/起飞。
   */
  bool check_health_preflight();

  /**
   * @brief 飞行过程中 安全检查
   * @return true 可继续；false 紧急降落。
   */
  bool check_health();

  /**
   * @brief 判定是否满足起飞条件。
   * @return true 可起飞；false 不可起飞。
   */
  bool can_takeoff() const;

  /**
   * @brief OFF 阶段的里程计来源合法性检查（占位）。
   * @return true 当前默认返回 true，后续在 cpp 中实现真实检查。
   */
  bool validate_offstage_odometry_source() const { return true; }

  /**
   * @brief 执行状态转移，作为public中handle_event函数的底层实现
   * @param next_state 目标状态。
   * @return true 转移成功（含自环）。
   */
  bool transition_to(SunrayState next_state);

  /**
   * @brief 获取已注册的控制器实例。
   * @return 控制器智能指针；未注册时返回 nullptr。
   */
  std::shared_ptr<uav_control::Base_Controller> get_controller() const;

  /** -----------------私有变量---------------------- */
  ros::NodeHandle nh_;        ///< ROS 节点句柄。
  SunrayState current_state_; ///< 当前状态。
	SunrayFSM_ParamConfig param_config_; 	// yaml文件中写入的参数
  /** -----------------px4数据读取与参数管理---------------------- */
  PX4_DataReader px4_data_reader;
  PX4_ParamManager px4_param_manager_;
  /** -----------------控制器相关--------------------- */
  std::shared_ptr<uav_control::Base_Controller>
      controller_; ///< 全局唯一控制器实例。
  ros::Timer controller_update_timer_;
  double controller_update_hz__;
  uav_control::Sunray_Control_Arbiter arbiter_; ///< 控制输出仲裁与发布层。
  /** -----------------MAVROS offboard/arming 接管相关--------------------- */
  ros::ServiceClient arming_client_;   ///< /<uav_ns>/mavros/cmd/arming
  ros::ServiceClient set_mode_client_; ///< /<uav_ns>/mavros/set_mode
  OffboardRetryConfig px4_offboard_retry_state_;
};

}; // namespace sunray_fsm