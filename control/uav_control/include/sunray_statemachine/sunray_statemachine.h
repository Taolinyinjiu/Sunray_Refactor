#pragma once

#include <memory>
#include <string>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>

#include "controller/base_controller/base_controller.hpp"
#include "sunray_control_arbiter/sunray_control_arbiter.h"

/**
 * @brief PX4 参数管理器前向声明。
 * @note 仅在本头文件中以智能指针形式持有，避免引入完整定义造成编译依赖膨胀。
 */
class PX4_ParamManager;

/**
 * @brief Sunray 状态机主状态集合。
 */
enum class SunrayState {
  OFF = 0,            ///< 待机/未激活状态。
  TAKEOFF,            ///< 起飞过程状态。
  HOVER,              ///< 悬停状态（主稳态）。
  LAND,               ///< 降落过程状态。
  EMERGENCY_LAND,     ///< 紧急降落状态。
  VELOCITY_CONTROL,   ///< 速度控制子状态。
  POSE_CONTROL,       ///< 位姿控制子状态。
  REFERENCE_CONTROL,  ///< 参考量控制子状态。
  TRAJECTORY_CONTROL, ///< 轨迹控制子状态。
  BREAKING,           ///< 控制退出后的减速过渡状态。
};

/**
 * @brief 触发状态机转移的事件集合。
 */
enum class SunrayEvent {
  TAKEOFF_REQUEST = 0,      ///< 请求起飞。
  TAKEOFF_COMPLETED,        ///< 起飞完成。
  LAND_REQUEST,             ///< 请求降落。
  LAND_COMPLETED,           ///< 降落完成。
  EMERGENCY_REQUEST,        ///< 请求紧急降落。
  EMERGENCY_COMPLETED,      ///< 紧急降落完成。
  WATCHDOG_ERROR,           ///< 看门狗异常。
  ENTER_VELOCITY_CONTROL,   ///< 进入速度控制。
  ENTER_POSE_CONTROL,       ///< 进入位姿控制。
  ENTER_REFERENCE_CONTROL,  ///< 进入参考量控制。
  ENTER_TRAJECTORY_CONTROL, ///< 进入轨迹控制。
  EXIT_CONTROL_MODE,        ///< 退出当前控制子模式。
  TRAJECTORY_COMPLETED,     ///< 轨迹执行完成。
  BREAKING_COMPLETED,       ///< 减速阶段完成。
};

/**
 * @brief 里程计来源类型。
 * @note 用于 OFF 阶段与 PX4 参数进行一致性校验。
 */
enum class OdometrySource {
  UNKNOWN = 0,  ///< 尚未注册来源。
  USER_DEFINED, ///< 用户自定义来源规则。
  MOCAP,        ///< 动捕系统里程计。
  Fast_LIO,     ///< Fast-LIO 输出里程计。
  VINS,         ///< VINS 输出里程计。
  VIOBOT,       ///< VIOBOT 输出里程计。
  GNSS,         ///< GNSS 里程计来源。
  OPTICAL_FLOW  ///< 光流定位来源。
};

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
public:
  /**
   * @brief 构造状态机。
   * @param nh ROS 节点句柄。
   */
  explicit Sunray_StateMachine(ros::NodeHandle &nh);

  /**
   * @brief 注册全局唯一控制器实例。
   * @param controller 控制器实例（不能为空）。
   * @return true 注册成功；false 注册失败。
   * @note 状态机所有状态共享同一个控制器对象，以避免多控制器状态不一致问题。
   */
  bool register_controller(
      const std::shared_ptr<uav_control::Base_Controller> &controller);

  /**
   * @brief 状态机事件入口。
   * @param event 输入事件。
   * @return true 发生有效转移；false 事件被忽略或不满足转移条件。
   */
  bool dispatch(SunrayEvent event);

  /**
   * @brief 状态机周期更新函数。
   * @note 按当前状态调用控制器的 takeoff/land/emergency_land/update。
   */
  void update();

  /**
   * @brief 设置状态可用标志。
   * @param ready true 表示满足状态可用前置条件。
   */
  void set_state_available(bool ready);

  /**
   * @brief 设置起飞回调就绪标志。
   * @param ready true 表示起飞回调已就绪。
   */
  void set_takeoff_callback_ready(bool ready);

  /**
   * @brief 获取当前状态。
   * @return 当前状态枚举值。
   */
  SunrayState current_state() const;

  /**
   * @brief 注册里程计来源。
   * @param source 里程计来源类型。
   */
  void register_odometry_source(OdometrySource source) {
    odometry_source_ = source;
  }

  /**
   * @brief 查询当前里程计来源。
   * @return 当前里程计来源。
   */
  OdometrySource odometry_source() const { return odometry_source_; }

  /**
   * @brief 注册 PX4 参数管理器句柄。
   * @param manager PX4 参数管理器实例。
   */
  void register_px4_param_manager(
      const std::shared_ptr<::PX4_ParamManager> &manager) {
    px4_param_manager_ = manager;
  }

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

private:
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
   * @brief 执行状态转移。
   * @param next_state 目标状态。
   * @return true 转移成功（含自环）。
   */
  bool transition_to(SunrayState next_state);

  /**
   * @brief 获取已注册的控制器实例。
   * @return 控制器智能指针；未注册时返回 nullptr。
   */
  std::shared_ptr<uav_control::Base_Controller> get_controller() const;

  ros::NodeHandle nh_;          ///< ROS 节点句柄。
  SunrayState current_state_;   ///< 当前状态。
  bool state_available_;        ///< 状态可用标志。
  bool takeoff_callback_ready_; ///< 起飞回调就绪标志。
  OdometrySource odometry_source_{
      OdometrySource::UNKNOWN}; ///< 当前里程计来源。
  std::shared_ptr<::PX4_ParamManager>
      px4_param_manager_; ///< PX4 参数管理器句柄。
  std::shared_ptr<uav_control::Base_Controller>
      controller_; ///< 全局唯一控制器实例。
  uav_control::Sunray_Control_Arbiter arbiter_; ///< 控制输出仲裁与发布层。

  // MAVROS offboard/arming 接管相关
  std::string uav_ns_;                     ///< 解析出的 UAV 命名空间（如 uav1）
  ros::Subscriber mavros_state_sub_;       ///< /<uav_ns>/mavros/state
  ros::ServiceClient arming_client_;       ///< /<uav_ns>/mavros/cmd/arming
  ros::ServiceClient set_mode_client_;     ///< /<uav_ns>/mavros/set_mode
  mavros_msgs::State mavros_state_;        ///< 最近一次飞控状态
  bool mavros_state_received_{false};      ///< MAVROS 状态是否已收到
  ros::Time last_set_mode_req_time_{};     ///< 最近一次 set_mode 请求时间
  ros::Time last_arm_req_time_{};          ///< 最近一次 arming 请求时间
  double set_mode_retry_interval_s_{1.0};  ///< set_mode 重试间隔（秒）
  double arm_retry_interval_s_{1.0};       ///< arming 重试间隔（秒）
  bool enable_offboard_control_{true};     ///< 是否启用 OFFBOARD/ARM 接管
};
