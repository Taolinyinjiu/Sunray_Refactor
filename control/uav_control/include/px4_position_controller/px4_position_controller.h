#pragma once

/**
 * @file px4_position_controller.h
 * @brief PX4 位置控制器封装。
 *
 * 本文件定义 `PX4_Position_Controller` 类，用于对接 PX4 内部位置控制能力。
 */

#include "base_controller/base_controller.h"

namespace uav_controller {

/**
 * @class PX4_Position_Controller
 * @brief 基于 PX4 位置环的控制器实现。
 */
class PX4_Position_Controller : public Base_Controller {
public:
  /**
   * @brief 控制器飞行阶段。
   */
  enum class FlightStage {
    GROUND = 0, ///< 在地面阶段
    TAKEOFF,    ///< 起飞阶段
    AIR,        ///< 空中巡航/任务阶段
    LANDING,    ///< 降落阶段
    EMERGENCY,  ///< 紧急处理阶段（优先级最高）
  };

  /**
   * @brief 控制器参数集合。
   *
   * 约定：
   * - 单位默认使用 SI；
   * - 误差阈值用于阶段切换判据；
   * - 状态超时用于防止使用过期状态继续控制。
   */
  struct Config {
    float takeoff_height_m{1.5F};             ///< 起飞目标相对高度（m）。
    float takeoff_climb_mps{0.6F};            ///< 起飞爬升速度（m/s）。
    float landing_descent_mps{0.5F};          ///< 降落下降速度（m/s）。
    float emergency_descent_mps{1.2F};        ///< 紧急下降速度（m/s）。
    float position_error_tolerance_x_m{0.15F}; ///< X 方向到位阈值（m）。
    float position_error_tolerance_y_m{0.15F}; ///< Y 方向到位阈值（m）。
    float position_error_tolerance_z_m{0.15F}; ///< Z 方向到位阈值（m）。
    float state_timeout_s{0.2F};              ///< 输入状态超时阈值（s）。
  };

  /**
   * @brief 构造 PX4 位置控制器。
   */
  PX4_Position_Controller() = default;

  /** @brief 加载控制参数。 */
  bool load_param(ros::NodeHandle &nh) override;

  /**
   * @brief 加载控制参数，并可选下发 PID 到 PX4。
   * @param nh ROS 节点句柄。
   * @param push_pid_params_to_px4 true 表示同步 PID 参数到 PX4。
   */
  bool load_param(ros::NodeHandle &nh, bool push_pid_params_to_px4 = false);

  /** @brief 切换到起飞模式。 */
  bool set_takeoff_mode() override;

  /** @brief 切换到降落模式。 */
  bool set_land_mode() override;

  /** @brief 切换到紧急模式。 */
  bool set_emergency_mode() override;

  /** @brief 根据当前/期望状态更新控制输出。 */
  uav_controller::ControlOutput update() override;

  /** @brief 判定起飞阶段是否完成。 */
  bool ensure_takeoff_completed() const override;

  /** @brief 判定降落阶段是否完成。 */
  bool ensure_land_completed() const override;

  /** @brief 判定紧急降落是否完成。 */
  bool ensure_emergency_land_completed() const override;

  /**
   * @brief 设置当前飞行阶段。
   * @param stage 飞行阶段枚举值。
   */
  void set_flight_stage(FlightStage stage) { flight_stage_ = stage; }

  /**
   * @brief 获取当前飞行阶段。
   * @return 当前飞行阶段。
   */
  FlightStage flight_stage() const { return flight_stage_; }

  /**
   * @brief 获取控制器当前配置。
   * @return 配置只读引用。
   */
  const Config &config() const { return config_; }

private:
  /**
   * @brief 内部检查：输入状态是否满足控制更新前提。
   * @return true 表示可以进行控制计算。
   */
  bool validate_state_inputs() const;

  /**
   * @brief 检查指定状态是否超时。
   * @param state 待检查状态。
   * @param now 当前时刻。
   * @return true 表示状态新鲜；false 表示超时。
   */
  bool is_state_fresh(const uav_common::UAVStateEstimate &state,
                      const ros::Time &now) const;

  /**
   * @brief 判断当前位置是否已到达目标。
   * @param target 目标状态。
   * @return true 表示位置误差在阈值内。
   */
  bool is_position_reached(const uav_common::UAVStateEstimate &target) const;

  /**
   * @brief 生成地面待机阶段控制输出。
   */
  ControlOutput build_ground_output() const;

  /**
   * @brief 生成起飞阶段控制输出。
   */
  ControlOutput build_takeoff_output() const;

  /**
   * @brief 生成空中控制阶段输出。
   */
  ControlOutput build_air_output() const;

  /**
   * @brief 生成降落阶段控制输出。
   */
  ControlOutput build_landing_output() const;

  /**
   * @brief 生成紧急阶段控制输出。
   */
  ControlOutput build_emergency_output() const;

  /**
   * @brief 基于判据更新阶段机（不直接修改状态机，仅控制内部阶段）。
   */
  void update_flight_stage_by_conditions();

  Config config_{};                           ///< 参数配置缓存。
  FlightStage flight_stage_{FlightStage::GROUND}; ///< 控制器内部阶段。
  ros::Time stage_enter_time_{};              ///< 当前阶段进入时间。
  double takeoff_start_z_m_{0.0};             ///< 起飞起始高度（用于相对高度判据）。
};

} // namespace uav_controller
