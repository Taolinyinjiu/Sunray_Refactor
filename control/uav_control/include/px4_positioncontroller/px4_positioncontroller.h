#pragma once

/**
 * @file px4_positioncontroller.h
 * @brief PX4 位置控制器封装。
 *
 * 本文件定义 `PX4_PositionController` 类，用于对接 PX4 内部位置控制能力。
 */

#include "base_controller/base_controller.h"

namespace uav_controller {

/**
 * @class PX4_PositionController
 * @brief 基于 PX4 位置环的控制器实现。
 */
class PX4_PositionController : public Base_Controller {
public:
  /**
   * @brief 控制器飞行阶段。
   */
  enum class FlightStage {
    TAKEOFF = 0, ///< 起飞阶段
    AIR,         ///< 空中巡航/任务阶段
    LANDING,     ///< 降落阶段
  };

  /** @brief 加载控制参数。 */
  bool load_param(ros::NodeHandle &nh) override;

  /**
   * @brief 加载控制参数，并可选下发 PID 到 PX4。
   * @param nh ROS 节点句柄。
   * @param push_pid_params_to_px4 true 表示同步 PID 参数到 PX4。
   */
  bool load_param(ros::NodeHandle &nh, bool push_pid_params_to_px4);

  /** @brief 切换到起飞模式。 */
  bool set_takeoff_mode() override;

  /** @brief 切换到降落模式。 */
  bool set_land_mode() override;

  /** @brief 切换到紧急模式。 */
  bool set_emergency_mode() override;

  /** @brief 根据当前/期望状态更新控制输出。 */
  uav_controller::ControlOutput update() override;

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

private:
  FlightStage flight_stage_{FlightStage::TAKEOFF};

  // Takeoff parameters
  float takeoff_height_m{0.0F};
  float takeoff_climb_mps{0.0F};

  // Position error tolerances
  float position_error_tolerance_x_m{0.0F};
  float position_error_tolerance_y_m{0.0F};
  float position_error_tolerance_z_m{0.0F};
};

} // namespace uav_controller
