#pragma once

/**
 * @file px4_position_controller.h
 * @brief PX4 位置控制器封装。
 *
 * 本文件定义 `Position_Controller` 类，用于对接 PX4 内部位置控制能力。
 */

#include "controller/base_controller/base_controller.hpp"
#include "ros/time.h"
#include "utils/quintic_curve.hpp"

namespace uav_control {

// 从基类控制器继承，设计基于px4位置环的控制器
class Position_Controller : public Base_Controller {
public:
  bool load_param(ros::NodeHandle &nh) override; // 重写加载参数函数
  ControllerOutput update(void) override;        // 重写控制器更新函数

private:
  void reset_takeoff_context_if_needed();
  void reset_land_context_if_needed();
  ControllerOutput handle_undefined_state();
  ControllerOutput handle_off_state();
  ControllerOutput handle_takeoff_state();
  ControllerOutput handle_hover_state();
  ControllerOutput handle_move_state();
  ControllerOutput handle_land_state();

  // 设计一个五次项曲线生成器，用来生成轨迹
  Quintic_Curve quintic_curve_generation;

  // 降落末段接管上下文（曲线 -> 恒速下沉 -> 触地判定）
  bool land_velocity_takeover_active_{false};
  bool land_takeover_speed_peak_reached_{false};
  double last_land_altitude_m_{0.0};
  ros::Time last_land_altitude_time_{0};
  ros::Time land_low_velocity_start_time_{0};
  ros::Time land_no_descent_start_time_{0};
  double last_land_vz_mps_{0.0};
  bool last_land_vz_valid_{false};
		
};

} // namespace uav_control
