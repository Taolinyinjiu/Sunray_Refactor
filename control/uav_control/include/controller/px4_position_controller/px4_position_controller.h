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
		
};

} // namespace uav_control
