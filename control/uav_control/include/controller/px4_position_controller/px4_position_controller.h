#pragma once

/**
 * @file px4_position_controller.h
 * @brief PX4 位置控制器封装。
 *
 * 本文件定义 `PX4_Position_Controller` 类，用于对接 PX4 内部位置控制能力。
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
	// 设计一个五次项曲线生成器，用来生成轨迹	
	Quintic_Curve	quintic_curve_generation;

};

// TODO：代码待完善
inline bool Position_Controller::load_param(ros::NodeHandle &nh) {
  if (!nh.getParam("/uav_ns", uav_ns)) {
    return false;
  }

  // 规范化命名空间：支持 "uav1" 或 "/uav1"
  std::string ns = uav_ns;
  if (ns.empty())
    return false;
  if (ns.front() != '/')
    ns = "/" + ns;
  if (ns.back() == '/')
    ns.pop_back();

  const std::string takeoff_key = ns + "/takeoff_param";
  const std::string tol_key = ns + "/error_tolerance";
  const std::string hold_key = ns + "/takeoff_holdtime";

  if (!nh.getParam(takeoff_key, takeoff_position))
    return false;
  if (!nh.getParam(tol_key, error_tolerance))
    return false;
  if (!nh.getParam(hold_key, takeoff_holdtime_param))
    return false;

  return true;
}

inline ControllerOutput Position_Controller::update(void) {
  // 首先构造输出
  ControllerOutput temp_output;
  // 根据当前的状态进行更新
  switch (controller_state_) {
  case ControllerState::UNDEFINED: {
    // 如果是UNDEFINE阶段，则说明无人机并没有做好准备，因此此时什么都不执行
    return temp_output;
  };
  case ControllerState::OFF: {
    // OFF阶段，此时无人机在地面上静止，不进行输出
    return temp_output;
  };
  case ControllerState::TAKEOFF: {
    // 未解锁：持续发零速度，保持控制链路活跃
    if (!px4_arm_state_) {
      temp_output.channel_enable(ControllerOutputMask::VELOCITY);
      temp_output.velocity.x() = 0.0;
      temp_output.velocity.y() = 0.0;
      temp_output.velocity.z() = 0.0;
      return temp_output;
    }

    // 已解锁，越界保护，如果参数没有正常加载，则依旧输出零速度
    if (takeoff_param.size() < 3 || error_tolerance.size() < 3) {
      takeoff_holdstart_time = ros::Time(0);
      takeoff_holdkeep_time = ros::Time(0);

      temp_output.channel_enable(ControllerOutputMask::VELOCITY);
      temp_output.velocity.x() = 0.0;
      temp_output.velocity.y() = 0.0;
      temp_output.velocity.z() = 0.0;
      return temp_output;
    }

    // 111的二进制是0x07
    constexpr uint8_t kTakeoffReadyMask = 0x07U;
    uint8_t takeoff_ready = 0U;
    // 计算误差
    const double ex = std::abs(takeoff_param[0] - current_state_.position.x());
    const double ey = std::abs(takeoff_param[1] - current_state_.position.y());
    const double ez = std::abs(takeoff_param[2] - current_state_.position.z());
    // 如果误差在容许的范围内，置位
    if (ex < error_tolerance[0])
      takeoff_ready |= (1U << 0);
    if (ey < error_tolerance[1])
      takeoff_ready |= (1U << 1);
    if (ez < error_tolerance[2])
      takeoff_ready |= (1U << 2);

    // 根据在期望的起飞位置误差内的持续时间来判断是否达到了稳定的阶段
    const ros::Time now = ros::Time::now();
    if (takeoff_ready == kTakeoffReadyMask) {
      if (takeoff_holdstart_time.isZero()) {
        takeoff_holdstart_time = now;
        takeoff_holdkeep_time = now;
      } else {
        takeoff_holdkeep_time = now;
        const ros::Duration hold_time = now - takeoff_holdstart_time;
        // 最低要保证2s的稳定时间
        const double hold_required =
            (takeoff_holdtime_param > 2.0) ? takeoff_holdtime_param : 2.0;
        if (hold_time.toSec() >= hold_required) {
          // 设置轨迹点为当前起飞参数
          trajectory_.position.x() = takeoff_param[0];
          trajectory_.position.y() = takeoff_param[1];
          trajectory_.position.z() = takeoff_param[2];
          // 切换到HOVER状态
          controller_state_ = ControllerState::HOVER;
        }
      }
    } else {
      // 任一轴超出容差，重置计时
      takeoff_holdstart_time = ros::Time(0);
      takeoff_holdkeep_time = ros::Time(0);
    }

    // 持续输出起飞目标点
    temp_output.channel_enable(ControllerOutputMask::POSITION);
    temp_output.position.x() = takeoff_param[0];
    temp_output.position.y() = takeoff_param[1];
    temp_output.position.z() = takeoff_param[2];
    return temp_output;
  }
  case ControllerState::HOVER: {
    // HOVER状态下，设置输出为当前轨迹点
    temp_output.channel_enable(ControllerOutputMask::POSITION);
    temp_output.position = trajectory_.position;
    return temp_output;
  };
  case ControllerState::MOVE: {
    // 当切换到MOVE时，通常是接受到了相关的控制指令
    if (!trajectory_.position.isZero()) {
      temp_output.channel_enable(ControllerOutputMask::POSITION);
      temp_output.position = trajectory_.position;
    }
    if (!trajectory_.velocity.isZero()) {
      temp_output.channel_enable(ControllerOutputMask::VELOCITY);
      temp_output.velocity = trajectory_.velocity;
    }
    if (!trajectory_.acceleration.isZero()) {
      temp_output.channel_enable(ControllerOutputMask::ACCELERATION);
      temp_output.acceleration_or_force = trajectory_.acceleration;
    }
    if (trajectory_.yaw != 0.0) {
      temp_output.channel_enable(ControllerOutputMask::YAW);
      temp_output.yaw = trajectory_.yaw;
    }

    if (trajectory_.yaw_rate != 0.0) {
      temp_output.channel_enable(ControllerOutputMask::YAW_RATE);
      temp_output.yaw = trajectory_.yaw_rate;
    }

    // position_controller
    // 不支持对姿态，推力，加加速度，加加加速度进行调整
    return temp_output;
  };
  case ControllerState::LAND: {
		// 构造五次项曲线
		// 首先判断是否为第一次进入LAND,通过判断起点是否为trajecotry_,终点是否为land_point
		if(quintic_curve_generation.get_start_position() != trajectory_.position && quintic_curve_generation.get_end_position()!= land_position)
		{
			// 清除原有轨迹生成器参数
			quintic_curve_generation.clear_all();
			// 注入降落轨迹参数
			quintic_curve_generation.set_start_position(trajectory_.position);
			quintic_curve_generation.set_end_position(land_position);
		}



		// 如果开始时间为0，说明是是刚进入land模式，设置开始时间为当前
		if(quintic_curve_generation.get_start_time() == ros::Time(0))
		{
			quintic_curve_generation.clear_all();
			quintic_curve_generation.set_start_time(ros::Time::now());

		}// 将当前轨迹点作为起点
		// 即使开始时间存在，需要考虑是否为之前控制器没有清除时间参数，需要检查开始position
		else if(quintic_curve_generation.)
		quintic_curve_generation.set_start_position(trajectory_.position);
		// 将当前轨迹点的xy位置作为降落的xy位置，设置z轴位置为0
		quintic_curve_generation.set_end_position(Eigen::Vector3d(trajectory_.position.x(),trajectory_.position.y(),takeoff_param[2]));
    // 
		
		// 使能位置，速度，加速度输出
		temp_output.channel_enable(ControllerOutputMask::POSITION);
		temp_output.channel_enable(ControllerOutputMask::VELOCITY);
		temp_output.channel_enable(ControllerOutputMask::ACCELERATION);
		// 设置输出量
		temp_output.position = quintic_curve_generation.get_position();
		temp_output.velocity = quintic_curve_generation.get_velocity();
		temp_output.acceleration_or_force = quintic_curve_generation.get_acceleration();
		// 返回输出量
		return temp_output;
  };
  case ControllerState::EMERGENCY_LAND: {
    break;
  };
  }

  return temp_output;
};

} // namespace uav_control
