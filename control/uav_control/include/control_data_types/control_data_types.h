/***
        @brief
   与sunray_fsm有关的数据类型，主要指的是sunray_fsm自身会用到的数据类型以及向发布的数据类型
*/
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "ros/duration.h"
#include <Eigen/Dense>
#include <cstdint>
#include <ros/time.h>
#include <string>
#include <vector>

namespace uav_control {

/**
 * @brief 状态机 状态定义
 */
enum class ControlState {
  UNDEFINED,
  OFF,
  TAKEOFF,
  LAND,
  EMERGENCY_LAND,
  RETURN,
  HOVER,
  POSITION_CONTROL,
  VELOCITY_CONTROL,
  ATTITUDE_CONTROL,
  COMPLEX_CONTROL,
  TRAJECTORY_CONTROL
};

// 这个是控制器内部的状态机，设计这个类型的原因是，
// 无人机在起飞降落阶段和悬停运动阶段，具有不同的动力学特性，控制器需要考虑到这一点
enum class ControllerState {
  UNDEFINED,
  OFF,
  TAKEOFF,
  LAND,
  EMERGENCY_LAND,
  HOVER,
  MOVE
};

// 控制器标准的输入(全量轨迹)，根据不同的任务情况构造不同的轨迹参数（构造指的是FSM进行构造）
// controller.get_input(std::vector<uav_control::TrajectoryPoint> tarjectory)

/* 1. 队列优先级和抢占优先级
队列优先级指的是，同时有两个控制量到达，一个是ego_planner这种外部导航算法输出的轨迹，另一个是像遥控器/地面站这种输出的命令，本质上都会转换成控制器的标准轨迹输入
但是我们认为这两者之间是有优先级的，举例
当ego_planner进行导航时，我们突然希望他做一些别的事情，此时来自遥控器/地面站的控制量会打断原有的ego_planner的控制


抢占优先级指的是，同一队列优先级的控制量，最新的控制量会覆盖原来的控制量，举例
在地面站指点飞行，无人机进行位置控制模式，此时在运动过程中，无人机尚未到达目标点，地面站设置停止，希望在当前位置悬停，切换到HOVER
此时，新的控制量会打断旧的控制量
*/
// 单个时刻的参考点（轨迹点）
struct TrajectoryPoint {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ros::Duration time_from_start;
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};     // m
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};     // m/s
  Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()}; // m/s^2
  Eigen::Vector3d jerk{Eigen::Vector3d::Zero()};         // m/s^3
  Eigen::Vector3d snap{Eigen::Vector3d::Zero()};         // m/s^4
  double yaw{0.0};                                       // rad
  double yaw_rate{0.0};                                  // rad/s
  double yaw_acc{0.0};                                   // rad/s^2
  uint32_t trajectory_id;                                // 区分不同时刻的轨迹
  // 来源优先级与抢占优先级
  // uint8_t source_priority;
  // uint8_t preemption_priority;
};

/**
 * @brief 控制输出掩码位定义。
 */
enum class ControllerOutputMask : uint32_t {
  UNDEFINED = 0U,
  POSITION = 1U << 0, ///< position 字段有效
  VELOCITY = 1U << 1, ///< velocity 字段有效
  ACCELERATION = 1U << 2,
  FORCE = 1U << 3,
  YAW = 1U << 4,
  YAW_RATE = 1U << 5,
  ATTITUDE = 1U << 6, ///< attitude 字段有效
  BODY_RATE = 1U << 7,
  THRUST = 1U << 8 ///< thrust 字段有效
};

// ControllerOutput output;
// output.channel_enable(uav_control::ControllerOutputMase);

/**
 * @brief 控制器标准输出。
 * @note 各字段是否有效由 output_mask 指示，FSM/执行器应按掩码消费数据。
 */
struct ControllerOutput {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ControllerOutput() = default;
  ~ControllerOutput() = default;

  /**
   * @brief 使能某个输出子项。
   * @param item 需要使能的掩码位。
   */
  void channel_enable(ControllerOutputMask item) {
    output_mask |= static_cast<uint32_t>(item);
  }

  /**
   * @brief 关闭某个输出子项。
   * @param item 需要关闭的掩码位。
   */
  void channel_disable(ControllerOutputMask item) {
    output_mask &= ~static_cast<uint32_t>(item);
  }

  /**
   * @brief 判断某个输出子项是否已使能。
   * @param item 掩码位。
   * @return true 已使能；false 未使能。
   */
  bool is_channel_enabled(ControllerOutputMask item) const {
    return (output_mask & static_cast<uint32_t>(item)) != 0U;
  }

  // PositionTarget 对应字段
  Eigen::Vector3d position = Eigen::Vector3d::Zero(); ///< POSITION
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero(); ///< VELOCITY
  Eigen::Vector3d acceleration_or_force =
      Eigen::Vector3d::Zero(); ///< ACCELERATION / FORCE
  double yaw = 0.0;            ///< YAW
  double yaw_rate = 0.0;       ///< YAW_RATE

  // AttitudeTarget 对应字段
  Eigen::Quaterniond attitude = Eigen::Quaterniond::Identity(); ///< ATTITUDE
  Eigen::Vector3d body_rate = Eigen::Vector3d::Zero();          ///< BODY_RATE
  double thrust = 0.0;                                          ///< THRUST

  uint32_t output_mask = static_cast<uint32_t>(ControllerOutputMask::UNDEFINED);
};

// 是否应当在这里表示出控制器的期望输出，或者说控制器的期望输出是什么样的？

}; // namespace uav_control
