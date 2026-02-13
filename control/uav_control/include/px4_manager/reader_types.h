/**
 * @file reader_types.h
 * @brief 各种的数据结构
 *
 * @details
 * 这个头文件提供了在 UAV 控制中常用的数据类型转换函数，用于将 MAVROS
 * 消息格式与 Eigen 线性代数库之间进行转换。
 *
 * 设计意图：
 * - 简化 ROS MAVROS 节点与 Eigen 库的数据交互
 * - 提供类型安全的转换接口
 * - 支持位置、速度、四元数等 3D 控制常见的数据类型
 * - 降低代码复杂度，提高代码可维护性
 *
 * @author taolinyinjiu
 * @date 2026-02-10
 * @version 0.1
 *
 * @see https://docs.ros.org/noetic/api/mavros/html/
 * @see https://eigen.tuxfamily.org/dox-3.4/
 * @see
 * https://yundrone.feishu.cn/wiki/RKMSw79HbigKLOkDo4Rc8WWtn9g?from=from_copylink
 */

#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "Eigen/Dense"
#include "Eigen/src/Core/Matrix.h"
#include "mavros_msgs/ExtendedState.h"
#include "mavros_msgs/State.h"

namespace reader_types {
enum class FlightMode : uint8_t {
  UNKNOWN = 0,
  MANUAL,
  ACRO,
  ALTCTL,
  POSCTL,
  OFFBOARD,
  STABILIZED,
  RATTITUDE,
  MISSION,
  LOITER,
  RTL,
  LAND,
  TAKEOFF,
  READY
};

enum class LandedState : uint8_t {
  UNDEFINED = 0,
  ON_GROUND,
  IN_AIR,
  TAKEOFF,
  LANDING
};

struct system_state_ {
  uint8_t uav_id;
  bool connected;
  bool armed;
  bool rc_input;
  float system_load;
	float voltage;
  float current;
  float percent;
  FlightMode flight_mode;
  LandedState landed_state;
};

struct ekf2_state_ {
  uint32_t state_codes;
  bool allow_stabilize;
  bool allow_altitude;
  bool allow_position;
};

struct flow_state_ {
  float distance;
  uint8_t quality;
  uint32_t integration_time_us;
  float integrated_x;
  float integrated_y;
  float integrated_xgyro;
  float integrated_ygyro;
  float integrated_zgyro;
};

struct pose_ {
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
};

struct velocity_ {
  Eigen::Vector3d linear;
  Eigen::Vector3d angular;
};

struct ekf2_param_ {
  int ev_ctrl;
  int hgt_ref;
  float ev_delay;
};

struct pid_param_ {
  float kp;
  float ki;
  float kd;
};

struct attitude_param_ {
  pid_param_ roll;
  pid_param_ pitch;
  pid_param_ yaw;
};

struct velocity_param_ {
  pid_param_ mpc_xy;
  pid_param_ mpc_z;
};

struct position_param_ {
  pid_param_ mpc_xy;
  pid_param_ mpc_z;
};

}  // namespace reader_types
