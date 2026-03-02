/**
 * @file px4_data_types.h
 * @brief Sunray PX4 模块使用的数据类型定义
 */

#pragma once

#include <Eigen/Dense>
#include <cstdint>

#include "mavros_msgs/OpticalFlowRad.h"
#include "ros/time.h"

namespace px4_data {

enum class FlightMode : uint8_t {
  kUndefined = 0,
  kManual,
  kAcro,
  kAltctl,
  kPosctl,
  kOffboard,
  kStabilized,
  kRattitude,
  kAutoMission,
  kAutoLoiter,
  kAutoRtl,
  kAutoLand,
  kAutoRtgs,
  kAutoReady,
  kAutoTakeoff
};

enum class LandedState : uint8_t {
  kUndefined = 0,
  kOnGround,
  kInAir,
  kTakeoff,
  kLanding
};

struct SystemState {
  uint8_t uav_id = 0;
  bool connected = false;
  bool armed = false;
  bool rc_input = false;
  uint8_t system_load = 0;
  float voltage = 0.0f;
  float current = 0.0f;
  float percent = 0.0f;
  FlightMode flight_mode = FlightMode::kUndefined;
  LandedState landed_state = LandedState::kUndefined;
};

struct Ekf2State {
  uint32_t state_codes = 0;
  bool allow_stabilize = false;
  bool allow_altitude = false;
  bool allow_position = false;
};

struct OpticalFlowRaw {
  double timestamp;
  uint8_t quality;
  uint32_t integration_time_us;
  float integrated_x;
  float integrated_y;
  float integrated_xgyro;
  float integrated_ygyro;
  float integrated_zgyro;
  uint32_t time_delta_distance_us;
  float distance;

  OpticalFlowRaw();
  explicit OpticalFlowRaw(const mavros_msgs::OpticalFlowRad& msg);
};

struct OpticalFlowState {
  double timestamp = 0.0;
  bool valid = false;
  uint8_t quality = 0;
  float distance = 0.0f;
  float dt_s = 0.0f;
  float vx_raw = 0.0f;
  float vy_raw = 0.0f;
  float vx = 0.0f;
  float vy = 0.0f;
  uint32_t sample_count = 0;
};

struct Pose {
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
};

struct Velocity {
  Eigen::Vector3d linear;
  Eigen::Vector3d angular;
};

struct Odometry {
  ros::Time timestamp;
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
  Eigen::Vector3d linear;
  Eigen::Vector3d angular;
};

/**
 * @brief EKF2_EV_CTRL 位掩码封装。
 *
 * 目的：
 * - 避免业务代码直接写“魔法数字”；
 * - 提供可读的 enable/disable 接口；
 * - 需要写入参数时可通过 toInt() 得到整型值。
 *
 * 用法示例：
 * @code
 * px4_data::ev_ctrl param;
 * param.enable_Horizontalposition();
 * param.enable_Yaw();
 * int v = param.toInt();  // 可直接用于 EKF2_EV_CTRL
 * @endcode
 */
struct EvCtrlParam {
  // 位定义（按 PX4 EKF2_EV_CTRL 的常见约定）
  static constexpr uint32_t kHorizontalPosition = 1u << 0;
  static constexpr uint32_t kVerticalPosition = 1u << 1;
  static constexpr uint32_t kVelocity = 1u << 2;
  static constexpr uint32_t kYaw = 1u << 3;

  uint32_t mask = 0u;

  void clear() { mask = 0u; }

  // 你期望的风格：param.enable_Horizontalposition();
  void enable_Horizontalposition() { mask |= kHorizontalPosition; }
  void disable_Horizontalposition() { mask &= ~kHorizontalPosition; }
  bool has_Horizontalposition() const { return (mask & kHorizontalPosition) != 0u; }

  void enable_Verticalposition() { mask |= kVerticalPosition; }
  void disable_Verticalposition() { mask &= ~kVerticalPosition; }
  bool has_Verticalposition() const { return (mask & kVerticalPosition) != 0u; }

  void enable_Velocity() { mask |= kVelocity; }
  void disable_Velocity() { mask &= ~kVelocity; }
  bool has_Velocity() const { return (mask & kVelocity) != 0u; }

  void enable_Yaw() { mask |= kYaw; }
  void disable_Yaw() { mask &= ~kYaw; }
  bool has_Yaw() const { return (mask & kYaw) != 0u; }

  int toInt() const { return static_cast<int>(mask); }
  static EvCtrlParam fromInt(int value) {
    EvCtrlParam out;
    out.mask = static_cast<uint32_t>(value);
    return out;
  }
};

// 便于写成：ev_ctrl param;
using ev_ctrl = EvCtrlParam;

struct Ekf2Params {
  int ev_ctrl = 0;
  int hgt_ref = 0;
  float ev_delay = 0.0f;

  void set_ev_ctrl(const EvCtrlParam& ctrl) { ev_ctrl = ctrl.toInt(); }
};

struct PidParams {
  float kp = 0.0f;
  float ki = 0.0f;
  float kd = 0.0f;
};

struct AttitudeParams {
  PidParams roll;
  PidParams pitch;
  PidParams yaw;
};

struct VelocityParams {
  PidParams mpc_xy;
  PidParams mpc_z;
};

struct PositionParams {
  PidParams mpc_xy;
  PidParams mpc_z;
};

// 兼容旧命名，避免一次性改动影响其他模块。
using system_state_ = SystemState;
using ekf2_state_ = Ekf2State;
using opflow_raw_ = OpticalFlowRaw;
using opflow_state_ = OpticalFlowState;
using pose_ = Pose;
using velocity_ = Velocity;
using odom_ = Odometry;
using ekf2_param_ = Ekf2Params;
using pid_param_ = PidParams;
using attitude_param_ = AttitudeParams;
using velocity_param_ = VelocityParams;
using position_param_ = PositionParams;

}  // namespace px4_data
