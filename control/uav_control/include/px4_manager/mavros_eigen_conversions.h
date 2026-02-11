/**
 * @file mavros_eigen_conversions.h
 * @brief MAVROS 消息和 Eigen 向量转换工具库
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

#include <cstdint>
#include <string>
#include <vector>

#include "geometry_msgs/TwistStamped.h"
#include "mavros_msgs/EstimatorStatus.h"
#include "mavros_msgs/ExtendedState.h"
#include "mavros_msgs/OpticalFlowRad.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/SysStatus.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "sensor_msgs/BatteryState.h"
#include "sensor_msgs/Imu.h"

namespace px4_common {
struct State {
  State();
  State(const mavros_msgs::State &state_msg);

  bool connected;
  bool armed;
  bool guided;
  bool manual_input;
  std::string mode;
  bool system_status;
};

struct ExtendedState {
  ExtendedState();
  ExtendedState(const mavros_msgs::ExtendedState &extendedstate_msg);

  uint8_t vtol_state;
  uint8_t landed_state;
};

struct BatteryState {
  BatteryState();
  BatteryState(const sensor_msgs::BatteryState &batterystate_msg);

  float voltage;
  float temperature;
  float current;
  float charge;
  float capacity;
  float design_capacity;
  float percentage;
  uint8_t power_supply_status;
  uint8_t power_supply_health;
  uint8_t power_supply_technology;
  bool present;
  std::vector<float> cell_voltage;
  std::vector<float> cell_temperature;
  std::string location;
  std::string serial_number;
};

struct SysStatus {
  SysStatus();
  SysStatus(const mavros_msgs::SysStatus &sysstatus_msg);

  uint32_t sensors_present;
  uint32_t sensors_enabled;
  uint32_t sensors_health;
  uint16_t load;
  uint16_t voltage_battery;
  int16_t current_battery;
  int8_t battery_remaining;
  uint16_t drop_rate_comm;
  uint16_t errors_comm;
  uint16_t errors_count1;
  uint16_t errors_count2;
  uint16_t errors_count3;
  uint16_t errors_count4;
};

struct EstimatorStatus {
  EstimatorStatus();
  EstimatorStatus(const mavros_msgs::EstimatorStatus &estimatorstatus_msg);

  bool attitude_status_flag;
  bool velocity_horiz_status_flag;
  bool velocity_vert_status_flag;
  bool pos_horiz_rel_status_flag;
  bool pos_horiz_abs_status_flag;
  bool pos_vert_abs_status_flag;
  bool pos_vert_agl_status_flag;
  bool const_pos_mode_status_flag;
  bool pred_pos_horiz_rel_status_flag;
  bool pred_pos_horiz_abs_status_flag;
  bool gps_glitch_status_flag;
  bool accel_error_status_flag;
};

struct OpticalFlowRad {
  OpticalFlowRad();
  OpticalFlowRad(const mavros_msgs::OpticalFlowRad &of_msg);

  uint32_t integration_time_us;
  float integrated_x;
  float integrated_y;
  float integrated_xgyro;
  float integrated_ygyro;
  float integrated_zgyro;
	int16_t temperature;
	uint8_t quality;
  uint32_t time_delta_distance_us;
  float distance;
};

struct Odometry {
  Odometry();
  Odometry(const nav_msgs::Odometry &odom_msg);

  // header
  uint32_t header_sec;
  uint32_t header_nsec;
  std::string header_frame_id;
  std::string child_frame_id;

  double position[3];
  double orientation[4];
  double pose_covariance[36];

  double linear_velocity[3];
  double angular_velocity[3];
  double twist_covariance[36];
};

struct TwistStamped {
  TwistStamped();
  TwistStamped(const geometry_msgs::TwistStamped &twist_msg);

  uint32_t header_sec;
  uint32_t header_nsec;
  std::string header_frame_id;

  double linear[3];
  double angular[3];
};

struct Imu {
  Imu();
  Imu(const sensor_msgs::Imu &imu_msg);

  double orientation[4];
  double orientation_covariance[9];

  double angular_velocity[3];
  double angular_velocity_covariance[9];

  double linear_acceleration[3];
  double linear_acceleration_covariance[9];
};

}; // namespace px4_common