#include "px4_manager/mavros_eigen_conversions.h"
#include "mavros_msgs/State.h"
#include <cmath>
#include <cstddef>
#include <ros/ros.h>

namespace px4_common {
// 基础状态
State::State() {
  connected = false;
  armed = false;
  guided = false;
  manual_input = false;
  mode = "NULL";
  system_status = false;
}

State::State(const mavros_msgs::State &state_msg) {
  connected = state_msg.connected;
  armed = state_msg.armed;
  guided = state_msg.guided;
  manual_input = state_msg.manual_input;
  mode = state_msg.mode;
  system_status = state_msg.system_status;
}
// 扩展状态
ExtendedState::ExtendedState() {
  vtol_state = 0;
  landed_state = 0;
}

ExtendedState::ExtendedState(
    const mavros_msgs::ExtendedState &extendedstate_msg) {
  vtol_state = extendedstate_msg.vtol_state;
  landed_state = extendedstate_msg.landed_state;
}
// 电池状态
BatteryState::BatteryState() {
  voltage = 0.0;
  temperature = 0.0;
  current = 0.0;
  charge = 0.0;
  capacity = 0.0;
  design_capacity = 0.0;
  percentage = 0.0;
  power_supply_status = 0.0;
  power_supply_health = 0.0;
  power_supply_technology = 0.0;
  present = false;
  cell_voltage.clear();
  cell_temperature.clear();
  location = "NULL";
  serial_number = "NULL";
}
BatteryState::BatteryState(const sensor_msgs::BatteryState &batterystate_msg) {
  voltage = batterystate_msg.voltage;
  temperature = batterystate_msg.temperature;
  current = batterystate_msg.current;
  charge = batterystate_msg.charge;
  capacity = batterystate_msg.capacity;
  design_capacity = batterystate_msg.design_capacity;
  percentage = batterystate_msg.percentage;
  power_supply_status = batterystate_msg.power_supply_status;
  power_supply_health = batterystate_msg.power_supply_health;
  power_supply_technology = batterystate_msg.power_supply_technology;
  present = batterystate_msg.present;
  cell_voltage = batterystate_msg.cell_voltage;
  cell_temperature = batterystate_msg.cell_temperature;
  location = batterystate_msg.location;
  serial_number = batterystate_msg.serial_number;
}
// 系统状态
SysStatus::SysStatus() {
  sensors_present = 0;
  sensors_enabled = 0;
  sensors_health = 0;
  load = 0;
  voltage_battery = 0;
  current_battery = 0;
  battery_remaining = 0;
  drop_rate_comm = 0;
  errors_comm = 0;
  errors_count1 = 0;
  errors_count2 = 0;
  errors_count3 = 0;
  errors_count4 = 0;
}

SysStatus::SysStatus(const mavros_msgs::SysStatus &sysstatus_msg) {
  sensors_present = sysstatus_msg.sensors_present;
  sensors_enabled = sysstatus_msg.sensors_enabled;
  sensors_health = sysstatus_msg.sensors_health;
  load = sysstatus_msg.load;
  voltage_battery = sysstatus_msg.voltage_battery;
  current_battery = sysstatus_msg.current_battery;
  battery_remaining = sysstatus_msg.battery_remaining;
  drop_rate_comm = sysstatus_msg.drop_rate_comm;
  errors_comm = sysstatus_msg.errors_comm;
  errors_count1 = sysstatus_msg.errors_count1;
  errors_count2 = sysstatus_msg.errors_count2;
  errors_count3 = sysstatus_msg.errors_count3;
  errors_count4 = sysstatus_msg.errors_count4;
}

// 估计器状态
EstimatorStatus::EstimatorStatus() {
  attitude_status_flag = false;
  velocity_horiz_status_flag = false;
  velocity_vert_status_flag = false;
  pos_horiz_rel_status_flag = false;
  pos_horiz_abs_status_flag = false;
  pos_vert_abs_status_flag = false;
  pos_vert_agl_status_flag = false;
  const_pos_mode_status_flag = false;
  pred_pos_horiz_rel_status_flag = false;
  pred_pos_horiz_abs_status_flag = false;
  gps_glitch_status_flag = false;
  accel_error_status_flag = false;
}

EstimatorStatus::EstimatorStatus(
    const mavros_msgs::EstimatorStatus &estimatorstatus_msg) {
  attitude_status_flag = estimatorstatus_msg.attitude_status_flag;
  velocity_horiz_status_flag = estimatorstatus_msg.velocity_horiz_status_flag;
  velocity_vert_status_flag = estimatorstatus_msg.velocity_vert_status_flag;
  pos_horiz_rel_status_flag = estimatorstatus_msg.pos_horiz_rel_status_flag;
  pos_horiz_abs_status_flag = estimatorstatus_msg.pos_horiz_abs_status_flag;
  pos_vert_abs_status_flag = estimatorstatus_msg.pos_vert_abs_status_flag;
  pos_vert_agl_status_flag = estimatorstatus_msg.pos_vert_agl_status_flag;
  const_pos_mode_status_flag = estimatorstatus_msg.const_pos_mode_status_flag;
  pred_pos_horiz_rel_status_flag =
      estimatorstatus_msg.pred_pos_horiz_rel_status_flag;
  pred_pos_horiz_abs_status_flag =
      estimatorstatus_msg.pred_pos_horiz_abs_status_flag;
  gps_glitch_status_flag = estimatorstatus_msg.gps_glitch_status_flag;
  accel_error_status_flag = estimatorstatus_msg.accel_error_status_flag;
}

// 光流
OpticalFlowRad::OpticalFlowRad() {
  integration_time_us = 0;
  integrated_x = 0.0f;
  integrated_y = 0.0f;
  integrated_xgyro = 0.0f;
  integrated_ygyro = 0.0f;
  integrated_zgyro = 0.0f;
	temperature = 0;
	quality = 0;
  time_delta_distance_us = 0;
  distance = 0.0f;
}

OpticalFlowRad::OpticalFlowRad(const mavros_msgs::OpticalFlowRad &of_msg) {
  integration_time_us = of_msg.integration_time_us;
  integrated_x = of_msg.integrated_x;
  integrated_y = of_msg.integrated_y;
  integrated_xgyro = of_msg.integrated_xgyro;
  integrated_ygyro = of_msg.integrated_ygyro;
  integrated_zgyro = of_msg.integrated_zgyro;
	temperature = of_msg.temperature;
	quality = of_msg.quality;
  time_delta_distance_us = of_msg.time_delta_distance_us;
  distance = of_msg.distance;
}

// 里程计
Odometry::Odometry() {
  header_sec = 0;
  header_nsec = 0;
  header_frame_id.clear();
  child_frame_id.clear();
  for (std::size_t i = 0; i < 3; ++i) {
    position[i] = 0.0;
    linear_velocity[i] = 0.0;
    angular_velocity[i] = 0.0;
  }
  for (std::size_t i = 0; i < 4; ++i)
    orientation[i] = 0.0;
  for (std::size_t i = 0; i < 36; ++i) {
    pose_covariance[i] = 0.0;
    twist_covariance[i] = 0.0;
  }
}

Odometry::Odometry(const nav_msgs::Odometry &odom_msg) {
  header_sec = odom_msg.header.stamp.sec;
  header_nsec = odom_msg.header.stamp.nsec;
  header_frame_id = odom_msg.header.frame_id;
  child_frame_id = odom_msg.child_frame_id;
  position[0] = odom_msg.pose.pose.position.x;
  position[1] = odom_msg.pose.pose.position.y;
  position[2] = odom_msg.pose.pose.position.z;
  orientation[0] = odom_msg.pose.pose.orientation.x;
  orientation[1] = odom_msg.pose.pose.orientation.y;
  orientation[2] = odom_msg.pose.pose.orientation.z;
  orientation[3] = odom_msg.pose.pose.orientation.w;
  for (std::size_t i = 0; i < 36; ++i)
    pose_covariance[i] = odom_msg.pose.covariance[i];
  linear_velocity[0] = odom_msg.twist.twist.linear.x;
  linear_velocity[1] = odom_msg.twist.twist.linear.y;
  linear_velocity[2] = odom_msg.twist.twist.linear.z;
  angular_velocity[0] = odom_msg.twist.twist.angular.x;
  angular_velocity[1] = odom_msg.twist.twist.angular.y;
  angular_velocity[2] = odom_msg.twist.twist.angular.z;
  for (std::size_t i = 0; i < 36; ++i)
    twist_covariance[i] = odom_msg.twist.covariance[i];
}

// TwistStamped
TwistStamped::TwistStamped() {
  header_sec = 0;
  header_nsec = 0;
  header_frame_id.clear();
  for (int i = 0; i < 3; ++i) {
    linear[i] = 0.0;
    angular[i] = 0.0;
  }
}

TwistStamped::TwistStamped(const geometry_msgs::TwistStamped &twist_msg) {
  header_sec = twist_msg.header.stamp.sec;
  header_nsec = twist_msg.header.stamp.nsec;
  header_frame_id = twist_msg.header.frame_id;
  linear[0] = twist_msg.twist.linear.x;
  linear[1] = twist_msg.twist.linear.y;
  linear[2] = twist_msg.twist.linear.z;
  angular[0] = twist_msg.twist.angular.x;
  angular[1] = twist_msg.twist.angular.y;
  angular[2] = twist_msg.twist.angular.z;
}

// IMU
Imu::Imu() {
  for (int i = 0; i < 4; ++i)
    orientation[i] = 0.0;
  for (int i = 0; i < 9; ++i)
    orientation_covariance[i] = 0.0;
  for (int i = 0; i < 3; ++i)
    angular_velocity[i] = 0.0;
  for (int i = 0; i < 9; ++i)
    angular_velocity_covariance[i] = 0.0;
  for (int i = 0; i < 3; ++i)
    linear_acceleration[i] = 0.0;
  for (int i = 0; i < 9; ++i)
    linear_acceleration_covariance[i] = 0.0;
}

Imu::Imu(const sensor_msgs::Imu &imu_msg) {
  orientation[0] = imu_msg.orientation.x;
  orientation[1] = imu_msg.orientation.y;
  orientation[2] = imu_msg.orientation.z;
  orientation[3] = imu_msg.orientation.w;
  for (int i = 0; i < 9; ++i)
    orientation_covariance[i] = imu_msg.orientation_covariance[i];
  angular_velocity[0] = imu_msg.angular_velocity.x;
  angular_velocity[1] = imu_msg.angular_velocity.y;
  angular_velocity[2] = imu_msg.angular_velocity.z;
  for (int i = 0; i < 9; ++i)
    angular_velocity_covariance[i] = imu_msg.angular_velocity_covariance[i];
  linear_acceleration[0] = imu_msg.linear_acceleration.x;
  linear_acceleration[1] = imu_msg.linear_acceleration.y;
  linear_acceleration[2] = imu_msg.linear_acceleration.z;
  for (int i = 0; i < 9; ++i)
    linear_acceleration_covariance[i] =
        imu_msg.linear_acceleration_covariance[i];
}

} // namespace px4_common