#include <mutex>

#include "mavros_msgs/ExtendedState.h"
#include "px4_manager/px4_datatypes.h"
#include "px4_manager/px4_reader.h"

// 标准状态回调函数
void PX4_Reader::stateCallback(const mavros_msgs::State::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(system_state_mtx);
  system_state.connected = msg->connected;
  system_state.armed = msg->armed;
  // 基于最小实现原则，这里没有guided
  system_state.rc_input = msg->manual_input;
  system_state.flight_mode = flightmode_fromString(msg->mode);
  // 基于最小实现原则，这里没有system_status
}
// 扩展状态回调函数
void PX4_Reader::exstateCallback(
    const mavros_msgs::ExtendedState::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(system_state_mtx);
  system_state.landed_state =
      static_cast<px4_data::LandedState>(msg->landed_state);
  // 基于最小实现原则，这里没有vtol状态，也不需要实现它
}
// 系统状态回调函数
void PX4_Reader::sysCallback(const mavros_msgs::SysStatus::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(system_state_mtx);
  system_state.system_load = msg->load;
  system_state.voltage = msg->voltage_battery / 1000.0;
  system_state.current = msg->current_battery / 100.0;
  system_state.percent = msg->battery_remaining;
}
// ekf2估计器状态回调函数
void PX4_Reader::ekf2statusCallback(
    const mavros_msgs::EstimatorStatus::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(ekf2_state_mtx);
  if (msg->accel_error_status_flag == true) {
    // 如果为 true，说明加速度计校准有问题或受高频振动影响。
    // TODO: 打印日志，显示报错，做一些操作
  }
  // TODO: 需要根据不同的传感器或者什么取设计一个state_code表
  ekf2_state.state_codes = 0.0;
  // 自稳模式许可，要求姿态估计有效
  ekf2_state.allow_stabilize = msg->attitude_status_flag;
  // 定高模式，要求姿态估计有效+垂直速度有效+绝对高度有效
  ekf2_state.allow_altitude = msg->attitude_status_flag &&
                              msg->velocity_vert_status_flag &&
                              msg->pos_vert_abs_status_flag;
  // 定点模式，要求姿态估计有效+水平速度+水平位置
  ekf2_state.allow_position = msg->attitude_status_flag &&
                              msg->velocity_horiz_status_flag &&
                              msg->pos_horiz_rel_status_flag;

  // 请注意，当加速度计错误时，依赖与惯性导航的模式应当被设置为false
  if (msg->accel_error_status_flag == true) {
    // 自稳模式看情况吧
    // ekf2_state.allow_stabilize = false;
    ekf2_state.allow_altitude = false;
    ekf2_state.allow_position = false;
  }
}
// 光流数据回调函数
void PX4_Reader::opflowCallback(
    const mavros_msgs::OpticalFlowRad::ConstPtr& msg) {
  // 防御式检查，避免异常情况下解引用空指针
  if (!msg) {
    return;
  }
  // 将 MAVROS 光流消息转换为项目内原始结构体
  px4_data::opflow_raw_ temp(*msg);
  // 写入光流环形缓冲区，供后续窗口滤波与状态估计使用
  opflow_buffer_.push(temp);
  // 保留最新一帧原始数据（用于调试或回溯）
  std::lock_guard<std::mutex> lock(opflow_mtx);
  opflow_raw = temp;
}

// local系下里程计回调函数
void PX4_Reader::localOdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  std::lock_guard<std::mutex> lock(local_pose_mtx);
	// 时间戳
	local_odom.timestamp = msg->header.stamp;
	// 位置信息
	local_odom.position.x() = msg->pose.pose.position.x;
	local_odom.position.y() = msg->pose.pose.position.y;
	local_odom.position.z() = msg->pose.pose.position.z;
	// 姿态信息(四元数)
	local_odom.orientation.w() = msg->pose.pose.orientation.w;
	local_odom.orientation.x() = msg->pose.pose.orientation.x;
	local_odom.orientation.y() = msg->pose.pose.orientation.y;
	local_odom.orientation.z() = msg->pose.pose.orientation.z;
	// 速度信息
	local_odom.linear.x() = msg->twist.twist.linear.x;
	local_odom.linear.y() = msg->twist.twist.linear.y;
	local_odom.linear.z() = msg->twist.twist.linear.z;
	// 角速度信息
	local_odom.angular.x() = msg->twist.twist.angular.x;
	local_odom.angular.y() = msg->twist.twist.angular.y;
	local_odom.angular.z() = msg->twist.twist.angular.z;

	// 单独拉出来位置+姿态
	local_pose.position = local_odom.position;
	local_pose.orientation = local_odom.orientation;
}
// local系下速度的回调函数
void PX4_Reader::localVelCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
  std::lock_guard<std::mutex> lock(local_velocity_mtx);
	// 线速度
	local_velocity.linear.x() = msg->twist.linear.x;
	local_velocity.linear.y() = msg->twist.linear.y;
	local_velocity.linear.z() = msg->twist.linear.z;
	// 角速度
	local_velocity.angular.x() = msg->twist.angular.x;
	local_velocity.angular.y() = msg->twist.angular.y;
	local_velocity.angular.z() = msg->twist.angular.z;
}
// body系下的姿态回调函数
void PX4_Reader::bodyAttCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
  std::lock_guard<std::mutex> lock(body_pose_mtx);
	// 位置
	body_pose.position.x() = -1;
	body_pose.position.y() = -1;
	body_pose.position.z() = -1;
	// 姿态
	body_pose.orientation.w() = msg->orientation.w;
	body_pose.orientation.x() = msg->orientation.x;
	body_pose.orientation.y() = msg->orientation.y;
	body_pose.orientation.z() = msg->orientation.z;
}
// body系下速度的回调函数
void PX4_Reader::bodyVelCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
  std::lock_guard<std::mutex> lock(body_velocity_mtx);
	// 线速度
	body_velocity.linear.x() = msg->twist.linear.x;
	body_velocity.linear.y() = msg->twist.linear.y;
	body_velocity.linear.z() = msg->twist.linear.z;
	// 角速度
	body_velocity.angular.x() = msg->twist.angular.x;
	body_velocity.angular.y() = msg->twist.angular.y;
	body_velocity.angular.z() = msg->twist.angular.z;
}
