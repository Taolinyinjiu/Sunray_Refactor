#include <mutex>

#include "mavros_msgs/ExtendedState.h"
#include "px4_manager/px4_data_types.h"
#include "px4_manager/px4_data_reader.h"

// 标准状态回调函数
void Px4DataReader::state_callback(const mavros_msgs::State::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(system_state_mutex_);
  system_state_cache_.connected = msg->connected;
  system_state_cache_.armed = msg->armed;
  // 基于最小实现原则，这里没有guided
  system_state_cache_.rc_input = msg->manual_input;
  system_state_cache_.flight_mode = flight_mode_from_string(msg->mode);
  // 基于最小实现原则，这里没有system_status
}
// 扩展状态回调函数
void Px4DataReader::extended_state_callback(
    const mavros_msgs::ExtendedState::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(system_state_mutex_);
  system_state_cache_.landed_state =
      static_cast<px4_data::LandedState>(msg->landed_state);
  // 基于最小实现原则，这里没有vtol状态，也不需要实现它
}
// 系统状态回调函数
void Px4DataReader::system_status_callback(const mavros_msgs::SysStatus::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(system_state_mutex_);
  system_state_cache_.system_load = msg->load;
  system_state_cache_.voltage = msg->voltage_battery / 1000.0;
  system_state_cache_.current = msg->current_battery / 100.0;
  system_state_cache_.percent = msg->battery_remaining;
}
// ekf2估计器状态回调函数
void Px4DataReader::ekf2_status_callback(
    const mavros_msgs::EstimatorStatus::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(ekf2_state_mutex_);
  if (msg->accel_error_status_flag == true) {
    // 如果为 true，说明加速度计校准有问题或受高频振动影响。
    // TODO: 打印日志，显示报错，做一些操作
  }
  // TODO: 需要根据不同的传感器或者什么取设计一个state_code表
  ekf2_state_cache_.state_codes = 0.0;
  // 自稳模式许可，要求姿态估计有效
  ekf2_state_cache_.allow_stabilize = msg->attitude_status_flag;
  // 定高模式，要求姿态估计有效+垂直速度有效+绝对高度有效
  ekf2_state_cache_.allow_altitude = msg->attitude_status_flag &&
                              msg->velocity_vert_status_flag &&
                              msg->pos_vert_abs_status_flag;
  // 定点模式，要求姿态估计有效+水平速度+水平位置
  ekf2_state_cache_.allow_position = msg->attitude_status_flag &&
                              msg->velocity_horiz_status_flag &&
                              msg->pos_horiz_rel_status_flag;

  // 请注意，当加速度计错误时，依赖与惯性导航的模式应当被设置为false
  if (msg->accel_error_status_flag == true) {
    // 自稳模式看情况吧
    // ekf2_state_cache_.allow_stabilize = false;
    ekf2_state_cache_.allow_altitude = false;
    ekf2_state_cache_.allow_position = false;
  }
}
// 光流数据回调函数
void Px4DataReader::optical_flow_callback(
    const mavros_msgs::OpticalFlowRad::ConstPtr& msg) {
  // 防御式检查，避免异常情况下解引用空指针
  if (!msg) {
    return;
  }
  // 将 MAVROS 光流消息转换为项目内原始结构体
  px4_data::OpticalFlowRaw temp(*msg);
  // 写入光流环形缓冲区，供后续窗口滤波与状态估计使用
  opflow_buffer_.push(temp);
  // 保留最新一帧原始数据（用于调试或回溯）
  std::lock_guard<std::mutex> lock(opflow_mutex_);
  latest_opflow_raw_ = temp;
}

// local系下里程计回调函数
void Px4DataReader::local_odometry_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
  std::lock_guard<std::mutex> lock(local_pose_mutex_);
	// 时间戳
	local_odometry_cache_.timestamp = msg->header.stamp;
	// 位置信息
	local_odometry_cache_.position.x() = msg->pose.pose.position.x;
	local_odometry_cache_.position.y() = msg->pose.pose.position.y;
	local_odometry_cache_.position.z() = msg->pose.pose.position.z;
	// 姿态信息(四元数)
	local_odometry_cache_.orientation.w() = msg->pose.pose.orientation.w;
	local_odometry_cache_.orientation.x() = msg->pose.pose.orientation.x;
	local_odometry_cache_.orientation.y() = msg->pose.pose.orientation.y;
	local_odometry_cache_.orientation.z() = msg->pose.pose.orientation.z;
	// 速度信息
	local_odometry_cache_.linear.x() = msg->twist.twist.linear.x;
	local_odometry_cache_.linear.y() = msg->twist.twist.linear.y;
	local_odometry_cache_.linear.z() = msg->twist.twist.linear.z;
	// 角速度信息
	local_odometry_cache_.angular.x() = msg->twist.twist.angular.x;
	local_odometry_cache_.angular.y() = msg->twist.twist.angular.y;
	local_odometry_cache_.angular.z() = msg->twist.twist.angular.z;

	// 单独拉出来位置+姿态
	local_pose_cache_.position = local_odometry_cache_.position;
	local_pose_cache_.orientation = local_odometry_cache_.orientation;
}
// local系下速度的回调函数
void Px4DataReader::local_velocity_callback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
  std::lock_guard<std::mutex> lock(local_velocity_mutex_);
	// 线速度
	local_velocity_cache_.linear.x() = msg->twist.linear.x;
	local_velocity_cache_.linear.y() = msg->twist.linear.y;
	local_velocity_cache_.linear.z() = msg->twist.linear.z;
	// 角速度
	local_velocity_cache_.angular.x() = msg->twist.angular.x;
	local_velocity_cache_.angular.y() = msg->twist.angular.y;
	local_velocity_cache_.angular.z() = msg->twist.angular.z;
}
// body系下的姿态回调函数
void Px4DataReader::body_attitude_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
  std::lock_guard<std::mutex> lock(body_pose_mutex_);
	// 位置
	body_pose_cache_.position.x() = -1;
	body_pose_cache_.position.y() = -1;
	body_pose_cache_.position.z() = -1;
	// 姿态
	body_pose_cache_.orientation.w() = msg->orientation.w;
	body_pose_cache_.orientation.x() = msg->orientation.x;
	body_pose_cache_.orientation.y() = msg->orientation.y;
	body_pose_cache_.orientation.z() = msg->orientation.z;
}
// body系下速度的回调函数
void Px4DataReader::body_velocity_callback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
  std::lock_guard<std::mutex> lock(body_velocity_mutex_);
	// 线速度
	body_velocity_cache_.linear.x() = msg->twist.linear.x;
	body_velocity_cache_.linear.y() = msg->twist.linear.y;
	body_velocity_cache_.linear.z() = msg->twist.linear.z;
	// 角速度
	body_velocity_cache_.angular.x() = msg->twist.angular.x;
	body_velocity_cache_.angular.y() = msg->twist.angular.y;
	body_velocity_cache_.angular.z() = msg->twist.angular.z;
}
