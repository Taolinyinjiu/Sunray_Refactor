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
	system_state.system_load = msg->load/10.0;
	system_state.voltage = msg->voltage_battery / 1000.0;
	system_state.current = msg->current_battery / 100.0;
	system_state.percent = msg->battery_remaining;
}
// ekf2估计器状态回调函数
void PX4_Reader::ekf2statusCallback(const mavros_msgs::EstimatorStatus::ConstPtr &msg){
	if(msg->accel_error_status_flag == true)
	{
		// 如果为 true，说明加速度计校准有问题或受高频振动影响。
		// TODO: 打印日志，显示报错，做一些操作

	}
	// TODO: 需要根据不同的传感器或者什么取设计一个state_code表
	ekf2_state.state_codes = 0.0;
	// 自稳模式许可，要求姿态估计有效
	ekf2_state.allow_stabilize = msg->attitude_status_flag;
	// 定高模式，要求姿态估计有效+垂直速度有效+绝对高度有效
	ekf2_state.allow_altitude = msg->attitude_status_flag && msg->velocity_vert_status_flag && msg->pos_vert_abs_status_flag;
	// 定点模式，要求姿态估计有效+水平速度+水平位置
	ekf2_state.allow_position = msg->attitude_status_flag && msg->velocity_horiz_status_flag && msg->pos_horiz_rel_status_flag;

	// 请注意，当加速度计错误时，依赖与惯性导航的模式应当被设置为false
	if(msg->accel_error_status_flag == true)
	{
		// 自稳模式看情况吧
		// ekf2_state.allow_stabilize = false;
		ekf2_state.allow_altitude = false;
		ekf2_state.allow_position = false;
	}
}
// 光流数据回调函数
void PX4_Reader::opflowCallback(const mavros_msgs::OpticalFlowRad::ConstPtr &msg)
{
	// 通过结构体的构造函数快速提取光流数据(感觉只是形式上的快速)
	px4_data::opflow_raw_ temp(*msg);
	// 丢进环形缓冲区

}