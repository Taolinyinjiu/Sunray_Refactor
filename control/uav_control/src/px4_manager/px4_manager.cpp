#include "px4_manager/px4_manager.h"
#include "px4_manager/mavros_eigen_conversions.h"

PX4_StateManager::PX4_StateManager() {
  // default factor
}

// factor with ros nodehandle
PX4_StateManager::PX4_StateManager(ros::NodeHandle &nh) {
	init_sub(nh);
}

void PX4_StateManager::init_sub(ros::NodeHandle &nh) {
	if (!nh.getParam("uav_id", uav_id)) {
		ROS_FATAL("Failed to get uav_id parameter. Shutting down...");
		ros::shutdown();
		return;
	}
	ROS_INFO("Initializing PX4_StateManager for UAV: %s", uav_id.c_str());

	// 注册订阅者
	state_sub_ =
	    nh.subscribe("/mavros/state", 10, &PX4_StateManager::stateCallback, this);
	exstate_sub_ = nh.subscribe("/mavros/extended_state", 10,
	                            &PX4_StateManager::exstateCallback, this);
	battery_sub_ = nh.subscribe("/mavros/battery", 10,
	                            &PX4_StateManager::batteryCallback, this);
	sys_sub_ = nh.subscribe("/mavros/sys_status", 10,
	                        &PX4_StateManager::sysCallback, this);
	ekf2status_sub_ = nh.subscribe("/mavros/estimator_status", 10,
	                               &PX4_StateManager::ekf2statusCallback, this);
	opflow_sub_ = nh.subscribe("/mavros/optical_flow/rad", 10,
	                           &PX4_StateManager::opflowCallback, this);
	local_odom_sub_ = nh.subscribe("/mavros/local_position/odom", 10,
	                               &PX4_StateManager::localOdomCallback, this);
	local_vel_sub_ = nh.subscribe("/mavros/local_position/velocity_local", 10,
	                              &PX4_StateManager::localVelCallback, this);
	body_att_sub_ = nh.subscribe("/mavros/imu/data", 10,
	                             &PX4_StateManager::bodyAttCallback, this);
	body_vel_sub_ = nh.subscribe("/mavros/local_position/velocity_body", 10,
	                             &PX4_StateManager::bodyVelCallback, this);
}

// 状态回调函数实现
void PX4_StateManager::stateCallback(const mavros_msgs::State::ConstPtr &msg) {
	// TODO: 处理状态消息
	px4_common::State temp_msg(*msg);
	
	
}

void PX4_StateManager::exstateCallback(
    const mavros_msgs::ExtendedState::ConstPtr &msg) {
	// TODO: 处理扩展状态消息
}

void PX4_StateManager::batteryCallback(
    const sensor_msgs::BatteryState::ConstPtr &msg) {
	// TODO: 处理电池状态消息
}

void PX4_StateManager::sysCallback(const mavros_msgs::SysStatus::ConstPtr &msg) {
	// TODO: 处理系统状态消息
}

void PX4_StateManager::ekf2statusCallback(
    const mavros_msgs::EstimatorStatus::ConstPtr &msg) {
	// TODO: 处理估计器状态消息
}

void PX4_StateManager::opflowCallback(
    const mavros_msgs::OpticalFlowRad::ConstPtr &msg) {
	// TODO: 处理光流消息
}

void PX4_StateManager::localOdomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
	// TODO: 处理本地里程计消息
}

void PX4_StateManager::localVelCallback(
    const geometry_msgs::TwistStamped::ConstPtr &msg) {
	// TODO: 处理本地速度消息
}

void PX4_StateManager::bodyAttCallback(const sensor_msgs::Imu::ConstPtr &msg) {
	// TODO: 处理IMU消息
}

void PX4_StateManager::bodyVelCallback(
    const geometry_msgs::TwistStamped::ConstPtr &msg) {
	// TODO: 处理机体速度消息
}
