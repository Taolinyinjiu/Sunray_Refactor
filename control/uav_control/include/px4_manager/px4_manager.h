/**
 * @file px4_datareader.h
 * @brief PX4数据读取类
 *
 * @details
 * 这个头文件实现了一个PX4的数据读取类，该类通过订阅一些mavros消息，获取当前飞控的状态，并通过一些结构体变量向外暴露，
 * 供用户快速的判断px4飞控的状态，同时提供切换模式，写入参数等函数，方便用户的使用
 *
 * 设计意图：
 * - 简化其他节点与Mavros的交互
 * - 降低代码复杂度，提高代码可维护性
 *
 * @author taolinyinjiu
 * @date 2026-02-11
 * @version 0.1
 *
 * @see https://docs.ros.org/noetic/api/mavros/html/
 * @see https://eigen.tuxfamily.org/dox-3.4/
 * @see
 * https://yundrone.feishu.cn/wiki/RKMSw79HbigKLOkDo4Rc8WWtn9g?from=from_copylink
 */
#pragma once

#include "mavros_eigen_conversions.h"
#include "ros/node_handle.h"

class PX4_StateManager {
public:
  PX4_StateManager();
  PX4_StateManager(ros::NodeHandle &nh);

  ~PX4_StateManager() {};

  void init_sub(ros::NodeHandle &nh);

private:
	std::string uav_id;
  // 根据	https://yundrone.feishu.cn/wiki/RKMSw79HbigKLOkDo4Rc8WWtn9g
  // subscriber 
  ros::Subscriber state_sub_;
  ros::Subscriber exstate_sub_;
  ros::Subscriber battery_sub_;
  ros::Subscriber sys_sub_;
  ros::Subscriber ekf2status_sub_;
  ros::Subscriber opflow_sub_;
  ros::Subscriber local_odom_sub_;
  ros::Subscriber local_vel_sub_;
  ros::Subscriber body_att_sub_;
  ros::Subscriber body_vel_sub_;
	// data
	px4_common::State mavros_state_;
	px4_common::ExtendedState mavros_ex_state_;
	px4_common:: BatteryState mavros_battery_;
	px4_common::SysStatus mavros_system_status;
	px4_common::EstimatorStatus mavros_ekf2status; 
  // callback fusion
  void stateCallback(const mavros_msgs::State::ConstPtr &msg);
  void exstateCallback(const mavros_msgs::ExtendedState::ConstPtr &msg);
  void batteryCallback(const sensor_msgs::BatteryState::ConstPtr &msg);
  void sysCallback(const mavros_msgs::SysStatus::ConstPtr &msg);
  void ekf2statusCallback(const mavros_msgs::EstimatorStatus::ConstPtr &msg);
  void opflowCallback(const mavros_msgs::OpticalFlowRad::ConstPtr &msg);
  void localOdomCallback(const nav_msgs::Odometry::ConstPtr &msg);
  void localVelCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);
  void bodyAttCallback(const sensor_msgs::Imu::ConstPtr &msg);
  void bodyVelCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);
};