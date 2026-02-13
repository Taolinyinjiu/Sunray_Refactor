/**
 * @file px4_reader.h
 * @brief 快速的的读取px4飞控的各种数据
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

#include "mavros_eigen_conversions.h"
#include "reader_types.h"
#include "ros/node_handle.h"

struct reader_list_ {
  bool read_system_state;
  bool read_ekf2_state;
  bool read_flow_state;
  bool read_localpose;
  bool read_localvel;
  bool read_bodypose;
  bool read_bodyvel;
  bool read_ekf2_param;
  bool read_attitude_param;
  bool read_velocity_param;
  bool read_position_param;

  void enable_all() {
    // std::fill 方法要求结构体全部由bool类型组成
    std::fill(&read_system_state, &read_position_param + 1, true);
  }
  void disable_all() {
    std::fill(&read_system_state, &read_position_param + 1, false);
  }
};

class PX4_Reader {
 public:
  PX4_Reader();
  ~PX4_Reader();
  // 手动初始化，对于enable_list应该是一个可选的参数,当不带该参数时，默认全订阅
  void init(ros::NodeHandle& nh);
  void init(ros::NodeHandle& nh, reader_list_ enable_list_);

	// get前缀，立即返回
  reader_types::system_state_ get_system_state(void);
  reader_types::ekf2_state_ get_ekf2_state(void);
  reader_types::flow_state_ get_flow_state(void);
  reader_types::pose_ get_local_pose(void);
  reader_types::velocity_ get_local_velocity(void);
  reader_types::pose_ get_body_pose(void);
  reader_types::velocity_ get_body_velocity(void);
  // fetch前缀，实时调用mavros服务，阻塞返回
	reader_types::ekf2_param_ fetch_ekf2_param(void);
  reader_types::attitude_param_ fetch_attitude_param(void);
  reader_types::attitude_param_ fetch_velocity_param(void);
  reader_types::attitude_param_ fetch_position_param(void);

 	static reader_types::FlightMode flightmode_fromString(const std::string& mode); 
 private:
	// 互斥锁 (使用 mutable 允许在 const 函数中使用)
	mutable std::mutex system_state_mtx;
	// 是否成功读取到无人机id
	bool read_uavid_flag;
  // 系统基本状态
  reader_types::system_state_ system_state;
  // ekf2估计状态
  reader_types::ekf2_state_ ekf2_state;
  // 光流数据
  reader_types::flow_state_ flow_state;
  // 惯性系位置与姿态
  reader_types::pose_ local_pose;
  // 惯性系速度
  reader_types::velocity_ local_velocity;
  // 机体系姿态
  reader_types::pose_ body_pose;
  // 机体系速度
  reader_types::velocity_ body_velocity;
  // ekf2相关参数
  reader_types::ekf2_param_ ekf2_param;
  // 姿态控制器相关参数
  reader_types::attitude_param_ attitude_param;
  // 速度控制器相关参数
  reader_types::velocity_param_ velocity_param;
  // 位置控制器相关参数
  reader_types::position_param_ position_param;
  // 订阅者
  ros::Subscriber state_sub_;
  ros::Subscriber exstate_sub_;
  ros::Subscriber sys_sub_;
  ros::Subscriber ekf2status_sub_;
  ros::Subscriber opflow_sub_;
  ros::Subscriber local_odom_sub_;
  ros::Subscriber local_vel_sub_;
  ros::Subscriber body_att_sub_;
  ros::Subscriber body_vel_sub_;
  // 回调函数
  void stateCallback(const mavros_msgs::State::ConstPtr& msg);
  void exstateCallback(const mavros_msgs::ExtendedState::ConstPtr& msg);
  void sysCallback(const mavros_msgs::SysStatus::ConstPtr& msg);
  void ekf2statusCallback(const mavros_msgs::EstimatorStatus::ConstPtr& msg);
  void opflowCallback(const mavros_msgs::OpticalFlowRad::ConstPtr& msg);
  void localOdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void localVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
  void bodyAttCallback(const sensor_msgs::Imu::ConstPtr& msg);
  void bodyVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
	// 服务端
};
