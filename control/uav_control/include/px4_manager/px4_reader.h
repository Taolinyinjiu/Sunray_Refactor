/**
 * @file px4_reader.h
 * @brief 快速的的读取px4飞控的各种数据
 *
 * @details
 * px4_reader.h
 * 该文件设计的初衷为简化用户的操作，通过px4_reader可以快速的读取所需要的飞控数据，而不必关心底层实现
 * @author taolinyinjiu
 * @date 2026-02-28
 * @version 0.1
 *
 * @see https://docs.ros.org/noetic/api/mavros/html/
 * @see https://eigen.tuxfamily.org/dox-3.4/
 * @see
 * https://yundrone.feishu.cn/wiki/RKMSw79HbigKLOkDo4Rc8WWtn9g?from=from_copylink
 */

#pragma once

#include "mavros_eigen_conversions.h"
// #include "px4_data.h"
#include "px4_datatypes.h"
#include "utils/opflow_ringbuffer.hpp"
#include "ros/node_handle.h"
#include "mavros_msgs/ParamGet.h"
#include <algorithm>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>

// 设计一个reader_list_结构体，变量使用bool类型，用于表示PX4_Reader实例需要读取的数据类型，默认全部不订阅，要求用户按需求进行订阅
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
  // 默认构造函数
  reader_list_() { disable_all(); };

  void enable_all() {
    read_system_state = true;
    read_ekf2_state = true;
    read_flow_state = true;
    read_localpose = true;
    read_localvel = true;
    read_bodypose = true;
    read_bodyvel = true;
    read_ekf2_param = true;
    read_attitude_param = true;
    read_velocity_param = true;
    read_position_param = true;
  }
  void disable_all() {
    read_system_state = false;
    read_ekf2_state = false;
    read_flow_state = false;
    read_localpose = false;
    read_localvel = false;
    read_bodypose = false;
    read_bodyvel = false;
    read_ekf2_param = false;
    read_attitude_param = false;
    read_velocity_param = false;
    read_position_param = false;
  }
};

class PX4_Reader {
public:
  // 构造函数，当不带有reader_list_结构体参数时，默认全订阅
  PX4_Reader(ros::NodeHandle &nh);
  PX4_Reader(ros::NodeHandle &nh, reader_list_ enable_list_);
  // 实现一个初始化订阅者的函数，从而提高后期的可维护性
  void initSubscribers(ros::NodeHandle &nh, reader_list_ enable_list_);
  // 析构函数
  ~PX4_Reader();

  // get前缀，立即返回
  px4_data::system_state_ get_system_state(void);
  px4_data::ekf2_state_ get_ekf2_state(void);
  px4_data::opflow_state_ get_flow_state(void);     // 该函数得到的是环形缓冲区输出的光流数据而非光流原始的数据
  px4_data::pose_ get_local_pose(void);
  px4_data::velocity_ get_local_velocity(void);
  px4_data::pose_ get_body_pose(void);
  px4_data::velocity_ get_body_velocity(void);
  // fetch前缀，实时调用mavros服务，得到参数后才返回，需要等待，也就是阻塞
  px4_data::ekf2_param_ fetch_ekf2_param(void);
  px4_data::attitude_param_ fetch_attitude_param(void);
  px4_data::velocity_param_ fetch_velocity_param(void);
  px4_data::position_param_ fetch_position_param(void);

  static px4_data::FlightMode flightmode_fromString(const std::string &mode);

private:
  // 初始化所有状态的默认值，避免首次回调前读取到未定义数据
  void resetStateDefaults();
  // 系统状态互斥锁 (使用 mutable 允许在 const 函数中使用)
  mutable std::mutex system_state_mtx;
  // ekf2状态互斥锁
  mutable std::mutex ekf2_state_mtx;
  // 光流原始数据互斥锁
  mutable std::mutex opflow_mtx;
  // local系位姿互斥锁
  mutable std::mutex local_pose_mtx;
  // local系速度互斥锁
  mutable std::mutex local_velocity_mtx;
  // body系姿态互斥锁
  mutable std::mutex body_pose_mtx;
  // body系速度互斥锁
  mutable std::mutex body_velocity_mtx;
  // 是否成功读取到无人机id
  int uav_id;
  std::string uav_name;
  // 系统基本状态
  px4_data::system_state_ system_state;
  // ekf2估计状态
  px4_data::ekf2_state_ ekf2_state;
  // 光流数据(原始，保留最近一帧用于调试或回溯)
  px4_data::opflow_raw_ opflow_raw;
  // 光流环形缓冲区（用于滤波与窗口统计）
  Opflow_Buffer opflow_buffer_{32};
  // 惯性系位置与姿态
  px4_data::pose_ local_pose;
  // 惯性系速度
  px4_data::velocity_ local_velocity;
	// 惯性系下里程计
	px4_data::odom_ local_odom;
  // 机体系姿态
  px4_data::pose_ body_pose;
  // 机体系速度
  px4_data::velocity_ body_velocity;
  // ekf2相关参数
  px4_data::ekf2_param_ ekf2_param;
  // 姿态控制器相关参数
  px4_data::attitude_param_ attitude_param;
  // 速度控制器相关参数
  px4_data::velocity_param_ velocity_param;
  // 位置控制器相关参数
  px4_data::position_param_ position_param;
  // 定义表项结构
  struct SubscribeEntry {
    bool reader_list_::* flag;              // 指向开关的成员指针
    ros::Subscriber PX4_Reader::* handle;   // 指向句柄的成员指针
    std::function<ros::Subscriber()> make; // 订阅执行器
  };
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
  void stateCallback(const mavros_msgs::State::ConstPtr &msg);
  void exstateCallback(const mavros_msgs::ExtendedState::ConstPtr &msg);
  void sysCallback(const mavros_msgs::SysStatus::ConstPtr &msg);
  void ekf2statusCallback(const mavros_msgs::EstimatorStatus::ConstPtr &msg);
  void opflowCallback(const mavros_msgs::OpticalFlowRad::ConstPtr &msg);
  void localOdomCallback(const nav_msgs::Odometry::ConstPtr &msg);
  void localVelCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);
  void bodyAttCallback(const sensor_msgs::Imu::ConstPtr &msg);
  void bodyVelCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);
  // 服务端
  ros::NodeHandle nh_;
  ros::ServiceClient param_get_client_;

  void initServiceClients();
  bool fetchParamInt(const std::string& param_name, int& out_value);
  bool fetchParamFloat(const std::string& param_name, float& out_value);
};
