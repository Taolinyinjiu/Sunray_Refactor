/**
 * @file px4_data_reader.h
 * @brief 快速的的读取px4飞控的各种数据
 *
 * @details
 * px4_data_reader.h
 * 该文件设计的初衷为简化用户的操作，通过px4_data_reader可以快速的读取所需要的飞控数据，而不必关心底层实现
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
#include "mavros_msgs/ParamGet.h"
#include "px4_data_types.h"
#include "ros/node_handle.h"
#include "utils/opflow_ringbuffer.hpp"

#include <functional>
#include <mutex>
#include <string>

// Px4DataReader 的订阅开关列表：默认全部关闭，由用户按需开启。
struct ReaderOptions {
  bool read_system_state;
  bool read_ekf2_state;
  bool read_flow_state;
  bool read_local_pose;
  bool read_local_velocity;
  bool read_body_pose;
  bool read_body_velocity;
  bool read_ekf2_params;
  bool read_attitude_params;
  bool read_velocity_params;
  bool read_position_params;
  // 默认构造函数
  ReaderOptions() { disable_all(); };

  void enable_all();
  void disable_all();
};

class Px4DataReader {
public:
  // 默认构造：全量订阅
  Px4DataReader(ros::NodeHandle &nh);
  // 按开关列表进行选择性订阅
  Px4DataReader(ros::NodeHandle &nh, ReaderOptions options);

  // 初始化订阅者映射表并注册对应 topic
  void init_subscribers(ros::NodeHandle &nh, ReaderOptions options);

  ~Px4DataReader();

  // 立即返回缓存状态
  px4_data::SystemState get_system_state(void);
  px4_data::Ekf2State get_ekf2_state(void);
  // 返回环形缓冲区融合后的光流状态（不是原始单帧）
  px4_data::OpticalFlowState get_flow_state(void);
  px4_data::Pose get_local_pose(void);
  px4_data::Velocity get_local_velocity(void);
  px4_data::Pose get_body_pose(void);
  px4_data::Velocity get_body_velocity(void);

  // 实时调用 mavros 参数服务（阻塞直到返回）
  px4_data::Ekf2Params fetch_ekf2_params(void);
  px4_data::AttitudeParams fetch_attitude_params(void);
  px4_data::VelocityParams fetch_velocity_params(void);
  px4_data::PositionParams fetch_position_params(void);

  static px4_data::FlightMode flight_mode_from_string(const std::string &mode);

private:
  // --- 内部类型 ---
  // 订阅表项：根据开关动态初始化对应订阅者
  struct SubscribeEntry {
    bool ReaderOptions::* flag;             // 开关位
    ros::Subscriber Px4DataReader::* handle;  // 订阅者成员句柄
    std::function<ros::Subscriber()> make; // 订阅构造器
  };

  // --- 初始化辅助函数 ---
  // 初始化所有状态默认值，避免首次回调前读取未定义数据
  void reset_state_defaults();
  void init_service_clients();
	// --- mavros查询参数接口函数 ---
  bool fetch_param_int(const std::string& param_name, int& out_value);
  bool fetch_param_float(const std::string& param_name, float& out_value);

  // --- 回调函数 ---
  void state_callback(const mavros_msgs::State::ConstPtr &msg);
  void extended_state_callback(const mavros_msgs::ExtendedState::ConstPtr &msg);
  void system_status_callback(const mavros_msgs::SysStatus::ConstPtr &msg);
  void ekf2_status_callback(const mavros_msgs::EstimatorStatus::ConstPtr &msg);
  void optical_flow_callback(const mavros_msgs::OpticalFlowRad::ConstPtr &msg);
  void local_odometry_callback(const nav_msgs::Odometry::ConstPtr &msg);
  void local_velocity_callback(const geometry_msgs::TwistStamped::ConstPtr &msg);
  void body_attitude_callback(const sensor_msgs::Imu::ConstPtr &msg);
  void body_velocity_callback(const geometry_msgs::TwistStamped::ConstPtr &msg);

  // --- ROS 接口句柄 ---
  ros::NodeHandle nh_;
  ros::ServiceClient param_get_client_;

  // 订阅者句柄
  ros::Subscriber state_sub_;
  ros::Subscriber exstate_sub_;
  ros::Subscriber sys_sub_;
  ros::Subscriber ekf2status_sub_;
  ros::Subscriber opflow_sub_;
  ros::Subscriber local_odom_sub_;
  ros::Subscriber local_vel_sub_;
  ros::Subscriber body_att_sub_;
  ros::Subscriber body_vel_sub_;

  // --- 并发访问保护 ---
  mutable std::mutex system_state_mutex_;
  mutable std::mutex ekf2_state_mutex_;
  mutable std::mutex opflow_mutex_;
  mutable std::mutex local_pose_mutex_;
  mutable std::mutex local_velocity_mutex_;
  mutable std::mutex body_pose_mutex_;
  mutable std::mutex body_velocity_mutex_;

  // --- 识别信息 ---
  int uav_id_;
  std::string uav_namespace_;

  // --- 状态缓存 ---
  px4_data::SystemState system_state_cache_;
  px4_data::Ekf2State ekf2_state_cache_;
  px4_data::OpticalFlowRaw latest_opflow_raw_; // 原始光流（保留最近一帧）
  Opflow_Buffer opflow_buffer_{32}; // 光流环形缓冲区（滤波/窗口统计）
  px4_data::Pose local_pose_cache_;
  px4_data::Velocity local_velocity_cache_;
  px4_data::Odometry local_odometry_cache_;
  px4_data::Pose body_pose_cache_;
  px4_data::Velocity body_velocity_cache_;
  px4_data::Ekf2Params ekf2_params_cache_;
  px4_data::AttitudeParams attitude_params_cache_;
  px4_data::VelocityParams velocity_params_cache_;
  px4_data::PositionParams position_params_cache_;
};
