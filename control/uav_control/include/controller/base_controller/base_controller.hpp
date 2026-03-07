#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "control_data_types/control_data_types.h"
#include "control_data_types/uav_state_estimate.hpp"
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

namespace uav_control {

/**
 * @class Base_Controller
 * @brief 无人机控制器抽象基类，定义了起飞、降落及核心运动接口。
 * @note 所有子类控制器必须实现参数加载逻辑，确保读取无人机配置参数。
 */
class Base_Controller {
public:
  Base_Controller() : has_loadparam(false), is_emergency(false) {}
  virtual ~Base_Controller() {} // 必须为虚析构

  /**
   * @brief 从 ROS 参数服务器加载配置
   * @return true 加载成功；false 加载失败，FSM 应拒绝切换至此控制器
   */
  virtual bool load_param(ros::NodeHandle &nh) = 0;

  /** @brief 设置切换到起飞模式 */
  virtual bool set_takeoff_mode(void);
  /** @brief 获取飞控的解锁状态 */
  virtual bool get_arm_state(bool arm_state_);
  /** @brief 设置切换到着陆模式 */
  virtual bool set_land_mode(void);

  /** @brief 切换到紧急降落模式 */
  virtual bool set_emergency_mode(void);

  /** @brief 向外反馈控制器当前状态 */
  virtual ControllerState get_controller_state();

  /** @brief 设置无人机当前里程计 */
  virtual bool set_current_odom(const nav_msgs::Odometry &current_state_msg);

  /** @brief 传入无人机当前姿态(此处从px4飞控拿到imu姿态数据) */
  virtual bool set_px4_attitude(const sensor_msgs::Imu &imu_msg);

  /** @brief 控制器的期望，设计为全状态的轨迹点 */
  virtual bool set_trajectory(const TrajectoryPoint &tarjectory_);
  // 当我们谈到传入轨迹的时候，我们实际上在讨论什么？

  /**
   * @brief 控制律核心更新循环，由 FSM 定时调用。
   * @return 控制输出（位置+速度+姿态+推力+输出掩码）。
   */
  virtual ControllerOutput update(void) = 0;

protected:            // 修改为 protected，方便子类状态检查
  bool has_loadparam; ///< 初始化状态位，执行 takeoff 前需检查
  bool is_emergency;  ///< 紧急状态标志，使能时强制进入 emergency_land
	
	bool arm_state_; ///< 无人机当前是否解锁，请注意，当切换到TAKEOFF模式而未解锁时，根据PX4的控制逻辑，控制器需要自行考虑如何根据控制器的特性设置输出
	/// < example px4位置控制器，在解锁前，takeoff阶段，输出的控制量更新为(home_x,home_y,takeoff_z,home_yaw)
	/// < example px4姿态控制器，在解锁前，takeoff阶段，输出的控制量更新为(姿态四元数)+（怠速推力）
  
	// 控制器内部状态机
  ControllerState controller_state_ = ControllerState::OFF;
  // 构造函数保证初始化时为0或者单位姿态
  UAVStateEstimate current_state_;
	Eigen::Quaterniond px4_attitude;
};

} // namespace uav_control


namespace uav_control {

// 设置起飞模式，仅允许在OFF模式下切换到TAKEOFF
inline bool Base_Controller::set_takeoff_mode(void){
	if(controller_state_ == ControllerState::OFF){
		controller_state_ = ControllerState::TAKEOFF;
		return true;
	}
	return false;
};

// 设置LAND模式，允许在除OFF以外的任何模式下切换到LAND模式
inline bool Base_Controller::set_land_mode(){
	if(controller_state_ == ControllerState::OFF)
		return false;
	controller_state_ = ControllerState::LAND;
	return true;
};
inline bool Base_Controller::set_emergency_mode(){
	controller_state_ = ControllerState::EMERGENCY_LAND;
	return true;
};

}