#pragma once

#include <cstdint>
#include <vector>
#include <string>

#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include "control_data_types/uav_state_estimate.hpp"
#include "control_data_types/control_data_types.h"

namespace uav_control {


/**
 * @class Base_Controller
 * @brief 无人机控制器抽象基类，定义了起飞、降落及核心运动接口。
 * @note 所有子类控制器必须实现参数加载逻辑，确保读取无人机配置参数。
 */
class Base_Controller {
  public:
    Base_Controller() : has_loadparam(false), is_emergency(false) {}
    virtual ~Base_Controller() {}  // 必须为虚析构

    /**
     * @brief 从 ROS 参数服务器加载配置
     * @return true 加载成功；false 加载失败，FSM 应拒绝切换至此控制器
     */
    virtual bool load_param(ros::NodeHandle& nh) = 0;

    /** @brief 设置切换到起飞模式 */
    virtual bool set_takeoff_mode(void) = 0;

    /** @brief 设置切换到着陆模式 */
    virtual bool set_land_mode(void) = 0;

    /** @brief 切换到紧急降落模式 */
    virtual bool set_emergency_mode(void) = 0;

    /** @brief 确认已成功起飞 */
    virtual bool ensure_takeoff_completed() const;

    /** @brief 确认已成功降落 */
    virtual bool ensure_land_completed() const;

    /** @brief 确认已成功紧急降落*/
    virtual bool ensure_emergency_land_completed() const;
	
    /**
     * @brief 控制律核心更新循环，由 FSM 定时调用。
     * @return 控制输出（位置+速度+姿态+推力+输出掩码）。
     */
    virtual ControllerOutput update(void) = 0;

    virtual bool set_currentstate(const nav_msgs::Odometry& current_state_msg);
    virtual bool set_emergencystate(const nav_msgs::Odometry& emergency_state_msg);
    
		const UAVStateEstimate& get_current_state() const { return current_state_; }
    const UAVStateEstimate& get_emergency_state() const { return emergency_state_; }
		// 简化控制器开发，开发者只需要关心如何对轨迹进行控制就好了
		virtual bool set_trajectory(std::vector<TrajectoryPoint> tarjectory_);

  protected:             // 修改为 protected，方便子类状态检查
    bool has_loadparam;  ///< 初始化状态位，执行 takeoff 前需检查
    bool is_emergency;   ///< 紧急状态标志，使能时强制进入 emergency_land
		// 控制器内部状态机
		ControllerState controller_state_ = ControllerState::OFF;
		// 构造函数保证初始化时为0或者单位姿态
    UAVStateEstimate current_state_;
    UAVStateEstimate emergency_state_;
};

}  // namespace uav_controller
