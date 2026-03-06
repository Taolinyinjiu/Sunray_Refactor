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
    // 当我们谈到传入轨迹的时候，我们实际上在讨论什么？
    // 1. 对于ego_planner 在未达到目的点时，会一直发送PositonCommand
    // 2。 对于手动输入的轨迹来说，只会输入一次轨迹，并等待无人机达到期望的点位
    // 3. 然而，我们通常希望轨迹是可以被打断的，因此我们得到这样的一个基本的观点
    //    我们可以传入完整的轨迹，也可以传入一部分轨迹，对于轨迹来说，我们要求他是抢占性的，也就是新轨迹的优先级是大于旧轨迹的
    //    也就是说当我们传入一条新轨迹的时候，我们希望他能够立即生效，而不需要等到旧轨迹执行完毕
    //    这就要求我们在控制器内部需要有一个轨迹缓冲区，来存储当前的轨迹和待生效的轨迹
    //    这样我们就可以在控制器内部实现一个双缓冲的机制
    //    也就是说我们有一个active轨迹和一个pending轨迹，active轨迹是当前正在执行的轨迹，pending轨迹是待生效的轨迹
    //    当我们传入一条新轨迹的时候，我们将他放入pending轨迹中，当active轨迹执行完毕或者达到某个条件的时候，我们将pending轨迹切换到active轨迹中，这样就实现了轨迹的抢占性和实时性
    //    这样我们就可以保证我们的控制器能够及时响应新的轨迹输入，同时也能够保证轨迹的连续性和稳定性
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
