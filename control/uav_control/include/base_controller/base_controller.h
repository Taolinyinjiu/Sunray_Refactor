#pragma once

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include "utils/uav_state_estimate.hpp"

namespace uav_controller {

/**
 * @class Base_Controller
 * @brief 无人机控制器抽象基类，定义了起飞、降落及核心更新接口。
 * @note 所有子类控制器必须实现参数加载逻辑，确保从私有命名空间读取配置。
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

    /** @brief 执行默认起飞逻辑，使用配置文件中的起飞参数 */
    virtual bool takeoff(void) = 0;

    /** @brief 执行默认降落逻辑，使用配置文件中的降落参数 */
    virtual bool land(void) = 0;

    /** @brief 紧急降落接口，应具有最高执行优先级 */
    virtual bool emergency_land(void) = 0;

    /** @brief 控制律核心更新循环，由 FSM 定时调用 */
    virtual void update(void) = 0;

    virtual void set_currentstate(const nav_msgs::Odometry& current_state_msg);
    virtual void set_emergencystate(const nav_msgs::Odometry& emergency_state_msg);
    virtual void set_desiredstate(const nav_msgs::Odometry& desired_state_msg);

  protected:             // 修改为 protected，方便子类状态检查
    bool has_loadparam;  ///< 初始化状态位，执行 takeoff 前需检查
    bool is_emergency;   ///< 紧急状态标志，使能时强制进入 emergency_land
    uav_common::UAVStateEstimate current_state;
    uav_common::UAVStateEstimate emergency_state;
    uav_common::UAVStateEstimate desired_state;
};

}  // namespace uav_controller
