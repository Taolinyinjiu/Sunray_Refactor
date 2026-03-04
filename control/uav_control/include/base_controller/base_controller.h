#pragma once

#include <cstdint>
#include <vector>
#include <string>

#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include "utils/uav_state_estimate.hpp"

namespace uav_controller {

/**
 * @brief 控制输出掩码位定义。
 */
enum class ControlOutputMask : uint32_t {
    UNDEFINED = 0U,
    POSITION = 1U << 0,     ///< position 字段有效
    VELOCITY = 1U << 1,     ///< velocity 字段有效
    ATTITUDE = 1U << 2,     ///< attitude 字段有效
    THRUST = 1U << 3,       ///< thrust 字段有效
};

/**
 * @brief 控制器标准输出。
 * @note 各字段是否有效由 output_mask 指示，FSM/执行器应按掩码消费数据。
 */
struct ControlOutput {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ControlOutput()
        : position(Eigen::Vector3d::Zero()),
          velocity(Eigen::Vector3d::Zero()),
          attitude(Eigen::Quaterniond::Identity()),
          thrust(0.0),
          output_mask(static_cast<uint32_t>(ControlOutputMask::UNDEFINED)) {}

    /**
     * @brief 使能某个输出子项。
     * @param item 需要使能的掩码位。
     */
    void enable(ControlOutputMask item) {
        output_mask |= static_cast<uint32_t>(item);
    }

    /**
     * @brief 关闭某个输出子项。
     * @param item 需要关闭的掩码位。
     */
    void disable(ControlOutputMask item) {
        output_mask &= ~static_cast<uint32_t>(item);
    }

    /**
     * @brief 判断某个输出子项是否已使能。
     * @param item 掩码位。
     * @return true 已使能；false 未使能。
     */
    bool is_enabled(ControlOutputMask item) const {
        return (output_mask & static_cast<uint32_t>(item)) != 0U;
    }

    /**
     * @brief 清空所有输出掩码位。
     */
    void clear_mask() {
        output_mask = static_cast<uint32_t>(ControlOutputMask::UNDEFINED);
    }

    /**
     * @brief 解码输出掩码为字段名列表。
     * @return 例如 {"POSITION", "VELOCITY"}；若空则返回 {"UNDEFINED"}。
     */
    std::vector<std::string> decode_output_mask() const {
        std::vector<std::string> decoded;
        if (is_enabled(ControlOutputMask::POSITION)) {
            decoded.emplace_back("POSITION");
        }
        if (is_enabled(ControlOutputMask::VELOCITY)) {
            decoded.emplace_back("VELOCITY");
        }
        if (is_enabled(ControlOutputMask::ATTITUDE)) {
            decoded.emplace_back("ATTITUDE");
        }
        if (is_enabled(ControlOutputMask::THRUST)) {
            decoded.emplace_back("THRUST");
        }
        if (decoded.empty()) {
            decoded.emplace_back("UNDEFINED");
        }
        return decoded;
    }

    /**
     * @brief 解码输出掩码为字符串。
     * @param delim 项之间的分隔符。
     * @return 例如 "POSITION|VELOCITY"。
     */
    std::string decode_output_mask_to_string(const std::string& delim = "|") const {
        const std::vector<std::string> parts = decode_output_mask();
        std::string result;
        for (size_t i = 0; i < parts.size(); ++i) {
            result += parts[i];
            if (i + 1 < parts.size()) {
                result += delim;
            }
        }
        return result;
    }

    Eigen::Vector3d position;      ///< 期望位置（世界系）
    Eigen::Vector3d velocity;      ///< 期望速度（世界系）
    Eigen::Quaterniond attitude;   ///< 期望姿态
    double thrust;                 ///< 期望总推力（归一化或物理量由下游约定）
    uint32_t output_mask;          ///< 输出掩码，组合 ControlOutputMask
};

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

    /** @brief 确认已成功起飞（由子类给出判据） */
    virtual bool ensure_takeoff_completed() const;

    /** @brief 确认已成功降落（由子类给出判据） */
    virtual bool ensure_land_completed() const;

    /** @brief 确认已成功紧急降落（由子类给出判据） */
    virtual bool ensure_emergency_land_completed() const;
	
    /**
     * @brief 控制律核心更新循环，由 FSM 定时调用。
     * @return 控制输出（位置+速度+姿态+推力+输出掩码）。
     */
    virtual ControlOutput update(void) = 0;

    virtual void set_currentstate(const nav_msgs::Odometry& current_state_msg);
    virtual void set_emergencystate(const nav_msgs::Odometry& emergency_state_msg);
    virtual void set_desiredstate(const nav_msgs::Odometry& desired_state_msg);
    const uav_common::UAVStateEstimate& get_current_state() const { return current_state; }
    const uav_common::UAVStateEstimate& get_emergency_state() const { return emergency_state; }
    const uav_common::UAVStateEstimate& get_desired_state() const { return desired_state; }

  protected:             // 修改为 protected，方便子类状态检查
    bool has_loadparam;  ///< 初始化状态位，执行 takeoff 前需检查
    bool is_emergency;   ///< 紧急状态标志，使能时强制进入 emergency_land
    uav_common::UAVStateEstimate current_state;
    uav_common::UAVStateEstimate emergency_state;
    uav_common::UAVStateEstimate desired_state;
};

}  // namespace uav_controller
