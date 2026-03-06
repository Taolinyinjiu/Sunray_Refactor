#pragma once
#include <Eigen/Dense>
#include <cstdint>
#include <ros/node_handle.h>
#include "control_data_types/uav_state_estimate.hpp"

// 控制输出掩码
class ControlMask {
public:
  // 强制类型枚举
  enum class Bit : uint32_t {
    POSITION = 1u << 0,
    VELOCITY = 1u << 1,
    ATTITUDE = 1u << 2,
    THRUST = 1u << 3,
    ACCELERATION = 1u << 4,
    TORQUE = 1u << 5,
  };
  // 默认构造函数
  ControlMask() : mask_(0u) {}
  // 使能不同的控制环节
  ControlMask &position(bool enable) {
    set(Bit::POSITION, enable);
    return *this;
  }
  ControlMask &velocity(bool enable) {
    set(Bit::VELOCITY, enable);
    return *this;
  }
  ControlMask &attitude(bool enable) {
    set(Bit::ATTITUDE, enable);
    return *this;
  }
  ControlMask &thrust(bool enable) {
    set(Bit::THRUST, enable);
    return *this;
  }

  // 互斥：开加速度时关力矩
  ControlMask &acceleration(bool enable) {
    set(Bit::ACCELERATION, enable);
    if (enable)
      set(Bit::TORQUE, false);
    return *this;
  }

  // 互斥：开力矩时关加速度
  ControlMask &torque(bool enable) {
    set(Bit::TORQUE, enable);
    if (enable)
      set(Bit::ACCELERATION, false);
    return *this;
  }
  // 查询函数，查询当前使能那些控制环节
  bool has_position() const { return has(Bit::POSITION); }
  bool has_velocity() const { return has(Bit::VELOCITY); }
  bool has_attitude() const { return has(Bit::ATTITUDE); }
  bool has_thrust() const { return has(Bit::THRUST); }
  bool has_acceleration() const { return has(Bit::ACCELERATION); }
  bool has_torque() const { return has(Bit::TORQUE); }
  // 输出掩码
  uint32_t value() const { return mask_; }
  // 清除掩码
  void clear() { mask_ = 0u; }

private:
  // 设置掩码
  void set(Bit b, bool enable) {
    const uint32_t v = static_cast<uint32_t>(b);
    if (enable)
      mask_ |= v;
    else
      mask_ &= ~v;
  }
  // 查询掩码
  bool has(Bit b) const { return (mask_ & static_cast<uint32_t>(b)) != 0u; }

  uint32_t mask_;
};

// 控制器输出变量
struct Control_Output {
  // EIGEN 对齐
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // 默认构造函数
  Control_Output() = default;

  // 首先是掩码，构造默认为不使能任何控制环
  ControlMask mask_;
  // -------setpoint_raw/local----------
  // 位置环控制
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  // 速度环控制
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  // 加速度控制
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
  // 力矩控制
  Eigen::Vector3d force = Eigen::Vector3d::Zero();
  // yaw角控制
  float yaw = 0.0f;
  float yaw_rate = 0.0f;
  // -------setpoint_raw/attitude----------
  // 四元数表示姿态
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  // 机体角速度(绕轴)
  Eigen::Vector3d body_rate = Eigen::Vector3d::Zero();
  // 归一化推力
  float thrust = 0.0f;
};

// 控制器基本模式，所有子类控制器应该兼容
enum class Base_Controller_Stage {
  GROUND = 0, ///< 在地面阶段
  ARM,        ///< 解锁阶段
  TAKEOFF,    ///< 起飞阶段
  HOVER,      ///< 悬停
  MOVE,       ///< 运动
  LANDING,    ///< 降落阶段
  EMERGENCY,  ///< 紧急处理阶段（优先级最高）
};



class Base_Controller {
	
public:
	Base_Controller(ros::NodeHandle &nh);
  virtual ~Base_Controller() = default;
	// virtual 前缀为派生类可重写函数
	// virtual function() = 0 为纯虚函数，子类必须自己重写
	// 加载参数
  virtual bool load_para() = 0;
	
	// 设置控制器的模式
	virtual void set_mode(Base_Controller_Stage stage_);

	// 设置当前里程计信息
	virtual void set_currentstate(const uav_common::UAVStateEstimate& uav_state_);
	// 设置期望的目标
	virtual void set_desiredstate(const uav_common::UAVStateEstimate& des_state_);
	// 控制器停止输出
	virtual void stop();

	// 检查是否解锁
	virtual bool has_arm() const;
	// 检查是否起飞成功
	virtual bool has_takeoff() const;

	virtual Control_Output update() = 0;

private:
  // 节点句柄
  ros::NodeHandle nh;
  // 控制器模式
  uint8_t controller_mode;
  // 控制器输出
  Control_Output controller_output_;
  // 里程计输入
	uav_common::UAVStateEstimate uav_state_;
	uav_common::UAVStateEstimate des_state_;
	// 解锁状态
	bool arm_state = false;
	// 起飞状态
	bool takeoff_state = false;
	// 降落状态 
	bool land_state = false;
};
