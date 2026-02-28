/**
 * @file px4_datatypes.h
 * @brief 提供Sunray项目 自定义的数据类型
 *
 * @details
 * px4_datatypes.h
 * 文件为Sunray项目中的PX4状态管理模块提供向外接口的自定义数据类型，设计的初衷在于简化用户的操作
 * 通过PX4状态管理模块中的Reader与Manager将底层与任务模块解藕，使用户可以专心上层结构设计，而不必关心飞控底层数据内容
 *
 * @author taolinyinjiu
 * @date 2026-02-27
 * @version 0.1
 *
 * @see
 * https://yundrone.feishu.cn/wiki/RKMSw79HbigKLOkDo4Rc8WWtn9g?from=from_copylink
 */

#pragma once
#include "mavros_msgs/OpticalFlowRad.h"
#include <Eigen/Dense>
#include <cstdint>
// 使用命名空间？or不使用
namespace px4_data {
// 控制模式结构体，使用class强类型枚举
enum class FlightMode : uint8_t {
  MANUAL = 0,   // 手动模式
  ACRO,         // 特技模式
  ALTCTL,       // 定高模式
  POSCTL,       // 定点模式
  OFFBOARD,     // 外部控制
  STABILIZED,   // 自稳模式
  RATTITUDE,    // 半特技模式
  AUTO_MISSION, // 任务模式
  AUTO_LOITER,  // (固定翼)悬停模式
  AUTO_RTL,     // 返航并降落
  AUTO_LAND,    // 降落
  AUTO_RTGS,    // (固定翼)返航至基地
  AUTO_READY,   // 准备就绪
  AUTO_TAKEOFF  // 起飞
};
// 着陆状态结构体，使用class强类型枚举
enum class LandedState : uint8_t {
  UNDEFINED = 0, // 未定义的状态
  ON_GROUND,     // 在地面
  IN_AIR,        // 在空中
  TAKEOFF,       // 起飞阶段
  LANDING        // 降落阶段
};
// 系统状态结构体
struct system_state_ {
  uint8_t uav_id;           // 无人机id
  bool connected;           // 无人机是否响应mavros心跳包 or px4是否连接成功
  bool armed;               // 无人机是否解锁
  bool rc_input;            // 无人机是否连接到遥控器
  float system_load;        // 无人机飞控cpu负载
  float voltage;            // 无人机电池电压
  float current;            // 无人机电池电流
  float percent;            // 无人机电池百分比
  FlightMode flight_mode;   // 控制模式
  LandedState landed_state; // 着陆检测状态
};
// ekf2估计器状态结构体
struct ekf2_state_ {
  uint32_t state_codes;
  bool allow_stabilize;
  bool allow_altitude;
  bool allow_position;
};

// 光流原始数据结构体
struct opflow_raw_ {
	// 光流消息时间戳
		double timestamp;
  // 光流数据质量
  uint8_t quality;
  // 光流两次采样之间的积分时间
  uint32_t integration_time_us;
  // 图像平面绕 X 轴和 Y 轴的累积像素位移，单位为弧度
  float integrated_x;
  float integrated_y;
  // 在 integration_time_us 时间内，传感器自带陀螺仪测得的旋转角度，单位为弧度
  float integrated_xgyro;
  float integrated_ygyro;
  float integrated_zgyro;
  // 距离传感器两次采样之间的时间间隔
  uint32_t time_delta_distance_us;
  // 传感器到地面的垂直距离
  float distance;
  // 构造函数
  opflow_raw_();
  opflow_raw_(const mavros_msgs::OpticalFlowRad &ConstPtr);
};

// 光流数据结构体
// TODO：修改为用户友好类型
struct opflow_state_ {

  uint32_t time_delta_distance_us;
  float distance;
  uint8_t quality;
  uint32_t integration_time_us;
  float integrated_x;
  float integrated_y;
  float integrated_xgyro;
  float integrated_ygyro;
  float integrated_zgyro;
};

// 位姿结构体
struct pose_ {
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
};
// 速度结构体
struct velocity_ {
  Eigen::Vector3d linear;
  Eigen::Vector3d angular;
};
// ekf2 参数结构体
struct ekf2_param_ {
  int ev_ctrl;
  int hgt_ref;
  float ev_delay;
};
// pid参数结构体
struct pid_param_ {
  float kp;
  float ki;
  float kd;
};
// 姿态环控制器PID参数
struct attitude_param_ {
  pid_param_ roll;
  pid_param_ pitch;
  pid_param_ yaw;
};
// 速度环控制器PID参数
struct velocity_param_ {
  pid_param_ mpc_xy;
  pid_param_ mpc_z;
};
// 位置环控制器PID参数
struct position_param_ {
  pid_param_ mpc_xy;
  pid_param_ mpc_z;
};
}; // namespace px4_data