/***
	@brief quintic_curve 五次项曲线
	输出连续的 位置 速度 加速度
*/

#pragma once

#include "control_data_types/curve_data_types.h"
#include "ros/time.h"
#include <Eigen/Dense>

namespace uav_control {

Curve_Output get_quintic_curve(const Eigen::Vector3d &start_point,
                               const Eigen::Vector3d &end_point,
                               double start_time, double keep_time,
                               double current_time);

/*** @brief QUintic_Curve 构造五次多项式曲线，输出平滑的位置速度加速度
使用方式：
*/
class Quintic_Curve {
public:
  // 为起飞降落阶段设计的构造函数，通过对z轴数据的判断来决定后续的更新
  Quintic_Curve() = default;
  ~Quintic_Curve() = default;

  // 设置起始点
  void set_start_position(Eigen::Vector3d point_position);
  // 设置结束点
  void set_end_position(Eigen::Vector3d point_position);
  // 设置运动持续时间(根据时间来计算速度)
  void set_keep_time(double keep_time);
  // 设置开始运动的时间
  bool set_start_time(ros::Time start_time);
  // 清除时间参数
  void clear_time();
  // 清除位置参数
  void clear_position();
	// 清除所有参数
  void clear_all();
  // 通过传入当前时间戳，生成当前时间对应的运动参数
  bool generate_by_current_time(ros::Time current_time);
  // 测试接口：使用重参数化，让速度峰值接近 0.75T
  bool generate_land_curve(ros::Time current_time);
	// 得到开始运动时间
	ros::Time get_start_time();
	// 得到持续运动的时间
	double get_keep_time();
	// 得到起点位置
	Eigen::Vector3d get_start_position();
	// 得到终点位置
	Eigen::Vector3d get_end_position();
  // 得到生成的位置
  Eigen::Vector3d get_position();
  // 得到生成的速度
  Eigen::Vector3d get_velocity();
  // 得到生成的加速度
  Eigen::Vector3d get_acceleration();

private:
  double keep_time_ = 0.0;
  double start_time_ = 0.0;
  ros::Time log_start_time_ = ros::Time(0);
	Eigen::Vector3d start_position_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d end_position_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d position_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d acceleration_ = Eigen::Vector3d::Zero();
};

} // namespace uav_control
