#pragma once

#include "control_data_types/Curve_data_types.h"
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

namespace uav_control {

inline Curve_Output get_quintic_curve(const Eigen::Vector3d &start_point,
                                      const Eigen::Vector3d &end_point,
                                      double start_time, double keep_time,
                                      double current_time) {
  Curve_Output temp_output;
  temp_output.position = start_point;
  temp_output.velocity.setZero();
  temp_output.acceleration.setZero();
  // 如果运动持续时间小于等于零，或者当前时间小于零，说明传入参数有问题，拒绝生成
  if (keep_time <= 0.0 || keep_time < 0) {
    temp_output.curve_status = false;
    temp_output.position = start_point;
    return temp_output;
  }
	// 使用当前时间减去起始时间，再除以持续时间，计算进度
  const double progress_raw = (current_time - start_time) / keep_time;
  const double progress = std::max(0.0, std::min(1.0, progress_raw));

  const double p2 = progress * progress;
  const double p3 = p2 * progress;
  const double p4 = p3 * progress;
  const double p5 = p4 * progress;

  // Quintic smoothstep with zero velocity/acceleration at start/end.
  const double blend = 6.0 * p5 - 15.0 * p4 + 10.0 * p3;
  const double d_blend = 30.0 * p4 - 60.0 * p3 + 30.0 * p2;
  const double dd_blend = 120.0 * p3 - 180.0 * p2 + 60.0 * progress;

  const Eigen::Vector3d delta = end_point - start_point;
  temp_output.position = start_point + blend * delta;

  const double inv_keep_time = 1.0 / keep_time;
  const double inv_keep_time2 = inv_keep_time * inv_keep_time;
  temp_output.velocity = d_blend * inv_keep_time * delta;
  temp_output.acceleration = dd_blend * inv_keep_time2 * delta;

  return temp_output;
}

/*** @brief QUintic_Curve 构造五次多项式曲线，输出平滑的位置速度加速度
使用方式：
*/
class Quintic_Curve {
public:
  // 为起飞降落阶段设计的构造函数，通过对z轴数据的判断来决定后续的更新
  Quintic_Curve() = default;
  ~Quintic_Curve() {}

  // 设置起始点
  void set_start_position(Eigen::Vector3d point_position) {
    start_position_ = point_position;
  };
  // 设置结束点
  void set_end_position(Eigen::Vector3d point_position) {
    end_position_ = point_position;
  };
	// 设置运动持续时间(根据时间来计算速度)
	void set_keep_time(double start_time);
  // 通过传入当前z轴高度的方式生成曲线
  bool generate_by_current_time(double current_time);
  // 得到生成的位置
  Eigen::Vector3d get_position() { return position_; };
  // 得到生成的速度
  Eigen::Vector3d get_velocity() { return velocity_; };
  // 得到生成的加速度
  Eigen::Vector3d get_acceleration() { return acceleration_; };

private:
	double start_time_;
	double current_time_;
  Eigen::Vector3d start_position_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d end_position_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d position_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d acceleration_ = Eigen::Vector3d::Zero();
  double start_time = 0.0; // 轨迹进度
};

} // namespace uav_control
