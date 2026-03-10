#pragma once

#include <Eigen/Dense>

namespace uav_control {
namespace curve {

/**
 * @file quintic_curve.hpp
 * @brief 五次多项式曲线通用求解接口。
 *
 * @details
 * 本文件提供基于边界条件的五次曲线评估函数。由于 API 仅输入位置与速度，
 * 默认采用 `a(0)=0`、`a(T)=0` 作为额外约束，从而得到唯一解。
 */

/**
 * @brief 五次曲线在某一时刻的评估结果。
 *
 * @note
 * - `elapsed_s`/`normalized_time` 基于区间裁剪后的时间计算；
 * - 当 `current_time_s` 超出 `[start_time_s, start_time_s + duration_s]` 时，
 *   `clamped=true`，并输出区间端点状态。
 */
struct QuinticCurveState {
  /** @brief 结果是否有效（输入合法且输出为有限值）。 */
  bool valid{false};
  /** @brief 是否发生时间裁剪（当前时刻超出规划区间）。 */
  bool clamped{false};
  /** @brief 从起始时刻开始的裁剪后历时，单位秒。 */
  double elapsed_s{0.0};
  /** @brief 归一化时间，范围 `[0, 1]`。 */
  double normalized_time{0.0};
  /** @brief 位置，单位米。 */
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  /** @brief 速度，单位米每秒。 */
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  /** @brief 加速度，单位米每二次方秒。 */
  Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
  /** @brief 加加速度（jerk），单位米每三次方秒。 */
  Eigen::Vector3d jerk{Eigen::Vector3d::Zero()};
  /** @brief 四阶导（snap），单位米每四次方秒。 */
  Eigen::Vector3d snap{Eigen::Vector3d::Zero()};
};

/**
 * @brief 评估五次曲线在当前时刻的状态。
 *
 * @details
 * 曲线模型为：
 * `p(t) = c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4 + c5*t^5`，
 * 系数由以下边界条件确定：
 * - `p(0) = start_position`
 * - `v(0) = start_velocity`
 * - `a(0) = 0`
 * - `p(T) = end_position`
 * - `v(T) = end_velocity`
 * - `a(T) = 0`
 *
 * @param start_position 起点位置（m）。
 * @param start_velocity 起点速度（m/s）。
 * @param end_position 终点位置（m）。
 * @param end_velocity 终点速度（m/s）。
 * @param start_time_s 轨迹起始时刻（秒，统一时间基准）。
 * @param duration_s 轨迹持续时间（秒，必须大于 0）。
 * @param current_time_s 当前评估时刻（秒，统一时间基准）。
 * @return QuinticCurveState 当前时刻的曲线状态。
 *
 * @note
 * 当输入时间非法或向量存在非有限值时，返回 `valid=false`。
 */
QuinticCurveState evaluate_quintic_curve(
    const Eigen::Vector3d &start_position,
    const Eigen::Vector3d &start_velocity,
    const Eigen::Vector3d &end_position,
    const Eigen::Vector3d &end_velocity, double start_time_s,
    double duration_s, double current_time_s);

} // namespace curve
} // namespace uav_control
