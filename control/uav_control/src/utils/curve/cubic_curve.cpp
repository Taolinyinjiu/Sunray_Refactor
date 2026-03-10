#include "utils/curve/cubic_curve.hpp"

#include <algorithm>
#include <cmath>

namespace uav_control {
namespace curve {
namespace {

bool is_finite_scalar(double value) { return std::isfinite(value); }

bool is_finite_vector(const Eigen::Vector3d &value) { return value.allFinite(); }

} // namespace

CubicCurveState evaluate_cubic_curve(
    const Eigen::Vector3d &start_position,
    const Eigen::Vector3d &start_velocity,
    const Eigen::Vector3d &end_position, const Eigen::Vector3d &end_velocity,
    double start_time_s, double duration_s, double current_time_s) {
  CubicCurveState output;
  if (!is_finite_scalar(start_time_s) || !is_finite_scalar(duration_s) ||
      !is_finite_scalar(current_time_s) || duration_s <= 0.0 ||
      !is_finite_vector(start_position) || !is_finite_vector(start_velocity) ||
      !is_finite_vector(end_position) || !is_finite_vector(end_velocity)) {
    return output;
  }

  const double elapsed_s = current_time_s - start_time_s;
  const double clamped_elapsed_s =
      std::max(0.0, std::min(duration_s, elapsed_s));
  const double t = clamped_elapsed_s;
  const double t2 = t * t;
  const double t3 = t2 * t;

  const double T = duration_s;
  const double T2 = T * T;
  const double T3 = T2 * T;

  const Eigen::Vector3d a0 = start_position;
  const Eigen::Vector3d a1 = start_velocity;
  const Eigen::Vector3d delta_p = end_position - start_position;
  const Eigen::Vector3d a2 =
      (3.0 * delta_p - (2.0 * start_velocity + end_velocity) * T) / T2;
  const Eigen::Vector3d a3 =
      (-2.0 * delta_p + (start_velocity + end_velocity) * T) / T3;

  output.position = a0 + a1 * t + a2 * t2 + a3 * t3;
  output.velocity = a1 + 2.0 * a2 * t + 3.0 * a3 * t2;
  output.acceleration = 2.0 * a2 + 6.0 * a3 * t;
  output.jerk = 6.0 * a3;
  output.elapsed_s = clamped_elapsed_s;
  output.normalized_time = clamped_elapsed_s / duration_s;
  output.clamped = (std::abs(clamped_elapsed_s - elapsed_s) > 1e-9);
  output.valid =
      is_finite_vector(output.position) && is_finite_vector(output.velocity) &&
      is_finite_vector(output.acceleration) && is_finite_vector(output.jerk);
  return output;
}

} // namespace curve
} // namespace uav_control

