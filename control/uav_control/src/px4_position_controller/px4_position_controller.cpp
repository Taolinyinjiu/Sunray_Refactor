#include "controller/px4_position_controller/px4_position_controller.h"

#include <cmath>

namespace uav_controller {

bool PX4_Position_Controller::load_param(ros::NodeHandle &nh) {
  return load_param(nh, false);
}

bool PX4_Position_Controller::load_param(ros::NodeHandle &nh,
                                         bool push_pid_params_to_px4) {
  nh.param("takeoff_height_m", config_.takeoff_height_m, config_.takeoff_height_m);
  nh.param("takeoff_climb_mps", config_.takeoff_climb_mps, config_.takeoff_climb_mps);
  nh.param("landing_descent_mps", config_.landing_descent_mps,
           config_.landing_descent_mps);
  nh.param("emergency_descent_mps", config_.emergency_descent_mps,
           config_.emergency_descent_mps);
  nh.param("position_error_tolerance_x_m", config_.position_error_tolerance_x_m,
           config_.position_error_tolerance_x_m);
  nh.param("position_error_tolerance_y_m", config_.position_error_tolerance_y_m,
           config_.position_error_tolerance_y_m);
  nh.param("position_error_tolerance_z_m", config_.position_error_tolerance_z_m,
           config_.position_error_tolerance_z_m);
  nh.param("state_timeout_s", config_.state_timeout_s, config_.state_timeout_s);

  const bool valid =
      (config_.takeoff_height_m > 0.0F) && (config_.takeoff_climb_mps > 0.0F) &&
      (config_.landing_descent_mps > 0.0F) &&
      (config_.emergency_descent_mps > 0.0F) &&
      (config_.position_error_tolerance_x_m > 0.0F) &&
      (config_.position_error_tolerance_y_m > 0.0F) &&
      (config_.position_error_tolerance_z_m > 0.0F) &&
      (config_.state_timeout_s > 0.0F);

  if (!valid) {
    ROS_ERROR("[PX4PositionController] invalid params detected");
    has_loadparam = false;
    return false;
  }

  has_loadparam = true;
  if (stage_enter_time_.isZero()) {
    stage_enter_time_ = ros::Time::now();
  }

  if (push_pid_params_to_px4) {
    ROS_WARN("[PX4PositionController] push_pid_params_to_px4 requested, but PID "
             "push is not implemented yet");
  }

  ROS_INFO("[PX4PositionController] params loaded: takeoff_height=%.2f "
           "takeoff_climb=%.2f landing_descent=%.2f emergency_descent=%.2f "
           "timeout=%.2f",
           config_.takeoff_height_m, config_.takeoff_climb_mps,
           config_.landing_descent_mps, config_.emergency_descent_mps,
           config_.state_timeout_s);
  return true;
}

bool PX4_Position_Controller::set_takeoff_mode() {
  if (!has_loadparam) {
    ROS_WARN("[PX4PositionController] reject set_takeoff_mode: params not loaded");
    return false;
  }
  if (flight_stage_ == FlightStage::TAKEOFF) {
    return true;
  }
  is_emergency = false;
  flight_stage_ = FlightStage::TAKEOFF;
  stage_enter_time_ = ros::Time::now();
  takeoff_start_z_m_ = current_state.position.z();
  return true;
}

bool PX4_Position_Controller::set_land_mode() {
  if (!has_loadparam) {
    ROS_WARN("[PX4PositionController] reject set_land_mode: params not loaded");
    return false;
  }
  if (flight_stage_ == FlightStage::LANDING) {
    return true;
  }
  flight_stage_ = FlightStage::LANDING;
  stage_enter_time_ = ros::Time::now();
  return true;
}

bool PX4_Position_Controller::set_emergency_mode() {
  if (!has_loadparam) {
    ROS_WARN(
        "[PX4PositionController] reject set_emergency_mode: params not loaded");
    return false;
  }
  if (flight_stage_ == FlightStage::EMERGENCY) {
    return true;
  }
  is_emergency = true;
  flight_stage_ = FlightStage::EMERGENCY;
  stage_enter_time_ = ros::Time::now();
  return true;
}

ControlOutput PX4_Position_Controller::update() {
  if (is_emergency && flight_stage_ != FlightStage::EMERGENCY) {
    flight_stage_ = FlightStage::EMERGENCY;
    stage_enter_time_ = ros::Time::now();
  }

  if (!validate_state_inputs()) {
    ROS_WARN_THROTTLE(
        1.0,
        "[PX4PositionController] input state invalid/stale, fallback output "
        "applied");
    return (flight_stage_ == FlightStage::EMERGENCY) ? build_emergency_output()
                                                      : build_ground_output();
  }

  update_flight_stage_by_conditions();
  switch (flight_stage_) {
  case FlightStage::GROUND:
    return build_ground_output();
  case FlightStage::TAKEOFF:
    return build_takeoff_output();
  case FlightStage::AIR:
    return build_air_output();
  case FlightStage::LANDING:
    return build_landing_output();
  case FlightStage::EMERGENCY:
    return build_emergency_output();
  default:
    return build_ground_output();
  }
}

bool PX4_Position_Controller::ensure_takeoff_completed() const {
  if (!current_state.isValid()) {
    return false;
  }
  const double rel_height = current_state.position.z() - takeoff_start_z_m_;
  return rel_height >=
         static_cast<double>(config_.takeoff_height_m -
                             config_.position_error_tolerance_z_m);
}

bool PX4_Position_Controller::ensure_land_completed() const {
  if (!current_state.isValid()) {
    return false;
  }
  const double z_threshold =
      takeoff_start_z_m_ + static_cast<double>(config_.position_error_tolerance_z_m);
  return (current_state.position.z() <= z_threshold) &&
         (std::abs(current_state.velocity.z()) < 0.15);
}

bool PX4_Position_Controller::ensure_emergency_land_completed() const {
  if (!current_state.isValid()) {
    return false;
  }
  const double z_threshold =
      takeoff_start_z_m_ + static_cast<double>(config_.position_error_tolerance_z_m);
  return (current_state.position.z() <= z_threshold) &&
         (current_state.velocity.norm() < 0.3);
}

bool PX4_Position_Controller::validate_state_inputs() const {
  if (!has_loadparam) {
    ROS_WARN_THROTTLE(1.0,
                      "[PX4PositionController][debug] invalid input: params not loaded");
    return false;
  }
  if (!current_state.isValid()) {
    ROS_WARN_THROTTLE(
        1.0,
        "[PX4PositionController][debug] invalid current_state: frame=%d stamp=%.3f "
        "pos=[%.3f %.3f %.3f] vel=[%.3f %.3f %.3f] q_norm=%.6f",
        static_cast<int>(current_state.coordinate_frame),
        current_state.timestamp.toSec(), current_state.position.x(),
        current_state.position.y(), current_state.position.z(),
        current_state.velocity.x(), current_state.velocity.y(),
        current_state.velocity.z(), current_state.orientation.norm());
    return false;
  }

  const ros::Time now = ros::Time::now();
  if (!is_state_fresh(current_state, now)) {
    const double age_s = (now - current_state.timestamp).toSec();
    ROS_WARN_THROTTLE(
        1.0,
        "[PX4PositionController][debug] stale current_state: age=%.3f timeout=%.3f stamp=%.3f now=%.3f",
        age_s, static_cast<double>(config_.state_timeout_s),
        current_state.timestamp.toSec(), now.toSec());
    return false;
  }

  if (flight_stage_ == FlightStage::AIR) {
    if (!desired_state.isValid()) {
      ROS_WARN_THROTTLE(
          1.0,
          "[PX4PositionController][debug] invalid desired_state in AIR: frame=%d stamp=%.3f "
          "pos=[%.3f %.3f %.3f] vel=[%.3f %.3f %.3f] q_norm=%.6f",
          static_cast<int>(desired_state.coordinate_frame),
          desired_state.timestamp.toSec(), desired_state.position.x(),
          desired_state.position.y(), desired_state.position.z(),
          desired_state.velocity.x(), desired_state.velocity.y(),
          desired_state.velocity.z(), desired_state.orientation.norm());
      return false;
    }
    if (!is_state_fresh(desired_state, now)) {
      const double age_s = (now - desired_state.timestamp).toSec();
      ROS_WARN_THROTTLE(
          1.0,
          "[PX4PositionController][debug] stale desired_state in AIR: age=%.3f timeout=%.3f stamp=%.3f now=%.3f",
          age_s, static_cast<double>(config_.state_timeout_s),
          desired_state.timestamp.toSec(), now.toSec());
      return false;
    }
  }

  return true;
}

bool PX4_Position_Controller::is_state_fresh(
    const uav_common::UAVStateEstimate &state, const ros::Time &now) const {
  if (state.timestamp.isZero()) {
    return false;
  }
  const double age_s = (now - state.timestamp).toSec();
  return age_s >= 0.0 && age_s <= static_cast<double>(config_.state_timeout_s);
}

bool PX4_Position_Controller::is_position_reached(
    const uav_common::UAVStateEstimate &target) const {
  if (!current_state.isValid() || !target.isValid()) {
    return false;
  }

  const Eigen::Vector3d error = target.position - current_state.position;
  return (std::abs(error.x()) <=
          static_cast<double>(config_.position_error_tolerance_x_m)) &&
         (std::abs(error.y()) <=
          static_cast<double>(config_.position_error_tolerance_y_m)) &&
         (std::abs(error.z()) <=
          static_cast<double>(config_.position_error_tolerance_z_m));
}

ControlOutput PX4_Position_Controller::build_ground_output() const {
  ControlOutput output;
  output.position = current_state.position;
  output.attitude = current_state.orientation;
  output.enable(ControlOutputMask::POSITION);
  output.enable(ControlOutputMask::ATTITUDE);
  return output;
}

ControlOutput PX4_Position_Controller::build_takeoff_output() const {
  ControlOutput output;
  output.position = current_state.position;
  output.position.z() = takeoff_start_z_m_ + static_cast<double>(config_.takeoff_height_m);
  output.velocity =
      Eigen::Vector3d(0.0, 0.0, static_cast<double>(config_.takeoff_climb_mps));
  output.attitude = current_state.orientation;
  output.enable(ControlOutputMask::POSITION);
  output.enable(ControlOutputMask::VELOCITY);
  output.enable(ControlOutputMask::ATTITUDE);
  return output;
}

ControlOutput PX4_Position_Controller::build_air_output() const {
  ControlOutput output;
  output.position = desired_state.position;
  output.velocity = desired_state.velocity;
  output.attitude = desired_state.orientation;
  output.enable(ControlOutputMask::POSITION);
  output.enable(ControlOutputMask::VELOCITY);
  output.enable(ControlOutputMask::ATTITUDE);
  return output;
}

ControlOutput PX4_Position_Controller::build_landing_output() const {
  ControlOutput output;
  output.velocity =
      Eigen::Vector3d(0.0, 0.0, -static_cast<double>(config_.landing_descent_mps));
  output.attitude = current_state.orientation;
  output.enable(ControlOutputMask::VELOCITY);
  output.enable(ControlOutputMask::ATTITUDE);
  return output;
}

ControlOutput PX4_Position_Controller::build_emergency_output() const {
  ControlOutput output;
  output.velocity =
      Eigen::Vector3d(0.0, 0.0, -static_cast<double>(config_.emergency_descent_mps));
  output.attitude = current_state.orientation;
  output.enable(ControlOutputMask::VELOCITY);
  output.enable(ControlOutputMask::ATTITUDE);
  return output;
}

void PX4_Position_Controller::update_flight_stage_by_conditions() {
  const FlightStage prev = flight_stage_;
  switch (flight_stage_) {
  case FlightStage::GROUND:
    break;
  case FlightStage::TAKEOFF:
    if (ensure_takeoff_completed()) {
      flight_stage_ = FlightStage::AIR;
      is_emergency = false;
    }
    break;
  case FlightStage::AIR:
    break;
  case FlightStage::LANDING:
    if (ensure_land_completed()) {
      flight_stage_ = FlightStage::GROUND;
      is_emergency = false;
    }
    break;
  case FlightStage::EMERGENCY:
    if (ensure_emergency_land_completed()) {
      flight_stage_ = FlightStage::GROUND;
      is_emergency = false;
    }
    break;
  default:
    flight_stage_ = FlightStage::GROUND;
    is_emergency = false;
    break;
  }

  if (prev != flight_stage_) {
    stage_enter_time_ = ros::Time::now();
  }
}

} // namespace uav_controller
