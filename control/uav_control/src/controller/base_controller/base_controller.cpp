#include "controller/base_controller/base_controller.hpp"

namespace uav_control {

bool Base_Controller::set_takeoff_mode(void) {
  if (controller_state_ != ControllerState::OFF) {
    return false;
  }
  if (!uav_current_state_.isValid()) {
    return false;
  }

  // 以当前位置信息作为起飞参考，并叠加相对起飞高度。
  takeoff_position_ = uav_current_state_.position;
  takeoff_position_.z() += takeoff_height_;

  // 进入 TAKEOFF 前重置起飞过程上下文。
  takeoff_initialized_ = false;
  takeoff_holdstart_time_ = ros::Time(0);
  takeoff_holdkeep_time_ = ros::Time(0);
  controller_state_ = ControllerState::TAKEOFF;
  return true;
}

bool Base_Controller::set_takeoff_mode(double relative_takeoff_height_m) {
  if(controller_state_ != ControllerState::OFF)
		return false;
	if (relative_takeoff_height_m <= 0.0) {
    return false;
  }
  takeoff_height_ = relative_takeoff_height_m;
  return set_takeoff_mode();
}

bool Base_Controller::set_land_mode() {
  if (controller_state_ == ControllerState::LAND) {
    return true;
  }
  if (controller_state_ == ControllerState::OFF ||
      controller_state_ == ControllerState::UNDEFINED ||
      !uav_current_state_.isValid()) {
    return false;
  }

  // 以当前位置作为降落参考，目标地面高度固定为 z=0。
  land_position_ = uav_current_state_.position;
  land_position_.z() = 0.0;
  land_initialized_ = false;
  land_holdstart_time_ = ros::Time(0);
  land_holdkeep_time_ = ros::Time(0);
  controller_state_ = ControllerState::LAND;
  return true;
}

bool Base_Controller::set_emergency_mode() {
  if (controller_state_ == ControllerState::OFF ||
      controller_state_ == ControllerState::UNDEFINED) {
    return false;
  }
  controller_state_ = ControllerState::EMERGENCY_LAND;
  return true;
}

bool Base_Controller::set_px4_arm_state(bool arm_state) {
  px4_arm_state_ = arm_state;
  return true;
}

bool Base_Controller::set_current_odom(
    const UAVStateEstimate &current_state) {
  uav_current_state_ = current_state;
  return true;
}

const UAVStateEstimate &Base_Controller::get_current_state() const {
  return uav_current_state_;
}

bool Base_Controller::set_px4_attitude(const sensor_msgs::Imu &imu_msg) {
  px4_attitude_.x() = imu_msg.orientation.x;
  px4_attitude_.y() = imu_msg.orientation.y;
  px4_attitude_.z() = imu_msg.orientation.z;
  px4_attitude_.w() = imu_msg.orientation.w;
  return true;
}

bool Base_Controller::set_trajectory(const TrajectoryPoint &trajectory) {
  trajectory_ = trajectory;
  if (trajectory_.valid_mask ==
      static_cast<uint32_t>(TrajectoryPoint::ValidMask::UNDEFINED)) {
    trajectory_.infer_valid_mask_from_nonzero();
  }
  return true;
}

ControllerState Base_Controller::get_controller_state() const {
  return controller_state_;
}

bool Base_Controller::is_takeoff_completed() const {
  if(controller_state_ == ControllerState::HOVER)
		return true;
	return false;
}

bool Base_Controller::is_land_completed() const {
    if(controller_state_ == ControllerState::OFF)
		return true;
	return false;
}

bool Base_Controller::is_emergency_completed() const {
    if(controller_state_ == ControllerState::OFF)
		return true;
	return false;
}

bool Base_Controller::is_healthy() const {
  return has_loadparam_ && uav_current_state_.isValid();
}

} // namespace uav_control
