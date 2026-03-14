#include "sunray_helper/sunray_helper.h"

#include <algorithm>
#include <ros/ros.h>

#include <Eigen/Geometry>
#include <cmath>

#include <uav_control/AttitudeCmd.h>
#include <uav_control/ComplexCmd.h>
#include <uav_control/Land.h>
#include <uav_control/LandCmd.h>
#include <uav_control/PositionCmd.h>
#include <uav_control/PositionRequest.h>
#include <uav_control/ReturnHome.h>
#include <uav_control/ReturnHomeCmd.h>
#include <uav_control/Takeoff.h>
#include <uav_control/TakeoffCmd.h>
#include <uav_control/TrajectoryCmd.h>
#include <uav_control/VelocityCmd.h>

namespace {
std::string normalize_ns(const std::string &ns) {
  if (!ns.empty() && ns.front() == '/') {
    return ns.substr(1);
  }
  return ns;
}

std::string resolve_uav_namespace(ros::NodeHandle &nh) {
  std::string key;
  std::string ns;
  if (nh.searchParam("uav_ns", key) && nh.getParam(key, ns) && !ns.empty()) {
    return normalize_ns(ns);
  }

  std::string name;
  int id = 0;
  bool ok_name = false;
  bool ok_id = false;
  if (nh.searchParam("uav_name", key)) {
    ok_name = nh.getParam(key, name) && !name.empty();
  }
  if (nh.searchParam("uav_id", key)) {
    ok_id = nh.getParam(key, id);
  }
  if (ok_name && ok_id) {
    return name + std::to_string(id);
  }

  // fallback to global params
  name = "uav";
  id = 1;
  nh.param("/uav_name", name, name);
  nh.param("/uav_id", id, id);
  return name + std::to_string(id);
}

Eigen::Quaterniond quat_from_yaw(double yaw) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
}

Eigen::Vector3d rpy_from_quat(const Eigen::Quaterniond &q) {
  return q.toRotationMatrix().eulerAngles(0, 1, 2);
}

const char *fsm_state_to_string(sunray_fsm::SunrayState state) {
  switch (state) {
  case sunray_fsm::SunrayState::OFF:
    return "OFF";
  case sunray_fsm::SunrayState::TAKEOFF:
    return "TAKEOFF";
  case sunray_fsm::SunrayState::HOVER:
    return "HOVER";
  case sunray_fsm::SunrayState::RETURN:
    return "RETURN";
  case sunray_fsm::SunrayState::LAND:
    return "LAND";
  case sunray_fsm::SunrayState::EMERGENCY_LAND:
    return "EMERGENCY_LAND";
  case sunray_fsm::SunrayState::POSITION_CONTROL:
    return "POSITION_CONTROL";
  case sunray_fsm::SunrayState::VELOCITY_CONTROL:
    return "VELOCITY_CONTROL";
  case sunray_fsm::SunrayState::ATTITUDE_CONTROL:
    return "ATTITUDE_CONTROL";
  case sunray_fsm::SunrayState::COMPLEX_CONTROL:
    return "COMPLEX_CONTROL";
  case sunray_fsm::SunrayState::TRAJECTORY_CONTROL:
    return "TRAJECTORY_CONTROL";
  default:
    return "UNKNOWN_STATE";
  }
}
} // namespace

Sunray_Helper::Sunray_Helper(ros::NodeHandle &nh) : nh_(nh) {
  nh_.param("takeoff_wait_s", takeoff_wait_timeout_s_, takeoff_wait_timeout_s_);
  nh_.param("takeoff_block_timeout_s", takeoff_wait_timeout_s_,
            takeoff_wait_timeout_s_);
  nh_.param("position_reached_tolerance_m", position_reached_tolerance_m_,
            position_reached_tolerance_m_);
  nh_.param("move_wait_s", block_wait_timeout_s_, block_wait_timeout_s_);
  nh_.param("block_wait_timeout_s", block_wait_timeout_s_, block_wait_timeout_s_);
  nh_.param("land_wait_s", land_wait_timeout_s_, land_wait_timeout_s_);
  nh_.param("land_block_timeout_s", land_wait_timeout_s_, land_wait_timeout_s_);
  nh_.param("wait_poll_hz", wait_poll_hz_, wait_poll_hz_);

  uav_ns_ = resolve_uav_namespace(nh_);
  const std::string ctrl_ns =
      uav_ns_.empty() ? "/sunray_control" : ("/" + uav_ns_ + "/sunray_control");
  ctrl_nh_ = ros::NodeHandle(ctrl_ns);

  takeoff_pub_ = ctrl_nh_.advertise<uav_control::TakeoffCmd>("takeoff_cmd", 10);
  land_pub_ = ctrl_nh_.advertise<uav_control::LandCmd>("land_cmd", 10);
  return_pub_ =
      ctrl_nh_.advertise<uav_control::ReturnHomeCmd>("return_cmd", 10);

  position_cmd_pub_ =
      ctrl_nh_.advertise<uav_control::PositionCmd>("position_cmd", 10);
  velocity_cmd_pub_ =
      ctrl_nh_.advertise<uav_control::VelocityCmd>("velocity_cmd", 10);
  attitude_cmd_pub_ =
      ctrl_nh_.advertise<uav_control::AttitudeCmd>("attitude_cmd", 10);
  trajectory_cmd_pub_ =
      ctrl_nh_.advertise<uav_control::TrajectoryCmd>("trajectory_cmd", 10);
  complex_cmd_pub_ =
      ctrl_nh_.advertise<uav_control::ComplexCmd>("complex_cmd", 10);
  fsm_state_sub_ =
      ctrl_nh_.subscribe("fsm_state", 10, &Sunray_Helper::fsm_state_cb, this);

  takeoff_client_ =
      ctrl_nh_.serviceClient<uav_control::Takeoff>("takeoff_request");
  land_client_ = ctrl_nh_.serviceClient<uav_control::Land>("land_request");
  return_client_ =
      ctrl_nh_.serviceClient<uav_control::ReturnHome>("return_request");
  position_cmd_client_ =
      ctrl_nh_.serviceClient<uav_control::PositionRequest>("position_request");

  try {
    ros::NodeHandle reader_nh =
        uav_ns_.empty() ? nh_ : ros::NodeHandle("/" + uav_ns_);
    px4_data_reader_.reset(new PX4_DataReader(reader_nh));
    px4_reader_ready_ = true;
  } catch (const std::exception &e) {
    ROS_WARN("Sunray_Helper: PX4_DataReader init failed: %s", e.what());
    px4_reader_ready_ = false;
  }
}

void Sunray_Helper::set_cached_fsm_state(sunray_fsm::SunrayState state) {
  std::lock_guard<std::mutex> lock(fsm_state_mutex_);
  fsm_state_ = state;
  fsm_state_received_ = true;
}

void Sunray_Helper::fsm_state_cb(const std_msgs::UInt8::ConstPtr &msg) {
  if (!msg) {
    return;
  }

  std::lock_guard<std::mutex> lock(fsm_state_mutex_);
  fsm_state_ = static_cast<sunray_fsm::SunrayState>(msg->data);
  fsm_state_received_ = true;
}

bool Sunray_Helper::wait_for_fsm_state(sunray_fsm::SunrayState expected_state,
                                       double timeout_s) {
  const double effective_timeout_s =
      (timeout_s > 0.0) ? timeout_s : block_wait_timeout_s_;
  ros::Rate rate(std::max(1.0, wait_poll_hz_));
  const ros::WallTime deadline =
      ros::WallTime::now() + ros::WallDuration(effective_timeout_s);

  while (ros::ok() && ros::WallTime::now() <= deadline) {
    {
      std::lock_guard<std::mutex> lock(fsm_state_mutex_);
      if (fsm_state_received_ && fsm_state_ == expected_state) {
        return true;
      }
    }
    rate.sleep();
  }

  sunray_fsm::SunrayState current_state = sunray_fsm::SunrayState::OFF;
  bool has_state = false;
  {
    std::lock_guard<std::mutex> lock(fsm_state_mutex_);
    current_state = fsm_state_;
    has_state = fsm_state_received_;
  }

  ROS_WARN("Sunray_Helper: wait for FSM state timeout, expected=%s current=%s "
           "received=%d timeout=%.3f",
           fsm_state_to_string(expected_state), fsm_state_to_string(current_state),
           static_cast<int>(has_state), effective_timeout_s);
  return false;
}

bool Sunray_Helper::wait_for_position_reached(
    const Eigen::Vector3d &target_position, double timeout_s) {
  if (!px4_reader_ready_ || !px4_data_reader_) {
    ROS_WARN_THROTTLE(1.0,
                      "Sunray_Helper: PX4 reader unavailable, skip position "
                      "completion wait");
    return true;
  }

  const double effective_timeout_s =
      (timeout_s > 0.0) ? timeout_s : block_wait_timeout_s_;
  ros::Rate rate(std::max(1.0, wait_poll_hz_));
  const ros::WallTime deadline =
      ros::WallTime::now() + ros::WallDuration(effective_timeout_s);

  while (ros::ok() && ros::WallTime::now() <= deadline) {
    const Eigen::Vector3d current_position = get_uav_position();
    if ((current_position - target_position).norm() <=
        position_reached_tolerance_m_) {
      return true;
    }
    rate.sleep();
  }

  const Eigen::Vector3d current_position = get_uav_position();
  ROS_WARN(
      "Sunray_Helper: wait for position timeout, current=(%.3f, %.3f, %.3f) "
      "target=(%.3f, %.3f, %.3f) tol=%.3f timeout=%.3f",
      current_position.x(), current_position.y(), current_position.z(),
      target_position.x(), target_position.y(), target_position.z(),
      position_reached_tolerance_m_, effective_timeout_s);
  return false;
}

bool Sunray_Helper::wait_for_landed(double timeout_s) {
  if (!px4_reader_ready_ || !px4_data_reader_) {
    ROS_WARN_THROTTLE(1.0,
                      "Sunray_Helper: PX4 reader unavailable, skip landing "
                      "completion wait");
    return true;
  }

  const double effective_timeout_s =
      (timeout_s > 0.0) ? timeout_s : land_wait_timeout_s_;
  ros::Rate rate(std::max(1.0, wait_poll_hz_));
  const ros::WallTime deadline =
      ros::WallTime::now() + ros::WallDuration(effective_timeout_s);

  while (ros::ok() && ros::WallTime::now() <= deadline) {
    const px4_data_types::SystemState system_state =
        px4_data_reader_->get_system_state();
    if (system_state.landed_state == px4_data_types::LandedState::kOnGround ||
        !system_state.armed) {
      return true;
    }
    rate.sleep();
  }

  const px4_data_types::SystemState system_state =
      px4_data_reader_->get_system_state();
  ROS_WARN(
      "Sunray_Helper: wait for landing timeout, armed=%d landed_state=%d "
      "timeout=%.3f",
      static_cast<int>(system_state.armed),
      static_cast<int>(system_state.landed_state), effective_timeout_s);
  return false;
}

bool Sunray_Helper::takeoff_async() {
  uav_control::TakeoffCmd msg;
  msg.takeoff_relative_height = 0.0;
  msg.takeoff_max_velocity = 0.0;
  takeoff_pub_.publish(msg);
  set_cached_fsm_state(sunray_fsm::SunrayState::TAKEOFF);
  return true;
}

bool Sunray_Helper::takeoff_block() {
  if (!takeoff_client_.exists() &&
      !takeoff_client_.waitForExistence(ros::Duration(1.0))) {
    return false;
  }
  uav_control::Takeoff srv;
  srv.request.takeoff_relative_height = 0.0;
  srv.request.takeoff_max_velocity = 0.0;
  if (!takeoff_client_.call(srv)) {
    return false;
  }
  if (srv.response.accepted) {
    set_cached_fsm_state(sunray_fsm::SunrayState::TAKEOFF);
    return wait_for_fsm_state(sunray_fsm::SunrayState::HOVER,
                              takeoff_wait_timeout_s_);
  }
  return false;
}

bool Sunray_Helper::land_async(int land_type, double land_max_velocity) {
  uav_control::LandCmd msg;
  msg.land_type = land_type;
  msg.land_max_velocity = land_max_velocity;
  land_pub_.publish(msg);
  set_cached_fsm_state(sunray_fsm::SunrayState::LAND);
  return true;
}

bool Sunray_Helper::land_block(int land_type, double land_max_velocity) {
  if (!land_client_.exists() &&
      !land_client_.waitForExistence(ros::Duration(1.0))) {
    return false;
  }
  uav_control::Land srv;
  srv.request.land_type = land_type;
  srv.request.land_max_velocity = land_max_velocity;
  if (!land_client_.call(srv)) {
    return false;
  }
  if (srv.response.accepted) {
    set_cached_fsm_state(sunray_fsm::SunrayState::LAND);
    if (!wait_for_fsm_state(sunray_fsm::SunrayState::OFF,
                            land_wait_timeout_s_)) {
      return false;
    }
    return wait_for_landed(1.0);
  }
  return false;
}

bool Sunray_Helper::return_async() {
  uav_control::ReturnHomeCmd msg;
  msg.use_takeoff_homepoint = true;
  msg.target_position.x = 0.0;
  msg.target_position.y = 0.0;
  msg.target_position.z = 0.0;
  msg.yaw_ctrl = false;
  msg.yaw = 0.0;
  msg.land_max_velocity = 0.0;
  return_pub_.publish(msg);
  set_cached_fsm_state(sunray_fsm::SunrayState::RETURN);
  return true;
}

bool Sunray_Helper::return_block() {
  if (!return_client_.exists() &&
      !return_client_.waitForExistence(ros::Duration(1.0))) {
    return false;
  }
  uav_control::ReturnHome srv;
  srv.request.use_takeoff_homepoint = true;
  srv.request.target_position.x = 0.0;
  srv.request.target_position.y = 0.0;
  srv.request.target_position.z = 0.0;
  srv.request.yaw_ctrl = false;
  srv.request.yaw = 0.0;
  srv.request.land_max_velocity = 0.0;
  if (!return_client_.call(srv)) {
    return false;
  }
  if (srv.response.accepted) {
    set_cached_fsm_state(sunray_fsm::SunrayState::RETURN);
  }
  return srv.response.accepted;
}

bool Sunray_Helper::return_async(Eigen::Vector3d target_position,
                                 bool yaw_ctrl,
                                 double yaw,
                                 double land_max_velocity) {
  uav_control::ReturnHomeCmd msg;
  msg.use_takeoff_homepoint = false;
  msg.target_position.x = target_position.x();
  msg.target_position.y = target_position.y();
  msg.target_position.z = target_position.z();
  msg.yaw_ctrl = yaw_ctrl;
  msg.yaw = yaw;
  msg.land_max_velocity = land_max_velocity;
  return_pub_.publish(msg);

  uav_target_.timestamp = ros::Time::now();
  uav_target_.coordinate_frame =
      uav_control::UAVStateEstimate::CoordinateFrame::LOCAL;
  uav_target_.position = target_position;
  if (yaw_ctrl) {
    uav_target_.orientation = quat_from_yaw(yaw);
  }
  set_cached_fsm_state(sunray_fsm::SunrayState::RETURN);
  return true;
}

bool Sunray_Helper::return_block(Eigen::Vector3d target_position,
                                 bool yaw_ctrl,
                                 double yaw,
                                 double land_max_velocity) {
  if (!return_client_.exists() &&
      !return_client_.waitForExistence(ros::Duration(1.0))) {
    return false;
  }
  uav_control::ReturnHome srv;
  srv.request.use_takeoff_homepoint = false;
  srv.request.target_position.x = target_position.x();
  srv.request.target_position.y = target_position.y();
  srv.request.target_position.z = target_position.z();
  srv.request.yaw_ctrl = yaw_ctrl;
  srv.request.yaw = yaw;
  srv.request.land_max_velocity = land_max_velocity;
  if (!return_client_.call(srv)) {
    return false;
  }
  if (srv.response.accepted) {
    uav_target_.timestamp = ros::Time::now();
    uav_target_.coordinate_frame =
        uav_control::UAVStateEstimate::CoordinateFrame::LOCAL;
    uav_target_.position = target_position;
    if (yaw_ctrl) {
      uav_target_.orientation = quat_from_yaw(yaw);
    }
    set_cached_fsm_state(sunray_fsm::SunrayState::RETURN);
  }
  return srv.response.accepted;
}

bool Sunray_Helper::set_position_async(Eigen::Vector3d position_) {
  uav_control::PositionCmd msg;
  msg.target_position.x = position_.x();
  msg.target_position.y = position_.y();
  msg.target_position.z = position_.z();
  msg.yaw_ctrl = false;
  msg.yaw = 0.0;
  position_cmd_pub_.publish(msg);

  uav_target_.timestamp = ros::Time::now();
  uav_target_.coordinate_frame =
      uav_control::UAVStateEstimate::CoordinateFrame::LOCAL;
  uav_target_.position = position_;
  set_cached_fsm_state(sunray_fsm::SunrayState::POSITION_CONTROL);
  return true;
}

bool Sunray_Helper::set_position_block(Eigen::Vector3d position_) {
  if (!position_cmd_client_.exists() &&
      !position_cmd_client_.waitForExistence(ros::Duration(1.0))) {
    return false;
  }
  uav_control::PositionRequest srv;
  srv.request.target_position.x = position_.x();
  srv.request.target_position.y = position_.y();
  srv.request.target_position.z = position_.z();
  srv.request.yaw_ctrl = false;
  srv.request.yaw = 0.0;
  if (!position_cmd_client_.call(srv)) {
    return false;
  }
  if (srv.response.accepted) {
    uav_target_.timestamp = ros::Time::now();
    uav_target_.coordinate_frame =
        uav_control::UAVStateEstimate::CoordinateFrame::LOCAL;
    uav_target_.position = position_;
    set_cached_fsm_state(sunray_fsm::SunrayState::POSITION_CONTROL);
    return wait_for_position_reached(position_, block_wait_timeout_s_);
  }
  return false;
}

bool Sunray_Helper::set_position_list_async(
    std::vector<Eigen::Vector3d> position_list_) {
  if (position_list_.empty()) {
    return false;
  }
  if (position_list_.size() > 1) {
    ROS_WARN_THROTTLE(1.0,
                      "Sunray_Helper: position list not supported, use last");
  }
  return set_position_async(position_list_.back());
}

bool Sunray_Helper::set_position_list_block(
    std::vector<Eigen::Vector3d> position_list_) {
  if (position_list_.empty()) {
    return false;
  }
  if (position_list_.size() > 1) {
    ROS_WARN_THROTTLE(1.0,
                      "Sunray_Helper: position list not supported, use last");
  }
  return set_position_block(position_list_.back());
}

bool Sunray_Helper::set_position_async(Eigen::Vector3d position_, float yaw) {
  uav_control::PositionCmd msg;
  msg.target_position.x = position_.x();
  msg.target_position.y = position_.y();
  msg.target_position.z = position_.z();
  msg.yaw_ctrl = true;
  msg.yaw = yaw;
  position_cmd_pub_.publish(msg);

  uav_target_.timestamp = ros::Time::now();
  uav_target_.coordinate_frame =
      uav_control::UAVStateEstimate::CoordinateFrame::LOCAL;
  uav_target_.position = position_;
  uav_target_.orientation = quat_from_yaw(yaw);
  set_cached_fsm_state(sunray_fsm::SunrayState::POSITION_CONTROL);
  return true;
}

bool Sunray_Helper::set_position_block(Eigen::Vector3d position_, float yaw) {
  if (!position_cmd_client_.exists() &&
      !position_cmd_client_.waitForExistence(ros::Duration(1.0))) {
    return false;
  }
  uav_control::PositionRequest srv;
  srv.request.target_position.x = position_.x();
  srv.request.target_position.y = position_.y();
  srv.request.target_position.z = position_.z();
  srv.request.yaw_ctrl = true;
  srv.request.yaw = yaw;
  if (!position_cmd_client_.call(srv)) {
    return false;
  }
  if (srv.response.accepted) {
    uav_target_.timestamp = ros::Time::now();
    uav_target_.coordinate_frame =
        uav_control::UAVStateEstimate::CoordinateFrame::LOCAL;
    uav_target_.position = position_;
    uav_target_.orientation = quat_from_yaw(yaw);
    set_cached_fsm_state(sunray_fsm::SunrayState::POSITION_CONTROL);
    return wait_for_position_reached(position_, block_wait_timeout_s_);
  }
  return false;
}

bool Sunray_Helper::set_position_list_async(
    std::vector<std::pair<Eigen::Vector3d, float>> point_list_) {
  if (point_list_.empty()) {
    return false;
  }
  if (point_list_.size() > 1) {
    ROS_WARN_THROTTLE(1.0,
                      "Sunray_Helper: position list not supported, use last");
  }
  return set_position_async(point_list_.back().first, point_list_.back().second);
}

bool Sunray_Helper::set_position_list_block(
    std::vector<std::pair<Eigen::Vector3d, float>> point_list_) {
  if (point_list_.empty()) {
    return false;
  }
  if (point_list_.size() > 1) {
    ROS_WARN_THROTTLE(1.0,
                      "Sunray_Helper: position list not supported, use last");
  }
  return set_position_block(point_list_.back().first, point_list_.back().second);
}

bool Sunray_Helper::set_linear_velocity_async(Eigen::Vector3d velocity_) {
  uav_control::VelocityCmd msg;
  msg.position_ctrl = false;
  msg.target_linear_velocity.x = velocity_.x();
  msg.target_linear_velocity.y = velocity_.y();
  msg.target_linear_velocity.z = velocity_.z();
  msg.target_angular_velocity.x = 0.0;
  msg.target_angular_velocity.y = 0.0;
  msg.target_angular_velocity.z = 0.0;
  velocity_cmd_pub_.publish(msg);

  uav_target_.timestamp = ros::Time::now();
  uav_target_.coordinate_frame =
      uav_control::UAVStateEstimate::CoordinateFrame::LOCAL;
  uav_target_.velocity = velocity_;
  set_cached_fsm_state(sunray_fsm::SunrayState::VELOCITY_CONTROL);
  return true;
}

bool Sunray_Helper::set_angular_velocity_async(Eigen::Vector3d velocity_) {
  uav_control::VelocityCmd msg;
  msg.position_ctrl = false;
  msg.target_linear_velocity.x = 0.0;
  msg.target_linear_velocity.y = 0.0;
  msg.target_linear_velocity.z = 0.0;
  msg.target_angular_velocity.x = velocity_.x();
  msg.target_angular_velocity.y = velocity_.y();
  msg.target_angular_velocity.z = velocity_.z();
  velocity_cmd_pub_.publish(msg);

  uav_target_.timestamp = ros::Time::now();
  uav_target_.coordinate_frame =
      uav_control::UAVStateEstimate::CoordinateFrame::LOCAL;
  uav_target_.bodyrates = velocity_;
  set_cached_fsm_state(sunray_fsm::SunrayState::VELOCITY_CONTROL);
  return true;
}

bool Sunray_Helper::set_position_velocity_async(Eigen::Vector3d position_,
                                                float velocity_) {
  Eigen::Vector3d cmd_vel = Eigen::Vector3d::Zero();
  if (velocity_ > 0.0f) {
    Eigen::Vector3d current = get_uav_position();
    Eigen::Vector3d diff = position_ - current;
    const double norm = diff.norm();
    if (norm > 1e-6) {
      cmd_vel = diff / norm * static_cast<double>(velocity_);
    }
  }
  uav_control::VelocityCmd msg;
  msg.position_ctrl = true;
  msg.target_position.x = position_.x();
  msg.target_position.y = position_.y();
  msg.target_position.z = position_.z();
  msg.target_linear_velocity.x = cmd_vel.x();
  msg.target_linear_velocity.y = cmd_vel.y();
  msg.target_linear_velocity.z = cmd_vel.z();
  msg.target_angular_velocity.x = 0.0;
  msg.target_angular_velocity.y = 0.0;
  msg.target_angular_velocity.z = 0.0;
  velocity_cmd_pub_.publish(msg);

  uav_target_.timestamp = ros::Time::now();
  uav_target_.coordinate_frame =
      uav_control::UAVStateEstimate::CoordinateFrame::LOCAL;
  uav_target_.position = position_;
  uav_target_.velocity = cmd_vel;
  set_cached_fsm_state(sunray_fsm::SunrayState::VELOCITY_CONTROL);
  return true;
}

bool Sunray_Helper::set_position_velocity_block(Eigen::Vector3d position_,
                                                float velocity_) {
  return set_position_velocity_async(position_, velocity_);
}

bool Sunray_Helper::set_position_velocity_list_async(
    std::vector<std::pair<Eigen::Vector3d, float>> point_list_) {
  if (point_list_.empty()) {
    return false;
  }
  if (point_list_.size() > 1) {
    ROS_WARN_THROTTLE(
        1.0,
        "Sunray_Helper: position+velocity list not supported, use last");
  }
  return set_position_velocity_async(point_list_.back().first,
                                     point_list_.back().second);
}

bool Sunray_Helper::set_position_velocity_list_block(
    std::vector<std::pair<Eigen::Vector3d, float>> point_list_) {
  if (point_list_.empty()) {
    return false;
  }
  if (point_list_.size() > 1) {
    ROS_WARN_THROTTLE(
        1.0,
        "Sunray_Helper: position+velocity list not supported, use last");
  }
  return set_position_velocity_block(point_list_.back().first,
                                     point_list_.back().second);
}

bool Sunray_Helper::set_yaw_async(float yaw_) {
  uav_control::AttitudeCmd msg;
  msg.yaw_ctrl_types = true;
  msg.yaw = yaw_;
  attitude_cmd_pub_.publish(msg);

  uav_target_.timestamp = ros::Time::now();
  uav_target_.coordinate_frame =
      uav_control::UAVStateEstimate::CoordinateFrame::LOCAL;
  uav_target_.orientation = quat_from_yaw(yaw_);
  set_cached_fsm_state(sunray_fsm::SunrayState::ATTITUDE_CONTROL);
  return true;
}

bool Sunray_Helper::set_yaw_block(float yaw_) {
  return set_yaw_async(yaw_);
}

bool Sunray_Helper::set_yaw_adjust_async(float adjust_yaw_) {
  uav_control::AttitudeCmd msg;
  msg.yaw_ctrl_types = false;
  msg.yaw = adjust_yaw_;
  attitude_cmd_pub_.publish(msg);

  const Eigen::Vector3d rpy = get_uav_attitude_rpy_rad();
  const double yaw = rpy.z() + static_cast<double>(adjust_yaw_);
  uav_target_.timestamp = ros::Time::now();
  uav_target_.coordinate_frame =
      uav_control::UAVStateEstimate::CoordinateFrame::LOCAL;
  uav_target_.orientation = quat_from_yaw(yaw);
  set_cached_fsm_state(sunray_fsm::SunrayState::ATTITUDE_CONTROL);
  return true;
}

bool Sunray_Helper::set_yaw_adjust_block(float adjust_yaw_) {
  return set_yaw_adjust_async(adjust_yaw_);
}

bool Sunray_Helper::set_trajectory_asycn() {
  ROS_WARN_THROTTLE(1.0,
                    "Sunray_Helper: trajectory control requires trajectory data");
  return false;
}

bool Sunray_Helper::set_trajectory_block() {
  ROS_WARN_THROTTLE(1.0,
                    "Sunray_Helper: trajectory control requires trajectory data");
  return false;
}

bool Sunray_Helper::set_complex_control() {
  uav_control::ComplexCmd msg;
  msg.header.stamp = ros::Time::now();
  msg.mode = 0;
  complex_cmd_pub_.publish(msg);
  set_cached_fsm_state(sunray_fsm::SunrayState::COMPLEX_CONTROL);
  return true;
}

uav_control::UAVStateEstimate Sunray_Helper::get_uav_odometry() {
  if (px4_reader_ready_ && px4_data_reader_) {
    const auto pose = px4_data_reader_->get_local_pose();
    const auto vel = px4_data_reader_->get_local_velocity();
    const auto body_vel = px4_data_reader_->get_body_velocity();
    uav_odometry_.timestamp = ros::Time::now();
    uav_odometry_.coordinate_frame =
        uav_control::UAVStateEstimate::CoordinateFrame::LOCAL;
    uav_odometry_.position = pose.position;
    uav_odometry_.orientation = pose.orientation;
    uav_odometry_.velocity = vel.linear;
    uav_odometry_.bodyrates = body_vel.angular;
  }
  return uav_odometry_;
}

Eigen::Vector3d Sunray_Helper::get_uav_position() {
  return get_uav_odometry().position;
}

Eigen::Vector3d Sunray_Helper::get_uav_velocity_linear() {
  return get_uav_odometry().velocity;
}

Eigen::Vector3d Sunray_Helper::get_uav_velocity_angular() {
  return get_uav_odometry().bodyrates;
}

Eigen::Vector3d Sunray_Helper::get_uav_attitude_rpy_rad() {
  return rpy_from_quat(get_uav_odometry().orientation);
}

Eigen::Vector3d Sunray_Helper::get_uav_attitude_rpy_deg() {
  return get_uav_attitude_rpy_rad() * (180.0 / M_PI);
}

Eigen::Quaterniond Sunray_Helper::get_uav_attitude_quat() {
  return get_uav_odometry().orientation;
}

Eigen::Vector3d Sunray_Helper::get_target_position() {
  return uav_target_.position;
}

Eigen::Vector3d Sunray_Helper::get_target_velocity_linear() {
  return uav_target_.velocity;
}

Eigen::Vector3d Sunray_Helper::get_target_velocity_angular() {
  return uav_target_.bodyrates;
}

Eigen::Vector3d Sunray_Helper::get_target_attitude_rpy_rad() {
  return rpy_from_quat(uav_target_.orientation);
}

Eigen::Vector3d Sunray_Helper::get_target_attitude_rpy_deg() {
  return get_target_attitude_rpy_rad() * (180.0 / M_PI);
}

Eigen::Quaterniond Sunray_Helper::get_target_attitude_quat() {
  return uav_target_.orientation;
}

float Sunray_Helper::get_target_thrust() {
  return 0.0f;
}

sunray_fsm::SunrayState Sunray_Helper::get_statemachine_state() {
  std::lock_guard<std::mutex> lock(fsm_state_mutex_);
  return fsm_state_;
}
