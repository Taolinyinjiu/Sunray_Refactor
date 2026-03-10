#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include "controller/px4_position_controller/px4_position_controller.h"
#include "sunray_statemachine/sunray_statemachine.h"

namespace {

double yaw_from_quaternion(const geometry_msgs::Quaternion &q) {
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double yaw_from_quaternion(const Eigen::Quaterniond &q) {
  const double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
  const double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  return std::atan2(siny_cosp, cosy_cosp);
}

std::string normalize_token(const std::string &token) {
  std::string out;
  out.reserve(token.size());
  for (char c : token) {
    if (!std::isspace(static_cast<unsigned char>(c))) {
      out.push_back(c);
    }
  }
  return out;
}

nav_msgs::Odometry normalize_odom(const nav_msgs::Odometry &input) {
  nav_msgs::Odometry out = input;
  if (out.header.stamp.isZero()) {
    out.header.stamp = ros::Time::now();
  }
  if (out.header.frame_id.empty()) {
    out.header.frame_id = "world";
  } else if (out.header.frame_id.front() == '/') {
    out.header.frame_id = out.header.frame_id.substr(1);
  }
  if (out.child_frame_id.empty()) {
    out.child_frame_id = "body";
  }
  return out;
}

} // namespace

class UavControlDemoNode {
public:
  enum class PostTakeoffMissionPhase {
    IDLE = 0,
    FORWARD_ACTIVE,
    LEFT_ACTIVE,
    LAND_REQUESTED,
    COMPLETED
  };

  struct TimedEvent {
    SunrayEvent event;
    double trigger_time_s;
    std::string raw_name;
  };

  explicit UavControlDemoNode(ros::NodeHandle &nh)
      : nh_(nh), pnh_("~"), fsm_(nh),
        controller_(std::make_shared<uav_control::Position_Controller>()) {
    std::string odom_topic = "/uav1/sunray/gazebo_pose";
    std::string desired_topic = "/uav1/sunray_desired_odom_in";
    std::string event_topic = "/uav1/sunray_fsm_event";
    pnh_.param("odom_topic", odom_topic, odom_topic);
    pnh_.param("desired_topic", desired_topic, desired_topic);
    pnh_.param("event_topic", event_topic, event_topic);
    pnh_.param("simulate_armed", simulate_armed_, simulate_armed_);
    pnh_.param("auto_seed_desired_from_odom", auto_seed_desired_from_odom_,
               auto_seed_desired_from_odom_);
    pnh_.param("post_takeoff_mission_enable", post_takeoff_mission_enable_,
               post_takeoff_mission_enable_);
    pnh_.param("post_takeoff_forward_m", post_takeoff_forward_m_,
               post_takeoff_forward_m_);
    pnh_.param("post_takeoff_left_m", post_takeoff_left_m_,
               post_takeoff_left_m_);
    pnh_.param("post_takeoff_pos_tol_m", post_takeoff_pos_tol_m_,
               post_takeoff_pos_tol_m_);
    pnh_.param("post_takeoff_hold_s", post_takeoff_hold_s_,
               post_takeoff_hold_s_);
    pnh_.param("param_reload_retry_s", param_reload_retry_s_,
               param_reload_retry_s_);

    try_load_controller_param(true);
    (void)fsm_.register_controller(controller_);

    odom_sub_ =
        nh_.subscribe(odom_topic, 20, &UavControlDemoNode::odom_cb, this);
    desired_sub_ =
        nh_.subscribe(desired_topic, 20, &UavControlDemoNode::desired_cb, this);
    event_sub_ =
        nh_.subscribe(event_topic, 20, &UavControlDemoNode::event_cb, this);

    double update_hz = 50.0;
    pnh_.param("update_hz", update_hz, update_hz);
    update_timer_ = nh_.createTimer(ros::Duration(1.0 / update_hz),
                                    &UavControlDemoNode::update_timer_cb, this);

    bool auto_takeoff = false;
    pnh_.param("auto_takeoff", auto_takeoff, auto_takeoff);
    if (auto_takeoff) {
      double auto_takeoff_delay_s = 3.0;
      pnh_.param("auto_takeoff_delay_s", auto_takeoff_delay_s,
                 auto_takeoff_delay_s);
      auto_takeoff_timer_ =
          nh_.createTimer(ros::Duration(auto_takeoff_delay_s),
                          &UavControlDemoNode::auto_takeoff_cb, this, true);
    }

    std::string test_script;
    pnh_.param("test_sequence_script", test_script, test_script);
    pnh_.param("test_sequence_repeat", test_sequence_repeat_,
               test_sequence_repeat_);
    pnh_.param("test_sequence_enable", test_sequence_enable_,
               test_sequence_enable_);
    if (test_sequence_enable_) {
      if (parse_test_script(test_script)) {
        test_start_time_ = ros::Time::now();
        test_timer_ = nh_.createTimer(ros::Duration(0.02),
                                      &UavControlDemoNode::test_timer_cb, this);
        ROS_INFO("[UavControlDemo] test sequence enabled: '%s' repeat=%s",
                 test_script.c_str(), test_sequence_repeat_ ? "true" : "false");
      } else {
        ROS_WARN("[UavControlDemo] test sequence parse failed, disabled");
      }
    }

    ROS_INFO("[UavControlDemo] subscribed odom='%s', desired='%s', event='%s'",
             odom_topic.c_str(), desired_topic.c_str(), event_topic.c_str());
  }

private:
  void odom_cb(const nav_msgs::OdometryConstPtr &msg) {
    const nav_msgs::Odometry odom_msg = normalize_odom(*msg);
    last_odom_msg_ = odom_msg;
    has_last_odom_ = true;
    const uav_control::UAVStateEstimate odom(odom_msg);
    (void)controller_->set_current_odom(odom);

    if (simulate_armed_) {
      (void)controller_->set_px4_arm_state(true);
    }

    if (!state_available_) {
      state_available_ = true;
      fsm_.set_state_available(true);
    }

    if (!takeoff_callback_ready_) {
      takeoff_callback_ready_ = true;
      fsm_.set_takeoff_callback_ready(true);
    }

    // 仅在首次里程计时进行一次 seed，避免后续周期性覆盖任务目标。
    if (auto_seed_desired_from_odom_ && !desired_input_received_ &&
        !auto_seed_initialized_) {
      uav_control::TrajectoryPoint hold_point;
      hold_point.set_position(odom.position);
      hold_point.set_yaw(yaw_from_quaternion(odom_msg.pose.pose.orientation));
      (void)controller_->set_trajectory(hold_point);
      auto_seed_initialized_ = true;
      ROS_INFO_ONCE(
          "[UavControlDemo] seeded desired trajectory from first odom sample");
    }
  }

  void desired_cb(const nav_msgs::OdometryConstPtr &msg) {
    desired_input_received_ = true;

    uav_control::TrajectoryPoint desired_point;
    desired_point.set_position(Eigen::Vector3d(
        msg->pose.pose.position.x, msg->pose.pose.position.y,
        msg->pose.pose.position.z));
    desired_point.set_velocity(Eigen::Vector3d(msg->twist.twist.linear.x,
                                               msg->twist.twist.linear.y,
                                               msg->twist.twist.linear.z));
    desired_point.set_yaw(yaw_from_quaternion(msg->pose.pose.orientation));
    (void)controller_->set_trajectory(desired_point);

    if (!takeoff_callback_ready_) {
      takeoff_callback_ready_ = true;
      fsm_.set_takeoff_callback_ready(true);
    }
  }

  void event_cb(const std_msgs::StringConstPtr &msg) {
    SunrayEvent event;
    if (parse_event_name(msg->data, &event)) {
      dispatch_event(event, msg->data, "topic");
      return;
    }
    ROS_WARN_THROTTLE(1.0, "[UavControlDemo] unsupported event: %s",
                      msg->data.c_str());
  }

  void update_timer_cb(const ros::TimerEvent &) {
    if (!controller_param_loaded_) {
      const ros::Time now = ros::Time::now();
      if (last_param_retry_time_.isZero() ||
          (now - last_param_retry_time_).toSec() >= param_reload_retry_s_) {
        last_param_retry_time_ = now;
        (void)try_load_controller_param(false);
      }
      return;
    }
    fsm_.update();
    dispatch_completion_events();
    update_post_takeoff_mission();
  }

  void auto_takeoff_cb(const ros::TimerEvent &) {
    dispatch_event(SunrayEvent::TAKEOFF_REQUEST, "TAKEOFF_REQUEST",
                   "auto_takeoff");
  }

  void dispatch_completion_events() {
    const SunrayState current = fsm_.current_state();

    if (current != SunrayState::TAKEOFF) {
      takeoff_completed_sent_ = false;
    }
    if (current != SunrayState::LAND) {
      land_completed_sent_ = false;
    }
    if (current != SunrayState::EMERGENCY_LAND) {
      emergency_completed_sent_ = false;
    }

    if (current == SunrayState::TAKEOFF && !takeoff_completed_sent_ &&
        controller_->is_takeoff_completed()) {
      dispatch_event(SunrayEvent::TAKEOFF_COMPLETED, "TAKEOFF_COMPLETED",
                     "controller");
      takeoff_completed_sent_ = true;
      maybe_start_post_takeoff_mission();
    }

    if (current == SunrayState::LAND && !land_completed_sent_ &&
        controller_->is_land_completed()) {
      dispatch_event(SunrayEvent::LAND_COMPLETED, "LAND_COMPLETED",
                     "controller");
      land_completed_sent_ = true;
    }

    if (current == SunrayState::EMERGENCY_LAND && !emergency_completed_sent_ &&
        controller_->is_emergency_completed()) {
      dispatch_event(SunrayEvent::EMERGENCY_COMPLETED, "EMERGENCY_COMPLETED",
                     "controller");
      emergency_completed_sent_ = true;
    }
  }

  bool parse_event_name(const std::string &name, SunrayEvent *event) const {
    if (event == nullptr) {
      return false;
    }

    if (name == "TAKEOFF_REQUEST") {
      *event = SunrayEvent::TAKEOFF_REQUEST;
      return true;
    }
    if (name == "TAKEOFF_COMPLETED") {
      *event = SunrayEvent::TAKEOFF_COMPLETED;
      return true;
    }
    if (name == "LAND_REQUEST") {
      *event = SunrayEvent::LAND_REQUEST;
      return true;
    }
    if (name == "LAND_COMPLETED") {
      *event = SunrayEvent::LAND_COMPLETED;
      return true;
    }
    if (name == "EMERGENCY_REQUEST") {
      *event = SunrayEvent::EMERGENCY_REQUEST;
      return true;
    }
    if (name == "EMERGENCY_COMPLETED") {
      *event = SunrayEvent::EMERGENCY_COMPLETED;
      return true;
    }
    if (name == "WATCHDOG_ERROR") {
      *event = SunrayEvent::WATCHDOG_ERROR;
      return true;
    }
    if (name == "ENTER_VELOCITY_CONTROL") {
      *event = SunrayEvent::ENTER_VELOCITY_CONTROL;
      return true;
    }
    if (name == "ENTER_POSE_CONTROL") {
      *event = SunrayEvent::ENTER_POSE_CONTROL;
      return true;
    }
    if (name == "ENTER_REFERENCE_CONTROL") {
      *event = SunrayEvent::ENTER_REFERENCE_CONTROL;
      return true;
    }
    if (name == "ENTER_TRAJECTORY_CONTROL") {
      *event = SunrayEvent::ENTER_TRAJECTORY_CONTROL;
      return true;
    }
    if (name == "EXIT_CONTROL_MODE") {
      *event = SunrayEvent::EXIT_CONTROL_MODE;
      return true;
    }
    if (name == "TRAJECTORY_COMPLETED") {
      *event = SunrayEvent::TRAJECTORY_COMPLETED;
      return true;
    }
    if (name == "BREAKING_COMPLETED") {
      *event = SunrayEvent::BREAKING_COMPLETED;
      return true;
    }
    return false;
  }

  bool parse_test_script(const std::string &script) {
    timed_events_.clear();
    next_timed_event_index_ = 0U;
    if (script.empty()) {
      return false;
    }

    std::stringstream ss(script);
    std::string token;
    while (std::getline(ss, token, ',')) {
      const std::string trimmed = normalize_token(token);
      if (trimmed.empty()) {
        continue;
      }

      const std::size_t at_pos = trimmed.find('@');
      if (at_pos == std::string::npos || at_pos == 0U ||
          at_pos + 1U >= trimmed.size()) {
        ROS_WARN("[UavControlDemo] invalid test token: '%s'",
                 trimmed.c_str());
        return false;
      }

      const std::string event_name = trimmed.substr(0, at_pos);
      const std::string time_s = trimmed.substr(at_pos + 1U);
      SunrayEvent evt;
      if (!parse_event_name(event_name, &evt)) {
        ROS_WARN("[UavControlDemo] unknown event in test script: '%s'",
                 event_name.c_str());
        return false;
      }

      char *end_ptr = nullptr;
      const double trigger_time = std::strtod(time_s.c_str(), &end_ptr);
      if (end_ptr == time_s.c_str() || trigger_time < 0.0) {
        ROS_WARN("[UavControlDemo] invalid trigger time in test script: '%s'",
                 time_s.c_str());
        return false;
      }

      timed_events_.push_back(TimedEvent{evt, trigger_time, event_name});
    }

    if (timed_events_.empty()) {
      return false;
    }

    std::sort(timed_events_.begin(), timed_events_.end(),
              [](const TimedEvent &a, const TimedEvent &b) {
                return a.trigger_time_s < b.trigger_time_s;
              });
    return true;
  }

  void test_timer_cb(const ros::TimerEvent &) {
    if (timed_events_.empty()) {
      return;
    }

    const double elapsed_s = (ros::Time::now() - test_start_time_).toSec();
    while (next_timed_event_index_ < timed_events_.size() &&
           elapsed_s >= timed_events_[next_timed_event_index_].trigger_time_s) {
      const TimedEvent &te = timed_events_[next_timed_event_index_];
      dispatch_event(te.event, te.raw_name, "test_sequence");
      ++next_timed_event_index_;
    }

    if (next_timed_event_index_ >= timed_events_.size()) {
      if (test_sequence_repeat_) {
        next_timed_event_index_ = 0U;
        test_start_time_ = ros::Time::now();
        return;
      }
      test_timer_.stop();
      ROS_INFO("[UavControlDemo] test sequence completed");
    }
  }

  void dispatch_event(SunrayEvent event, const std::string &name,
                      const std::string &source) {
    const bool accepted = fsm_.dispatch(event);
    ROS_INFO("[UavControlDemo][event] source=%s event=%s accepted=%s",
             source.c_str(), name.c_str(), accepted ? "true" : "false");
  }

  bool try_load_controller_param(bool first_attempt) {
    if (controller_param_loaded_) {
      return true;
    }
    if (controller_->load_param(nh_)) {
      controller_param_loaded_ = true;
      ROS_INFO("[UavControlDemo] controller load_param success");
      return true;
    }
    if (first_attempt) {
      ROS_WARN(
          "[UavControlDemo] controller load_param failed, waiting params and retrying...");
    } else {
      ROS_WARN_THROTTLE(
          2.0,
          "[UavControlDemo] controller load_param still not ready, retrying...");
    }
    return false;
  }

  void maybe_start_post_takeoff_mission() {
    if (!post_takeoff_mission_enable_ || post_takeoff_mission_started_) {
      return;
    }

    const uav_control::UAVStateEstimate current_state =
        controller_->get_current_state();
    if (!current_state.isValid()) {
      ROS_WARN(
          "[UavControlDemo] post-takeoff mission skipped: current state invalid");
      return;
    }

    const double yaw = yaw_from_quaternion(current_state.orientation);
    const Eigen::Vector3d forward_dir(std::cos(yaw), std::sin(yaw), 0.0);
    const Eigen::Vector3d left_dir(-std::sin(yaw), std::cos(yaw), 0.0);

    mission_forward_target_ =
        current_state.position + post_takeoff_forward_m_ * forward_dir;
    mission_left_target_ =
        mission_forward_target_ + post_takeoff_left_m_ * left_dir;

    command_position_target(mission_forward_target_);
    mission_phase_ = PostTakeoffMissionPhase::FORWARD_ACTIVE;
    mission_target_hold_start_time_ = ros::Time(0);
    post_takeoff_mission_started_ = true;

    ROS_INFO(
        "[UavControlDemo] post-takeoff mission started: forward=%.2fm, left=%.2fm",
        post_takeoff_forward_m_, post_takeoff_left_m_);
  }

  void update_post_takeoff_mission() {
    if (!post_takeoff_mission_enable_ || !post_takeoff_mission_started_) {
      return;
    }

    const SunrayState current = fsm_.current_state();
    if (current == SunrayState::OFF &&
        mission_phase_ == PostTakeoffMissionPhase::LAND_REQUESTED) {
      mission_phase_ = PostTakeoffMissionPhase::COMPLETED;
      post_takeoff_mission_started_ = false;
      ROS_INFO("[UavControlDemo] post-takeoff mission completed");
      return;
    }

    if (current != SunrayState::HOVER) {
      return;
    }

    if (mission_phase_ == PostTakeoffMissionPhase::FORWARD_ACTIVE) {
      if (is_target_reached_with_hold(mission_forward_target_)) {
        command_position_target(mission_left_target_);
        mission_phase_ = PostTakeoffMissionPhase::LEFT_ACTIVE;
        mission_target_hold_start_time_ = ros::Time(0);
        ROS_INFO("[UavControlDemo] forward waypoint reached, switch to left");
      }
      return;
    }

    if (mission_phase_ == PostTakeoffMissionPhase::LEFT_ACTIVE) {
      if (is_target_reached_with_hold(mission_left_target_)) {
        dispatch_event(SunrayEvent::LAND_REQUEST, "LAND_REQUEST",
                       "post_takeoff_mission");
        mission_phase_ = PostTakeoffMissionPhase::LAND_REQUESTED;
        mission_target_hold_start_time_ = ros::Time(0);
      }
      return;
    }
  }

  bool is_target_reached_with_hold(const Eigen::Vector3d &target) {
    const uav_control::UAVStateEstimate current_state =
        controller_->get_current_state();
    if (!current_state.isValid()) {
      mission_target_hold_start_time_ = ros::Time(0);
      return false;
    }

    const double distance = (current_state.position - target).norm();
    const ros::Time now = ros::Time::now();
    if (distance > post_takeoff_pos_tol_m_) {
      mission_target_hold_start_time_ = ros::Time(0);
      return false;
    }

    if (mission_target_hold_start_time_.isZero()) {
      mission_target_hold_start_time_ = now;
      return false;
    }

    return (now - mission_target_hold_start_time_).toSec() >=
           post_takeoff_hold_s_;
  }

  void command_position_target(const Eigen::Vector3d &target_position) {
    uav_control::TrajectoryPoint target;
    target.set_position(target_position);
    // 标记已存在显式目标，防止 fallback seed 覆盖任务位置指令。
    desired_input_received_ = true;
    if (has_last_odom_) {
      target.set_yaw(yaw_from_quaternion(last_odom_msg_.pose.pose.orientation));
    }
    (void)controller_->set_trajectory(target);
    ROS_INFO("[UavControlDemo] set target position: [%.3f, %.3f, %.3f]",
             target_position.x(), target_position.y(), target_position.z());
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  Sunray_StateMachine fsm_;
  std::shared_ptr<uav_control::Position_Controller> controller_;

  ros::Subscriber odom_sub_;
  ros::Subscriber desired_sub_;
  ros::Subscriber event_sub_;
  ros::Timer update_timer_;
  ros::Timer auto_takeoff_timer_;
  ros::Timer test_timer_;

  bool simulate_armed_{true};
  bool state_available_{false};
  bool takeoff_callback_ready_{false};
  bool auto_seed_desired_from_odom_{true};
  bool auto_seed_initialized_{false};
  bool desired_input_received_{false};
  bool test_sequence_enable_{false};
  bool test_sequence_repeat_{false};
  bool controller_param_loaded_{false};
  bool has_last_odom_{false};
  bool post_takeoff_mission_enable_{true};
  bool post_takeoff_mission_started_{false};
  double post_takeoff_forward_m_{1.0};
  double post_takeoff_left_m_{1.0};
  double post_takeoff_pos_tol_m_{0.15};
  double post_takeoff_hold_s_{0.5};
  double param_reload_retry_s_{0.5};
  ros::Time last_param_retry_time_{0};
  ros::Time mission_target_hold_start_time_{0};
  nav_msgs::Odometry last_odom_msg_;
  Eigen::Vector3d mission_forward_target_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d mission_left_target_ = Eigen::Vector3d::Zero();
  PostTakeoffMissionPhase mission_phase_{PostTakeoffMissionPhase::IDLE};

  bool takeoff_completed_sent_{false};
  bool land_completed_sent_{false};
  bool emergency_completed_sent_{false};

  ros::Time test_start_time_;
  std::vector<TimedEvent> timed_events_;
  std::size_t next_timed_event_index_{0U};
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "uav_control_demo_node");
  ros::NodeHandle nh;
  UavControlDemoNode node(nh);
  ros::spin();
  return 0;
}
