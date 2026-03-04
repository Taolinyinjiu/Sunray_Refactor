#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <algorithm>
#include <cstdlib>
#include <cctype>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include "px4_position_controller/px4_position_controller.h"
#include "sunray_statemachine/sunray_statemachine.h"

namespace {
std::string trim_leading_slash(const std::string &s) {
  if (!s.empty() && s.front() == '/') {
    return s.substr(1);
  }
  return s;
}

std::string resolve_uav_ns(ros::NodeHandle &nh) {
  std::string key;
  std::string uav_ns;
  if (nh.searchParam("uav_ns", key) && nh.getParam(key, uav_ns) && !uav_ns.empty()) {
    return trim_leading_slash(uav_ns);
  }

  std::string uav_name;
  int uav_id = 0;
  bool ok_name = false;
  bool ok_id = false;
  if (nh.searchParam("uav_name", key)) {
    ok_name = nh.getParam(key, uav_name) && !uav_name.empty();
  }
  if (nh.searchParam("uav_id", key)) {
    ok_id = nh.getParam(key, uav_id);
  }
  if (ok_name && ok_id) {
    return trim_leading_slash(uav_name + std::to_string(uav_id));
  }
  return "";
}

std::string make_topic(const std::string &uav_ns, const std::string &suffix) {
  const std::string clean_suffix =
      (!suffix.empty() && suffix.front() == '/') ? suffix.substr(1) : suffix;
  if (uav_ns.empty()) {
    return "/" + clean_suffix;
  }
  return "/" + uav_ns + "/" + clean_suffix;
}
} // namespace

class UavControlDemoNode {
public:
  struct TimedEvent {
    SunrayEvent event;
    double trigger_time_s;
    std::string raw_name;
  };

  explicit UavControlDemoNode(ros::NodeHandle &nh)
      : nh_(nh), pnh_("~"), fsm_(nh), controller_(std::make_shared<uav_controller::PX4_Position_Controller>()) {
    uav_ns_ = resolve_uav_ns(nh_);
    ROS_INFO("[UavControlDemo] resolved uav_ns='%s'", uav_ns_.c_str());

    std::string odom_topic = make_topic(uav_ns_, "sunray_odom_in");
    std::string desired_topic = make_topic(uav_ns_, "sunray_desired_odom_in");
    std::string event_topic = make_topic(uav_ns_, "sunray_fsm_event");
    pnh_.param("odom_topic", odom_topic, odom_topic);
    pnh_.param("desired_topic", desired_topic, desired_topic);
    pnh_.param("event_topic", event_topic, event_topic);

    if (!controller_->load_param(nh_, false)) {
      ROS_WARN("[UavControlDemo] controller load_param failed, will continue for debug");
    }
    (void)fsm_.register_controller(controller_);

    odom_sub_ = nh_.subscribe(odom_topic, 20, &UavControlDemoNode::odom_cb, this);
    desired_sub_ = nh_.subscribe(desired_topic, 20, &UavControlDemoNode::desired_cb, this);
    event_sub_ = nh_.subscribe(event_topic, 20, &UavControlDemoNode::event_cb, this);

    double update_hz = 50.0;
    pnh_.param("update_hz", update_hz, update_hz);
    update_timer_ = nh_.createTimer(ros::Duration(1.0 / update_hz),
                                    &UavControlDemoNode::update_timer_cb, this);

    bool auto_takeoff = false;
    pnh_.param("auto_takeoff", auto_takeoff, auto_takeoff);
    if (auto_takeoff) {
      double auto_takeoff_delay_s = 3.0;
      pnh_.param("auto_takeoff_delay_s", auto_takeoff_delay_s, auto_takeoff_delay_s);
      auto_takeoff_timer_ = nh_.createTimer(ros::Duration(auto_takeoff_delay_s),
                                            &UavControlDemoNode::auto_takeoff_cb, this, true);
    }

    std::string test_script;
    pnh_.param("test_sequence_script", test_script, test_script);
    pnh_.param("test_sequence_repeat", test_sequence_repeat_, false);
    pnh_.param("test_sequence_enable", test_sequence_enable_, false);
    pnh_.param("auto_seed_desired_from_odom", auto_seed_desired_from_odom_, true);
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
    controller_->set_currentstate(*msg);
    fsm_.set_state_available(true);

    if (auto_seed_desired_from_odom_ && !desired_input_received_) {
      controller_->set_desiredstate(*msg);
      fsm_.set_takeoff_callback_ready(true);
      ROS_INFO_THROTTLE(
          1.0,
          "[UavControlDemo][debug] desired auto-seeded from odom for takeoff gate");
    }

    ROS_INFO_THROTTLE(
        1.0,
        "[UavControlDemo][debug] odom in: topic stamp=%.3f frame='%s' child='%s' pos_z=%.3f",
        msg->header.stamp.toSec(), msg->header.frame_id.c_str(),
        msg->child_frame_id.c_str(), msg->pose.pose.position.z);
  }

  void desired_cb(const nav_msgs::OdometryConstPtr &msg) {
    desired_input_received_ = true;
    controller_->set_desiredstate(*msg);
    fsm_.set_takeoff_callback_ready(true);
    ROS_INFO_THROTTLE(
        1.0,
        "[UavControlDemo][debug] desired in: stamp=%.3f frame='%s' child='%s' pos=[%.3f %.3f %.3f]",
        msg->header.stamp.toSec(), msg->header.frame_id.c_str(),
        msg->child_frame_id.c_str(), msg->pose.pose.position.x,
        msg->pose.pose.position.y, msg->pose.pose.position.z);
  }

  void event_cb(const std_msgs::StringConstPtr &msg) {
    SunrayEvent event;
    if (parse_event_name(msg->data, &event)) {
      dispatch_event(event, msg->data, "topic");
    } else {
      ROS_WARN_THROTTLE(1.0, "[UavControlDemo] unsupported event: %s", msg->data.c_str());
    }
  }

  void update_timer_cb(const ros::TimerEvent &) { fsm_.update(); }

  void auto_takeoff_cb(const ros::TimerEvent &) {
    ROS_INFO("[UavControlDemo] auto takeoff trigger");
    dispatch_event(SunrayEvent::TAKEOFF_REQUEST, "TAKEOFF_REQUEST", "auto_takeoff");
  }

  bool parse_event_name(const std::string &name, SunrayEvent *event) const {
    if (event == nullptr) {
      return false;
    }
    if (name == "TAKEOFF_REQUEST") {
      *event = SunrayEvent::TAKEOFF_REQUEST;
      return true;
    }
    if (name == "LAND_REQUEST") {
      *event = SunrayEvent::LAND_REQUEST;
      return true;
    }
    if (name == "EMERGENCY_REQUEST") {
      *event = SunrayEvent::EMERGENCY_REQUEST;
      return true;
    }
    if (name == "ENTER_VELOCITY_CONTROL") {
      *event = SunrayEvent::ENTER_VELOCITY_CONTROL;
      return true;
    }
    if (name == "EXIT_CONTROL_MODE") {
      *event = SunrayEvent::EXIT_CONTROL_MODE;
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
    return false;
  }

  bool parse_test_script(const std::string &script) {
    timed_events_.clear();
    next_timed_event_index_ = 0U;
    if (script.empty()) {
      ROS_WARN("[UavControlDemo] test_sequence_script is empty");
      return false;
    }

    std::stringstream ss(script);
    std::string token;
    while (std::getline(ss, token, ',')) {
      token.erase(std::remove_if(token.begin(), token.end(), ::isspace), token.end());
      if (token.empty()) {
        continue;
      }
      const std::size_t at_pos = token.find('@');
      if (at_pos == std::string::npos || at_pos == 0U || at_pos + 1U >= token.size()) {
        ROS_WARN("[UavControlDemo] invalid test token: '%s' (expect EVENT@TIME)", token.c_str());
        return false;
      }

      const std::string event_name = token.substr(0, at_pos);
      const std::string time_s = token.substr(at_pos + 1U);
      SunrayEvent evt;
      if (!parse_event_name(event_name, &evt)) {
        ROS_WARN("[UavControlDemo] unknown event in test script: '%s'", event_name.c_str());
        return false;
      }

      char *end_ptr = nullptr;
      const double trigger_time = std::strtod(time_s.c_str(), &end_ptr);
      if (end_ptr == time_s.c_str() || trigger_time < 0.0) {
        ROS_WARN("[UavControlDemo] invalid trigger time in test script: '%s'", time_s.c_str());
        return false;
      }

      timed_events_.push_back(TimedEvent{evt, trigger_time, event_name});
    }

    if (timed_events_.empty()) {
      ROS_WARN("[UavControlDemo] no valid timed events in test script");
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
        ROS_INFO("[UavControlDemo] test sequence cycle completed, restart");
        next_timed_event_index_ = 0U;
        test_start_time_ = ros::Time::now();
      } else {
        ROS_INFO("[UavControlDemo] test sequence completed");
        test_timer_.stop();
      }
    }
  }

  void dispatch_event(SunrayEvent event, const std::string &name, const std::string &source) {
    const bool accepted = fsm_.dispatch(event);
    ROS_INFO("[UavControlDemo][event] source=%s event=%s accepted=%s",
             source.c_str(), name.c_str(), accepted ? "true" : "false");
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::string uav_ns_;

  Sunray_StateMachine fsm_;
  std::shared_ptr<uav_controller::PX4_Position_Controller> controller_;

  ros::Subscriber odom_sub_;
  ros::Subscriber desired_sub_;
  ros::Subscriber event_sub_;
  ros::Timer update_timer_;
  ros::Timer auto_takeoff_timer_;
  ros::Timer test_timer_;
  bool test_sequence_enable_{false};
  bool test_sequence_repeat_{false};
  bool auto_seed_desired_from_odom_{true};
  bool desired_input_received_{false};
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
