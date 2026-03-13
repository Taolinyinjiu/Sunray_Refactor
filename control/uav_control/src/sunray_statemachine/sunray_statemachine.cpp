#include "sunray_statemachine/sunray_statemachine.h"
#include "controller/px4_position_controller/px4_position_controller.h"
#include <algorithm>
namespace sunray_fsm {

Sunray_StateMachine::Sunray_StateMachine(ros::NodeHandle &nh)
    : nh_(nh), uav_ns_(resolve_uav_namespace()),
      fsm_current_state_(SunrayState::OFF), fsm_param_config_{},
      px4_data_reader_(nh_), px4_param_manager_(nh_), px4_arming_client_(),
      px4_set_mode_client_(), px4_offboard_retry_state_{},
      enable_offboard_control_(true), sunray_controller_(nullptr),
      external_odom_sub_(), latest_external_odom_{}, controller_update_timer_(),
      arbiter_() {
  // 1) 读取 FSM 参数（兼容旧/新 yaml key）
  nh_.param("uav_name", fsm_param_config_.uav_name, fsm_param_config_.uav_name);
  nh_.param("uav_id", fsm_param_config_.uav_id, fsm_param_config_.uav_id);
  nh_.param("mass_kg", fsm_param_config_.mass_kg, fsm_param_config_.mass_kg);
  nh_.param("gravity", fsm_param_config_.gravity, fsm_param_config_.gravity);

  nh_.param("odom_topic_name", fsm_param_config_.odom_topic_name,
            fsm_param_config_.odom_topic_name);
  nh_.param("fuse_odom_to_px4", fsm_param_config_.fuse_odom_to_px4,
            fsm_param_config_.fuse_odom_to_px4);
  nh_.param("fuse_odom_frequency", fsm_param_config_.fuse_odom_frequency_hz,
            fsm_param_config_.fuse_odom_frequency_hz);

  nh_.param("low_voltage", fsm_param_config_.low_voltage_v,
            fsm_param_config_.low_voltage_v);
  nh_.param("low_voltage_operate", fsm_param_config_.low_voltage_action,
            fsm_param_config_.low_voltage_action);
  nh_.param("control_with_no_rc", fsm_param_config_.control_with_no_rc,
            fsm_param_config_.control_with_no_rc);
  nh_.param("lost_with_rc", fsm_param_config_.lost_with_rc_action,
            fsm_param_config_.lost_with_rc_action);
  nh_.param("arm_with_code", fsm_param_config_.arm_with_code,
            fsm_param_config_.arm_with_code);
  nh_.param("takeoff_with_code", fsm_param_config_.takeoff_with_code,
            fsm_param_config_.takeoff_with_code);
  nh_.param("check_flip", fsm_param_config_.check_flip,
            fsm_param_config_.check_flip);

  nh_.param("electronic_fence/x_max", fsm_param_config_.fence_x_max,
            fsm_param_config_.fence_x_max);
  nh_.param("electronic_fence/x_min", fsm_param_config_.fence_x_min,
            fsm_param_config_.fence_x_min);
  nh_.param("electronic_fence/y_max", fsm_param_config_.fence_y_max,
            fsm_param_config_.fence_y_max);
  nh_.param("electronic_fence/y_min", fsm_param_config_.fence_y_min,
            fsm_param_config_.fence_y_min);
  nh_.param("electronic_fence/z_max", fsm_param_config_.fence_z_max,
            fsm_param_config_.fence_z_max);
  nh_.param("electronic_fence/z_min", fsm_param_config_.fence_z_min,
            fsm_param_config_.fence_z_min);

  nh_.param("msg_timeout/odom", fsm_param_config_.timeout_odom_s,
            fsm_param_config_.timeout_odom_s);
  nh_.param("msg_timeout/rc", fsm_param_config_.timeout_rc_s,
            fsm_param_config_.timeout_rc_s);
  nh_.param("msg_timeout/control_heartbeat",
            fsm_param_config_.timeout_control_hb_s,
            fsm_param_config_.timeout_control_hb_s);
  nh_.param("msg_timeout/imu", fsm_param_config_.timeout_imu_s,
            fsm_param_config_.timeout_imu_s);
  nh_.param("msg_timeout/battery", fsm_param_config_.timeout_battery_s,
            fsm_param_config_.timeout_battery_s);

  nh_.param("error_tolerance/pos_x", fsm_param_config_.error_tolerance_pos_x_m,
            fsm_param_config_.error_tolerance_pos_x_m);
  nh_.param("error_tolerance/pos_y", fsm_param_config_.error_tolerance_pos_y_m,
            fsm_param_config_.error_tolerance_pos_y_m);
  nh_.param("error_tolerance/pos_z", fsm_param_config_.error_tolerance_pos_z_m,
            fsm_param_config_.error_tolerance_pos_z_m);

  nh_.param("max_velocity/x_vel", fsm_param_config_.max_velocity_x_mps,
            fsm_param_config_.max_velocity_x_mps);
  nh_.param("max_velocity/y_vel", fsm_param_config_.max_velocity_y_mps,
            fsm_param_config_.max_velocity_y_mps);
  nh_.param("max_velocity/z_vel", fsm_param_config_.max_velocity_z_mps,
            fsm_param_config_.max_velocity_z_mps);
  nh_.param("max_velocity_with_rc/x_vel",
            fsm_param_config_.max_velocity_with_rc_x_mps,
            fsm_param_config_.max_velocity_with_rc_x_mps);
  nh_.param("max_velocity_with_rc/y_vel",
            fsm_param_config_.max_velocity_with_rc_y_mps,
            fsm_param_config_.max_velocity_with_rc_y_mps);
  nh_.param("max_velocity_with_rc/z_vel",
            fsm_param_config_.max_velocity_with_rc_z_mps,
            fsm_param_config_.max_velocity_with_rc_z_mps);
  nh_.param("tilt_angle_max", fsm_param_config_.tilt_angle_max_deg,
            fsm_param_config_.tilt_angle_max_deg);

  nh_.param("land_type", fsm_param_config_.land_type,
            fsm_param_config_.land_type);
  nh_.param("land_max_vel_mps", fsm_param_config_.land_max_vel_mps,
            fsm_param_config_.land_max_vel_mps);
  nh_.param("land_max_velocity", fsm_param_config_.land_max_vel_mps,
            fsm_param_config_.land_max_vel_mps);

  nh_.param("controller_type", fsm_param_config_.controller_type,
            fsm_param_config_.controller_type);
  nh_.param("controller_types", fsm_param_config_.controller_type,
            fsm_param_config_.controller_type);
  nh_.param("controller_update_hz", fsm_param_config_.controller_update_hz,
            fsm_param_config_.controller_update_hz);
  nh_.param("controller_update_frequency",
            fsm_param_config_.controller_update_hz,
            fsm_param_config_.controller_update_hz);
  nh_.param("takeoff_height_m", fsm_param_config_.takeoff_height_m,
            fsm_param_config_.takeoff_height_m);
  nh_.param("takeoff_relative_height", fsm_param_config_.takeoff_height_m,
            fsm_param_config_.takeoff_height_m);
  nh_.param("takeoff_max_vel_mps", fsm_param_config_.takeoff_max_vel_mps,
            fsm_param_config_.takeoff_max_vel_mps);
  nh_.param("takeoff_max_velocity", fsm_param_config_.takeoff_max_vel_mps,
            fsm_param_config_.takeoff_max_vel_mps);
  nh_.param("/fsm/enable_offboard_control", enable_offboard_control_,
            enable_offboard_control_);
  nh_.param("fsm/enable_offboard_control", enable_offboard_control_,
            enable_offboard_control_);

  nh_.param("set_mode_retry_interval_s",
            px4_offboard_retry_state_.set_mode_retry_interval_s,
            px4_offboard_retry_state_.set_mode_retry_interval_s);
  nh_.param("arm_retry_interval_s",
            px4_offboard_retry_state_.arm_retry_interval_s,
            px4_offboard_retry_state_.arm_retry_interval_s);

  std::string external_odom_topic = fsm_param_config_.odom_topic_name;
  if (external_odom_topic.empty()) {
    external_odom_topic = uav_ns_.empty() ? "/sunray_odom_in"
                                          : ("/" + uav_ns_ + "/sunray_odom_in");
  }
  nh_.param("/fsm/odom_topic", external_odom_topic, external_odom_topic);
  nh_.param("fsm/odom_topic", external_odom_topic, external_odom_topic);
  fsm_param_config_.odom_topic_name = external_odom_topic;

  // 2) 初始化 MAVROS service client
  const std::string mavros_ns =
      uav_ns_.empty() ? "/mavros" : ("/" + uav_ns_ + "/mavros");
  px4_arming_client_ =
      nh_.serviceClient<mavros_msgs::CommandBool>(mavros_ns + "/cmd/arming");
  px4_set_mode_client_ =
      nh_.serviceClient<mavros_msgs::SetMode>(mavros_ns + "/set_mode");
  external_odom_sub_ = nh_.subscribe(
      external_odom_topic, 20, &Sunray_StateMachine::external_odom_cb, this);

  // 3) 初始化仲裁器
  if (!arbiter_.init(nh_)) {
    ROS_WARN("[SunrayFSM] arbiter init failed");
  }

  // 4) 注册控制器 + 启动周期 update 定时器
  (void)register_controller(fsm_param_config_.controller_type);

  const double hz = std::max(50.0, fsm_param_config_.controller_update_hz);
  controller_update_timer_ =
      nh_.createTimer(ros::Duration(1.0 / hz),
                      &Sunray_StateMachine::controller_update_timer_cb, this);

  ROS_INFO("[SunrayFSM] init done, uav_ns='%s', state=OFF, offboard_gate=%s, "
           "odom='%s'",
           uav_ns_.c_str(), enable_offboard_control_ ? "true" : "false",
           external_odom_topic.c_str());
}

bool Sunray_StateMachine::register_controller(int controller_types) {
  // controller_type 约定（与 yaml 对齐）:
  // 0: PX4 position controller（已实现）
  // 1: Sunray attitude controller（预留）
  // 2: controller slot #2（预留）
  // 3: controller slot #3（预留）
  std::shared_ptr<uav_control::Base_Controller> selected_controller;

  switch (controller_types) {
  case 0:
    selected_controller = std::make_shared<uav_control::Position_Controller>();
    break;
  case 1:
    ROS_WARN("[SunrayFSM] controller_type=1 reserved, fallback to type=0 "
             "(PX4 position controller)");
    selected_controller = std::make_shared<uav_control::Position_Controller>();
    break;
  case 2:
    ROS_WARN("[SunrayFSM] controller_type=2 reserved, fallback to type=0 "
             "(PX4 position controller)");
    selected_controller = std::make_shared<uav_control::Position_Controller>();
    break;
  case 3:
    ROS_WARN("[SunrayFSM] controller_type=3 reserved, fallback to type=0 "
             "(PX4 position controller)");
    selected_controller = std::make_shared<uav_control::Position_Controller>();
    break;
  default:
    ROS_WARN("[SunrayFSM] unknown controller_type=%d, fallback to type=0 "
             "(PX4 position controller)",
             controller_types);
    selected_controller = std::make_shared<uav_control::Position_Controller>();
    break;
  }

  if (!selected_controller) {
    ROS_ERROR("[SunrayFSM] register_controller failed: create controller "
              "instance failed (type=%d)",
              controller_types);
    return false;
  }

  sunray_controller_ = selected_controller;
  ROS_INFO("[SunrayFSM] register global controller success, type=%d",
           controller_types);
  return true;
}

bool Sunray_StateMachine::handle_event(SunrayEvent event) {
  const SunrayState prev = fsm_current_state_;

  switch (fsm_current_state_) {
  case SunrayState::OFF:
    if (event == SunrayEvent::TAKEOFF_REQUEST && can_takeoff()) {
      return transition_to(SunrayState::TAKEOFF);
    }
    break;

  case SunrayState::TAKEOFF:
    if (event == SunrayEvent::TAKEOFF_COMPLETED) {
      return transition_to(SunrayState::HOVER);
    }
    if (event == SunrayEvent::WATCHDOG_ERROR ||
        event == SunrayEvent::EMERGENCY_REQUEST) {
      return transition_to(SunrayState::EMERGENCY_LAND);
    }
    break;

  case SunrayState::HOVER:
    if (event == SunrayEvent::LAND_REQUEST) {
      return transition_to(SunrayState::LAND);
    }
    if (event == SunrayEvent::RETURN_REQUEST) {
      return transition_to(SunrayState::RETURN);
    }
    if (event == SunrayEvent::WATCHDOG_ERROR ||
        event == SunrayEvent::EMERGENCY_REQUEST) {
      return transition_to(SunrayState::EMERGENCY_LAND);
    }
    if (event == SunrayEvent::ENTER_POSITION_CONTROL) {
      return transition_to(SunrayState::POSITION_CONTROL);
    }
    if (event == SunrayEvent::ENTER_VELOCITY_CONTROL) {
      return transition_to(SunrayState::VELOCITY_CONTROL);
    }
    if (event == SunrayEvent::ENTER_ATTITUDE_CONTROL) {
      return transition_to(SunrayState::ATTITUDE_CONTROL);
    }
    if (event == SunrayEvent::ENTER_COMPLEX_CONTROL) {
      return transition_to(SunrayState::COMPLEX_CONTROL);
    }
    if (event == SunrayEvent::ENTER_TRAJECTORY_CONTROL) {
      return transition_to(SunrayState::TRAJECTORY_CONTROL);
    }
    break;

  case SunrayState::RETURN:
    if (event == SunrayEvent::RETURN_COMPLETED) {
      return transition_to(SunrayState::HOVER);
    }
    if (event == SunrayEvent::LAND_REQUEST) {
      return transition_to(SunrayState::LAND);
    }
    if (event == SunrayEvent::WATCHDOG_ERROR ||
        event == SunrayEvent::EMERGENCY_REQUEST) {
      return transition_to(SunrayState::EMERGENCY_LAND);
    }
    break;

  case SunrayState::LAND:
    if (event == SunrayEvent::LAND_COMPLETED) {
      return transition_to(SunrayState::OFF);
    }
    if (event == SunrayEvent::WATCHDOG_ERROR ||
        event == SunrayEvent::EMERGENCY_REQUEST) {
      return transition_to(SunrayState::EMERGENCY_LAND);
    }
    break;

  case SunrayState::EMERGENCY_LAND:
    if (event == SunrayEvent::EMERGENCY_COMPLETED) {
      return transition_to(SunrayState::OFF);
    }
    break;

  case SunrayState::POSITION_CONTROL:
    if (event == SunrayEvent::POSITION_COMPLETED) {
      return transition_to(SunrayState::HOVER);
    }
    if (event == SunrayEvent::LAND_REQUEST) {
      return transition_to(SunrayState::LAND);
    }
    if (event == SunrayEvent::RETURN_REQUEST) {
      return transition_to(SunrayState::RETURN);
    }
    if (event == SunrayEvent::WATCHDOG_ERROR ||
        event == SunrayEvent::EMERGENCY_REQUEST) {
      return transition_to(SunrayState::EMERGENCY_LAND);
    }
    break;

  case SunrayState::VELOCITY_CONTROL:
    if (event == SunrayEvent::VELOCITY_COMPLETED) {
      return transition_to(SunrayState::HOVER);
    }
    if (event == SunrayEvent::LAND_REQUEST) {
      return transition_to(SunrayState::LAND);
    }
    if (event == SunrayEvent::RETURN_REQUEST) {
      return transition_to(SunrayState::RETURN);
    }
    if (event == SunrayEvent::WATCHDOG_ERROR ||
        event == SunrayEvent::EMERGENCY_REQUEST) {
      return transition_to(SunrayState::EMERGENCY_LAND);
    }
    break;

  case SunrayState::ATTITUDE_CONTROL:
    if (event == SunrayEvent::ATTITUDE_COMPLETED) {
      return transition_to(SunrayState::HOVER);
    }
    if (event == SunrayEvent::LAND_REQUEST) {
      return transition_to(SunrayState::LAND);
    }
    if (event == SunrayEvent::RETURN_REQUEST) {
      return transition_to(SunrayState::RETURN);
    }
    if (event == SunrayEvent::WATCHDOG_ERROR ||
        event == SunrayEvent::EMERGENCY_REQUEST) {
      return transition_to(SunrayState::EMERGENCY_LAND);
    }
    break;

  case SunrayState::COMPLEX_CONTROL:
    if (event == SunrayEvent::COMPLEX_COMPLETED) {
      return transition_to(SunrayState::HOVER);
    }
    if (event == SunrayEvent::LAND_REQUEST) {
      return transition_to(SunrayState::LAND);
    }
    if (event == SunrayEvent::RETURN_REQUEST) {
      return transition_to(SunrayState::RETURN);
    }
    if (event == SunrayEvent::WATCHDOG_ERROR ||
        event == SunrayEvent::EMERGENCY_REQUEST) {
      return transition_to(SunrayState::EMERGENCY_LAND);
    }
    break;

  case SunrayState::TRAJECTORY_CONTROL:
    if (event == SunrayEvent::TRAJECTORY_COMPLETED) {
      return transition_to(SunrayState::HOVER);
    }
    if (event == SunrayEvent::LAND_REQUEST) {
      return transition_to(SunrayState::LAND);
    }
    if (event == SunrayEvent::RETURN_REQUEST) {
      return transition_to(SunrayState::RETURN);
    }
    if (event == SunrayEvent::WATCHDOG_ERROR ||
        event == SunrayEvent::EMERGENCY_REQUEST) {
      return transition_to(SunrayState::EMERGENCY_LAND);
    }
    break;
  }

  ROS_WARN("[SunrayFSM] ignore event %s at state %s", to_string(event),
           to_string(prev));
  return false;
}

void Sunray_StateMachine::update() {
  if (!check_health()) {
    if (fsm_current_state_ != SunrayState::EMERGENCY_LAND) {
      transition_to(SunrayState::EMERGENCY_LAND);
    }
  }

  const std::shared_ptr<uav_control::Base_Controller> controller =
      get_controller();
  if (!controller) {
    ROS_WARN_THROTTLE(1.0, "[SunrayFSM] no controller registered");
    return;
  }

  const px4_data_types::SystemState px4_state =
      px4_data_reader_.get_system_state();
  (void)controller->set_px4_arm_state(enable_offboard_control_ ? px4_state.armed
                                                               : true);

  const ros::Time now = ros::Time::now();
  const bool external_odom_fresh =
      latest_external_odom_.isValid() &&
      !latest_external_odom_.timestamp.isZero() &&
      (now - latest_external_odom_.timestamp).toSec() <=
          fsm_param_config_.timeout_odom_s;
  if (!external_odom_fresh) {
    ROS_WARN_THROTTLE(
        1.0,
        "[SunrayFSM] external odom unavailable or timeout (timeout=%.3fs), "
        "skip control update",
        fsm_param_config_.timeout_odom_s);
    return;
  }
  (void)controller->set_current_odom(latest_external_odom_);

  if (requires_offboard() && !ensure_offboard_and_arm()) {
    ROS_WARN_THROTTLE(
        1.0,
        "[SunrayFSM] waiting for OFFBOARD/ARM before effective flight control");
  }

  switch (fsm_current_state_) {
  case SunrayState::TAKEOFF:
    (void)controller->set_takeoff_mode(fsm_param_config_.takeoff_height_m,
                                       fsm_param_config_.takeoff_max_vel_mps);
    break;
  case SunrayState::LAND:
    (void)controller->set_land_mode();
    break;
  case SunrayState::EMERGENCY_LAND:
    (void)controller->set_emergency_mode();
    break;
  default:
    break;
  }

  const uav_control::ControllerOutput control_output = controller->update();
  const bool has_effective_output =
      control_output.is_channel_enabled(
          uav_control::ControllerOutputMask::POSITION) ||
      control_output.is_channel_enabled(
          uav_control::ControllerOutputMask::VELOCITY) ||
      control_output.is_channel_enabled(
          uav_control::ControllerOutputMask::ATTITUDE) ||
      control_output.is_channel_enabled(
          uav_control::ControllerOutputMask::THRUST);

  if (!has_effective_output) {
    if (fsm_current_state_ == SunrayState::OFF) {
      return;
    }
    ROS_WARN_THROTTLE(
        1.0, "[SunrayFSM] controller produced empty output at state=%s",
        to_string(fsm_current_state_));
    return;
  }

  arbiter_.set_fsm_state(fsm_current_state_);
  arbiter_.set_uav_state(controller->get_current_state());
  if (fsm_current_state_ == SunrayState::EMERGENCY_LAND) {
    arbiter_.submit(
        uav_control::Sunray_Control_Arbiter::ControlSource::EMERGENCY,
        control_output, ros::Time::now(), 255U);
  } else {
    arbiter_.submit(
        uav_control::Sunray_Control_Arbiter::ControlSource::EXTERNAL,
        control_output, ros::Time::now(), 100U);
  }
  (void)arbiter_.arbitrate_and_publish();
}

void Sunray_StateMachine::controller_update_timer_cb(const ros::TimerEvent &) {
  update();
}

void Sunray_StateMachine::external_odom_cb(
    const nav_msgs::Odometry::ConstPtr &msg) {
  if (!msg) {
    return;
  }
  const uav_control::UAVStateEstimate odom_state(*msg);
  if (!odom_state.isValid()) {
    ROS_WARN_THROTTLE(1.0,
                      "[SunrayFSM] ignore invalid external odom frame='%s'",
                      msg->header.frame_id.c_str());
    return;
  }
  latest_external_odom_ = odom_state;
}

std::string Sunray_StateMachine::resolve_uav_namespace() const {
  std::string key;
  std::string ns;
  if (nh_.searchParam("uav_ns", key) && nh_.getParam(key, ns) && !ns.empty()) {
    if (!ns.empty() && ns.front() == '/') {
      return ns.substr(1);
    }
    return ns;
  }

  std::string name;
  int id = 0;
  bool ok_name = false;
  bool ok_id = false;
  if (nh_.searchParam("uav_name", key)) {
    ok_name = nh_.getParam(key, name) && !name.empty();
  }
  if (nh_.searchParam("uav_id", key)) {
    ok_id = nh_.getParam(key, id);
  }
  if (ok_name && ok_id) {
    return name + std::to_string(id);
  }
  return "";
}

bool Sunray_StateMachine::requires_offboard() const {
  if (!enable_offboard_control_) {
    return false;
  }

  switch (fsm_current_state_) {
  case SunrayState::TAKEOFF:
  case SunrayState::HOVER:
  case SunrayState::RETURN:
  case SunrayState::LAND:
  case SunrayState::EMERGENCY_LAND:
  case SunrayState::POSITION_CONTROL:
  case SunrayState::VELOCITY_CONTROL:
  case SunrayState::ATTITUDE_CONTROL:
  case SunrayState::COMPLEX_CONTROL:
  case SunrayState::TRAJECTORY_CONTROL:
    return true;
  case SunrayState::OFF:
  default:
    return false;
  }
}

bool Sunray_StateMachine::ensure_offboard_and_arm() {
  if (!enable_offboard_control_) {
    return true;
  }

  const px4_data_types::SystemState px4_state =
      px4_data_reader_.get_system_state();

  if (!px4_state.connected) {
    ROS_WARN_THROTTLE(1.0, "[SunrayFSM] PX4 is not connected");
    return false;
  }

  if (!px4_set_mode_client_.isValid() || !px4_arming_client_.isValid()) {
    ROS_WARN_THROTTLE(1.0, "[SunrayFSM] mavros service clients are not ready");
    return false;
  }

  const ros::Time now = ros::Time::now();

  if (px4_state.flight_mode != px4_data_types::FlightMode::kOffboard) {
    if (px4_offboard_retry_state_.last_set_mode_req_time.isZero() ||
        (now - px4_offboard_retry_state_.last_set_mode_req_time).toSec() >=
            px4_offboard_retry_state_.set_mode_retry_interval_s) {
      mavros_msgs::SetMode mode_cmd;
      mode_cmd.request.custom_mode = "OFFBOARD";
      if (px4_set_mode_client_.call(mode_cmd) && mode_cmd.response.mode_sent) {
        ROS_INFO("[SunrayFSM] OFFBOARD mode request sent");
      } else {
        ROS_WARN_THROTTLE(1.0, "[SunrayFSM] OFFBOARD mode request failed");
      }
      px4_offboard_retry_state_.last_set_mode_req_time = now;
    }
    return false;
  }

  if (!px4_state.armed) {
    if (px4_offboard_retry_state_.last_arm_req_time.isZero() ||
        (now - px4_offboard_retry_state_.last_arm_req_time).toSec() >=
            px4_offboard_retry_state_.arm_retry_interval_s) {
      mavros_msgs::CommandBool arm_cmd;
      arm_cmd.request.value = true;
      if (px4_arming_client_.call(arm_cmd) && arm_cmd.response.success) {
        ROS_INFO("[SunrayFSM] ARM request sent");
      } else {
        ROS_WARN_THROTTLE(1.0, "[SunrayFSM] ARM request failed");
      }
      px4_offboard_retry_state_.last_arm_req_time = now;
    }
    return false;
  }

  return true;
}

SunrayState Sunray_StateMachine::get_current_state() const {
  return fsm_current_state_;
}

const char *Sunray_StateMachine::to_string(SunrayState state) {
  switch (state) {
  case SunrayState::OFF:
    return "OFF";
  case SunrayState::TAKEOFF:
    return "TAKEOFF";
  case SunrayState::HOVER:
    return "HOVER";
  case SunrayState::RETURN:
    return "RETURN";
  case SunrayState::LAND:
    return "LAND";
  case SunrayState::EMERGENCY_LAND:
    return "EMERGENCY_LAND";
  case SunrayState::POSITION_CONTROL:
    return "POSITION_CONTROL";
  case SunrayState::VELOCITY_CONTROL:
    return "VELOCITY_CONTROL";
  case SunrayState::ATTITUDE_CONTROL:
    return "ATTITUDE_CONTROL";
  case SunrayState::COMPLEX_CONTROL:
    return "COMPLEX_CONTROL";
  case SunrayState::TRAJECTORY_CONTROL:
    return "TRAJECTORY_CONTROL";
  default:
    return "UNKNOWN_STATE";
  }
}

const char *Sunray_StateMachine::to_string(SunrayEvent event) {
  switch (event) {
  case SunrayEvent::TAKEOFF_REQUEST:
    return "TAKEOFF_REQUEST";
  case SunrayEvent::TAKEOFF_COMPLETED:
    return "TAKEOFF_COMPLETED";
  case SunrayEvent::LAND_REQUEST:
    return "LAND_REQUEST";
  case SunrayEvent::LAND_COMPLETED:
    return "LAND_COMPLETED";
  case SunrayEvent::EMERGENCY_REQUEST:
    return "EMERGENCY_REQUEST";
  case SunrayEvent::EMERGENCY_COMPLETED:
    return "EMERGENCY_COMPLETED";
  case SunrayEvent::RETURN_REQUEST:
    return "RETURN_REQUEST";
  case SunrayEvent::RETURN_COMPLETED:
    return "RETURN_COMPLETED";
  case SunrayEvent::WATCHDOG_ERROR:
    return "WATCHDOG_ERROR";
  case SunrayEvent::ENTER_POSITION_CONTROL:
    return "ENTER_POSITION_CONTROL";
  case SunrayEvent::ENTER_VELOCITY_CONTROL:
    return "ENTER_VELOCITY_CONTROL";
  case SunrayEvent::ENTER_ATTITUDE_CONTROL:
    return "ENTER_ATTITUDE_CONTROL";
  case SunrayEvent::ENTER_COMPLEX_CONTROL:
    return "ENTER_COMPLEX_CONTROL";
  case SunrayEvent::ENTER_TRAJECTORY_CONTROL:
    return "ENTER_TRAJECTORY_CONTROL";
  case SunrayEvent::POSITION_COMPLETED:
    return "POSITION_COMPLETED";
  case SunrayEvent::VELOCITY_COMPLETED:
    return "VELOCITY_COMPLETED";
  case SunrayEvent::ATTITUDE_COMPLETED:
    return "ATTITUDE_COMPLETED";
  case SunrayEvent::COMPLEX_COMPLETED:
    return "COMPLEX_COMPLETED";
  case SunrayEvent::TRAJECTORY_COMPLETED:
    return "TRAJECTORY_COMPLETED";
  default:
    return "UNKNOWN_EVENT";
  }
}

bool Sunray_StateMachine::check_health_preflight() {
  if (!sunray_controller_) {
    ROS_WARN(
        "[SunrayFSM] preflight check failed: controller is not registered");
    return false;
  }

  if (!validate_offstage_odometry_source()) {
    ROS_WARN("[SunrayFSM] preflight check failed: odometry source is invalid "
             "in OFF stage");
    return false;
  }

  return true;
}

bool Sunray_StateMachine::check_health() {
  // 当前为占位检查：
  // 1) 起飞前由 check_health_preflight() 执行完整门控；
  // 2) 飞行中后续可扩展里程计稳定性、控制器输出饱和等检查。
  if (fsm_current_state_ == SunrayState::OFF) {
    return true;
  }

  return sunray_controller_ != nullptr;
}

bool Sunray_StateMachine::can_takeoff() const {
  return validate_offstage_odometry_source() &&
         static_cast<bool>(sunray_controller_);
}

bool Sunray_StateMachine::transition_to(SunrayState next_state) {
  if (fsm_current_state_ == next_state) {
    return true;
  }

  if (fsm_current_state_ == SunrayState::OFF &&
      next_state == SunrayState::TAKEOFF && !check_health_preflight()) {
    ROS_WARN("[SunrayFSM] transition blocked: OFF -> TAKEOFF preflight check "
             "failed");
    return false;
  }

  ROS_INFO("[SunrayFSM] transition: %s -> %s", to_string(fsm_current_state_),
           to_string(next_state));
  fsm_current_state_ = next_state;
  return true;
}

std::shared_ptr<uav_control::Base_Controller>
Sunray_StateMachine::get_controller() const {
  return sunray_controller_;
}

} // namespace sunray_fsm
