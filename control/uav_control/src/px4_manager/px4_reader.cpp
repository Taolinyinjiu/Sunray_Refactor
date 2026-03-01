#include "px4_manager/px4_reader.h"

#include <vector>

void reader_list_::enable_all() {
  read_system_state = true;
  read_ekf2_state = true;
  read_flow_state = true;
  read_localpose = true;
  read_localvel = true;
  read_bodypose = true;
  read_bodyvel = true;
  read_ekf2_param = true;
  read_attitude_param = true;
  read_velocity_param = true;
  read_position_param = true;
}

void reader_list_::disable_all() {
  read_system_state = false;
  read_ekf2_state = false;
  read_flow_state = false;
  read_localpose = false;
  read_localvel = false;
  read_bodypose = false;
  read_bodyvel = false;
  read_ekf2_param = false;
  read_attitude_param = false;
  read_velocity_param = false;
  read_position_param = false;
}

void PX4_Reader::resetStateDefaults() {
  uav_id = 0;
  uav_name.clear();

  system_state = {};
  system_state.uav_id = 0;
  system_state.connected = false;
  system_state.armed = false;
  system_state.rc_input = false;
  system_state.system_load = 0.0f;
  system_state.voltage = 0.0f;
  system_state.current = 0.0f;
  system_state.percent = 0.0f;
  system_state.flight_mode = px4_data::FlightMode::UNDEFINED;
  system_state.landed_state = px4_data::LandedState::UNDEFINED;

  ekf2_state = {};
  ekf2_state.state_codes = 0;
  ekf2_state.allow_stabilize = false;
  ekf2_state.allow_altitude = false;
  ekf2_state.allow_position = false;

  opflow_raw = px4_data::opflow_raw_();
  opflow_buffer_.clear();

  local_pose.position.setZero();
  local_pose.orientation = Eigen::Quaterniond::Identity();
  local_velocity.linear.setZero();
  local_velocity.angular.setZero();

  local_odom.timestamp = ros::Time(0);
  local_odom.position.setZero();
  local_odom.orientation = Eigen::Quaterniond::Identity();
  local_odom.linear.setZero();
  local_odom.angular.setZero();

  body_pose.position.setZero();
  body_pose.orientation = Eigen::Quaterniond::Identity();
  body_velocity.linear.setZero();
  body_velocity.angular.setZero();

  ekf2_param = {};
  attitude_param = {};
  velocity_param = {};
  position_param = {};
}

void PX4_Reader::initServiceClients() {
  param_get_client_ =
      nh_.serviceClient<mavros_msgs::ParamGet>(uav_name + "/mavros/param/get");
}

bool PX4_Reader::fetchParamInt(const std::string& param_name, int& out_value) {
  if (!param_get_client_) {
    ROS_WARN("param_get service client is not initialized.");
    return false;
  }

  if (!param_get_client_.exists()) {
    param_get_client_.waitForExistence(ros::Duration(1.0));
  }

  mavros_msgs::ParamGet srv;
  srv.request.param_id = param_name;
  if (!param_get_client_.call(srv)) {
    ROS_WARN("Failed to call param get service for %s", param_name.c_str());
    return false;
  }

  if (!srv.response.success) {
    ROS_WARN("PX4 param %s not found or rejected by FCU", param_name.c_str());
    return false;
  }

  out_value = static_cast<int>(srv.response.value.integer);
  return true;
}

bool PX4_Reader::fetchParamFloat(const std::string& param_name,
                                 float& out_value) {
  if (!param_get_client_) {
    ROS_WARN("param_get service client is not initialized.");
    return false;
  }

  if (!param_get_client_.exists()) {
    param_get_client_.waitForExistence(ros::Duration(1.0));
  }

  mavros_msgs::ParamGet srv;
  srv.request.param_id = param_name;
  if (!param_get_client_.call(srv)) {
    ROS_WARN("Failed to call param get service for %s", param_name.c_str());
    return false;
  }

  if (!srv.response.success) {
    ROS_WARN("PX4 param %s not found or rejected by FCU", param_name.c_str());
    return false;
  }

  out_value = static_cast<float>(srv.response.value.real);
  return true;
}

// 实现以一个函数，入口参数为reader_list和rosnode，通过reader_list来实现将订阅者注册到ros环境中，由于订阅者都是类的成员，因此这个函数也放进类中
void PX4_Reader::initSubscribers(ros::NodeHandle& nh,
                                 reader_list_ enable_list_) {
  // 定义订阅表
  std::vector<SubscribeEntry> entries = {
      // 标准状态信息
      {&reader_list_::read_system_state, &PX4_Reader::state_sub_,
       [&]() {
         return nh.subscribe<mavros_msgs::State>(
             uav_name + "/mavros/state", 10, &PX4_Reader::stateCallback, this);
       }},
      // 扩展状态信息
      {&reader_list_::read_system_state, &PX4_Reader::exstate_sub_,
       [&]() {
         return nh.subscribe<mavros_msgs::ExtendedState>(
             uav_name + "/mavros/extended_state", 10,
             &PX4_Reader::exstateCallback, this);
       }},
      // 系统状态信息
      {&reader_list_::read_system_state, &PX4_Reader::sys_sub_,
       [&]() {
         return nh.subscribe<mavros_msgs::SysStatus>(
             uav_name + "/mavros/sys_status", 10, &PX4_Reader::sysCallback,
             this);
       }},
      // ekf2 估计器状态信息
      {&reader_list_::read_ekf2_state, &PX4_Reader::ekf2status_sub_,
       [&]() {
         return nh.subscribe<mavros_msgs::EstimatorStatus>(
             uav_name + "/mavros/estimator_status", 10,
             &PX4_Reader::ekf2statusCallback, this);
       }},
      // 光流数据
      {&reader_list_::read_flow_state, &PX4_Reader::opflow_sub_,
       [&]() {
         return nh.subscribe<mavros_msgs::OpticalFlowRad>(
             uav_name + "/mavros/px4flow/optical_flow_rad", 10,
             &PX4_Reader::opflowCallback, this);
       }},
      // local系下的里程计
      {&reader_list_::read_localpose, &PX4_Reader::local_odom_sub_,
       [&]() {
         return nh.subscribe<nav_msgs::Odometry>(
             uav_name + "/mavros/local_position/odom", 10,
             &PX4_Reader::localOdomCallback, this);
       }},
      // local系下的速度
      {&reader_list_::read_localvel, &PX4_Reader::local_vel_sub_,
       [&]() {
         return nh.subscribe<geometry_msgs::TwistStamped>(
             uav_name + "/mavros/local_position/velocity_local", 10,
             &PX4_Reader::localVelCallback, this);
       }},
      // body系下的姿态
      {&reader_list_::read_bodypose, &PX4_Reader::body_att_sub_,
       [&]() {
         return nh.subscribe<sensor_msgs::Imu>(uav_name + "/mavros/imu/data",
                                               10, &PX4_Reader::bodyAttCallback,
                                               this);
       }},
      // local系下的速度
      {&reader_list_::read_bodyvel, &PX4_Reader::body_vel_sub_,
       [&]() {
         return nh.subscribe<geometry_msgs::TwistStamped>(
             uav_name + "/mavros/local_position/velocity_body", 10,
             &PX4_Reader::bodyVelCallback, this);
       }},
  };

  // 遍历订阅表，根据用户传入的enable_list_自动执行订阅
  for (const auto& entries : entries) {
    // 检查enable_list中对应的参数是否为true
    if (enable_list_.*(entries.flag)) {
      // 执行lambda 表达式并将返回的Subscriber 负值给当前实例对应的成员变量
      this->*(entries.handle) = entries.make();
    }
  }
}

// 默认构造函数，全订阅
PX4_Reader::PX4_Reader(ros::NodeHandle& nh) {
  resetStateDefaults();
  nh_ = nh;
  // 首先读取全局参数，通过uav_id参数与uav_name是否存在来进行判断，当该参数存在时，认为参数文件正常加载
  try {
    // 检查uav_id参数是否存在
    if (!nh.getParam("uav_id", uav_id)) {
      throw std::runtime_error(
          "无法读取参数 'uav_id'。请检查 YAML "
          "配置文件是否正确加载到 Parameter Server。");
    }
    if (!nh.getParam("uav_name", uav_name)) {
      throw std::runtime_error(
          "无法读取参数 'uav_name'。请检查 YAML "
          "配置文件是否正确加载到 Parameter Server。");
    }
  } catch (const std::exception& e) {
    // 如果不存在，则抛出异常
    ROS_FATAL("PX4_Reader初始化失败: %s", e.what());
    // 继续向上抛出异常，如果main中没有异常处理，则节点结束运行
    throw;
  }

  // 运行到这里，认为参数文件正确加载，开始初始化各项订阅者以及回调函数，该构造函数没有reader_list因此默认为全部订阅
  uav_name = "/" + uav_name + std::to_string(uav_id);
  // 构造一个全订阅的enable_list
  reader_list_ enable_list;
  enable_list.enable_all();
  initServiceClients();
  initSubscribers(nh, enable_list);
};

// 带有reader_list的构造函数，选择性的订阅
PX4_Reader::PX4_Reader(ros::NodeHandle& nh, reader_list_ enable_list_) {
  resetStateDefaults();
  nh_ = nh;
  // 首先读取全局参数，通过uav_id参数与uav_name是否存在来进行判断，当该参数存在时，认为参数文件正常加载
  try {
    // 检查uav_id参数是否存在
    if (!nh.getParam("uav_id", uav_id)) {
      throw std::runtime_error(
          "无法读取参数 'uav_id'。请检查 YAML "
          "配置文件是否正确加载到 Parameter Server。");
    }
    if (!nh.getParam("uav_name", uav_name)) {
      throw std::runtime_error(
          "无法读取参数 'uav_name'。请检查 YAML "
          "配置文件是否正确加载到 Parameter Server。");
    }
  } catch (const std::exception& e) {
    // 如果不存在，则抛出异常
    ROS_FATAL("PX4_Reader初始化失败: %s", e.what());
    // 继续向上抛出异常，如果main中没有异常处理，则节点结束运行
    throw;
  }

  // 运行到这里，认为参数文件正确加载，开始初始化各项订阅者以及回调函数，该构造函数没有reader_list因此默认为全部订阅
  uav_name = "/" + uav_name + std::to_string(uav_id);
  // 传入enable_list
  initServiceClients();
  initSubscribers(nh, enable_list_);
};
// 析构函数，由于ROS中的订阅者会自动调用shutdown方法，因此PX4_Reader中析构函数没有实现的必要
PX4_Reader::~PX4_Reader() {};

// 请注意,订阅者的回调函数放置在px4_reader_callback.cpp中进行实现

px4_data::FlightMode PX4_Reader::flightmode_fromString(
    const std::string& mode) {
  if (mode == "OFFBOARD") return px4_data::FlightMode::OFFBOARD;
  if (mode == "POSCTL") return px4_data::FlightMode::POSCTL;
  if (mode == "ALTCTL") return px4_data::FlightMode::ALTCTL;
  if (mode == "AUTO.TAKEOFF") return px4_data::FlightMode::AUTO_TAKEOFF;
  if (mode == "AUTO.LAND") return px4_data::FlightMode::AUTO_LAND;
  if (mode == "AUTO.RTL") return px4_data::FlightMode::AUTO_RTL;
  if (mode == "AUTO.MISSION") return px4_data::FlightMode::AUTO_MISSION;
  if (mode == "AUTO.LOITER") return px4_data::FlightMode::AUTO_LOITER;
  if (mode == "MANUAL") return px4_data::FlightMode::MANUAL;
  if (mode == "STABILIZED") return px4_data::FlightMode::STABILIZED;
  if (mode == "ACRO") return px4_data::FlightMode::ACRO;
  return px4_data::FlightMode::UNDEFINED;
};

px4_data::system_state_ PX4_Reader::get_system_state(void) {
  std::lock_guard<std::mutex> lock(system_state_mtx);
  px4_data::system_state_ state = system_state;
  state.uav_id = static_cast<uint8_t>(uav_id);
  return state;
}

px4_data::opflow_state_ PX4_Reader::get_flow_state(void) {
  px4_data::opflow_state_ state;

  // 先尝试读取最新样本；若缓冲区为空则直接返回默认无效状态
  opflow_sample latest_sample;
  if (!opflow_buffer_.latest(latest_sample)) {
    return state;
  }

  state.timestamp = latest_sample.timestamp;
  state.quality = latest_sample.quality;
  state.distance = latest_sample.distance;
  state.dt_s = latest_sample.dt_s;
  state.vx_raw = latest_sample.vx_raw;
  state.vy_raw = latest_sample.vy_raw;

  // 优先使用最近窗口均值；若不满足质量门限则退化到最新样本
  constexpr size_t kMeanWindow = 5;
  constexpr uint8_t kMinQuality = 30;
  opflow_sample mean_sample;
  if (opflow_buffer_.mean(kMeanWindow, mean_sample, kMinQuality)) {
    state.vx = mean_sample.vx_raw;
    state.vy = mean_sample.vy_raw;

    const std::vector<opflow_sample> samples = opflow_buffer_.snapshot();
    const size_t start =
        samples.size() > kMeanWindow ? samples.size() - kMeanWindow : 0;
    uint32_t valid_count = 0;
    for (size_t i = start; i < samples.size(); ++i) {
      if (samples[i].quality >= kMinQuality) {
        ++valid_count;
      }
    }
    state.sample_count = valid_count;
    state.valid = (valid_count > 0);
    return state;
  }

  state.vx = latest_sample.vx_raw;
  state.vy = latest_sample.vy_raw;
  state.sample_count = 1;
  state.valid = (latest_sample.quality >= kMinQuality);
  return state;
}

px4_data::ekf2_state_ PX4_Reader::get_ekf2_state(void) {
  std::lock_guard<std::mutex> lock(ekf2_state_mtx);
  return ekf2_state;
}

px4_data::velocity_ PX4_Reader::get_local_velocity(void) {
  std::lock_guard<std::mutex> lock(local_velocity_mtx);
  return local_velocity;
}

px4_data::pose_ PX4_Reader::get_local_pose(void) {
  std::lock_guard<std::mutex> lock(local_pose_mtx);
  return local_pose;
}

px4_data::pose_ PX4_Reader::get_body_pose(void) {
  std::lock_guard<std::mutex> lock(body_pose_mtx);
  return body_pose;
}

px4_data::velocity_ PX4_Reader::get_body_velocity(void) {
  std::lock_guard<std::mutex> lock(body_velocity_mtx);
  return body_velocity;
}

px4_data::ekf2_param_ PX4_Reader::fetch_ekf2_param(void) {
  px4_data::ekf2_param_ out = ekf2_param;

  fetchParamInt("EKF2_EV_CTRL", out.ev_ctrl);
  fetchParamInt("EKF2_HGT_REF", out.hgt_ref);
  fetchParamFloat("EKF2_EV_DELAY", out.ev_delay);

  ekf2_param = out;
  return out;
}

px4_data::attitude_param_ PX4_Reader::fetch_attitude_param(void) {
  px4_data::attitude_param_ out = attitude_param;

  // 读取多旋翼角速度环 PID 参数
  fetchParamFloat("MC_ROLLRATE_P", out.roll.kp);
  fetchParamFloat("MC_ROLLRATE_I", out.roll.ki);
  fetchParamFloat("MC_ROLLRATE_D", out.roll.kd);

  fetchParamFloat("MC_PITCHRATE_P", out.pitch.kp);
  fetchParamFloat("MC_PITCHRATE_I", out.pitch.ki);
  fetchParamFloat("MC_PITCHRATE_D", out.pitch.kd);

  fetchParamFloat("MC_YAWRATE_P", out.yaw.kp);
  fetchParamFloat("MC_YAWRATE_I", out.yaw.ki);
  fetchParamFloat("MC_YAWRATE_D", out.yaw.kd);

  attitude_param = out;
  return out;
}

px4_data::velocity_param_ PX4_Reader::fetch_velocity_param(void) {
  px4_data::velocity_param_ out = velocity_param;

  fetchParamFloat("MPC_XY_VEL_P_ACC", out.mpc_xy.kp);
  fetchParamFloat("MPC_XY_VEL_I_ACC", out.mpc_xy.ki);
  fetchParamFloat("MPC_XY_VEL_D_ACC", out.mpc_xy.kd);

  fetchParamFloat("MPC_Z_VEL_P_ACC", out.mpc_z.kp);
  fetchParamFloat("MPC_Z_VEL_I_ACC", out.mpc_z.ki);
  fetchParamFloat("MPC_Z_VEL_D_ACC", out.mpc_z.kd);

  velocity_param = out;
  return out;
}

px4_data::position_param_ PX4_Reader::fetch_position_param(void) {
  px4_data::position_param_ out = position_param;

  // 位置环常用参数只有 P，I/D 保持为 0
  fetchParamFloat("MPC_XY_P", out.mpc_xy.kp);
  fetchParamFloat("MPC_Z_P", out.mpc_z.kp);

  out.mpc_xy.ki = 0.0f;
  out.mpc_xy.kd = 0.0f;
  out.mpc_z.ki = 0.0f;
  out.mpc_z.kd = 0.0f;

  position_param = out;
  return out;
}
