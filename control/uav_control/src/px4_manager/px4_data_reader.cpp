#include "px4_manager/px4_data_reader.h"

#include <cstdint>
#include <vector>

void ReaderOptions::enable_all() {
  read_system_state = true;
  read_ekf2_state = true;
  read_flow_state = true;
  read_local_pose = true;
  read_local_velocity = true;
  read_body_pose = true;
  read_body_velocity = true;
  read_ekf2_params = true;
  read_attitude_params = true;
  read_velocity_params = true;
  read_position_params = true;
}

void ReaderOptions::disable_all() {
  read_system_state = false;
  read_ekf2_state = false;
  read_flow_state = false;
  read_local_pose = false;
  read_local_velocity = false;
  read_body_pose = false;
  read_body_velocity = false;
  read_ekf2_params = false;
  read_attitude_params = false;
  read_velocity_params = false;
  read_position_params = false;
}

void Px4DataReader::reset_state_defaults() {
  uav_id_ = 0;
  uav_namespace_.clear();

  system_state_cache_ = {};
  system_state_cache_.uav_id = 0;
  system_state_cache_.connected = false;
  system_state_cache_.armed = false;
  system_state_cache_.rc_input = false;
  system_state_cache_.system_load = 0.0f;
  system_state_cache_.voltage = 0.0f;
  system_state_cache_.current = 0.0f;
  system_state_cache_.percent = 0.0f;
  system_state_cache_.flight_mode = px4_data::FlightMode::kUndefined;
  system_state_cache_.landed_state = px4_data::LandedState::kUndefined;

  ekf2_state_cache_ = {};
  ekf2_state_cache_.state_codes = 0;
  ekf2_state_cache_.allow_stabilize = false;
  ekf2_state_cache_.allow_altitude = false;
  ekf2_state_cache_.allow_position = false;

  latest_opflow_raw_ = px4_data::OpticalFlowRaw();
  opflow_buffer_.clear();

  local_pose_cache_.position.setZero();
  local_pose_cache_.orientation = Eigen::Quaterniond::Identity();
  local_velocity_cache_.linear.setZero();
  local_velocity_cache_.angular.setZero();

  local_odometry_cache_.timestamp = ros::Time(0);
  local_odometry_cache_.position.setZero();
  local_odometry_cache_.orientation = Eigen::Quaterniond::Identity();
  local_odometry_cache_.linear.setZero();
  local_odometry_cache_.angular.setZero();

  body_pose_cache_.position.setZero();
  body_pose_cache_.orientation = Eigen::Quaterniond::Identity();
  body_velocity_cache_.linear.setZero();
  body_velocity_cache_.angular.setZero();

  ekf2_params_cache_ = {};
  attitude_params_cache_ = {};
  velocity_params_cache_ = {};
  position_params_cache_ = {};
}

void Px4DataReader::init_service_clients() {
  param_get_client_ =
      nh_.serviceClient<mavros_msgs::ParamGet>(uav_namespace_ + "/mavros/param/get");
}

bool Px4DataReader::fetch_param_int(const std::string& param_name, int& out_value) {
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

bool Px4DataReader::fetch_param_float(const std::string& param_name,
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

// 根据 ReaderOptions 在 ROS 中按需注册订阅者。
void Px4DataReader::init_subscribers(ros::NodeHandle& nh,
                                    ReaderOptions options) {
  // 定义订阅表
  std::vector<SubscribeEntry> entries = {
      // 标准状态信息
      {&ReaderOptions::read_system_state, &Px4DataReader::state_sub_,
       [&]() {
         return nh.subscribe<mavros_msgs::State>(
             uav_namespace_ + "/mavros/state", 10, &Px4DataReader::state_callback, this);
       }},
      // 扩展状态信息
      {&ReaderOptions::read_system_state, &Px4DataReader::exstate_sub_,
       [&]() {
         return nh.subscribe<mavros_msgs::ExtendedState>(
             uav_namespace_ + "/mavros/extended_state", 10,
             &Px4DataReader::extended_state_callback, this);
       }},
      // 系统状态信息
      {&ReaderOptions::read_system_state, &Px4DataReader::sys_sub_,
       [&]() {
         return nh.subscribe<mavros_msgs::SysStatus>(
             uav_namespace_ + "/mavros/sys_status", 10, &Px4DataReader::system_status_callback,
             this);
       }},
      // ekf2 估计器状态信息
      {&ReaderOptions::read_ekf2_state, &Px4DataReader::ekf2status_sub_,
       [&]() {
         return nh.subscribe<mavros_msgs::EstimatorStatus>(
             uav_namespace_ + "/mavros/estimator_status", 10,
             &Px4DataReader::ekf2_status_callback, this);
       }},
      // 光流数据
      {&ReaderOptions::read_flow_state, &Px4DataReader::opflow_sub_,
       [&]() {
         return nh.subscribe<mavros_msgs::OpticalFlowRad>(
             uav_namespace_ + "/mavros/px4flow/optical_flow_rad", 10,
             &Px4DataReader::optical_flow_callback, this);
       }},
      // local系下的里程计
      {&ReaderOptions::read_local_pose, &Px4DataReader::local_odom_sub_,
       [&]() {
         return nh.subscribe<nav_msgs::Odometry>(
             uav_namespace_ + "/mavros/local_position/odom", 10,
             &Px4DataReader::local_odometry_callback, this);
       }},
      // local系下的速度
      {&ReaderOptions::read_local_velocity, &Px4DataReader::local_vel_sub_,
       [&]() {
         return nh.subscribe<geometry_msgs::TwistStamped>(
             uav_namespace_ + "/mavros/local_position/velocity_local", 10,
             &Px4DataReader::local_velocity_callback, this);
       }},
      // body系下的姿态
      {&ReaderOptions::read_body_pose, &Px4DataReader::body_att_sub_,
       [&]() {
         return nh.subscribe<sensor_msgs::Imu>(uav_namespace_ + "/mavros/imu/data",
                                               10, &Px4DataReader::body_attitude_callback,
                                               this);
       }},
      // local系下的速度
      {&ReaderOptions::read_body_velocity, &Px4DataReader::body_vel_sub_,
       [&]() {
         return nh.subscribe<geometry_msgs::TwistStamped>(
             uav_namespace_ + "/mavros/local_position/velocity_body", 10,
             &Px4DataReader::body_velocity_callback, this);
       }},
  };

  // 遍历订阅表，根据用户传入的 options 自动执行订阅
  for (const auto& entry : entries) {
    // 检查 options 中对应的参数是否为 true
    if (options.*(entry.flag)) {
      // 执行lambda 表达式并将返回的Subscriber 负值给当前实例对应的成员变量
      this->*(entry.handle) = entry.make();
    }
  }
}

// 默认构造函数，全订阅
Px4DataReader::Px4DataReader(ros::NodeHandle& nh) {
  reset_state_defaults();
  nh_ = nh;
  // 首先读取全局参数，通过uav_id_参数与uav_namespace_是否存在来进行判断，当该参数存在时，认为参数文件正常加载
  try {
    // 检查uav_id_参数是否存在
    if (!nh.getParam("uav_id", uav_id_)) {
      throw std::runtime_error(
          "无法读取参数 'uav_id'。请检查 YAML "
          "配置文件是否正确加载到 Parameter Server。");
    }
    if (!nh.getParam("uav_name", uav_namespace_)) {
      throw std::runtime_error(
          "无法读取参数 'uav_name'。请检查 YAML "
          "配置文件是否正确加载到 Parameter Server。");
    }
  } catch (const std::exception& e) {
    // 如果不存在，则抛出异常
    ROS_FATAL("Px4DataReader初始化失败: %s", e.what());
    // 继续向上抛出异常，如果main中没有异常处理，则节点结束运行
    throw;
  }

  // 运行到这里，认为参数文件正确加载，开始初始化各项订阅者以及回调函数，该构造函数没有reader_list因此默认为全部订阅
  uav_namespace_ = "/" + uav_namespace_ + std::to_string(uav_id_);
  // 构造一个全订阅的enable_list
  ReaderOptions enable_list;
  enable_list.enable_all();
  init_service_clients();
  init_subscribers(nh, enable_list);
};

// 带有reader_list的构造函数，选择性的订阅
Px4DataReader::Px4DataReader(ros::NodeHandle& nh, ReaderOptions options) {
  reset_state_defaults();
  nh_ = nh;
  // 首先读取全局参数，通过uav_id_参数与uav_namespace_是否存在来进行判断，当该参数存在时，认为参数文件正常加载
  try {
    // 检查uav_id_参数是否存在
    if (!nh.getParam("uav_id", uav_id_)) {
      throw std::runtime_error(
          "无法读取参数 'uav_id'。请检查 YAML "
          "配置文件是否正确加载到 Parameter Server。");
    }
    if (!nh.getParam("uav_name", uav_namespace_)) {
      throw std::runtime_error(
          "无法读取参数 'uav_name'。请检查 YAML "
          "配置文件是否正确加载到 Parameter Server。");
    }
  } catch (const std::exception& e) {
    // 如果不存在，则抛出异常
    ROS_FATAL("Px4DataReader初始化失败: %s", e.what());
    // 继续向上抛出异常，如果main中没有异常处理，则节点结束运行
    throw;
  }

  // 运行到这里，认为参数文件正确加载，开始初始化各项订阅者以及回调函数，该构造函数没有reader_list因此默认为全部订阅
  uav_namespace_ = "/" + uav_namespace_ + std::to_string(uav_id_);
  // 传入 options
  init_service_clients();
  init_subscribers(nh, options);
};
// 析构函数，由于ROS中的订阅者会自动调用shutdown方法，因此Px4DataReader中析构函数没有实现的必要
Px4DataReader::~Px4DataReader() {};

// 请注意,订阅者的回调函数放置在px4_data_reader_callback.cpp中进行实现

px4_data::FlightMode Px4DataReader::flight_mode_from_string(
    const std::string& mode) {
  if (mode == "OFFBOARD") return px4_data::FlightMode::kOffboard;
  if (mode == "POSCTL") return px4_data::FlightMode::kPosctl;
  if (mode == "ALTCTL") return px4_data::FlightMode::kAltctl;
  if (mode == "AUTO.TAKEOFF") return px4_data::FlightMode::kAutoTakeoff;
  if (mode == "AUTO.LAND") return px4_data::FlightMode::kAutoLand;
  if (mode == "AUTO.RTL") return px4_data::FlightMode::kAutoRtl;
  if (mode == "AUTO.MISSION") return px4_data::FlightMode::kAutoMission;
  if (mode == "AUTO.LOITER") return px4_data::FlightMode::kAutoLoiter;
  if (mode == "MANUAL") return px4_data::FlightMode::kManual;
  if (mode == "STABILIZED") return px4_data::FlightMode::kStabilized;
  if (mode == "ACRO") return px4_data::FlightMode::kAcro;
  return px4_data::FlightMode::kUndefined;
};

px4_data::SystemState Px4DataReader::get_system_state(void) {
  std::lock_guard<std::mutex> lock(system_state_mutex_);
  px4_data::SystemState state = system_state_cache_;
  state.uav_id = static_cast<uint8_t>(uav_id_);
  return state;
}

px4_data::OpticalFlowState Px4DataReader::get_flow_state(void) {
  px4_data::OpticalFlowState state;

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

px4_data::Ekf2State Px4DataReader::get_ekf2_state(void) {
  std::lock_guard<std::mutex> lock(ekf2_state_mutex_);
  return ekf2_state_cache_;
}

px4_data::Velocity Px4DataReader::get_local_velocity(void) {
  std::lock_guard<std::mutex> lock(local_velocity_mutex_);
  return local_velocity_cache_;
}

px4_data::Pose Px4DataReader::get_local_pose(void) {
  std::lock_guard<std::mutex> lock(local_pose_mutex_);
  return local_pose_cache_;
}

px4_data::Pose Px4DataReader::get_body_pose(void) {
  std::lock_guard<std::mutex> lock(body_pose_mutex_);
  return body_pose_cache_;
}

px4_data::Velocity Px4DataReader::get_body_velocity(void) {
  std::lock_guard<std::mutex> lock(body_velocity_mutex_);
  return body_velocity_cache_;
}

px4_data::Ekf2Params Px4DataReader::fetch_ekf2_params(void) {
  px4_data::Ekf2Params out = ekf2_params_cache_;

  fetch_param_int("EKF2_EV_CTRL", out.ev_ctrl);
  fetch_param_int("EKF2_HGT_REF", out.hgt_ref);
  fetch_param_float("EKF2_EV_DELAY", out.ev_delay);

  ekf2_params_cache_ = out;
  return out;
}

px4_data::AttitudeParams Px4DataReader::fetch_attitude_params(void) {
  px4_data::AttitudeParams out = attitude_params_cache_;

  // 读取多旋翼角速度环 PID 参数
  fetch_param_float("MC_ROLLRATE_P", out.roll.kp);
  fetch_param_float("MC_ROLLRATE_I", out.roll.ki);
  fetch_param_float("MC_ROLLRATE_D", out.roll.kd);

  fetch_param_float("MC_PITCHRATE_P", out.pitch.kp);
  fetch_param_float("MC_PITCHRATE_I", out.pitch.ki);
  fetch_param_float("MC_PITCHRATE_D", out.pitch.kd);

  fetch_param_float("MC_YAWRATE_P", out.yaw.kp);
  fetch_param_float("MC_YAWRATE_I", out.yaw.ki);
  fetch_param_float("MC_YAWRATE_D", out.yaw.kd);

  attitude_params_cache_ = out;
  return out;
}

px4_data::VelocityParams Px4DataReader::fetch_velocity_params(void) {
  px4_data::VelocityParams out = velocity_params_cache_;

  fetch_param_float("MPC_XY_VEL_P_ACC", out.mpc_xy.kp);
  fetch_param_float("MPC_XY_VEL_I_ACC", out.mpc_xy.ki);
  fetch_param_float("MPC_XY_VEL_D_ACC", out.mpc_xy.kd);

  fetch_param_float("MPC_Z_VEL_P_ACC", out.mpc_z.kp);
  fetch_param_float("MPC_Z_VEL_I_ACC", out.mpc_z.ki);
  fetch_param_float("MPC_Z_VEL_D_ACC", out.mpc_z.kd);

  velocity_params_cache_ = out;
  return out;
}

px4_data::PositionParams Px4DataReader::fetch_position_params(void) {
  px4_data::PositionParams out = position_params_cache_;

  // 位置环常用参数只有 P，I/D 保持为 0
  fetch_param_float("MPC_XY_P", out.mpc_xy.kp);
  fetch_param_float("MPC_Z_P", out.mpc_z.kp);

  out.mpc_xy.ki = 0.0f;
  out.mpc_xy.kd = 0.0f;
  out.mpc_z.ki = 0.0f;
  out.mpc_z.kd = 0.0f;

  position_params_cache_ = out;
  return out;
}
