#include "px4_manager/px4_reader.h"

#include <vector>

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
  initSubscribers(nh, enable_list);
};

// 带有reader_list的构造函数，选择性的订阅
PX4_Reader::PX4_Reader(ros::NodeHandle& nh, reader_list_ enable_list_) {
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
};

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
