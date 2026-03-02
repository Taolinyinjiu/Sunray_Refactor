#include "px4_manager/px4_manager.h"

#include "ros/ros.h"

#include <stdexcept>

// 默认构造：对象可先创建，后续再显式 init。
PX4_StateManager::PX4_StateManager() = default;

// 便捷构造：创建即初始化，适合简单测试或单一 UAV 节点。
PX4_StateManager::PX4_StateManager(ros::NodeHandle& nh) { init(nh); }

void PX4_StateManager::init(ros::NodeHandle& nh) {
  // 1) 读取目标 UAV 标识。缺失时直接抛异常，避免静默落到错误命名空间。
  if (!nh.getParam("uav_id", uav_id_)) {
    throw std::runtime_error("PX4_StateManager init failed: missing param uav_id");
  }
  if (!nh.getParam("uav_name", uav_name_)) {
    throw std::runtime_error(
        "PX4_StateManager init failed: missing param uav_name");
  }

  // 2) 构建 MAVROS 命名空间。
  //    例如 uav_name_="uav", uav_id_=1 -> /uav1/mavros
  const std::string ns = "/" + uav_name_ + std::to_string(uav_id_) + "/mavros";

  // 3) 创建各功能对应的 service client。
  //    注意：这里只创建句柄，不会立即等待服务可用。
  arming_client_ = nh.serviceClient<mavros_msgs::CommandBool>(ns + "/cmd/arming");
  set_mode_client_ = nh.serviceClient<mavros_msgs::SetMode>(ns + "/set_mode");
  takeoff_client_ = nh.serviceClient<mavros_msgs::CommandTOL>(ns + "/cmd/takeoff");
  land_client_ = nh.serviceClient<mavros_msgs::CommandTOL>(ns + "/cmd/land");
  param_set_client_ = nh.serviceClient<mavros_msgs::ParamSet>(ns + "/param/set");

  // 4) 标记初始化完成，允许对外接口执行。
  initialized_ = true;
}

bool PX4_StateManager::ensureClientsReady() {
  // 所有 public 控制/参数接口统一经过这里做“快速失败”。
  if (!initialized_) {
    ROS_WARN("PX4_StateManager is not initialized. Call init(nh) first.");
    return false;
  }
  return true;
}

bool PX4_StateManager::setArm(bool arm) {
  // 初始化未完成时直接返回，避免访问未准备的 client。
  if (!ensureClientsReady()) return false;

  // CommandBool: request.value=true 解锁，false 上锁。
  mavros_msgs::CommandBool srv;
  srv.request.value = arm;
  if (!arming_client_.call(srv)) {
    ROS_WARN("Failed to call arming service");
    return false;
  }
  return srv.response.success;
}

bool PX4_StateManager::setMode(const std::string& mode) {
  if (!ensureClientsReady()) return false;

  // SetMode 使用 custom_mode 字符串（如 OFFBOARD / AUTO.LOITER）。
  mavros_msgs::SetMode srv;
  srv.request.custom_mode = mode;
  if (!set_mode_client_.call(srv)) {
    ROS_WARN("Failed to call set_mode service: %s", mode.c_str());
    return false;
  }
  return srv.response.mode_sent;
}

bool PX4_StateManager::setTakeoff(double altitude, double latitude,
                                  double longitude, double yaw) {
  if (!ensureClientsReady()) return false;

  // CommandTOL 在 MAVROS 中同时承载起飞/降落参数。
  // 对于仅需高度起飞的场景，可只传 altitude，其他参数留默认 0。
  mavros_msgs::CommandTOL srv;
  srv.request.altitude = altitude;
  srv.request.latitude = latitude;
  srv.request.longitude = longitude;
  srv.request.yaw = yaw;
  if (!takeoff_client_.call(srv)) {
    ROS_WARN("Failed to call takeoff service");
    return false;
  }
  return srv.response.success;
}

bool PX4_StateManager::setLand(double altitude, double latitude, double longitude,
                               double yaw) {
  if (!ensureClientsReady()) return false;

  // 与 setTakeoff 类似，CommandTOL 里填写降落相关字段。
  mavros_msgs::CommandTOL srv;
  srv.request.altitude = altitude;
  srv.request.latitude = latitude;
  srv.request.longitude = longitude;
  srv.request.yaw = yaw;
  if (!land_client_.call(srv)) {
    ROS_WARN("Failed to call land service");
    return false;
  }
  return srv.response.success;
}

bool PX4_StateManager::setParamInt(const std::string& name, int value) {
  if (!ensureClientsReady()) return false;

  // param/set 的 integer 与 real 字段由调用方按参数类型填写。
  // 这里明确走 integer，适用于 EKF2_EV_CTRL / EKF2_HGT_REF 这类整型参数。
  mavros_msgs::ParamSet srv;
  srv.request.param_id = name;
  srv.request.value.integer = value;
  if (!param_set_client_.call(srv)) {
    ROS_WARN("Failed to call param/set for %s", name.c_str());
    return false;
  }
  return srv.response.success;
}

bool PX4_StateManager::setParamFloat(const std::string& name, float value) {
  if (!ensureClientsReady()) return false;

  // 浮点参数写入路径，适用于 EKF2_EV_DELAY 等 float 参数。
  mavros_msgs::ParamSet srv;
  srv.request.param_id = name;
  srv.request.value.real = value;
  if (!param_set_client_.call(srv)) {
    ROS_WARN("Failed to call param/set for %s", name.c_str());
    return false;
  }
  return srv.response.success;
}
