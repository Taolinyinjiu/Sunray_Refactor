#include "controller/px4_position_controller/px4_position_controller.h"

#include <algorithm>
#include <cmath>

namespace uav_control {

namespace {

double axisDurationFromLimit(double displacement_m, double velocity_limit_mps) {
  if (!std::isfinite(displacement_m) || !std::isfinite(velocity_limit_mps) ||
      velocity_limit_mps <= 0.0) {
    return 0.0;
  }
  return std::abs(displacement_m) / (velocity_limit_mps * 2.0);
}

double curveDurationFromVelocityLimit(const Eigen::Vector3d &start_position,
                                      const Eigen::Vector3d &target_position,
                                      const Eigen::Vector3d &velocity_max) {
  const Eigen::Vector3d delta = target_position - start_position;
  const double tx = axisDurationFromLimit(delta.x(), velocity_max.x());
  const double ty = axisDurationFromLimit(delta.y(), velocity_max.y());
  const double tz = axisDurationFromLimit(delta.z(), velocity_max.z());
  return std::max(tx, std::max(ty, tz));
}

} // namespace


// 重写加载参数函数，目前只需要加载uav_name+uav_id
bool Position_Controller::load_param(ros::NodeHandle &nh) {
  if (!nh.getParam("/uav_ns", uav_ns_)) {
		// 如果没有得到uav_ns参数，就检查uav_name与uav_id
    std::string uav_name;
		int uav_id;
		// 如果至少有一个没有获取到，则返回false状态
		if(!nh.getParam("/uav_name", uav_name) || !nh.getParam("/uav_id", uav_id))
			return false;
		else {
			// 如果两个都检测到了，就进行拼接
			uav_ns_ = uav_name + std::to_string(uav_id);
		}
  }

  // 规范化命名空间：将 "uav1/" 或 "uav1"规范化输出为"/uav1"
  std::string ns = uav_ns_;
  if (ns.empty()) {
    return false;
  }
  if (ns.front() != '/') {
    ns = "/" + ns;
  }
  if (ns.back() == '/') {
    ns.pop_back();
  }
  uav_ns_ = ns;

  // 读取误差容限参数
  std::vector<double> error_param;
  if (!nh.getParam(uav_ns_ + "/error_tolerance", error_param)) {
    return false;
  }
  if (error_param.size() < 3) {
    return false;
  }
  // 将vector转换为Eigen
  error_tolerance_.x() = error_param[0];
  error_tolerance_.y() = error_param[1];
  error_tolerance_.z() = error_param[2];
  if (!error_tolerance_.allFinite() || (error_tolerance_.array() <= 0.0).any()) {
    return false;
  }

  std::vector<double> velocity_max_param;
  if (!nh.getParam(uav_ns_ + "/velocity_max", velocity_max_param)) {
    return false;
  }
  if (velocity_max_param.size() < 3) {
    return false;
  }
  velocity_max_.x() = velocity_max_param[0];
  velocity_max_.y() = velocity_max_param[1];
  velocity_max_.z() = velocity_max_param[2];
  if (!velocity_max_.allFinite() || (velocity_max_.array() <= 0.0).any()) {
    return false;
  }

  // 起飞相对高度（默认使用基类默认值）。
  nh.param(uav_ns_ + "/takeoff_height", takeoff_height_, takeoff_height_);
  nh.param(uav_ns_ + "/takeoff_time", takeoff_time_, takeoff_time_);
  nh.param(uav_ns_ + "/land_time", land_time_, land_time_);

  // if (!std::isfinite(takeoff_time_) || takeoff_time_ < 0.0) {
  //   return false;
  // }
  // if (!std::isfinite(land_time_) || land_time_ < 0.0) {
  //   return false;
  // }

  has_loadparam_ = true;
  return true;
}

// 根据控制器当前状态，进入对应的函数
ControllerOutput Position_Controller::update(void) {
  if (!has_loadparam_) {
    controller_state_ = ControllerState::EMERGENCY_LAND;
    ControllerOutput safe_output;
    safe_output.channel_enable(ControllerOutputMask::VELOCITY);
    safe_output.velocity = Eigen::Vector3d::Zero();
    return safe_output;
  }

  // 参数已加载且状态估计有效时，允许从 UNDEFINED 进入 OFF 待机态。
  if (controller_state_ == ControllerState::UNDEFINED &&
      uav_current_state_.isValid()) {
    controller_state_ = ControllerState::OFF;
  }

  reset_takeoff_context_if_needed();
  reset_land_context_if_needed();

  switch (controller_state_) {
  case ControllerState::UNDEFINED:
    return handle_undefined_state();
  case ControllerState::OFF:
    return handle_off_state();
  case ControllerState::TAKEOFF:
    return handle_takeoff_state();
  case ControllerState::HOVER:
    return handle_hover_state();
  case ControllerState::MOVE:
    return handle_move_state();
  case ControllerState::LAND:
    return handle_land_state();
  default:
    return ControllerOutput();
  }
}

void Position_Controller::reset_takeoff_context_if_needed() {
  // 如果当前状态不为TAKEOFF状态，但是TAKEOFF标识符指示初始化了，说明当前已经离开TAKEOFF阶段，需要重置起飞上下文参数
  if (controller_state_ != ControllerState::TAKEOFF && takeoff_initialized_) {
    // 重置起飞上下文标识
    takeoff_initialized_ = false;
    // 清空时间参数
    takeoff_holdstart_time_ = ros::Time(0);
    takeoff_holdkeep_time_ = ros::Time(0);
  }
}

void Position_Controller::reset_land_context_if_needed() {
  // 如果当前状态不为LAND状态，但是LAND标识符又指示初始化了，说明当前已经结束了LAND阶段，需要重置LAND参数
  if (controller_state_ != ControllerState::LAND && land_initialized_) {
    // 重置标识符
		land_initialized_ = false;
		// 清空land位置
		land_position_ = Eigen::Vector3d::Zero();
		// 清空时间参数
    land_holdstart_time_ = ros::Time(0);
		land_holdkeep_time_ = ros::Time(0);
    // 清空曲线参数
		quintic_curve_generation.clear_all();
  }
}

ControllerOutput Position_Controller::handle_undefined_state() {
  // 如果是UNDEFINE阶段，则说明无人机并没有做好准备，因此此时什么都不执行
  return ControllerOutput(); // 空构造函数，返回也是空结构体
}

ControllerOutput Position_Controller::handle_off_state() {
  // OFF阶段，此时无人机在地面上静止，不进行输出
  return ControllerOutput();
}

ControllerOutput Position_Controller::handle_takeoff_state() {
  ControllerOutput temp_output;

  // 首次进入TAKEOFF时，重置起飞计时上下文
  if (!takeoff_initialized_) {
    takeoff_initialized_ = true;
    takeoff_holdstart_time_ = ros::Time(0);
    takeoff_holdkeep_time_ = ros::Time(0);
  }

  // 未解锁：持续发零速度，保持控制链路活跃
  if (!px4_arm_state_) {
    temp_output.channel_enable(ControllerOutputMask::VELOCITY);
    temp_output.velocity.x() = 0.0;
    temp_output.velocity.y() = 0.0;
    temp_output.velocity.z() = 0.0;
    return temp_output;
  }

  // 111的二进制是0x07
  constexpr uint8_t kTakeoffReadyMask = 0x07U;
  uint8_t takeoff_ready = 0U;
  // 计算误差
  const double ex = std::abs(takeoff_position_.x() - uav_current_state_.position.x());
  const double ey = std::abs(takeoff_position_.y() - uav_current_state_.position.y());
  const double ez = std::abs(takeoff_position_.z() - uav_current_state_.position.z());
  // 如果误差在容许的范围内，置位
  if (ex < error_tolerance_.x()) {
    takeoff_ready |= (1U << 0);
  }
  if (ey < error_tolerance_.y()) {
    takeoff_ready |= (1U << 1);
  }
  if (ez < error_tolerance_.z()) {
    takeoff_ready |= (1U << 2);
  }

  // 根据在期望的起飞位置误差内的持续时间来判断是否达到了稳定的阶段
  const ros::Time now = ros::Time::now();
  // 强制要求至少要2s用于判断是否稳定悬停
	const double hold_required =
      (takeoff_success_time_ > 2.0) ? takeoff_success_time_ : 2.0;
	// 如果三轴上的误差都满足要求，开始计算持续时间是否满足要求
  if (takeoff_ready == kTakeoffReadyMask) {
		// 如果保持的开始时间为0
		if (takeoff_holdstart_time_.isZero()) {
      takeoff_holdstart_time_ = now;
      takeoff_holdkeep_time_ = now;
    } else {
      takeoff_holdkeep_time_ = now;
      const ros::Duration hold_time = now - takeoff_holdstart_time_;
      if (hold_time.toSec() >= hold_required) {
        // 设置轨迹点为当前起飞参数
        trajectory_.position = takeoff_position_;
        // 切换到HOVER状态
        controller_state_ = ControllerState::HOVER;
      }
    }
  } else {
    // 任一轴超出容差，重置计时
    takeoff_holdstart_time_ = ros::Time(0);
    takeoff_holdkeep_time_ = ros::Time(0);
  }

  // 持续输出起飞目标点
  temp_output.channel_enable(ControllerOutputMask::POSITION);
  temp_output.position = takeoff_position_;
  return temp_output;
}



ControllerOutput Position_Controller::handle_hover_state() {
  ControllerOutput temp_output;
  // HOVER状态下，设置输出为当前轨迹点
  temp_output.channel_enable(ControllerOutputMask::POSITION);
  temp_output.position = trajectory_.position;
  return temp_output;
}

ControllerOutput Position_Controller::handle_move_state() {
  ControllerOutput temp_output;
  // 当切换到MOVE时，通常是接受到了相关的控制指令
  if (trajectory_.is_channel_enabled(TrajectoryPoint::ValidMask::POSITION)) {
    temp_output.channel_enable(ControllerOutputMask::POSITION);
    temp_output.position = trajectory_.position;
  }
  if (trajectory_.is_channel_enabled(TrajectoryPoint::ValidMask::VELOCITY)) {
    temp_output.channel_enable(ControllerOutputMask::VELOCITY);
    temp_output.velocity = trajectory_.velocity;
  }
  if (trajectory_.is_channel_enabled(TrajectoryPoint::ValidMask::ACCELERATION)) {
    temp_output.channel_enable(ControllerOutputMask::ACCELERATION);
    temp_output.acceleration_or_force = trajectory_.acceleration;
  }
  if (trajectory_.is_channel_enabled(TrajectoryPoint::ValidMask::YAW)) {
    temp_output.channel_enable(ControllerOutputMask::YAW);
    temp_output.yaw = trajectory_.yaw;
  }
  if (trajectory_.is_channel_enabled(TrajectoryPoint::ValidMask::YAW_RATE)) {
    temp_output.channel_enable(ControllerOutputMask::YAW_RATE);
    temp_output.yaw_rate = trajectory_.yaw_rate;
  }

  // position_controller
  // 不支持对姿态，推力，加加速度，加加加速度进行调整
  return temp_output;
}

ControllerOutput Position_Controller::handle_land_state() {
  ControllerOutput temp_output;
  // LAND 阶段核心思路：
  // 1) 用五次曲线从当前点平滑过渡到 land 目标点；
  // 2) 曲线执行完后，按高度/竖直速度判定是否落地；
  // 3) 若还未落地，持续给一个保守的下降速度兜底。
  constexpr double kLandReplanPosEpsM = 0.05;
  constexpr double kLandReplanTimeEpsS = 0.1;
  constexpr double kLandingVelTolMps = 0.1;
  constexpr double kLandingFallbackDescentMps = -0.1;

  // 使用基类已有的降落参考变量：land_position_ + land_time_。
  const Eigen::Vector3d requested_target = land_position_;
  const double configured_min_land_duration_s =
      (land_time_ > 0.0) ? land_time_ : land_success_time_;
  if (configured_min_land_duration_s <= 0.0) {
    temp_output.channel_enable(ControllerOutputMask::VELOCITY);
    temp_output.velocity = Eigen::Vector3d::Zero();
    return temp_output;
  }

  if (error_tolerance_.z() <= 0.0) {
    temp_output.channel_enable(ControllerOutputMask::VELOCITY);
    temp_output.velocity = Eigen::Vector3d::Zero();
    return temp_output;
  }

  // 检查是否需要重建降落轨迹：
  // - 首次进入 LAND；
  // - 目标点变化超过阈值；
  // - 持续时间参数变化超过阈值；
  // - 曲线尚未配置起始时间。
  const ros::Time now = ros::Time::now();
  const Eigen::Vector3d duration_ref_start =
      land_initialized_ ? quintic_curve_generation.get_start_position()
                        : uav_current_state_.position;
  const double computed_land_duration_s = curveDurationFromVelocityLimit(
      duration_ref_start, requested_target, velocity_max_);
  const double requested_land_duration_s =
      std::max(computed_land_duration_s, configured_min_land_duration_s);

  const bool land_target_changed =
      (requested_target - quintic_curve_generation.get_end_position()).norm() >
      kLandReplanPosEpsM;
  const bool land_duration_changed =
      std::abs(requested_land_duration_s - quintic_curve_generation.get_keep_time()) >
      kLandReplanTimeEpsS;
  const bool curve_time_invalid = quintic_curve_generation.get_start_time().isZero();
  const bool need_reinit =
      !land_initialized_ || land_target_changed || land_duration_changed ||
      curve_time_invalid;

  if (need_reinit) {
    // 以“当前状态位置”为曲线起点，避免轨迹跳变。
    land_holdstart_time_ = now;
    land_holdkeep_time_ = now;

    const double reinit_computed_duration_s = curveDurationFromVelocityLimit(
        uav_current_state_.position, requested_target, velocity_max_);
    const double reinit_land_duration_s =
        std::max(reinit_computed_duration_s, configured_min_land_duration_s);

    // 重新配置五次曲线参数，并设置起始时间。
    quintic_curve_generation.clear_all();
    quintic_curve_generation.set_start_position(uav_current_state_.position);
    quintic_curve_generation.set_end_position(requested_target);
    quintic_curve_generation.set_keep_time(reinit_land_duration_s);
    if (!quintic_curve_generation.set_start_time(now)) {
      // 曲线初始化失败时走安全兜底：零速度，等待下周期重试。
      land_initialized_ = false;
      temp_output.channel_enable(ControllerOutputMask::VELOCITY);
      temp_output.velocity = Eigen::Vector3d::Zero();
      return temp_output;
    }
    land_initialized_ = true;
  }

  // 在曲线持续时间内，优先输出曲线的位置/速度/加速度前馈。
  const double active_land_duration_s = quintic_curve_generation.get_keep_time();
  const double delta_time = (now - land_holdstart_time_).toSec();
  if (delta_time <= active_land_duration_s) {
    const bool curve_state = quintic_curve_generation.generate_land_curve(now);
    if (curve_state) {
      temp_output.channel_enable(ControllerOutputMask::POSITION);
      temp_output.channel_enable(ControllerOutputMask::VELOCITY);
      temp_output.channel_enable(ControllerOutputMask::ACCELERATION);
      temp_output.position = quintic_curve_generation.get_position();
      temp_output.velocity = quintic_curve_generation.get_velocity();
      temp_output.acceleration_or_force = quintic_curve_generation.get_acceleration();
    } else {
      // 曲线生成失败时，退化为当前位置保持 + 零速度，避免突发控制输出。
      temp_output.channel_enable(ControllerOutputMask::POSITION);
      temp_output.channel_enable(ControllerOutputMask::VELOCITY);
      temp_output.position = uav_current_state_.position;
      temp_output.velocity = Eigen::Vector3d::Zero();
    }
    land_holdkeep_time_ = now;
    return temp_output;
  }

  // 超过轨迹时间后，改用末端速度策略判定并补偿下降
  // z_diff > 0 表示当前高度高于目标高度。
  const double z_vel_abs = std::abs(uav_current_state_.velocity.z());
  const double z_diff = uav_current_state_.position.z() - requested_target.z();
  const double z_err = std::abs(z_diff);
  const double z_tol = error_tolerance_.z();

  // 落地成功：高度误差与竖直速度均足够小
  if (z_vel_abs <= kLandingVelTolMps && z_err <= z_tol) {
    controller_state_ = ControllerState::OFF;
    temp_output.clear_all();
    return temp_output;
  }

  // 未落地：统一使用速度通道；
  // 仅在“明显高于目标高度”时给保守下降速度，否则先悬停等待收敛。
  temp_output.channel_enable(ControllerOutputMask::VELOCITY);
  if (z_diff > z_tol) {
    temp_output.velocity = Eigen::Vector3d(0.0, 0.0, kLandingFallbackDescentMps);
  } else {
    temp_output.velocity = Eigen::Vector3d::Zero();
  }
  land_holdkeep_time_ = now;
  return temp_output;
}

} // namespace uav_control
