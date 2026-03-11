#pragma once

/**
 * @file px4_position_controller.h
 * @brief PX4 位置控制器封装。
 *
 * 本文件定义 `Position_Controller` 类，用于对接 PX4 内部位置控制能力。
 */

#include "controller/base_controller/base_controller.hpp"
#include "ros/time.h"
#include "utils/curve/quintic_curve.hpp"

namespace uav_control {

/**
 * @class Position_Controller
 * @brief 基于 PX4 位置环语义的控制器实现。
 *
 * @details
 * 该类继承 `Base_Controller`，负责将控制状态机（起飞、悬停、移动、降落）
 * 转换为标准 `ControllerOutput` 输出。其主要行为包括：
 * - 从参数服务器加载控制参数（误差容限、最大速度、起降时长）；
 * - 在 `update()` 中按 `controller_state_` 分发到对应状态处理函数；
 * - 在降落阶段执行“曲线引导 + 末段速度接管 + 触地判定”的流程管理。
 */
class Position_Controller : public Base_Controller {
public:
  /**
   * @brief 从 ROS 参数服务器加载位置控制器参数。
   *
   * @param nh ROS 节点句柄。
   * @return true 参数加载成功；false 参数缺失或非法。
   *
   * @note
   * 典型读取项包含：
   * - `/uav_ns`（或 `/uav_name` + `/uav_id` 组合）；
   * - `<uav_ns>/error_tolerance`；
   * - `<uav_ns>/velocity_max`；
   * - `<uav_ns>/takeoff_height`、`takeoff_time`、`land_time`。
   */
  bool load_param(ros::NodeHandle &nh) override;

  /**
   * @brief 控制器周期更新入口。
   *
   * @return 当前周期控制输出，具体有效通道由 `output_mask` 指示。
   *
   * @details
   * 该函数会在必要时重置起飞/降落上下文，并根据 `controller_state_`
   * 路由到对应状态处理函数生成输出。
   */
  ControllerOutput update(void) override;

private:
  /**
   * @brief 离开起飞状态时重置起飞上下文。
   *
   * @details
   * 当控制器不处于 `TAKEOFF` 且起飞上下文曾初始化时，
   * 清理起飞相关标志与计时信息，避免状态切换后残留历史上下文。
   */
  void reset_takeoff_context_if_needed();

  /**
   * @brief 离开降落状态时重置降落上下文。
   *
   * @details
   * 当控制器不处于 `LAND` 且降落上下文存在时，清空降落过程中的
   * 末段接管与触地判定状态，确保下次进入降落阶段从干净上下文开始。
   */
  void reset_land_context_if_needed();

  /** @brief 处理 `UNDEFINED` 状态并返回对应控制输出。 */
  ControllerOutput handle_undefined_state();
  /** @brief 处理 `OFF` 状态并返回对应控制输出。 */
  ControllerOutput handle_off_state();
  /** @brief 处理 `TAKEOFF` 状态并返回对应控制输出。 */
  ControllerOutput handle_takeoff_state();
  /** @brief 处理 `HOVER` 状态并返回对应控制输出。 */
  ControllerOutput handle_hover_state();
  /** @brief 处理 `MOVE` 状态并返回对应控制输出。 */
  ControllerOutput handle_move_state();
  /** @brief 处理 `LAND` 状态并返回对应控制输出。 */
  ControllerOutput handle_land_state();

  /** @brief 降落末段是否已切换到速度接管阶段。 */
  bool land_velocity_takeover_active_{false};

  /** @brief 是否已达到下降峰值速度（用于二次穿越阈值判定）。 */
  bool land_takeover_speed_peak_reached_{false};

  /** @brief 上一周期记录的高度（m），用于估计观测下降率。 */
  double last_land_altitude_m_{0.0};

  /** @brief 上一周期高度记录时间戳。 */
  ros::Time last_land_altitude_time_{0};

  /** @brief 低速状态连续保持的起始时间戳。 */
  ros::Time land_low_velocity_start_time_{0};

  /** @brief “高度不再明显下降”连续保持的起始时间戳。 */
  ros::Time land_no_descent_start_time_{0};

  /** @brief 上一周期垂直速度 z 分量（m/s）。 */
  double last_land_vz_mps_{0.0};

  /** @brief `last_land_vz_mps_` 是否已被有效初始化。 */
  bool last_land_vz_valid_{false};
};

} // namespace uav_control
