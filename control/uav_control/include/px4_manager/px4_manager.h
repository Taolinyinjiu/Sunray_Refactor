/**
 * @file px4_manager.h
 * @brief PX4 状态与参数管理器（基于 MAVROS service）
 *
 * @details
 * 该类的目标是把“上层控制逻辑 -> MAVROS service 调用”这层胶水逻辑收敛到一个地方，
 * 以减少业务节点里重复的 service 拼接、request 填充和错误处理代码。
 *
 * 使用方式（推荐）：
 * 1. 在节点启动阶段构造对象（或默认构造后调用 init）。
 * 2. 保证参数服务器中存在：
 *    - uav_id（如 1）
 *    - uav_name（如 "uav"）
 * 3. 调用 init(nh) 完成内部 service client 初始化。
 * 4. 之后再调用 setArm / setMode / setTakeoff / setLand / setParamXXX。
 *
 * 命名空间约定：
 * 若 uav_name="uav" 且 uav_id=1，则目标 MAVROS 命名空间为：
 *   /uav1/mavros
 * 例如：
 *   /uav1/mavros/cmd/arming
 *   /uav1/mavros/set_mode
 *   /uav1/mavros/param/set
 *
 * 设计边界：
 * - 这里只封装 service 调用，不做状态机调度（例如先切模式再解锁这种流程控制）。
 * - 返回值只表达“这次 service 调用是否成功被远端接受”。
 * - 对需要更强一致性的场景（例如模式切换后再次确认当前模式）应由上层补做读回校验。
 *
 * 成功判定约定：
 * - setArm      -> srv.response.success
 * - setMode     -> srv.response.mode_sent
 * - setTakeoff  -> srv.response.success
 * - setLand     -> srv.response.success
 * - setParamInt/Float -> srv.response.success
 */
#pragma once

#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/CommandTOL.h"
#include "mavros_msgs/ParamValue.h"
#include "mavros_msgs/ParamSet.h"
#include "mavros_msgs/SetMode.h"
#include "ros/node_handle.h"

#include <string>

class PX4_StateManager {
public:
  /**
   * @brief 默认构造，不会自动创建 service client。
   *
   * 适用于“先创建对象，后续条件满足时再 init”场景。
   */
  PX4_StateManager();

  /**
   * @brief 便捷构造，内部直接调用 init(nh)。
   * @param nh ROS 句柄，要求能访问 uav_id / uav_name 参数。
   * @throw std::runtime_error 当关键参数缺失时抛异常。
   */
  explicit PX4_StateManager(ros::NodeHandle& nh);

  ~PX4_StateManager() = default;

  /**
   * @brief 初始化 manager：读取 UAV 参数并创建 service client。
   *
   * 该函数会：
   * 1. 从参数服务器读取 uav_id 和 uav_name。
   * 2. 组装 MAVROS 命名空间。
   * 3. 初始化各个 service client 句柄。
   * 4. 将 initialized_ 置为 true。
   *
   * @param nh ROS 句柄
   * @throw std::runtime_error 当参数缺失时抛异常
   */
  void init(ros::NodeHandle& nh);

  /**
   * @brief 解锁/上锁。
   * @param arm true=解锁, false=上锁
   * @return true 表示调用成功且 PX4 返回 success；false 表示初始化未完成、service 调用失败或 PX4 拒绝。
   */
  bool setArm(bool arm);

  /**
   * @brief 设置飞行模式（例如 OFFBOARD / AUTO.LOITER）。
   * @param mode PX4 模式字符串（custom_mode）
   * @return true 表示 mode_sent；false 表示未发出或被拒绝。
   */
  bool setMode(const std::string& mode);

  /**
   * @brief 发送起飞指令。
   * @param altitude 目标起飞高度
   * @param latitude 目标纬度（可选）
   * @param longitude 目标经度（可选）
   * @param yaw 目标航向（可选）
   */
  bool setTakeoff(double altitude, double latitude = 0.0,
                  double longitude = 0.0, double yaw = 0.0);

  /**
   * @brief 发送降落指令。
   * @param altitude 期望降落高度（可选）
   * @param latitude 期望降落纬度（可选）
   * @param longitude 期望降落经度（可选）
   * @param yaw 期望降落航向（可选）
   */
  bool setLand(double altitude = 0.0, double latitude = 0.0,
               double longitude = 0.0, double yaw = 0.0);

  /**
   * @brief 设置整型参数（走 /mavros/param/set）。
   * @param name 参数名（如 EKF2_EV_CTRL）
   * @param value 整数值
   * @return true 表示 PX4 接受此次设置；false 表示调用失败或被拒绝。
   */
  bool setParamInt(const std::string& name, int value);

  /**
   * @brief 设置浮点参数（走 /mavros/param/set）。
   * @param name 参数名（如 EKF2_EV_DELAY）
   * @param value 浮点值
   * @return true 表示 PX4 接受此次设置；false 表示调用失败或被拒绝。
   */
  bool setParamFloat(const std::string& name, float value);

private:
  /**
   * @brief 统一前置检查：是否已完成 init。
   * @return true 可继续调用，false 应立即返回失败。
   */
  bool ensureClientsReady();

  // ===== Service client 句柄 =====
  // 这些句柄在 init() 中按 UAV 命名空间创建。
  ros::ServiceClient arming_client_;
  ros::ServiceClient set_mode_client_;
  ros::ServiceClient takeoff_client_;
  ros::ServiceClient land_client_;
  ros::ServiceClient param_set_client_;

  // ===== 目标 UAV 标识 =====
  // 由参数服务器读取：
  // - uav_name_ 例如 "uav"
  // - uav_id_   例如 1
  // 最终拼装为 /uav1/mavros
  int uav_id_{0};
  std::string uav_name_;

  // 初始化完成标记。所有对外接口都会先检查该值。
  bool initialized_{false};
};
