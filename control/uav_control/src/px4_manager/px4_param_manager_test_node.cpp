#include "px4_manager/px4_param_manager.h"

#include "mavros_msgs/ParamGet.h"
#include "ros/ros.h"

#include <cmath>
#include <string>

namespace {

bool fetchParamInt(ros::ServiceClient& client, const std::string& name, int& out) {
  mavros_msgs::ParamGet srv;
  srv.request.param_id = name;
  if (!client.call(srv)) {
    ROS_ERROR("ParamGet call failed for %s", name.c_str());
    return false;
  }
  if (!srv.response.success) {
    ROS_ERROR("ParamGet response not success for %s", name.c_str());
    return false;
  }
  out = srv.response.value.integer;
  return true;
}

bool fetchParamFloat(ros::ServiceClient& client, const std::string& name, float& out) {
  mavros_msgs::ParamGet srv;
  srv.request.param_id = name;
  if (!client.call(srv)) {
    ROS_ERROR("ParamGet call failed for %s", name.c_str());
    return false;
  }
  if (!srv.response.success) {
    ROS_ERROR("ParamGet response not success for %s", name.c_str());
    return false;
  }
  out = srv.response.value.real;
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "px4_param_manager_test_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // 默认目标 UAV 命名空间：/uav1/mavros
  int uav_id = 1;
  std::string uav_name = "uav";
  pnh.param("uav_id", uav_id, uav_id);
  pnh.param("uav_name", uav_name, uav_name);
  nh.setParam("uav_id", uav_id);
  nh.setParam("uav_name", uav_name);
  const std::string mavros_ns = "/" + uav_name + std::to_string(uav_id) + "/mavros";
  ros::ServiceClient param_get_client =
      nh.serviceClient<mavros_msgs::ParamGet>(mavros_ns + "/param/get");

  // 你给出的测试值
  int ev_ctrl = 1;
  float ev_delay = 13.0f;
  int hgt_ref = 1;
  pnh.param("ev_ctrl", ev_ctrl, ev_ctrl);
  pnh.param("ev_delay", ev_delay, ev_delay);
  pnh.param("hgt_ref", hgt_ref, hgt_ref);

  PX4_ParamManager manager(nh);
  ros::Duration(1.0).sleep();

  px4_data::Ekf2Params param;
  // 演示位掩码风格：ev_ctrl param; param.enable_Horizontalposition();
  px4_data::ev_ctrl ev_ctrl_mask;
  ev_ctrl_mask.enable_Horizontalposition();
  ev_ctrl_mask.enable_Verticalposition();
  // 若你想完全由位掩码驱动，可忽略上面的 ~ev_ctrl 参数，直接用该值。
  param.set_ev_ctrl(ev_ctrl_mask);
  // 兼容旧参数入口：若显式传入 _ev_ctrl，则覆盖上面的演示值。
  param.ev_ctrl = ev_ctrl;
  param.ev_delay = ev_delay;
  param.hgt_ref = hgt_ref;

  ROS_INFO("PX4 param test: EKF2_EV_CTRL=%d EKF2_EV_DELAY=%.3f EKF2_HGT_REF=%d",
           param.ev_ctrl, param.ev_delay, param.hgt_ref);

  const bool ok = manager.set_param_ekf2(param);
  if (!ok) {
    ROS_ERROR("FAIL: set_param_ekf2 returned false");
    return 1;
  }

  int ev_ctrl_actual = 0;
  int hgt_ref_actual = 0;
  float ev_delay_actual = 0.0f;
  bool get_ok = fetchParamInt(param_get_client, "EKF2_EV_CTRL", ev_ctrl_actual) &&
                fetchParamFloat(param_get_client, "EKF2_EV_DELAY", ev_delay_actual) &&
                fetchParamInt(param_get_client, "EKF2_HGT_REF", hgt_ref_actual);
  if (!get_ok) {
    ROS_ERROR("FAIL: param readback failed");
    return 1;
  }

  const float kFloatTol = 1e-3f;
  const bool match = (ev_ctrl_actual == ev_ctrl) && (hgt_ref_actual == hgt_ref) &&
                     (std::fabs(ev_delay_actual - ev_delay) <= kFloatTol);

  ROS_INFO("Readback: EKF2_EV_CTRL=%d EKF2_EV_DELAY=%.3f EKF2_HGT_REF=%d",
           ev_ctrl_actual, ev_delay_actual, hgt_ref_actual);
  if (!match) {
    ROS_ERROR("FAIL: readback mismatch, target=(%d, %.3f, %d)", ev_ctrl, ev_delay,
              hgt_ref);
    return 1;
  }

  ROS_INFO("PASS: set_param_ekf2 completed and readback matched");
  return 0;
}
