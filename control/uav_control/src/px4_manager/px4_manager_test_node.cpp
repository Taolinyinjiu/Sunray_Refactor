#include "px4_manager/px4_manager.h"

#include "ros/ros.h"

#include <stdexcept>
#include <string>

int main(int argc, char** argv) {
  ros::init(argc, argv, "px4_manager_test_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // 与 reader 测试节点保持一致，默认使用 /uav1 命名空间
  nh.setParam("uav_id", 1);
  nh.setParam("uav_name", std::string("uav"));

  bool do_set_mode = true;
  std::string mode = "OFFBOARD";
  bool do_arm = true;
  bool arm_value = false;
  bool do_takeoff = false;
  double takeoff_alt = 2.0;
  bool do_land = false;
  bool do_set_param_int = true;
  std::string int_param_name = "EKF2_EV_CTRL";
  int int_param_value = 0;
  bool do_set_param_float = true;
  std::string float_param_name = "EKF2_EV_DELAY";
  double float_param_value = 10.0;

  pnh.param("do_set_mode", do_set_mode, do_set_mode);
  pnh.param("mode", mode, mode);
  pnh.param("do_arm", do_arm, do_arm);
  pnh.param("arm_value", arm_value, arm_value);
  pnh.param("do_takeoff", do_takeoff, do_takeoff);
  pnh.param("takeoff_alt", takeoff_alt, takeoff_alt);
  pnh.param("do_land", do_land, do_land);
  pnh.param("do_set_param_int", do_set_param_int, do_set_param_int);
  pnh.param("int_param_name", int_param_name, int_param_name);
  pnh.param("int_param_value", int_param_value, int_param_value);
  pnh.param("do_set_param_float", do_set_param_float, do_set_param_float);
  pnh.param("float_param_name", float_param_name, float_param_name);
  pnh.param("float_param_value", float_param_value, float_param_value);

  PX4_StateManager manager;
  try {
    manager.init(nh);
  } catch (const std::exception& e) {
    ROS_FATAL("Failed to init PX4_StateManager: %s", e.what());
    return 1;
  }

  ROS_INFO("px4_manager_test_node started. Target MAVROS namespace: /uav1/mavros");
  ROS_INFO("Safety defaults: all command switches are false unless set via private params.");

  bool mode_sent = false;
  bool arm_sent = false;
  bool takeoff_sent = false;
  bool land_sent = false;
  bool int_param_sent = false;
  bool float_param_sent = false;

  ros::Rate rate(2.0);
  while (ros::ok()) {
    ros::spinOnce();

    if (do_set_mode && !mode_sent) {
      const bool ok = manager.setMode(mode);
      ROS_INFO("[TEST] setMode(%s) -> %s", mode.c_str(), ok ? "success" : "failed");
      mode_sent = true;
    }

    if (do_arm && !arm_sent) {
      const bool ok = manager.setArm(arm_value);
      ROS_INFO("[TEST] setArm(%d) -> %s", arm_value, ok ? "success" : "failed");
      arm_sent = true;
    }

    if (do_takeoff && !takeoff_sent) {
      const bool ok = manager.setTakeoff(takeoff_alt);
      ROS_INFO("[TEST] setTakeoff(alt=%.2f) -> %s", takeoff_alt,
               ok ? "success" : "failed");
      takeoff_sent = true;
    }

    if (do_land && !land_sent) {
      const bool ok = manager.setLand();
      ROS_INFO("[TEST] setLand() -> %s", ok ? "success" : "failed");
      land_sent = true;
    }

    if (do_set_param_int && !int_param_sent) {
      const bool ok = manager.setParamInt(int_param_name, int_param_value);
      ROS_INFO("[TEST] setParamInt(%s=%d) -> %s", int_param_name.c_str(),
               int_param_value, ok ? "success" : "failed");
      int_param_sent = true;
    }

    if (do_set_param_float && !float_param_sent) {
      const bool ok =
          manager.setParamFloat(float_param_name, static_cast<float>(float_param_value));
      ROS_INFO("[TEST] setParamFloat(%s=%.4f) -> %s", float_param_name.c_str(),
               float_param_value, ok ? "success" : "failed");
      float_param_sent = true;
    }

    ROS_INFO_THROTTLE(2.0, "[TEST] waiting... set switches via private params to send commands.");
    rate.sleep();
  }

  return 0;
}
