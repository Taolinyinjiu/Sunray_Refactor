#include "px4_manager/px4_reader.h"

#include "ros/console.h"
#include "ros/ros.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "px4_reader_test_node");
  ros::NodeHandle nh;

  // 固定测试参数（按你的要求）
  nh.setParam("uav_id", 1);
  nh.setParam("uav_name", std::string("uav"));

  reader_list_ enable_list;
  enable_list.read_system_state = true;
  enable_list.read_ekf2_state = true;
  enable_list.read_flow_state = true;
  enable_list.read_localpose = true;
  enable_list.read_localvel = true;
  enable_list.read_bodypose = true;
  enable_list.read_bodyvel = true;

  PX4_Reader reader(nh, enable_list);

  ROS_INFO("px4_reader_test_node started. Expect MAVROS topics under /uav1/...");

  ros::Rate rate(10.0);
  while (ros::ok()) {
    ros::spinOnce();

    const px4_data::system_state_ sys = reader.get_system_state();
    const px4_data::ekf2_state_ ekf2 = reader.get_ekf2_state();
    const px4_data::opflow_state_ flow = reader.get_flow_state();
    const px4_data::pose_ local_pose = reader.get_local_pose();
    const px4_data::velocity_ local_vel = reader.get_local_velocity();
    const px4_data::pose_ body_pose = reader.get_body_pose();
    const px4_data::velocity_ body_vel = reader.get_body_velocity();
    const px4_data::ekf2_param_ ekf2_param = reader.fetch_ekf2_param();
    const px4_data::attitude_param_ att_param = reader.fetch_attitude_param();
    const px4_data::velocity_param_ vel_param = reader.fetch_velocity_param();
    const px4_data::position_param_ pos_param = reader.fetch_position_param();

    ROS_INFO_THROTTLE(
        1.0,
        "[SYS] connected=%d armed=%d rc=%d mode=%d landed=%d load=%d batt=%.2fV "
        "%.2fA %.1f%%",
        sys.connected, sys.armed, sys.rc_input, static_cast<int>(sys.flight_mode),
        static_cast<int>(sys.landed_state), sys.system_load, sys.voltage,
        sys.current, sys.percent);

    ROS_INFO_THROTTLE(1.0, "[EKF2] stabilize=%d altitude=%d position=%d",
                      ekf2.allow_stabilize, ekf2.allow_altitude,
                      ekf2.allow_position);

    ROS_INFO_THROTTLE(
        1.0,
        "[FLOW] valid=%d q=%u dist=%.2f dt=%.3f raw=(%.2f, %.2f) filt=(%.2f, %.2f) n=%u",
        flow.valid, flow.quality, flow.distance, flow.dt_s, flow.vx_raw,
        flow.vy_raw, flow.vx, flow.vy, flow.sample_count);

    ROS_INFO_THROTTLE(
        1.0,
        "[LOCAL] p=(%.2f, %.2f, %.2f) q=(%.3f, %.3f, %.3f, %.3f) v=(%.2f, %.2f, %.2f) w=(%.2f, %.2f, %.2f)",
        local_pose.position.x(), local_pose.position.y(), local_pose.position.z(),
        local_pose.orientation.w(), local_pose.orientation.x(),
        local_pose.orientation.y(), local_pose.orientation.z(),
        local_vel.linear.x(), local_vel.linear.y(), local_vel.linear.z(),
        local_vel.angular.x(), local_vel.angular.y(), local_vel.angular.z());

    ROS_INFO_THROTTLE(
        1.0,
        "[BODY ] q=(%.3f, %.3f, %.3f, %.3f) v=(%.2f, %.2f, %.2f) w=(%.2f, %.2f, %.2f)",
        body_pose.orientation.w(), body_pose.orientation.x(),
        body_pose.orientation.y(), body_pose.orientation.z(), body_vel.linear.x(),
        body_vel.linear.y(), body_vel.linear.z(), body_vel.angular.x(),
        body_vel.angular.y(), body_vel.angular.z());

    ROS_INFO_THROTTLE(
        2.0,
        "[PARAM-EKF2] EV_CTRL=%d HGT_REF=%d EV_DELAY=%.3f",
        ekf2_param.ev_ctrl, ekf2_param.hgt_ref, ekf2_param.ev_delay);

    ROS_INFO_THROTTLE(
        2.0,
        "[PARAM-ATT] R(%.3f, %.3f, %.3f) P(%.3f, %.3f, %.3f) Y(%.3f, %.3f, %.3f)",
        att_param.roll.kp, att_param.roll.ki, att_param.roll.kd, att_param.pitch.kp,
        att_param.pitch.ki, att_param.pitch.kd, att_param.yaw.kp, att_param.yaw.ki,
        att_param.yaw.kd);

    ROS_INFO_THROTTLE(
        2.0,
        "[PARAM-VEL] XY(%.3f, %.3f, %.3f) Z(%.3f, %.3f, %.3f)",
        vel_param.mpc_xy.kp, vel_param.mpc_xy.ki, vel_param.mpc_xy.kd,
        vel_param.mpc_z.kp, vel_param.mpc_z.ki, vel_param.mpc_z.kd);

    ROS_INFO_THROTTLE(
        2.0,
        "[PARAM-POS] XY(%.3f, %.3f, %.3f) Z(%.3f, %.3f, %.3f)",
        pos_param.mpc_xy.kp, pos_param.mpc_xy.ki, pos_param.mpc_xy.kd,
        pos_param.mpc_z.kp, pos_param.mpc_z.ki, pos_param.mpc_z.kd);

    ROS_INFO_THROTTLE(1.0, "...............................................");
    rate.sleep();
  }

  return 0;
}
