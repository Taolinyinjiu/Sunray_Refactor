#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <signal.h>
#include "sunray_log.hpp"

ros::Publisher mocap_odom_pub;
geometry_msgs::TwistStamped latest_twist_msg;
bool has_twist_msg = false;
bool require_twist = false;

// 中断信号
void MySigintHandler(int sig) {

    ros::shutdown();
}

void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg) {

    nav_msgs::Odometry odom_msg;

    odom_msg.header.stamp = pose_msg->header.stamp;
    odom_msg.header.frame_id = "world";
    odom_msg.child_frame_id = "base_link";

    // Pose
    odom_msg.pose.pose.position.x = pose_msg->pose.position.x;
    odom_msg.pose.pose.position.y = pose_msg->pose.position.y;
    odom_msg.pose.pose.position.z = pose_msg->pose.position.z;
    odom_msg.pose.pose.orientation.x = pose_msg->pose.orientation.x;
    odom_msg.pose.pose.orientation.y = pose_msg->pose.orientation.y;
    odom_msg.pose.pose.orientation.z = pose_msg->pose.orientation.z;
    odom_msg.pose.pose.orientation.w = pose_msg->pose.orientation.w;

    if (require_twist && !has_twist_msg) {
        ROS_WARN_THROTTLE(1.0, "No twist message received yet, skip odom publish.");
        return;
    }

    if (has_twist_msg) {
        // Fill with latest twist to avoid strict timestamp sync dependency.
        odom_msg.twist.twist.linear.x = latest_twist_msg.twist.linear.x;
        odom_msg.twist.twist.linear.y = latest_twist_msg.twist.linear.y;
        odom_msg.twist.twist.linear.z = latest_twist_msg.twist.linear.z;
        odom_msg.twist.twist.angular.x = latest_twist_msg.twist.angular.x;
        odom_msg.twist.twist.angular.y = latest_twist_msg.twist.angular.y;
        odom_msg.twist.twist.angular.z = latest_twist_msg.twist.angular.z;
    }

    mocap_odom_pub.publish(odom_msg);
}

void TwistCallback(const geometry_msgs::TwistStamped::ConstPtr& twist_msg) {
    latest_twist_msg = *twist_msg;
    has_twist_msg = true;
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "mocap_odom_node");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // 中断信号注册
    signal(SIGINT, MySigintHandler);

    int uav_id = 1;
    pnh.param<int>("uav_id", uav_id, 1);

    std::string odom_pub_topic_default = "/uav" + std::to_string(uav_id) + "/sunray/odometry";
    std::string odom_pub_topic = odom_pub_topic_default;
    pnh.param<std::string>("odom_topic", odom_pub_topic, odom_pub_topic_default);
    mocap_odom_pub = nh.advertise<nav_msgs::Odometry>(odom_pub_topic, 10);

    std::string pose_sub_topic_default = "/vrpn_client_node_1/uav" + std::to_string(uav_id) + "/pose";
    std::string twist_sub_topic_default = "/vrpn_client_node_1/uav" + std::to_string(uav_id) + "/twist";
    std::string pose_sub_topic = pose_sub_topic_default;
    std::string twist_sub_topic = twist_sub_topic_default;
    pnh.param<std::string>("pose_topic", pose_sub_topic, pose_sub_topic_default);
    pnh.param<std::string>("twist_topic", twist_sub_topic, twist_sub_topic_default);
    pnh.param<bool>("require_twist", require_twist, false);

    ros::Subscriber mocap_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(pose_sub_topic, 100, PoseCallback);
    ros::Subscriber mocap_twist_sub = nh.subscribe<geometry_msgs::TwistStamped>(twist_sub_topic, 100, TwistCallback);

    SUNRAY_INFO("Subscribe pose topic: {}", pose_sub_topic);
    SUNRAY_INFO("Subscribe twist topic: {}", twist_sub_topic);
    SUNRAY_INFO("Publish odometry topic: {}", odom_pub_topic);

    ros::spin();
    return 0;
}
