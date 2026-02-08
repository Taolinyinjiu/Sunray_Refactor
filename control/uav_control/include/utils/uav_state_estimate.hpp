#pragma once

#include <Eigen/Dense>
#include "utils/geometry_eigen_conversions.h"
#include <nav_msgs/Odometry.h>
#include <ros/time.h>

namespace uav_common {
struct UAVStateEstimate {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    UAVStateEstimate();
    UAVStateEstimate(const nav_msgs::Odometry& state_estimate_msg);

    nav_msgs::Odometry toRosMessage() const;
    bool isVaild() const;

    ros::Time timestamp;
    enum class CoordinateFrame { INVALID, WORLD, LOCAL } coordinate_frame;
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector3d bodyrates;
    Eigen::Quaterniond orientation;
};
}  // namespace uav_common

namespace uav_common {
inline UAVStateEstimate::UAVStateEstimate()
    : timestamp(ros::Time::now()), coordinate_frame(CoordinateFrame::INVALID),
      position(Eigen::Vector3d::Zero()), velocity(Eigen::Vector3d::Zero()),
      bodyrates(Eigen::Vector3d::Zero()), orientation(Eigen::Quaterniond::Identity()) {}

inline UAVStateEstimate::UAVStateEstimate(const nav_msgs::Odometry& state_estimate_msg) {
    timestamp = state_estimate_msg.header.stamp;
    coordinate_frame = CoordinateFrame::INVALID;
    // compare the frame
    if (state_estimate_msg.header.frame_id.compare("world") == 0) {
        coordinate_frame = CoordinateFrame::WORLD;
    } else if (state_estimate_msg.header.frame_id.compare("local") == 0) {
        coordinate_frame = CoordinateFrame::LOCAL;
    }
    position = geometryToEigen(state_estimate_msg.pose.pose.position);
    velocity = geometryToEigen(state_estimate_msg.twist.twist.linear);
    bodyrates = geometryToEigen(state_estimate_msg.twist.twist.angular);
    orientation = geometryToEigen(state_estimate_msg.pose.pose.orientation);
}

inline nav_msgs::Odometry UAVStateEstimate::toRosMessage() const {
    nav_msgs::Odometry msg;
    msg.header.stamp = timestamp;
    switch (coordinate_frame) {
    case CoordinateFrame::WORLD:
        msg.header.frame_id = "world";
        break;
    case CoordinateFrame::LOCAL:
        msg.header.frame_id = "local";
        break;
    default:
        msg.header.frame_id = "invalid";
        break;
    }
    msg.child_frame_id = "body";
    msg.pose.pose.position = vectorToPoint(eigenToGeometry(position));
    msg.twist.twist.linear = eigenToGeometry(velocity);
    msg.pose.pose.orientation = eigenToGeometry(orientation);
    msg.twist.twist.angular = eigenToGeometry(bodyrates);
    return msg;
}

inline bool UAVStateEstimate::isVaild() const {
    if (coordinate_frame == CoordinateFrame::INVALID) {
        return false;
    }
    // 欧里几得范数,即向量的长度,是否为NAN
    if (std::isnan(position.norm())) {
        return false;
    }
    if (std::isnan(velocity.norm())) {
        return false;
    }
    if (std::isnan(orientation.norm())) {
        return false;
    }
    if (std::isnan(bodyrates.norm())) {
        return false;
    }

    return true;
}

}  // namespace uav_common
