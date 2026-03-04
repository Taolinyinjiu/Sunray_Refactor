#include "base_controller/base_controller.h"

namespace uav_controller {

bool Base_Controller::ensure_takeoff_completed() const {
    return false;
}

bool Base_Controller::ensure_land_completed() const {
    return false;
}

bool Base_Controller::ensure_emergency_land_completed() const {
    return false;
}

void Base_Controller::set_currentstate(const nav_msgs::Odometry& current_state_msg) {
    current_state = uav_common::UAVStateEstimate(current_state_msg);
}

void Base_Controller::set_emergencystate(const nav_msgs::Odometry& emergency_state_msg) {
    emergency_state = uav_common::UAVStateEstimate(emergency_state_msg);
}

void Base_Controller::set_desiredstate(const nav_msgs::Odometry& desired_state_msg) {
    desired_state = uav_common::UAVStateEstimate(desired_state_msg);
}

}  // namespace uav_controller
