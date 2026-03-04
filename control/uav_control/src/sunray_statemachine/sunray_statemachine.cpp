#include "sunray_statemachine/sunray_statemachine.h"

Sunray_StateMachine::Sunray_StateMachine(ros::NodeHandle& nh)
    : nh_(nh), current_state_(SunrayState::OFF), state_available_(false), takeoff_callback_ready_(false) {}

bool Sunray_StateMachine::register_controller(const std::shared_ptr<uav_controller::Base_Controller>& controller) {
    if (!controller) {
        ROS_WARN("[SunrayFSM] register_controller failed: null controller");
        return false;
    }

    controller_ = controller;
    ROS_INFO("[SunrayFSM] register global controller success");
    return true;
}

bool Sunray_StateMachine::dispatch(SunrayEvent event) {
    const SunrayState prev = current_state_;

    switch (current_state_) {
        case SunrayState::OFF:
            if (event == SunrayEvent::TAKEOFF_REQUEST && can_takeoff()) {
                return transition_to(SunrayState::TAKEOFF);
            }
            break;

        case SunrayState::TAKEOFF:
            if (event == SunrayEvent::TAKEOFF_COMPLETED) {
                return transition_to(SunrayState::HOVER);
            }
            if (event == SunrayEvent::WATCHDOG_ERROR || event == SunrayEvent::EMERGENCY_REQUEST) {
                return transition_to(SunrayState::EMERGENCY_LAND);
            }
            break;

        case SunrayState::HOVER:
            if (event == SunrayEvent::LAND_REQUEST) {
                return transition_to(SunrayState::LAND);
            }
            if (event == SunrayEvent::WATCHDOG_ERROR || event == SunrayEvent::EMERGENCY_REQUEST) {
                return transition_to(SunrayState::EMERGENCY_LAND);
            }
            if (event == SunrayEvent::ENTER_VELOCITY_CONTROL) {
                return transition_to(SunrayState::VELOCITY_CONTROL);
            }
            if (event == SunrayEvent::ENTER_POSE_CONTROL) {
                return transition_to(SunrayState::POSE_CONTROL);
            }
            if (event == SunrayEvent::ENTER_REFERENCE_CONTROL) {
                return transition_to(SunrayState::REFERENCE_CONTROL);
            }
            if (event == SunrayEvent::ENTER_TRAJECTORY_CONTROL) {
                return transition_to(SunrayState::TRAJECTORY_CONTROL);
            }
            break;

        case SunrayState::LAND:
            if (event == SunrayEvent::LAND_COMPLETED) {
                return transition_to(SunrayState::OFF);
            }
            if (event == SunrayEvent::WATCHDOG_ERROR || event == SunrayEvent::EMERGENCY_REQUEST) {
                return transition_to(SunrayState::EMERGENCY_LAND);
            }
            break;

        case SunrayState::EMERGENCY_LAND:
            if (event == SunrayEvent::EMERGENCY_COMPLETED) {
                return transition_to(SunrayState::OFF);
            }
            break;

        case SunrayState::VELOCITY_CONTROL:
        case SunrayState::POSE_CONTROL:
        case SunrayState::REFERENCE_CONTROL:
            if (event == SunrayEvent::EXIT_CONTROL_MODE) {
                return transition_to(SunrayState::BREAKING);
            }
            if (event == SunrayEvent::WATCHDOG_ERROR || event == SunrayEvent::EMERGENCY_REQUEST) {
                return transition_to(SunrayState::EMERGENCY_LAND);
            }
            break;

        case SunrayState::TRAJECTORY_CONTROL:
            if (event == SunrayEvent::TRAJECTORY_COMPLETED) {
                return transition_to(SunrayState::HOVER);
            }
            if (event == SunrayEvent::EXIT_CONTROL_MODE) {
                return transition_to(SunrayState::BREAKING);
            }
            if (event == SunrayEvent::WATCHDOG_ERROR || event == SunrayEvent::EMERGENCY_REQUEST) {
                return transition_to(SunrayState::EMERGENCY_LAND);
            }
            break;

        case SunrayState::BREAKING:
            if (event == SunrayEvent::BREAKING_COMPLETED) {
                return transition_to(SunrayState::HOVER);
            }
            if (event == SunrayEvent::WATCHDOG_ERROR || event == SunrayEvent::EMERGENCY_REQUEST) {
                return transition_to(SunrayState::EMERGENCY_LAND);
            }
            break;
    }

    ROS_WARN("[SunrayFSM] ignore event %s at state %s", to_string(event), to_string(prev));
    return false;
}

void Sunray_StateMachine::update() {
    if (!check_health()) {
        if (current_state_ != SunrayState::EMERGENCY_LAND) {
            transition_to(SunrayState::EMERGENCY_LAND);
        }
    }

    const std::shared_ptr<uav_controller::Base_Controller> controller = get_controller();
    if (!controller) {
        ROS_WARN_THROTTLE(1.0, "[SunrayFSM] no controller registered");
        return;
    }

    switch (current_state_) {
        case SunrayState::TAKEOFF:
            controller->set_takeoff_mode();
            break;
        case SunrayState::LAND:
            controller->set_land_mode();
            break;
        case SunrayState::EMERGENCY_LAND:
            controller->set_emergency_mode();
            break;
        default:
            (void)controller->update();
            break;
    }
}

void Sunray_StateMachine::set_state_available(bool ready) {
    state_available_ = ready;
}

void Sunray_StateMachine::set_takeoff_callback_ready(bool ready) {
    takeoff_callback_ready_ = ready;
}

SunrayState Sunray_StateMachine::current_state() const {
    return current_state_;
}

const char* Sunray_StateMachine::to_string(SunrayState state) {
    switch (state) {
        case SunrayState::OFF:
            return "OFF";
        case SunrayState::TAKEOFF:
            return "TAKEOFF";
        case SunrayState::HOVER:
            return "HOVER";
        case SunrayState::LAND:
            return "LAND";
        case SunrayState::EMERGENCY_LAND:
            return "EMERGENCY_LAND";
        case SunrayState::VELOCITY_CONTROL:
            return "VELOCITY_CONTROL";
        case SunrayState::POSE_CONTROL:
            return "POSE_CONTROL";
        case SunrayState::REFERENCE_CONTROL:
            return "REFERENCE_CONTROL";
        case SunrayState::TRAJECTORY_CONTROL:
            return "TRAJECTORY_CONTROL";
        case SunrayState::BREAKING:
            return "BREAKING";
        default:
            return "UNKNOWN_STATE";
    }
}

const char* Sunray_StateMachine::to_string(SunrayEvent event) {
    switch (event) {
        case SunrayEvent::TAKEOFF_REQUEST:
            return "TAKEOFF_REQUEST";
        case SunrayEvent::TAKEOFF_COMPLETED:
            return "TAKEOFF_COMPLETED";
        case SunrayEvent::LAND_REQUEST:
            return "LAND_REQUEST";
        case SunrayEvent::LAND_COMPLETED:
            return "LAND_COMPLETED";
        case SunrayEvent::EMERGENCY_REQUEST:
            return "EMERGENCY_REQUEST";
        case SunrayEvent::EMERGENCY_COMPLETED:
            return "EMERGENCY_COMPLETED";
        case SunrayEvent::WATCHDOG_ERROR:
            return "WATCHDOG_ERROR";
        case SunrayEvent::ENTER_VELOCITY_CONTROL:
            return "ENTER_VELOCITY_CONTROL";
        case SunrayEvent::ENTER_POSE_CONTROL:
            return "ENTER_POSE_CONTROL";
        case SunrayEvent::ENTER_REFERENCE_CONTROL:
            return "ENTER_REFERENCE_CONTROL";
        case SunrayEvent::ENTER_TRAJECTORY_CONTROL:
            return "ENTER_TRAJECTORY_CONTROL";
        case SunrayEvent::EXIT_CONTROL_MODE:
            return "EXIT_CONTROL_MODE";
        case SunrayEvent::TRAJECTORY_COMPLETED:
            return "TRAJECTORY_COMPLETED";
        case SunrayEvent::BREAKING_COMPLETED:
            return "BREAKING_COMPLETED";
        default:
            return "UNKNOWN_EVENT";
    }
}

bool Sunray_StateMachine::check_health_preflight() {
    if (!controller_) {
        ROS_WARN("[SunrayFSM] preflight check failed: controller is not registered");
        return false;
    }

    if (!state_available_) {
        ROS_WARN("[SunrayFSM] preflight check failed: state is unavailable");
        return false;
    }

    if (!takeoff_callback_ready_) {
        ROS_WARN("[SunrayFSM] preflight check failed: takeoff callback is not ready");
        return false;
    }

    if (!validate_offstage_odometry_source()) {
        ROS_WARN("[SunrayFSM] preflight check failed: odometry source is invalid in OFF stage");
        return false;
    }

    return true;
}

bool Sunray_StateMachine::check_health() {
    // 当前为占位检查：
    // 1) 起飞前由 check_health_preflight() 执行完整门控；
    // 2) 飞行中后续可扩展里程计稳定性、控制器输出饱和等检查。
    if (current_state_ == SunrayState::OFF) {
        return true;
    }

    return controller_ != nullptr;
}

bool Sunray_StateMachine::can_takeoff() const {
    return state_available_ && takeoff_callback_ready_ && validate_offstage_odometry_source() &&
           static_cast<bool>(controller_);
}

bool Sunray_StateMachine::transition_to(SunrayState next_state) {
    if (current_state_ == next_state) {
        return true;
    }

    if (current_state_ == SunrayState::OFF && next_state == SunrayState::TAKEOFF && !check_health_preflight()) {
        ROS_WARN("[SunrayFSM] transition blocked: OFF -> TAKEOFF preflight check failed");
        return false;
    }

    ROS_INFO("[SunrayFSM] transition: %s -> %s", to_string(current_state_), to_string(next_state));
    current_state_ = next_state;
    return true;
}

std::shared_ptr<uav_controller::Base_Controller> Sunray_StateMachine::get_controller() const {
    return controller_;
}
