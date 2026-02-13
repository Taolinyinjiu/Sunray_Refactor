#include <mutex>

#include "mavros_msgs/ExtendedState.h"
#include "px4_manager/px4_reader.h"

void PX4_Reader::stateCallback(const mavros_msgs::State::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(system_state_mtx);
  system_state.connected = msg->connected;
  system_state.armed = msg->armed;
  // 基于最小实现原则，这里没有guided
  system_state.rc_input = msg->manual_input;
  system_state.flight_mode = flightmode_fromString(msg->mode);
  // 基于最小实现原则，这里没有system_status
}

void PX4_Reader::exstateCallback(
    const mavros_msgs::ExtendedState::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(system_state_mtx);
  system_state.landed_state =
      static_cast<reader_types::LandedState>(msg->landed_state);
  // 基于最小实现原则，这里没有vtol状态，也不需要实现它
}

void PX4_Reader::sysCallback(const mavros_msgs::SysStatus::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(system_state_mtx);\
	system_state.system_load = msg->load/10.0;
	system_state.voltage = msg->voltage_battery / 1000.0;
	system_state.current = msg->current_battery / 100.0;
	system_state.percent = msg->battery_remaining;
}