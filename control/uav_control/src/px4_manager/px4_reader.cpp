#include "px4_manager/px4_reader.h"

PX4_Reader::PX4_Reader(){};
PX4_Reader::~PX4_Reader(){};

void PX4_Reader::init(ros::NodeHandle& nh)
{
	int uav_id;
	read_uavid_flag = nh.getParam("uav_id",uav_id);
	
}















reader_types::FlightMode PX4_Reader::flightmode_fromString(const std::string& mode)
{
  if (mode == "OFFBOARD") return reader_types::FlightMode::OFFBOARD;
  if (mode == "POSCTL") return reader_types::FlightMode::POSCTL;
  if (mode == "ALTCTL") return reader_types::FlightMode::ALTCTL;
  if (mode == "AUTO.TAKEOFF") return reader_types::FlightMode::TAKEOFF;
  if (mode == "AUTO.LAND") return reader_types::FlightMode::LAND;
  if (mode == "AUTO.RTL") return reader_types::FlightMode::RTL;
  if (mode == "AUTO.MISSION") return reader_types::FlightMode::MISSION;
  if (mode == "AUTO.LOITER") return reader_types::FlightMode::LOITER;
  if (mode == "MANUAL") return reader_types::FlightMode::MANUAL;
  if (mode == "STABILIZED") return reader_types::FlightMode::STABILIZED;
  if (mode == "ACRO") return reader_types::FlightMode::ACRO;
  return reader_types::FlightMode::UNKNOWN;
}