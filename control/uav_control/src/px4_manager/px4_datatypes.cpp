#include "px4_manager/px4_datatypes.h"
#include "mavros_msgs/OpticalFlowRad.h"

namespace px4_data{
	opflow_raw_::opflow_raw_()
	{
		timestamp = 0.0;
		quality = 0;
		integration_time_us = 0;
		integrated_x = 0.0;
		integrated_y = 0.0;
		integrated_xgyro = 0.0;
		integrated_ygyro = 0.0;
		integrated_zgyro = 0.0;
		time_delta_distance_us = 0;
		distance = 0.0;
	}

	opflow_raw_::opflow_raw_(const mavros_msgs::OpticalFlowRad &ConstPtr)
	{
		timestamp = ConstPtr.header.stamp.toSec();
		quality = ConstPtr.quality;
		integration_time_us = ConstPtr.integration_time_us;
		integrated_x = ConstPtr.integrated_x;
		integrated_y = ConstPtr.integrated_y;
		integrated_xgyro = ConstPtr.integrated_xgyro;
		integrated_ygyro = ConstPtr.integrated_ygyro;
		integrated_zgyro = ConstPtr.integrated_zgyro;
		time_delta_distance_us = ConstPtr.time_delta_distance_us;
		distance = ConstPtr.distance;
	}
}