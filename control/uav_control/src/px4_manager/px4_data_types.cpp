#include "px4_manager/px4_data_types.h"

namespace px4_data {

OpticalFlowRaw::OpticalFlowRaw()
    : timestamp(0.0),
      quality(0),
      integration_time_us(0),
      integrated_x(0.0f),
      integrated_y(0.0f),
      integrated_xgyro(0.0f),
      integrated_ygyro(0.0f),
      integrated_zgyro(0.0f),
      time_delta_distance_us(0),
      distance(0.0f) {}

OpticalFlowRaw::OpticalFlowRaw(const mavros_msgs::OpticalFlowRad& msg)
    : timestamp(msg.header.stamp.toSec()),
      quality(msg.quality),
      integration_time_us(msg.integration_time_us),
      integrated_x(msg.integrated_x),
      integrated_y(msg.integrated_y),
      integrated_xgyro(msg.integrated_xgyro),
      integrated_ygyro(msg.integrated_ygyro),
      integrated_zgyro(msg.integrated_zgyro),
      time_delta_distance_us(msg.time_delta_distance_us),
      distance(msg.distance) {}

}  // namespace px4_data
