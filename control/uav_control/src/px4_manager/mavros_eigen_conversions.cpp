#include "px4_manager/mavros_eigen_conversions.h"
#include <ros/ros.h>

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "test_node");
	ros::NodeHandle nh;
	
	ROS_INFO("Test node started successfully!");
	
	ros::spin();
	
	return 0;
}