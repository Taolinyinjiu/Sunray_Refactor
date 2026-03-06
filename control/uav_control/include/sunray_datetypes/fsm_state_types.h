/***
	@brief 与sunray_fsm有关的数据类型，主要指的是sunray_fsm自身会用到的数据类型以及向发布的数据类型
*/
#pragma once

namespace sunray_control{

// 状态机的状态声明，使用强类型进行枚举
enum class control_state{
	OFF,			
	TAKEOFF,
	LAND,
	EMERGENCY_LAND,
	RETURN,
	HOVER,
	POSITION_CONTROL,
	VELOCITY_CONTROL,
	ATTITUDE_CONTROL,
	COMPLEX_CONTROL,
	TRAJECTORY_CONTROL
};




};