1. 我们需要FSM为我们提供什么？
a. 当前的FSM自身状态
b. 当前的无人机定位数据
c. 


```cpp
// Sunray_FSM以10Hz的频率向外发布状态机状态，数据类型为
struct Sunray_FSM_Status{
	// 当前状态机的状态,起飞，悬停，降落，运动...
	sunray_fsm::FSM_State uav_state;
	// 目标的数据信息？
	Eigen::Vector3d position;	
	Eigen::Vector3d velocity;	
	

	// 当前无人机的状态
	Eigen::Vector3d position;
	Eigen::Vector3d velocity_linear;
	Eigen::Vector3d velocity_angular;
	Eigen::Vector3d attitude_rpy_deg;
	Eigen::Quaterniond attitude_quat;
	// 
}
```

https://robomaster-dev.readthedocs.io/zh-cn/latest/python_sdk/robomaster.html#module-robomaster.flight

Sunray_Helper本质上是对Sunray_Drone动作类接口的封装，根据无人机动作本身特性的不同，分为即时动作控制和任务动作控制
1. 任务动作控制：指的是需要持续一段时间才能完成的动作，比如无人机向前飞行一米，需要飞行一段时间才能到达对应的位置