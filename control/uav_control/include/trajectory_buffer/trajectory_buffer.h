#include <sys/types.h>

#include <vector>

#include "control_data_types/control_data_types.h"
#include "ros/time.h"

namespace uav_control {

// 1. 构造轨迹容器
struct TrajectoryVector {
  uint32_t trajectory_id{0};
  ros::Time recive_time;
  ros::Time effective_time;
  std::vector<TrajectoryPoint> points;
};

// 2.构造采样
struct TrajectorySample {
  bool valid{false};
  bool reached_end{false};

  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
  Eigen::Vector3d jerk{Eigen::Vector3d::Zero()};
  Eigen::Vector3d snap{Eigen::Vector3d::Zero()};
  double yaw{0.0};
  double yaw_rate{0.0};
  double yaw_acc{0.0};

  uint32_t trajectory_id{0};
};

// 3. 构造强类型枚举
enum class TrajectoryBufferStatus : uint8_t {
  EMPTY = 0,
  READY = 1,
  COMPLETED = 3,
  ABORT = 4,
  ILLEGAL_START = 5,
  ILLEGAL_FINAL = 6,
  IMPOSSIBLE = 7
};

// 4. 双线程缓存管理器
class TrajectoryBufferManager{
	public:
	// 后台写入，提交一条待生效的轨迹
	bool push_pending(TrajectoryVector traj_){

	}



};




};  // namespace uav_control