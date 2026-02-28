#pragma once
// 光流环形缓冲区
// 环形缓冲区的意义是，使用一段时间的数据替换突然点位的数据，或者说，用x[0]~x[9]的均值替换x[5]的数据，从而保证连续时间内数据的稳定性
// 环形缓冲区的最小存储单元应该是？

#include <cstdint>
#include <mutex>
#include <vector>
#include "px4_manager/px4_datatypes.h"
// 环形缓冲区最小存储单元
struct opflow_sample {
  // 光流时间戳
  double timestamp;
  // 光流的质量
  uint8_t quality;
  // x y方向的角速度
  float vx_raw, vy_raw;
  // 离地的高度
  float distance;
	opflow_sample(){};
	opflow_sample(px4_data::opflow_raw_ opflow_raw);
};

inline opflow_sample::opflow_sample(px4_data::opflow_raw_ opflow_raw)
{
	// 光流数据时间戳
	timestamp = opflow_raw.timestamp;
	// 光流数据质量
	quality = opflow_raw.quality;
	// 光流离地高度
	distance = opflow_raw.distance;
	// xy方向的角速度
	



};

// 环形缓冲区定义
class Opflow_Buffer {
public:
private:
  // 使用容器进行管理
  std::vector<opflow_sample> buffer_;
  size_t head_ = 0;
  size_t count_ = 0;
  const size_t capacity_;
  std::mutex mtx_;

};
