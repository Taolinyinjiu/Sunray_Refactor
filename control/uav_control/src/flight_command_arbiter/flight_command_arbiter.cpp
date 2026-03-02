#include "flight_command_arbiter/flight_command_arbiter.h"

#include <algorithm>

#include <ros/ros.h>

FlightCommandArbiter::CommandMeta::CommandMeta()
    // 默认策略：
    // - ttl=0.25s：输入流断开时可快速失效；
    // - preemptive=true：允许高优来源抢占；
    // - min_hold=0：默认不强制保持。
    : source(),
      priority(0),
      stamp(0.0),
      ttl(0.25),
      preemptive(true),
      min_hold(0.0) {}

FlightCommandArbiter::ArbitrationResult::ArbitrationResult()
    : has_output(false),
      type(CommandType::kLocal),
      winner_source(),
      winner_priority(0),
      attitude(),
      local() {}

FlightCommandArbiter::WinnerState::WinnerState()
    : valid(false), entry(), selected_at(0.0) {}

FlightCommandArbiter::FlightCommandArbiter()
    : initialized_(false),
      nh_(),
      uav_id_(0),
      uav_name_("uav"),
      attitude_pub_(),
      local_pub_(),
      default_output_type_(CommandType::kLocal),
      hold_last_duration_(0.2),
      has_failsafe_attitude_(false),
      has_failsafe_local_(false),
      failsafe_attitude_(),
      failsafe_local_(),
      entries_(),
      source_priority_overrides_(),
      winner_(),
      last_result_(),
      has_last_result_(false) {}

FlightCommandArbiter::FlightCommandArbiter(ros::NodeHandle& nh)
    : FlightCommandArbiter() {
  init(nh);
}

void FlightCommandArbiter::init(ros::NodeHandle& nh) {
  // 复制 NodeHandle，后续用于发布与参数读取。
  nh_ = nh;

  // 允许参数缺省：uav0。
  nh_.param<int>("uav_id", uav_id_, 0);
  nh_.param<std::string>("uav_name", uav_name_, std::string("uav"));

  const std::string ns = "/" + uav_name_ + std::to_string(uav_id_) + "/mavros";

  // 仲裁器只负责最终单路输出：
  // - 姿态输出到 setpoint_raw/attitude
  // - 本地输出到 setpoint_raw/local
  attitude_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget>(
      ns + "/setpoint_raw/attitude", 10);
  local_pub_ =
      nh_.advertise<mavros_msgs::PositionTarget>(ns + "/setpoint_raw/local", 10);

  initialized_ = true;
}

void FlightCommandArbiter::submitAttitude(
    const CommandMeta& meta, const mavros_msgs::AttitudeTarget& cmd) {
  if (!ensureInitialized("submitAttitude")) {
    return;
  }

  // 将外部输入封装为内部统一缓存单元。
  CommandEntry entry;
  entry.type = CommandType::kAttitude;
  entry.meta = meta;
  entry.attitude = cmd;
  entry.local = mavros_msgs::PositionTarget();

  // 若外部未提供时间戳，按接收时刻补齐，避免“零时间戳直接失效”。
  if (entry.meta.stamp.isZero()) {
    entry.meta.stamp = ros::Time::now();
  }

  // 相同 (source, type) 只保留最新一条。
  addOrReplaceEntry(entry);
}

void FlightCommandArbiter::submitLocal(const CommandMeta& meta,
                                       const mavros_msgs::PositionTarget& cmd) {
  if (!ensureInitialized("submitLocal")) {
    return;
  }

  // 与 submitAttitude 一致，这里仅消息体类型不同。
  CommandEntry entry;
  entry.type = CommandType::kLocal;
  entry.meta = meta;
  entry.local = cmd;
  entry.attitude = mavros_msgs::AttitudeTarget();

  if (entry.meta.stamp.isZero()) {
    entry.meta.stamp = ros::Time::now();
  }

  addOrReplaceEntry(entry);
}

bool FlightCommandArbiter::tick(const ros::Time& now, ArbitrationResult* out) {
  if (!ensureInitialized("tick")) {
    return false;
  }

  // 第一步：清理所有过期输入，缩小候选集合。
  removeExpiredEntries(now);

  // 第二步：从剩余有效输入中选择“静态最优候选”（优先级+时间戳）。
  CommandEntry candidate;
  const bool has_candidate = selectBestCandidate(now, &candidate);

  if (!has_candidate) {
    // 无任何有效输入：清空赢家并进入 failsafe 分支。
    winner_.valid = false;

    ArbitrationResult fallback;
    const bool has_fallback = buildFallbackResult(&fallback);
    if (!has_fallback) {
      if (out != nullptr) {
        *out = ArbitrationResult();
      }
      return false;
    }

    fallback.has_output = true;
    if (out != nullptr) {
      *out = fallback;
    }
    last_result_ = fallback;
    has_last_result_ = true;
    return true;
  }

  // 第三步：决定是否切换赢家。
  // 触发切换条件：
  // 1) 当前无赢家；
  // 2) 当前赢家已失效；
  // 3) candidate 满足 shouldSwitchWinner 的抢占/切换规则。
  if (!winner_.valid || !isEntryValid(winner_.entry, now) ||
      shouldSwitchWinner(candidate, now)) {
    winner_.valid = true;
    winner_.entry = candidate;
    // 记录“成为赢家”的时刻，用于 min_hold 判断。
    winner_.selected_at = now;
  }

  // 第四步：由赢家构建输出。
  ArbitrationResult result;
  if (!buildResultFromEntry(winner_.entry, &result)) {
    return false;
  }
  result.has_output = true;

  if (out != nullptr) {
    *out = result;
  }
  last_result_ = result;
  has_last_result_ = true;
  return true;
}

bool FlightCommandArbiter::tickAndPublish(const ros::Time& now,
                                          ArbitrationResult* out) {
  // 复用 tick 结果，避免发布路径与仲裁路径分叉。
  ArbitrationResult result;
  const bool ok = tick(now, &result);
  if (!ok) {
    if (out != nullptr) {
      *out = ArbitrationResult();
    }
    return false;
  }

  publish(result);

  if (out != nullptr) {
    *out = result;
  }
  return true;
}

void FlightCommandArbiter::setSourcePriority(const std::string& source,
                                             int priority) {
  // 运行期覆盖优先级（不改原始输入 meta）。
  source_priority_overrides_[source] = priority;
}

void FlightCommandArbiter::clearSource(const std::string& source) {
  // 删除该来源的全部命令（姿态/本地都清理）。
  entries_.erase(std::remove_if(entries_.begin(), entries_.end(),
                                [&source](const CommandEntry& entry) {
                                  return entry.meta.source == source;
                                }),
                 entries_.end());

  // 若当前赢家恰好来自该来源，则立即失效，等待下一次 tick 重选。
  if (winner_.valid && winner_.entry.meta.source == source) {
    winner_.valid = false;
  }
}

void FlightCommandArbiter::clearAll() {
  entries_.clear();
  winner_.valid = false;
}

void FlightCommandArbiter::setDefaultOutputType(CommandType type) {
  default_output_type_ = type;
}

void FlightCommandArbiter::setHoldLastDuration(
    const ros::Duration& hold_last_duration) {
  hold_last_duration_ = hold_last_duration;
}

void FlightCommandArbiter::setFailsafeAttitude(
    const mavros_msgs::AttitudeTarget& cmd) {
  failsafe_attitude_ = cmd;
  has_failsafe_attitude_ = true;
}

void FlightCommandArbiter::setFailsafeLocal(const mavros_msgs::PositionTarget& cmd) {
  failsafe_local_ = cmd;
  has_failsafe_local_ = true;
}

void FlightCommandArbiter::clearFailsafeAttitude() {
  has_failsafe_attitude_ = false;
}

void FlightCommandArbiter::clearFailsafeLocal() {
  has_failsafe_local_ = false;
}

bool FlightCommandArbiter::getLastWinner(ArbitrationResult* out) const {
  if (!has_last_result_ || out == nullptr) {
    return false;
  }
  *out = last_result_;
  return true;
}

void FlightCommandArbiter::addOrReplaceEntry(const CommandEntry& entry) {
  // 设计选择：每个来源每种类型仅维护最后一条，避免旧命令堆积导致仲裁噪声。
  for (std::size_t i = 0; i < entries_.size(); ++i) {
    CommandEntry& existing = entries_[i];
    if (existing.type == entry.type && existing.meta.source == entry.meta.source) {
      existing = entry;
      return;
    }
  }
  entries_.push_back(entry);
}

void FlightCommandArbiter::removeExpiredEntries(const ros::Time& now) {
  // 统一过期判定：复用 isEntryValid，避免逻辑分叉。
  entries_.erase(std::remove_if(entries_.begin(), entries_.end(),
                                [this, &now](const CommandEntry& entry) {
                                  return !isEntryValid(entry, now);
                                }),
                 entries_.end());
}

bool FlightCommandArbiter::selectBestCandidate(const ros::Time& now,
                                               CommandEntry* selected) const {
  if (selected == nullptr) {
    return false;
  }

  bool found = false;
  for (std::size_t i = 0; i < entries_.size(); ++i) {
    const CommandEntry& entry = entries_[i];
    if (!isEntryValid(entry, now)) {
      continue;
    }

    if (!found) {
      *selected = entry;
      found = true;
      continue;
    }

    // 排序规则：
    // 1) 优先级更高者胜；
    // 2) 同优先级时，时间戳更新者胜（新命令覆盖旧命令）。
    const int entry_prio = effectivePriority(entry);
    const int selected_prio = effectivePriority(*selected);

    if (entry_prio > selected_prio) {
      *selected = entry;
      continue;
    }

    if (entry_prio == selected_prio && entry.meta.stamp > selected->meta.stamp) {
      *selected = entry;
    }
  }

  return found;
}

bool FlightCommandArbiter::isEntryValid(const CommandEntry& entry,
                                        const ros::Time& now) const {
  if (entry.meta.stamp.isZero()) {
    return false;
  }

  // ttl<=0 约定为“永久有效”（由外部负责主动清除）。
  if (entry.meta.ttl.toSec() <= 0.0) {
    return true;
  }

  return (now - entry.meta.stamp) <= entry.meta.ttl;
}

bool FlightCommandArbiter::sameEntry(const CommandEntry& a,
                                     const CommandEntry& b) const {
  return a.type == b.type && a.meta.source == b.meta.source;
}

int FlightCommandArbiter::effectivePriority(const CommandEntry& entry) const {
  // 若存在 source 覆盖，则覆盖优先于命令自带 priority。
  std::map<std::string, int>::const_iterator it =
      source_priority_overrides_.find(entry.meta.source);
  if (it != source_priority_overrides_.end()) {
    return it->second;
  }
  return entry.meta.priority;
}

bool FlightCommandArbiter::shouldSwitchWinner(const CommandEntry& candidate,
                                              const ros::Time& now) const {
  // 基础兜底：无赢家或赢家无效，直接切换。
  if (!winner_.valid) {
    return true;
  }

  if (!isEntryValid(winner_.entry, now)) {
    return true;
  }

  if (sameEntry(winner_.entry, candidate)) {
    // 同一来源同一类型，不做“切换”，沿用当前赢家即可。
    return false;
  }

  // 规则A：赢家最小保持时间。保持窗内禁止切换。
  const ros::Duration hold_until = winner_.entry.meta.min_hold;
  if (hold_until.toSec() > 0.0 && (now - winner_.selected_at) < hold_until) {
    return false;
  }

  // 规则B：全局“最后命令保持窗口”。用于抑制短时抖动来源造成的频繁切换。
  if (hold_last_duration_.toSec() > 0.0 &&
      (now - winner_.entry.meta.stamp) <= hold_last_duration_) {
    return false;
  }

  const int winner_prio = effectivePriority(winner_.entry);
  const int candidate_prio = effectivePriority(candidate);

  // 规则C：优先级判定 + 抢占许可。
  if (candidate_prio > winner_prio) {
    return candidate.meta.preemptive;
  }

  // 同优先级下，只有“更晚命令且允许抢占”才切换。
  if (candidate_prio == winner_prio) {
    return candidate.meta.preemptive &&
           candidate.meta.stamp > winner_.entry.meta.stamp;
  }

  return false;
}

bool FlightCommandArbiter::buildResultFromEntry(const CommandEntry& entry,
                                                ArbitrationResult* out) const {
  if (out == nullptr) {
    return false;
  }

  ArbitrationResult result;
  result.has_output = true;
  result.type = entry.type;
  result.winner_source = entry.meta.source;
  result.winner_priority = effectivePriority(entry);

  // 按类型写入对应消息体，另一个消息体保持默认值即可。
  if (entry.type == CommandType::kAttitude) {
    result.attitude = entry.attitude;
  } else {
    result.local = entry.local;
  }

  *out = result;
  return true;
}

bool FlightCommandArbiter::buildFallbackResult(ArbitrationResult* out) const {
  if (out == nullptr) {
    return false;
  }

  ArbitrationResult result;
  result.has_output = true;
  result.winner_source = "failsafe";
  result.winner_priority = -1;

  // 兜底选择顺序：
  // 1) 优先采用 default_output_type_ 指向的 failsafe；
  // 2) 若该类型不存在，则退化到另一类型 failsafe；
  // 3) 都不存在则返回 false（表示本帧无输出）。
  if (default_output_type_ == CommandType::kLocal && has_failsafe_local_) {
    result.type = CommandType::kLocal;
    result.local = failsafe_local_;
    *out = result;
    return true;
  }

  if (default_output_type_ == CommandType::kAttitude && has_failsafe_attitude_) {
    result.type = CommandType::kAttitude;
    result.attitude = failsafe_attitude_;
    *out = result;
    return true;
  }

  if (has_failsafe_local_) {
    result.type = CommandType::kLocal;
    result.local = failsafe_local_;
    *out = result;
    return true;
  }

  if (has_failsafe_attitude_) {
    result.type = CommandType::kAttitude;
    result.attitude = failsafe_attitude_;
    *out = result;
    return true;
  }

  return false;
}

void FlightCommandArbiter::publish(const ArbitrationResult& result) {
  if (!result.has_output) {
    return;
  }

  // 只发布一个最终话题，避免 attitude/local 双流同时驱动导致冲突。
  if (result.type == CommandType::kAttitude) {
    attitude_pub_.publish(result.attitude);
    return;
  }

  local_pub_.publish(result.local);
}

bool FlightCommandArbiter::ensureInitialized(const char* api) const {
  if (initialized_) {
    return true;
  }

  // 统一入口告警，便于定位外部未调用 init 的使用错误。
  ROS_WARN("FlightCommandArbiter::%s called before init(nh)", api);
  return false;
}
