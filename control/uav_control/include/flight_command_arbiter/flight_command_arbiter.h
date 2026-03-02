/**
 * @file flight_command_arbiter.h
 * @brief PX4 控制指令仲裁类
 *
 * @details
 * 这个头文件实现了一个PX4 控制指令的仲裁类，该类接收Sunray框架中对PX4的控制指令，并根据输入中不同的优先级来仲裁，仲裁指的是当同时或者同一时间段内接受到来自两个不同输入接口的控制指令时，选择那个进行输出的阶段
 *
 * 设计意图：
 * - 简化其他节点与Mavros的交互,优化Sunray框架的流程
 * - 
 *
 * 实现功能：
 * 1. 实现flight_command_arbiter类，该类向ROS中注册一个仲裁节点，并且具有唯一性，当ROS环节中存在同名的仲裁节点时，停止当前节点的注册，并输出ERROR
 * 2. 实现控制信号结构体，包含优先级等 

 * @author taolinyinjiu
 * @date 2026-03-02
 * @version 0.1
 *
 */

#pragma once

#include <map>
#include <string>
#include <vector>

#include <ros/node_handle.h>
#include <ros/time.h>

#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>

class FlightCommandArbiter {
public:
  // 当前仲裁器支持两类输出命令：姿态推力 或 本地位置/速度类 setpoint。
  enum class CommandType {
    kAttitude,
    kLocal,
  };

  // 输入命令的元信息。仲裁时只基于该信息决定“谁赢”，消息体本身不参与排序。
  struct CommandMeta {
    // 命令来源标识（例如 "px4_manager"、"mission"、"avoidance"）。
    std::string source;
    // 基础优先级，值越大优先级越高。
    int priority;
    // 命令时间戳。若提交时为零，则在 submit* 中自动填充为 ros::Time::now()。
    ros::Time stamp;
    // 生存时间。now - stamp > ttl 则视为过期。ttl<=0 表示永不过期。
    ros::Duration ttl;
    // 是否允许该来源对当前赢家进行抢占。
    bool preemptive;
    // 该来源一旦成为赢家，最短保持时长；保持期内不允许切换到其他来源。
    ros::Duration min_hold;

    CommandMeta();
  };

  // 单次仲裁结果。为避免动态分配，两个消息体字段均保留，按 type 使用其一。
  struct ArbitrationResult {
    // 本次是否产生有效输出。
    bool has_output;
    // 当前输出命令类型。
    CommandType type;
    // 胜出来源名称；若进入兜底输出，通常为 "failsafe"。
    std::string winner_source;
    // 胜出优先级（已应用 source 覆盖后）。
    int winner_priority;
    // 当 type == kAttitude 时有效。
    mavros_msgs::AttitudeTarget attitude;
    // 当 type == kLocal 时有效。
    mavros_msgs::PositionTarget local;

    ArbitrationResult();
  };

  FlightCommandArbiter();
  explicit FlightCommandArbiter(ros::NodeHandle& nh);
  ~FlightCommandArbiter() = default;

  // 初始化：
  // 1) 读取 uav_id / uav_name 参数；
  // 2) 建立最终输出 publisher。
  void init(ros::NodeHandle& nh);

  // 提交姿态命令与本地命令。相同 (source, type) 会覆盖旧值，避免队列无限增长。
  void submitAttitude(const CommandMeta& meta,
                      const mavros_msgs::AttitudeTarget& cmd);
  void submitLocal(const CommandMeta& meta, const mavros_msgs::PositionTarget& cmd);

  // tick: 只做仲裁，不发布；tickAndPublish: 仲裁后立即发布。
  // 建议在外部主循环（例如 50~100Hz）周期调用。
  bool tick(const ros::Time& now, ArbitrationResult* out);
  bool tickAndPublish(const ros::Time& now, ArbitrationResult* out = nullptr);

  // 覆盖某来源的优先级（运行期动态调参用）。
  void setSourcePriority(const std::string& source, int priority);
  // 清理指定来源/全部来源输入缓存。
  void clearSource(const std::string& source);
  void clearAll();

  // 无有效输入时的输出偏好与“最后命令保持”窗口。
  void setDefaultOutputType(CommandType type);
  void setHoldLastDuration(const ros::Duration& hold_last_duration);

  // 设置/清空兜底命令。无候选命令时按 default_output_type_ 优先选择。
  void setFailsafeAttitude(const mavros_msgs::AttitudeTarget& cmd);
  void setFailsafeLocal(const mavros_msgs::PositionTarget& cmd);
  void clearFailsafeAttitude();
  void clearFailsafeLocal();

  // 获取上一帧仲裁结果（仅用于观测/调试）。
  bool getLastWinner(ArbitrationResult* out) const;

private:
  // 内部缓存单元：一次来源输入 + 元信息 + 消息体。
  struct CommandEntry {
    CommandType type;
    CommandMeta meta;
    mavros_msgs::AttitudeTarget attitude;
    mavros_msgs::PositionTarget local;
  };

  // 当前赢家状态，用于处理“保持窗口”和跨 tick 的稳定输出。
  struct WinnerState {
    bool valid;
    CommandEntry entry;
    // 成为赢家的时间，而非命令时间戳。
    ros::Time selected_at;

    WinnerState();
  };

  bool initialized_;
  ros::NodeHandle nh_;

  int uav_id_;
  std::string uav_name_;

  ros::Publisher attitude_pub_;
  ros::Publisher local_pub_;

  CommandType default_output_type_;
  ros::Duration hold_last_duration_;

  bool has_failsafe_attitude_;
  bool has_failsafe_local_;
  mavros_msgs::AttitudeTarget failsafe_attitude_;
  mavros_msgs::PositionTarget failsafe_local_;

  std::vector<CommandEntry> entries_;
  // 来源优先级覆盖表：source -> priority。
  std::map<std::string, int> source_priority_overrides_;

  WinnerState winner_;
  ArbitrationResult last_result_;
  bool has_last_result_;

  // 输入管理：写入/过期清理。
  void addOrReplaceEntry(const CommandEntry& entry);
  void removeExpiredEntries(const ros::Time& now);
  // 从有效命令中选出“静态最优候选”（尚未考虑当前赢家保持策略）。
  bool selectBestCandidate(const ros::Time& now, CommandEntry* selected) const;
  bool isEntryValid(const CommandEntry& entry, const ros::Time& now) const;
  bool sameEntry(const CommandEntry& a, const CommandEntry& b) const;
  int effectivePriority(const CommandEntry& entry) const;

  // 是否应将当前赢家切换为 candidate（包含 min_hold/hold_last/preemptive 规则）。
  bool shouldSwitchWinner(const CommandEntry& candidate, const ros::Time& now) const;
  // 构造结果：赢家结果 / 兜底结果。
  bool buildResultFromEntry(const CommandEntry& entry, ArbitrationResult* out) const;
  bool buildFallbackResult(ArbitrationResult* out) const;

  // 最终发布出口：按 type 只发布一个话题，避免双通道冲突。
  void publish(const ArbitrationResult& result);
  // API 入口保护：未 init 时统一告警。
  bool ensureInitialized(const char* api) const;
};
