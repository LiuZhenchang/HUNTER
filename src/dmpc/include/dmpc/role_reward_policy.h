#ifndef DMPC_ROLE_REWARD_POLICY_H
#define DMPC_ROLE_REWARD_POLICY_H

#include <algorithm>

inline double boundedRoleProjection(double projection, double remaining_distance) {
  if (remaining_distance <= 0.0) {
    return projection;
  }
  return std::min(projection, remaining_distance);
}

inline bool shouldDisableRoleReward(double target_distance, double target_tolerance) {
  return target_distance <= target_tolerance;
}

inline bool shouldDisableCompletedTargetReward(bool target_task_completed) {
  return target_task_completed;
}

inline bool shouldDecayRoleReward(bool is_tracker, double target_distance, double no_decay_distance) {
  return !is_tracker || target_distance <= no_decay_distance;
}

inline bool shouldUseTerminalTargetScore(double target_distance, double switch_distance) {
  return target_distance <= switch_distance;
}

inline double terminalTargetScore(double terminal_distance, double switch_distance, double arrival_distance) {
  if (terminal_distance <= arrival_distance) {
    return 1.0;
  }

  const double score_span = std::max(1e-6, switch_distance - arrival_distance);
  return std::max(-1.0, 1.0 - (terminal_distance - arrival_distance) / score_span);
}

#endif  // DMPC_ROLE_REWARD_POLICY_H
