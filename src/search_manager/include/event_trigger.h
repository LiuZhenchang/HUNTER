#ifndef SEARCH_MANAGER_EVENT_TRIGGER_H
#define SEARCH_MANAGER_EVENT_TRIGGER_H

#include <atomic>
#include <cmath>
#include <cstddef>
#include <mutex>
#include <optional>

struct EventTriggerConfig {
  double probability_weight;
  double uncertainty_weight;
  double probability_delta_weight;
  double uncertainty_delta_weight;
  double threshold;
};

struct EventTriggerResult {
  double max_probability;
  double mean_uncertainty;
  double probability_delta;
  double uncertainty_delta;
  double metric;
  bool triggered;
};

/**
 * @brief Evaluates the information-driven DMPC replanning condition.
 */
class EventTrigger {
 public:
  explicit EventTrigger(const EventTriggerConfig &config) : config_(config) {}

  void reset() {
    previous_probability_.reset();
    previous_uncertainty_.reset();
  }

  EventTriggerResult evaluate(double max_probability, double mean_uncertainty) {
    const double probability_delta = previous_probability_.has_value()
                                         ? std::abs(max_probability - *previous_probability_)
                                         : 0.0;
    const double uncertainty_delta = previous_uncertainty_.has_value()
                                         ? std::abs(mean_uncertainty - *previous_uncertainty_)
                                         : 0.0;
    previous_probability_ = max_probability;
    previous_uncertainty_ = mean_uncertainty;

    const double metric = config_.probability_weight * max_probability + config_.uncertainty_weight * mean_uncertainty +
                          config_.probability_delta_weight * probability_delta +
                          config_.uncertainty_delta_weight * uncertainty_delta;
    return {max_probability,   mean_uncertainty, probability_delta,
            uncertainty_delta, metric,           metric > config_.threshold};
  }

 private:
  EventTriggerConfig config_;
  std::optional<double> previous_probability_;
  std::optional<double> previous_uncertainty_;
};

/**
 * @brief Stores mission-level counters used by the event-trigger CSV log.
 */
class EventTriggerMetrics {
 public:
  void recordGaCall() {
    ++ga_call_count_;
  }

  void recordTargetDetection(double elapsed_seconds) {
    std::lock_guard<std::mutex> lock(target_time_mutex_);
    if (!first_target_detection_time_.has_value()) {
      first_target_detection_time_ = elapsed_seconds;
    }
  }

  std::size_t gaCallCount() const {
    return ga_call_count_.load();
  }

  std::optional<double> firstTargetDetectionTime() const {
    std::lock_guard<std::mutex> lock(target_time_mutex_);
    return first_target_detection_time_;
  }

 private:
  std::atomic<std::size_t> ga_call_count_{0};
  mutable std::mutex target_time_mutex_;
  std::optional<double> first_target_detection_time_;
};

#endif  // SEARCH_MANAGER_EVENT_TRIGGER_H
