#include "query/DirectQuery.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace RS_marching::query {

DirectStop::DirectStop(const RetainedView &view, const int target_x,
                       const int target_y, const State goal)
    : view_(view), target_x_(target_x), target_y_(target_y), goal_(goal) {}

void DirectStop::evaluate(const double key) {
  ++evaluations;
  const auto begin = std::chrono::steady_clock::now();
  auto route = answerFixedGoal(view_, target_x_, target_y_, goal_, work);
  evaluation_ms += std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - begin).count();
  const double goal_key = view_.distance(target_x_, target_y_);
  if (route.status == GoalStatus::Clear) {
    stop_key = std::min(stop_key, route.chain.segment_sum);
    if (best.status != GoalStatus::Clear || route.chain.segment_sum < best.chain.segment_sum)
      best = std::move(route);
    // One tightening evaluation halfway between the goal's key and the bound.
    next_key_ = evaluations == 1 && stop_key > goal_key
        ? goal_key + 0.5 * (stop_key - goal_key)
        : std::numeric_limits<double>::quiet_NaN();
  } else {
    next_key_ = key + std::max(2.0 * view_.radius(), 0.25 * key);
  }
}

bool DirectStop::operator()(const double key) {
  ++pops;
  if (key > stop_key + kDirectStopMargin * view_.radius()) return true;
  if (evaluations >= kDirectEvaluationLimit || !view_.reached(target_x_, target_y_))
    return false;
  const double goal_key = view_.distance(target_x_, target_y_);
  const bool due = evaluations == 0 ? key >= goal_key
                                    : std::isfinite(next_key_) && key >= next_key_;
  if (due) evaluate(key);
  return key > stop_key + kDirectStopMargin * view_.radius();
}

} // namespace RS_marching::query
