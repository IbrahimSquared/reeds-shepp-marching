// Copyright (c) 2026, Ibrahim Ibrahim
// SPDX-License-Identifier: Apache-2.0
#include "nav2_smac_planner/prefix_observer.hpp"
#include <unordered_map>

namespace rs_benchmark_observer
{
namespace
{
std::unordered_map<nav2_smac_planner::NodeLattice *, SavedPrefix> saved;
}

void reset()
{
  // Release buckets too: residency is bounded by this query's final expansion.
  decltype(saved) empty;
  saved.swap(empty);
}

void capture(nav2_smac_planner::NodeLattice * node)
{
  const auto * primitive = node->getMotionPrimitive();
  if (primitive) {
    saved.emplace(node, SavedPrefix{primitive, node->parent, node->pose, node->isBackward()});
  }
}

const SavedPrefix * surviving_prefix(nav2_smac_planner::NodeLattice * node)
{
  if (node->getMotionPrimitive()) {return nullptr;}
  const auto found = saved.find(node);
  if (found == saved.end()) {return nullptr;}
  const auto & value = found->second;
  if (node->parent != value.parent || node->pose.x != value.pose.x ||
    node->pose.y != value.pose.y || node->pose.theta != value.pose.theta ||
    node->isBackward() != value.backwards)
  {
    return nullptr;
  }
  return &value;
}

std::size_t captured_count() {return saved.size();}
std::size_t record_bytes() {return sizeof(decltype(saved)::value_type);}
}  // namespace rs_benchmark_observer
