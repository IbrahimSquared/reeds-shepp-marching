// Copyright (c) 2026, Ibrahim Ibrahim
// SPDX-License-Identifier: Apache-2.0
#ifndef RS_BENCHMARK_PREFIX_OBSERVER_HPP_
#define RS_BENCHMARK_PREFIX_OBSERVER_HPP_

#include <cstddef>
#include "nav2_smac_planner/node_lattice.hpp"

namespace rs_benchmark_observer
{
struct SavedPrefix
{
  const nav2_smac_planner::MotionPrimitive * primitive;
  nav2_smac_planner::NodeLattice * parent;
  nav2_smac_planner::NodeLattice::Coordinates pose;
  bool backwards;
};

// Single-threaded benchmark only. Reset before any query can invalidate graph/table pointers.
void reset();
// Called only at the final analytic-expansion cleanNode, before native metadata is erased.
void capture(nav2_smac_planner::NodeLattice * node);
const SavedPrefix * surviving_prefix(nav2_smac_planner::NodeLattice * node);
std::size_t captured_count();
std::size_t record_bytes();
}  // namespace rs_benchmark_observer
#endif
