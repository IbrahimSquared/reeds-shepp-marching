#ifndef QUERY_DIRECT_QUERY_HPP
#define QUERY_DIRECT_QUERY_HPP

#include "query/ReconstructionQuery.hpp"

#include <cstddef>
#include <limits>

namespace RS_marching::query {

// Stop rule of a direct one-to-one query. The start wave marches in its
// ordinary order, or with keys g + |c - goal| (A* order; the straight line is
// admissible from any heading, so the argument below holds for both); once
// the goal cell is reached and the popped key passes its distance, the
// fixed-goal answer on the wave's partial retained state gives a route of
// length U, and the wave stops when a popped key exceeds
// U + kDirectStopMargin * r. Every represented candidate costs at least the
// record or prefix it extends, so cells popped beyond U cannot shorten the
// route, except through later improvements of popped records, later births
// with smaller prefixes, deferred completion (which runs only if the queue
// drains before the bound), or endpoint proposals through a neighbour whose
// retained last leg is a fixed-terminal fallback rather than the free-heading
// optimum (such a route can cost less than the neighbour's key). The best
// clear route of the evaluations is kept: the bounded answer need not improve
// as the wave grows.
class DirectStop {
public:
  DirectStop(const RetainedView &view, int target_x, int target_y, State goal);

  // Called with each settled pop key; true stops the march.
  bool operator()(double key);

  std::size_t pops = 0;
  std::size_t evaluations = 0;
  double stop_key = std::numeric_limits<double>::infinity();
  // Work and time of the stop evaluations, beyond the final query's own.
  QueryWork work;
  double evaluation_ms = 0.0;
  // The shortest clear route any evaluation found; its geometry is complete
  // and stays valid as the wave continues.
  GoalRoute best;

private:
  void evaluate(double key);

  const RetainedView &view_;
  int target_x_, target_y_;
  State goal_;
  double next_key_ = std::numeric_limits<double>::quiet_NaN();
};

// Re-evaluations while no route exists are spaced geometrically, at most this
// many in total.
inline constexpr std::size_t kDirectEvaluationLimit = 8;
// The wave continues this many turning radii beyond the bound, because the
// label-correcting march can still improve records settled just below it.
inline constexpr double kDirectStopMargin = 0.5;

} // namespace RS_marching::query

#endif
