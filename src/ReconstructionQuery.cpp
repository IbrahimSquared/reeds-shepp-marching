#include "query/ReconstructionQuery.hpp"
#include "query/RetainedReplayMemo.hpp"

#include "accelerated/Solver.hpp"
#include "geometry/ConnectorClearance.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <span>
#include <string_view>
#include <utility>

namespace RS_marching::query {

namespace {

audit::ResolvedChainAuditResult replayRetainedChainImpl(
    const std::vector<State> &poses, const double accepted_distance,
    const double radius, QueryWork *const work,
    const std::vector<std::uint32_t> *const words,
    detail::RetainedReplayMemo *const memo) {
  audit::ResolvedChainAuditResult result;
  if (!std::isfinite(accepted_distance) || accepted_distance < 0.0 ||
      !std::isfinite(radius) || radius <= 0.0) {
    return result;
  }
  if (poses.size() < 2) {
    result.status = audit::ResolvedChainStatus::Incomplete;
    return result;
  }
  if (words != nullptr && words->size() != poses.size() - 1) {
    result.status = audit::ResolvedChainStatus::Incomplete;
    return result;
  }
  if (std::any_of(poses.begin(), poses.end(), [](const State &pose) {
        return !std::isfinite(pose.x) || !std::isfinite(pose.y) ||
               !std::isfinite(pose.theta);
      })) {
    return result;
  }
  result.segments.reserve(poses.size() - 1);
  result.segment_sum = 0.0;
  result.status = audit::ResolvedChainStatus::Clear;
  const auto retain = [&](const audit::ResolvedChainStatus status) {
    result.status = std::max(result.status, status);
  };
  for (std::size_t index = 0; index + 1 < poses.size(); ++index) {
    audit::LegacyPathAuditResult path;
    bool replay_hit = false;
    if (words != nullptr) {
      path = memo != nullptr
          ? memo->replay(poses[index], poses[index + 1], radius,
                         words->at(index), replay_hit)
          : audit::reconstructRetainedLeg(poses[index], poses[index + 1], radius,
                                         words->at(index));
      if (work) {
        if (replay_hit) ++work->retained_replay_hits;
        else ++work->solves;
      }
    } else {
      Solver solver;
      double distance = std::numeric_limits<double>::infinity();
      int condition = 0;
      int quadrant = 0;
      if (work) ++work->solves;
      solver.acceleratedRSPlanner(poses[index], poses[index + 1], distance,
                                  radius, condition, quadrant);
      path = audit::replaySelectedLegacyPath(
          solver, poses[index], poses[index + 1], distance, radius,
          condition, quadrant);
    }
    if (work && !replay_hit) ++work->materializations;
    if (!path.replayed()) {
      if (work) ++work->replay_failures;
      if (path.status == audit::LegacyReplayStatus::PrimitiveMaterializationFailed) {
        retain(audit::ResolvedChainStatus::Unsupported);
      } else if (path.status == audit::LegacyReplayStatus::ReplayMismatch) {
        retain(audit::ResolvedChainStatus::Incomplete);
      } else {
        retain(audit::ResolvedChainStatus::Invalid);
      }
    }
    result.segment_sum += path.distance;
    result.segments.push_back({path, {}});
  }
  result.complete_distance_residual =
      std::abs(result.segment_sum - accepted_distance);
  const double scale = std::max(radius, std::abs(accepted_distance));
  if (!std::isfinite(result.segment_sum) ||
      result.complete_distance_residual > audit::kReplayTolerance * scale) {
    retain(audit::ResolvedChainStatus::Incomplete);
  }
  return result;
}

// The retained prefix of source from the root: each ancestor's preceding pose
// and parent, and the incoming words of every prefix leg after the first
// (whose word is the root's). False for broken ancestry.
bool retainedPrefix(const RetainedView &view, const int source, std::vector<State> &poses,
                    std::vector<int> &sources, std::vector<std::uint32_t> &words,
                    std::size_t *const visits) {
  if (!view.validSource(source)) return false;
  int ancestor = source;
  for (std::size_t hops = 0; view.parent(ancestor) >= 0; ++hops) {
    if (hops >= view.sources() || !view.validSource(view.parent(ancestor))) return false;
    if (visits) ++*visits;
    poses.push_back(view.precedingPose(ancestor));
    sources.push_back(view.parent(ancestor));
    words.push_back(view.incomingWord(ancestor));
    ancestor = view.parent(ancestor);
  }
  std::reverse(poses.begin(), poses.end());
  std::reverse(sources.begin(), sources.end());
  std::reverse(words.begin(), words.end());
  if (!words.empty()) words.erase(words.begin());
  return true;
}

RetainedRecord replayRetainedRecordImpl(const RetainedView &view, const int x,
                                    const int y, QueryWork *const work,
                                    std::size_t *const ancestry_visits,
                                    detail::RetainedReplayMemo *const memo) {
  RetainedRecord record;
  const int source = view.owner(x, y);
  if (!retainedPrefix(view, source, record.poses, record.source_ids, record.words,
                      ancestry_visits))
    return {};
  if (!record.poses.empty()) record.words.push_back(view.recordIncomingWord(x, y));
  record.words.push_back(view.recordOutgoingWord(x, y));
  record.poses.push_back(view.recordPivotPose(x, y));
  record.source_ids.push_back(source);
  record.poses.push_back({static_cast<double>(x), static_cast<double>(y), view.heading(x, y)});
  record.source_ids.push_back(source);
  const bool root_target = record.poses.size() == 2 && view.rootCell(x, y);
  if (root_target) {
    record.chain.status = audit::ResolvedChainStatus::Clear;
    record.chain.segment_sum = 0.0;
    record.chain.complete_distance_residual = 0.0;
  } else {
    record.chain = replayRetainedChainImpl(record.poses, view.distance(x, y), view.radius(),
                                       work, &record.words, memo);
  }
  record.word_match = root_target || record.chain.segments.size() == record.words.size();
  for (std::size_t i = 0; i < record.chain.segments.size() && record.word_match; ++i)
    record.word_match = audit::selectedPathWord(record.chain.segments[i].path) == record.words[i];
  if (!record.word_match && record.chain.status == audit::ResolvedChainStatus::Clear)
    record.chain.status = audit::ResolvedChainStatus::Incomplete;
  record.valid = true;
  return record;
}

} // namespace

audit::ResolvedChainAuditResult replayRetainedChain(
    const std::vector<State> &poses, const double accepted_distance,
    const double radius, QueryWork *const work,
    const std::vector<std::uint32_t> *const words) {
  return replayRetainedChainImpl(poses, accepted_distance, radius, work, words, nullptr);
}

RetainedRecord replayRetainedRecord(const RetainedView &view, const int x,
                                    const int y, QueryWork *const work,
                                    std::size_t *const ancestry_visits) {
  return replayRetainedRecordImpl(view, x, y, work, ancestry_visits, nullptr);
}

// A certified replay of a retained chain with `legs` legs and stored distance
// g has selected distances d_i summing within tau * max(r, g) of g, and each
// leg's literal length within tau * max(r, d_i) of d_i (tau =
// kReplayTolerance; free-heading words are exact). Its physical length is
// therefore within tau * ((2 + tau) * max(r, g) + legs * r) of g, plus an
// allowance for summation rounding.
double replayResidualBound(const double g, const std::size_t legs, const double r) {
  constexpr double tau = audit::kReplayTolerance;
  const double n = static_cast<double>(legs);
  const double scale = std::max(r, std::abs(g)) + n * r;
  return tau * ((2.0 + tau) * std::max(r, std::abs(g)) + n * r) +
         4.0 * (n + 2.0) * std::numeric_limits<double>::epsilon() * scale;
}

namespace {

// std::nextafter(x, +infinity) as one bit step: the same result for every
// non-signaling x while MXCSR.DAZ is clear (never set here), signed zeros,
// subnormals and infinities included, without a libm call. It raises no
// floating-point flags, which no result here depends on.
double nextUp(const double x) {
  if (std::isnan(x) || x == std::numeric_limits<double>::infinity()) return x;
  if (x == 0.0) return std::numeric_limits<double>::denorm_min();
  const auto bits = std::bit_cast<std::uint64_t>(x);
  return std::bit_cast<double>(x > 0.0 ? bits + 1 : bits - 1);
}

// std::nextafter(x, -infinity), likewise.
double nextDown(const double x) { return -nextUp(-x); }

} // namespace

// Invert the certified residual before introducing a differently scaled leg.
// One outward step covers the residual comparison's subtraction rounding;
// the downward step keeps the resulting prefix envelope below that prefix.
double replayPrefixLowerBound(const double g, const std::size_t legs, const double r) {
  const double residual = replayResidualBound(g, legs, r);
  if (!std::isfinite(g) || !std::isfinite(residual)) return 0.0;
  return std::max(0.0, nextDown(g - nextUp(residual)));
}

// Addition is monotone in both operands. Compose the prefix envelope using
// the same operation as the selector's physical-prefix + physical-leg cost.
double replayExtensionLowerBound(const double g, const std::size_t legs,
                                 const double r, const double leg_bound) {
  if (!std::isfinite(leg_bound)) return 0.0;
  const double sum = replayPrefixLowerBound(g, legs, r) + leg_bound;
  return std::isfinite(sum) ? std::max(0.0, sum) : 0.0;
}

namespace {

[[nodiscard]] double physicalLength(const audit::ResolvedChainAuditResult &chain) {
  double total = 0.0;
  for (const auto &leg : chain.segments)
    for (std::size_t i = 0; i < leg.path.primitive_count; ++i)
      total += leg.path.primitives[i].length;
  return total;
}

[[nodiscard]] double angleError(const double a, const double b) {
  return std::abs(std::remainder(a - b, 2.0 * std::numbers::pi));
}

[[nodiscard]] audit::ContinuousClearanceResult qualifiedClear() {
  audit::ContinuousClearanceResult result;
  result.status = audit::ContinuousClearanceStatus::Clear;
  result.feature = audit::ContinuousClearanceFeature::None;
  return result;
}

// Adds the work counters of an abandoned pass to its repeat (the chain payload
// peak is the larger of the two); coverage (sources considered, spatial
// fallback and sources added, endpoint fallback and candidates, extension
// rank) describes the repeat alone.
void absorbCounters(GoalRoute &into, const GoalRoute &from) {
  into.ancestry_visits += from.ancestry_visits;
  into.prefix_legs += from.prefix_legs;
  into.prefix_failures += from.prefix_failures;
  into.sources_pruned += from.sources_pruned;
  into.sources_bound_pruned += from.sources_bound_pruned;
  into.extension_cells += from.extension_cells;
  into.extensions_ranked += from.extensions_ranked;
  into.extensions_priced += from.extensions_priced;
  into.extensions_qualified += from.extensions_qualified;
  into.spatial_source_visits += from.spatial_source_visits;
  into.authority_fallbacks += from.authority_fallbacks;
  into.peak_chain_payload = std::max(into.peak_chain_payload, from.peak_chain_payload);
}

struct Search {
  GoalRoute route;
  std::size_t new_legs = 0;
  double selected_distance = std::numeric_limits<double>::infinity();
};

[[nodiscard]] Search search(const RetainedView &view, const int target_x,
                            const int target_y, const State goal,
                            const RecordControl &record, QueryWork &work,
                            const GoalQueryOptions &options,
                            const QueryAuthority authority) {
  detail::RetainedReplayMemo replay_memo;
  Search found;
  GoalRoute &route = found.route;
  const double r = view.radius();
  const auto endpointMatches = [&](const State end) {
    return std::hypot(end.x - goal.x, end.y - goal.y) <= audit::kReplayTolerance * r &&
           (options.free_heading || angleError(end.theta, goal.theta) <= audit::kReplayTolerance);
  };
  const bool record_control = record.chain != nullptr && record.word_match &&
      record.chain->certified() && !record.poses->empty() &&
      endpointMatches(record.chain->segments.empty() ? record.poses->front()
                                                     : record.chain->segments.back().path.endpoint);
  double best_distance = record_control ? physicalLength(*record.chain)
                                        : std::numeric_limits<double>::infinity();
  route.best_control = best_distance;
  if (record_control) {
    route.poses = *record.poses;
    route.source_ids = *record.source_ids;
    route.chain = *record.chain;
    route.prefix_word_match = true;
    route.route_kind = "retained_record";
  }
  const auto payload = [](const std::vector<State> &p, const std::vector<int> &ids,
                          const audit::ResolvedChainAuditResult &c) {
    return p.capacity() * sizeof(State) + ids.capacity() * sizeof(int) +
           c.segments.capacity() * sizeof(audit::ResolvedSegmentAudit);
  };
  if (!view.validSource(0)) {
    route.status = GoalStatus::Incomplete;
    return found;
  }
  LegQualifier qualifier(view.map(), view.clearance().sharedSceneChecker(), r,
                         authority == QueryAuthority::Exact);
  const State start = view.rootPose();
  auto direct = options.free_heading ? qualifier.priceFree(start, goal.x, goal.y, work)
                                     : qualifier.leg(start, goal, work);
  if (options.free_heading && direct.priced() &&
      direct.path.distance < best_distance -
          1e-9 * std::max({1.0, r, std::isfinite(best_distance) ? best_distance : 0.0}))
    qualifier.qualify(direct, start, work);
  if (direct.clear()) {
    route.best_control = std::min(route.best_control, direct.path.distance);
    if (direct.path.distance < best_distance) {
      best_distance = direct.path.distance;
      route.poses = {start, options.free_heading
          ? State{goal.x, goal.y, direct.path.target_theta} : goal};
      route.source_ids = {0, 0};
      route.chain = {};
      route.chain.segments = {{direct.path, qualifiedClear()}};
      route.chain.status = audit::ResolvedChainStatus::Clear;
      route.prefix_word_match = true;
      route.route_kind = "direct_start";
      found.new_legs = 1;
    }
  }

  // Record extensions: a reached cell's retained record plus one new leg, in
  // order of g(c) + rsLengthLowerBound(c, goal).
  // The best record extension so far: its record is replayed before it is
  // kept, so cost is its physical length. It is adopted after every family
  // has been searched.
  struct PendingExtension {
    double cost = std::numeric_limits<double>::infinity();
    std::size_t rank = 0;
    QualifiedLeg leg;
    RetainedRecord retained;
  } pending;
  // Every pruning decision compares a certified lower bound on a candidate's
  // physical length with this physical incumbent.
  const auto incumbent = [&] { return std::min(best_distance, pending.cost); };
  // Sources are pruned only beyond the incumbent plus settleExtension's tie:
  // a pending extension within that tie of the best route is not adopted,
  // yet a route between the two must still be found.
  const auto pruneLimit = [&] {
    const double value = incumbent();
    const double best = std::isfinite(best_distance) ? best_distance : 0.0;
    return value + 1e-9 * std::max({1.0, r, std::abs(value), best});
  };
  // The fallback families (nearest pivots, then endpoint proposals) run only
  // when the retained, direct and pivot families found no route; a source
  // pruned by an extension's length counts as not found. Their eligibility
  // therefore depends on how much the certified bounds prune.
  const auto policyRouteFound = [&] { return std::isfinite(best_distance); };
  const auto pendingPayload = [&] {
    return payload(pending.retained.poses, pending.retained.source_ids, pending.retained.chain) +
           pending.retained.words.capacity() * sizeof(std::uint32_t);
  };
  // Legs of cell (x, y)'s retained record: its owner's ancestry plus the
  // outgoing leg (and the owner's incoming leg for a nonroot owner).
  const auto recordLegs = [&](const int x, const int y) {
    std::size_t legs = 1;
    for (int source = view.owner(x, y); view.validSource(source) && view.parent(source) >= 0 &&
                                        legs <= view.sources();
         source = view.parent(source)) {
      ++route.ancestry_visits;
      ++legs;
    }
    return legs;
  };
  // Whether cell (x, y)'s extension can reach limit: its physical length is at
  // least g - replayResidualBound(g, legs) + a, for a certified lower bound a
  // on the new leg. The bound shrinks as legs grow, so the ancestry is walked
  // only when the one-leg and longest-record bounds straddle limit; legs caches
  // the count (0 until walked).
  const auto extensionBelow = [&](const int x, const int y, const double g, const double a,
                                  const double limit, std::size_t &legs) {
    if (legs == 0) {
      if (replayExtensionLowerBound(g, 1, r, a) <= limit) return true;
      if (replayExtensionLowerBound(g, view.sources() + 1, r, a) > limit) return false;
      legs = recordLegs(x, y);
    }
    return replayExtensionLowerBound(g, legs, r, a) <= limit;
  };
  const auto extendRecords = [&] {
    // Only the kExtensionPriceLimit lowest bounds are ever priced, so a
    // bounded max-heap keeps the scratch O(K).
    struct Extension {
      double bound;  // g + leg_bound, the pricing order
      double leg_bound;
      std::size_t legs;
      int y, x;
      bool operator<(const Extension &other) const {
        return bound != other.bound ? bound < other.bound
                                    : (y != other.y ? y < other.y : x < other.x);
      }
    };
    std::array<Extension, kExtensionPriceLimit> extensions{};
    std::size_t extension_count = 0;
    const double limit = incumbent();
    // Cap in floating point before converting: even a finite radius can
    // exceed the integer range (or overflow 1.5*r to infinity).
    const int window = static_cast<int>(std::min(
        static_cast<double>(kExtensionWindowCap), std::ceil(1.5 * r) + 2.0));
    for (int y = std::max(0, target_y - window);
         y <= std::min(view.height() - 1, target_y + window); ++y) {
      for (int x = std::max(0, target_x - window);
           x <= std::min(view.width() - 1, target_x + window); ++x) {
        if (!view.free(x, y) || !view.reached(x, y)) continue;
        ++route.extension_cells;
        const double g = view.distance(x, y);
        // A cheap filter first: the Chebyshev distance is below the Euclidean
        // one, a new leg ends within replayPositionSlack of the goal, and no
        // record has more legs than there are sources plus one.
        const State cell{static_cast<double>(x), static_cast<double>(y), view.heading(x, y)};
        const double chebyshev = std::max(std::abs(x - goal.x), std::abs(y - goal.y)) -
                                 replayPositionSlack(cell, goal.x, goal.y, r);
        if (view.rootCell(x, y) ||
            replayExtensionLowerBound(g, view.sources() + 1, r, chebyshev) > limit)
          continue;
        const double leg_bound = options.free_heading ? 0.0
            : rsLengthLowerBound(cell, goal, r);
        // A free URS selector price ranks only the represented candidate;
        // it is not a rejecting lower bound on all free-heading paths.
        double ranking = leg_bound;
        if (options.free_heading) {
          const auto priced = qualifier.priceFree(cell, goal.x, goal.y, work);
          ++route.extensions_ranked;
          if (!priced.priced()) continue;
          ranking = priced.path.distance;
        }
        Extension extension{g + ranking, leg_bound, 0, y, x};
        // An infinite ranking key sorts last; it is not a rejecting bound.
        if (!extensionBelow(x, y, g, leg_bound, limit, extension.legs))
          continue;
        if (extension_count < extensions.size()) {
          extensions[extension_count++] = extension;
          std::push_heap(extensions.begin(), extensions.begin() + extension_count);
        } else if (extension < extensions.front()) {
          std::pop_heap(extensions.begin(), extensions.end());
          extensions.back() = extension;
          std::push_heap(extensions.begin(), extensions.end());
        }
      }
    }
    std::sort_heap(extensions.begin(), extensions.begin() + extension_count);
    for (std::size_t rank = 0; rank < extension_count; ++rank) {
      auto &extension = extensions[rank];
      const double g = view.distance(extension.x, extension.y);
      if (!extensionBelow(extension.x, extension.y, g, extension.leg_bound, incumbent(),
                          extension.legs))
        continue;
      const State from{static_cast<double>(extension.x), static_cast<double>(extension.y),
                       view.heading(extension.x, extension.y)};
      auto leg = options.free_heading ? qualifier.priceFree(from, goal.x, goal.y, work)
                                      : qualifier.price(from, goal, work);
      ++route.extensions_priced;
      // It must beat the pending extension, and settleExtension adopts it
      // over a finite best only below best - tie (endpoint proposals, whose
      // ties favour an extension, run only while no best exists, and best
      // only decreases).
      const double adoptable = std::isfinite(best_distance)
          ? best_distance - 1e-9 * std::max({1.0, r, best_distance})
          : std::numeric_limits<double>::infinity();
      if (!leg.priced() ||
          !extensionBelow(extension.x, extension.y, g, leg.path.distance,
                          std::min(pending.cost, adoptable), extension.legs))
        continue;
      if (!qualifier.qualify(leg, from, work)) continue;
      ++route.extensions_qualified;
      if (!endpointMatches(leg.path.endpoint)) continue;
      auto retained = replayRetainedRecordImpl(view, extension.x, extension.y, &work,
                                               &route.ancestry_visits, &replay_memo);
      // Counts visited logical legs, including copies from the replay memo.
      route.prefix_legs += retained.chain.segments.size();
      if (!retained.valid || !retained.word_match || !retained.chain.certified()) {
        ++route.prefix_failures;
        continue;
      }
      route.peak_chain_payload = std::max(route.peak_chain_payload,
          payload(route.poses, route.source_ids, route.chain) + pendingPayload() +
          payload(retained.poses, retained.source_ids, retained.chain) +
          retained.words.capacity() * sizeof(std::uint32_t));
      const double cost = physicalLength(retained.chain) + leg.path.distance;
      if (cost < pending.cost) pending = {cost, rank + 1, std::move(leg), std::move(retained)};
    }
  };
  // Selects the pending extension when it is physically shortest.
  const auto settleExtension = [&] {
    if (!std::isfinite(pending.cost)) return;
    auto &retained = pending.retained;
    const double distance = pending.cost;
    // An extension can split an existing route's geometry into two legs; adopt
    // it only when it is shorter beyond a tie tolerance.
    const double tie = std::isfinite(best_distance)
        ? 1e-9 * std::max({1.0, r, best_distance}) : 0.0;
    // Against an endpoint route (which can rebuild the extension's geometry
    // through the same cell), ties favour the extension.
    const bool endpoint_incumbent = std::string_view(route.route_kind) == "endpoint_terminal";
    // A tie never lets the answer exceed a reported control.
    const double control_limit = route.best_control +
        1e-10 * std::max({1.0, r, distance, route.best_control});
    if (endpoint_incumbent ? !(distance <= std::min(best_distance + tie, control_limit))
                           : !(distance < best_distance - tie))
      return;
    best_distance = distance;
    retained.poses.push_back(options.free_heading
        ? State{goal.x, goal.y, pending.leg.path.target_theta} : goal);
    retained.source_ids.push_back(retained.source_ids.back());
    retained.chain.segments.push_back({pending.leg.path, qualifiedClear()});
    route.poses = std::move(retained.poses);
    route.source_ids = std::move(retained.source_ids);
    route.chain = std::move(retained.chain);
    route.prefix_word_match = true;
    route.route_kind = "record_extension";
    route.extension_rank = pending.rank;
    found.new_legs = 1;
    route.peak_chain_payload = std::max(route.peak_chain_payload,
        payload(route.poses, route.source_ids, route.chain) +
        retained.words.capacity() * sizeof(std::uint32_t));
  };

  if (options.record_extensions) extendRecords();
  if (options.free_heading) {
    settleExtension();
    route.peak_chain_payload = std::max(route.peak_chain_payload,
        payload(route.poses, route.source_ids, route.chain));
    qualifier.reportExactWork(work);
    found.selected_distance = best_distance;
    route.status = std::isfinite(best_distance) ? GoalStatus::Clear : GoalStatus::Unsupported;
    return found;
  }
  struct SourceSeed {
    int source = -1;
    double theta = 0.0;
  };
  std::array<SourceSeed, kSourceLimit> candidates{};
  std::size_t candidate_count = 0;
  const auto addSource = [&](const int id, const double theta, const std::size_t limit) {
    if (!view.validSource(id) || view.parent(id) < 0 || candidate_count == limit) return;
    for (std::size_t i = 0; i < candidate_count; ++i)
      if (candidates[i].source == id) return;
    candidates[candidate_count++] = {id, theta};
  };
  const int source = view.reached(target_x, target_y) ? view.owner(target_x, target_y) : -1;
  if (view.validSource(source))
    addSource(source, view.pivotHeading(target_x, target_y), kInitialSourceLimit);
  for (int dy = -1; dy <= 1; ++dy) {
    for (int dx = -1; dx <= 1; ++dx) {
      const int x = target_x + dx, y = target_y + dy;
      if (!view.inside(x, y) || !view.updated(x, y) || !view.free(x, y)) continue;
      addSource(view.owner(x, y), view.pivotHeading(x, y), kInitialSourceLimit);
    }
  }
  int ancestor = source;
  for (std::size_t hops = 0; view.validSource(ancestor) &&
       candidate_count < kInitialSourceLimit && hops < view.sources(); ++hops) {
    ++route.ancestry_visits;
    ancestor = view.parent(ancestor);
    if (view.validSource(ancestor))
      addSource(ancestor, view.birthHeading(ancestor), kInitialSourceLimit);
  }

  // One pivot connector: the retained prefix to A, then A -> (p, psi) -> goal.
  // URS(A->p) + URS(goal->p) orders sources only: URS solver values are
  // achievable lengths, not certified minima.
  const auto freeHeadingPrice = [&](const State A, const State p) {
    work.bound_solves += 2;
    const auto first = audit::solveFreeHeadingCandidate(A, p.x, p.y, r);
    const auto second = audit::solveFreeHeadingCandidate(goal, p.x, p.y, r);
    return first.replayed() && second.replayed() ? first.distance + second.distance
                                                 : std::numeric_limits<double>::quiet_NaN();
  };
  // Certified lower bounds on A -> (p, psi) -> goal over every psi: the
  // displacements with each new leg's endpoint slack, then the joint heading
  // bound, which dominates them. Neither costs a solve; they differ in the
  // counter they report and in whether a pruned pivot source still reports its
  // retained-seed controls (sources_pruned: no; sources_bound_pruned: yes).
  const auto displacementBound = [&](const State A, const State p) {
    const double bound = std::hypot(A.x - p.x, A.y - p.y) + std::hypot(p.x - goal.x, p.y - goal.y) -
           replayPositionSlack(A, p.x, p.y, r) - replayPositionSlack(p, goal.x, goal.y, r);
    return std::isfinite(bound) ? bound : 0.0;
  };
  // Whether every connector through p exceeds limit; the scan starts at p's
  // seed heading.
  const auto connectorExceeds = [&](const State A, const State p,
                                    const double prefix, const double limit) {
    if (!std::isfinite(prefix) || !std::isfinite(limit)) return false;
    const double cutoff = limit - prefix;
    const double bound = rsConnectorLowerBound(A, p.x, p.y, goal, r, p.theta, cutoff);
    // A result <= cutoff is only an early interval witness, not the full
    // joint bound. Otherwise compare in the selector's forward direction;
    // rounded subtraction alone is not the inverse of rounded addition.
    return bound > cutoff && prefix + bound > limit;
  };
  // An endpoint proposal (endpoint_x >= 0) keeps the endpoint cell's record up
  // to its owner pivot and searches the heading at the endpoint cell instead.
  const auto trySource = [&](const int id, const double seed_theta, const int endpoint_x = -1,
                             const int endpoint_y = -1) {
    const bool endpoint = endpoint_x >= 0;
    const State pivot = endpoint
        ? State{static_cast<double>(endpoint_x), static_cast<double>(endpoint_y), seed_theta}
        : view.pivotPose(id);
    // A prefix is replayed once for this candidate, not requalified or changed.
    std::vector<State> prefix_poses;
    std::vector<int> prefix_sources;
    std::vector<std::uint32_t> prefix_words;
    const bool root_owner = view.parent(id) < 0;
    if (!retainedPrefix(view, id, prefix_poses, prefix_sources, prefix_words,
                        &route.ancestry_visits) ||
        (prefix_poses.empty() && !(endpoint && root_owner))) {
      ++route.prefix_failures;
      return;
    }
    double stored_prefix_distance = view.prefixDistance(id);
    if (endpoint) {
      prefix_poses.push_back(view.recordPivotPose(endpoint_x, endpoint_y));
      prefix_sources.push_back(id);
      if (!root_owner) {
        stored_prefix_distance += view.recordIncomingDistance(endpoint_x, endpoint_y);
        prefix_words.push_back(view.recordIncomingWord(endpoint_x, endpoint_y));
      }
    }
    if (endpoint && std::isfinite(incumbent())) {
      // Endpoint proposals run beside an incumbent: prune them before
      // replaying the prefix, on the certified replay bound of its stored
      // distance.
      const State owner = prefix_poses.back();
      const double scalar = replayPrefixLowerBound(
          stored_prefix_distance, prefix_words.size(), r);
      if (scalar + displacementBound(owner, pivot) > pruneLimit()) {
        ++route.sources_pruned;
        return;
      }
      if (connectorExceeds(owner, pivot, scalar, pruneLimit())) {
        ++route.sources_bound_pruned;
        return;
      }
    }
    audit::ResolvedChainAuditResult prefix_path;
    if (prefix_poses.size() == 1 && stored_prefix_distance == 0.0) {
      prefix_path.status = audit::ResolvedChainStatus::Clear;
      prefix_path.segment_sum = 0.0;
      prefix_path.complete_distance_residual = 0.0;
    } else {
      prefix_path = replayRetainedChainImpl(prefix_poses, stored_prefix_distance, r,
                                            &work, &prefix_words, &replay_memo);
    }
    // Counts visited logical legs, including copies from the replay memo.
    route.prefix_legs += prefix_path.segments.size();
    bool valid_prefix =
        prefix_path.certified() && prefix_path.segments.size() == prefix_words.size();
    for (std::size_t i = 0; i < prefix_path.segments.size() && valid_prefix; ++i)
      valid_prefix = audit::selectedPathWord(prefix_path.segments[i].path) == prefix_words[i];
    if (!valid_prefix) { ++route.prefix_failures; return; }
    route.peak_chain_payload = std::max(route.peak_chain_payload,
        payload(route.poses, route.source_ids, route.chain) + pendingPayload() +
        payload(prefix_poses, prefix_sources, prefix_path) +
        record.word_bytes + prefix_words.capacity() * sizeof(std::uint32_t));
    const double prefix_distance = physicalLength(prefix_path);
    const State A = prefix_poses.back();
    const bool bounded = std::isfinite(incumbent());
    if (bounded && prefix_distance + displacementBound(A, pivot) > pruneLimit()) {
      ++route.sources_pruned;
      return;
    }
    // An endpoint seed equals the searched pivot's own heading.
    const std::array<double, 1> seed_storage{seed_theta};
    const std::span<const double> extra_seeds = endpoint
        ? std::span<const double>{} : std::span<const double>(seed_storage);
    if (bounded) {
      if (connectorExceeds(A, pivot, prefix_distance, pruneLimit())) {
        ++route.sources_bound_pruned;
        // The skipped source cannot win, but its retained seeds remain
        // reported controls, as for a searched source. Endpoint seeds report
        // controls only when searched (their pre-replay prunes cannot).
        if (!endpoint) {
          const double control =
              retainedControlDistance(qualifier, A, pivot, goal, extra_seeds, work);
          if (std::isfinite(control))
            route.best_control = std::min(route.best_control, prefix_distance + control);
        }
        return;
      }
    }
    const auto terminal = searchPivotHeading(qualifier, A, pivot, goal, extra_seeds, work);
    if (std::isfinite(terminal.retained_control_distance))
      route.best_control = std::min(route.best_control,
                                    prefix_distance + terminal.retained_control_distance);
    route.peak_chain_payload = std::max(route.peak_chain_payload,
        payload(route.poses, route.source_ids, route.chain) + pendingPayload() +
        payload(prefix_poses, prefix_sources, prefix_path) +
        prefix_words.capacity() * sizeof(std::uint32_t));
    if (!terminal.clear() || !endpointMatches(terminal.legs[1].path.endpoint)) return;
    const double distance = prefix_distance + terminal.distance;
    if (distance >= best_distance) return;
    best_distance = distance;
    prefix_poses.push_back(terminal.pivot);
    prefix_poses.push_back(goal);
    prefix_sources.push_back(id);
    prefix_sources.push_back(id);
    for (const auto &leg : terminal.legs)
      prefix_path.segments.push_back({leg.path, qualifiedClear()});
    route.peak_chain_payload = std::max(route.peak_chain_payload,
        payload(route.poses, route.source_ids, route.chain) + pendingPayload() +
        payload(prefix_poses, prefix_sources, prefix_path) +
        record.word_bytes + prefix_words.capacity() * sizeof(std::uint32_t));
    route.poses = std::move(prefix_poses);
    route.source_ids = std::move(prefix_sources);
    route.chain = std::move(prefix_path);
    route.prefix_word_match = true; // Retained prefix words matched; new legs are literal.
    route.route_kind = endpoint ? "endpoint_terminal" : "pivot_terminal";
    found.new_legs = 2;
  };

  // Search policy sources in order of their free-heading price, so a good
  // route found early prunes the rest. Only equal-length ties can change.
  {
    std::array<std::pair<double, std::size_t>, kSourceLimit> order{};
    for (std::size_t i = 0; i < candidate_count; ++i) {
      const double price = freeHeadingPrice(view.precedingPose(candidates[i].source),
                                            view.pivotPose(candidates[i].source));
      order[i] = {std::isfinite(price) ? view.prefixDistance(candidates[i].source) + price
                                       : std::numeric_limits<double>::infinity(), i};
    }
    std::stable_sort(order.begin(), order.begin() + candidate_count);
    std::array<SourceSeed, kSourceLimit> sorted{};
    for (std::size_t i = 0; i < candidate_count; ++i) sorted[i] = candidates[order[i].second];
    candidates = sorted;
  }
  for (std::size_t candidate_index = 0;; ++candidate_index) {
    if (candidate_index == candidate_count) {
      if (route.spatial_fallback || policyRouteFound()) break;
      route.spatial_fallback = true;
      // An improved free-heading owner can hide a useful terminal-pose
      // pivot. Retained pivot positions remain available independently of
      // ownership. Preserve the original fill-to-12 policy; a full initial
      // pool receives exactly one additional nearest unused pivot.
      const std::size_t available = std::max<std::size_t>(
          1, kInitialSourceLimit - candidate_count);
      std::array<std::pair<double, int>, kSourceLimit> nearest;
      nearest.fill({std::numeric_limits<double>::infinity(), -1});
      for (std::size_t id = 1; id < view.sources(); ++id) {
        ++route.spatial_source_visits;
        const int identity = static_cast<int>(id);
        if (!view.validSource(identity) || view.parent(identity) < 0) continue;
        bool present = false;
        for (std::size_t i = 0; i < candidate_count; ++i)
          present |= candidates[i].source == identity;
        if (present) continue;
        const State position = view.pivotPose(identity);
        const std::pair<double, int> ranked{
            std::hypot(position.x - goal.x, position.y - goal.y), identity};
        const auto end = nearest.begin() + available;
        const auto slot = std::lower_bound(nearest.begin(), end, ranked);
        if (slot == end) continue;
        std::move_backward(slot, end - 1, end);
        *slot = ranked;
      }
      const std::size_t previous_count = candidate_count;
      for (std::size_t i = 0; i < available && nearest[i].second >= 0; ++i) {
        const int id = nearest[i].second;
        addSource(id, view.birthHeading(id), candidates.size());
      }
      route.spatial_sources_added = candidate_count - previous_count;
      if (candidate_index == candidate_count) break;
    }
    trySource(candidates[candidate_index].source, candidates[candidate_index].theta);
  }
  route.sources_considered = candidate_count;

  // Endpoint proposals, as in the main repository's fixed-heading query, on
  // the same condition as the nearest-pivot fallback; a pending extension
  // prunes them by its length.
  if (!policyRouteFound()) {
    route.endpoint_fallback = true;
    constexpr std::array<int, 16> neighbours{1, 0, 0, 1, -1, 0, 0, -1,
                                             1, 1, -1, 1, -1, -1, 1, -1};
    for (std::size_t k = 0; k < neighbours.size() &&
                            route.endpoint_candidates < kEndpointLimit; k += 2) {
      const int x = target_x + neighbours[k], y = target_y + neighbours[k + 1];
      if (!view.inside(x, y) || !view.free(x, y) || !view.reached(x, y) ||
          !view.validSource(view.owner(x, y)))
        continue;
      ++route.endpoint_candidates;
      trySource(view.owner(x, y), view.heading(x, y), x, y);
    }
  }

  settleExtension();
  route.peak_chain_payload = std::max(route.peak_chain_payload,
      payload(route.poses, route.source_ids, route.chain));
  qualifier.reportExactWork(work);
  found.selected_distance = best_distance;
  if (!std::isfinite(best_distance)) {
    route.status = GoalStatus::Unsupported;
    return found;
  }
  route.status = GoalStatus::Clear;
  return found;
}

} // namespace

bool confirmNewLegs(const Field<double> &map, const double radius,
                    const std::vector<State> &poses,
                    const audit::ResolvedChainAuditResult &chain,
                    const std::size_t new_legs, QueryWork &work) {
  if (new_legs > chain.segments.size() || poses.size() < chain.segments.size())
    return false;
  for (std::size_t index = chain.segments.size() - new_legs;
       index < chain.segments.size(); ++index) {
    const auto &path = chain.segments[index].path;
    ++work.confirmations;
    const auto exact = audit::continuousPathClearance(
        map, poses[index], radius, path.primitives, path.primitive_count);
    ++work.exact_checks;
    work.clearance_intervals += exact.visited_intervals;
    work.clearance_events += exact.visited_events;
    work.free_box_attempts += exact.free_box_attempts;
    work.free_box_cells += exact.free_box_cells;
    work.free_box_clears += exact.free_box_clears;
    if (!path.replayed() || !exact.clear()) return false;
  }
  return true;
}

GoalRoute reconstructGoal(const RetainedView &view, const int target_x,
                          const int target_y, const State goal,
                          const RecordControl &record, QueryWork &work,
                          const GoalQueryOptions &options,
                          const QueryAuthority authority) {
  auto found = search(view, target_x, target_y, goal, record, work, options, authority);
  GoalRoute &route = found.route;
  if (authority == QueryAuthority::Shared && route.status == GoalStatus::Clear &&
      found.new_legs > 0) {
    // The two authorities have agreed on every logged Clear/Blocked decision;
    // where the exact checker does not confirm a selected leg, recompute.
    if (options.reject_confirmation ||
        !confirmNewLegs(view.map(), view.radius(), route.poses, route.chain,
                        found.new_legs, work)) {
      auto exact = reconstructGoal(view, target_x, target_y, goal, record, work,
                                   options, QueryAuthority::Exact);
      absorbCounters(exact, route);
      exact.authority_fallbacks += 1;
      return exact;
    }
  }
  if (route.status != GoalStatus::Clear) return std::move(found.route);
  const double actual_length = physicalLength(route.chain);
  route.chain.segment_sum = actual_length;
  route.chain.complete_distance_residual = std::abs(actual_length - found.selected_distance);
  if (route.chain.complete_distance_residual >
      audit::kReplayTolerance * std::max(view.radius(), actual_length))
    route.status = GoalStatus::Incomplete;
  else if (std::isfinite(route.best_control) && actual_length > route.best_control +
      1e-10 * std::max({1.0, view.radius(), actual_length, route.best_control}))
    route.status = GoalStatus::Incomplete;
  return std::move(found.route);
}

GoalRoute answerFixedGoal(const RetainedView &view, const int target_x,
                          const int target_y, const State goal, QueryWork &work) {
  RetainedRecord retained;
  RecordControl record;
  if (view.inside(target_x, target_y) && view.reached(target_x, target_y) &&
      view.validSource(view.owner(target_x, target_y)) &&
      std::abs(std::remainder(view.heading(target_x, target_y) - goal.theta,
                              2.0 * std::numbers::pi)) <= audit::kReplayTolerance) {
    retained = replayRetainedRecord(view, target_x, target_y, &work);
    if (retained.valid)
      record = {&retained.poses, &retained.source_ids, &retained.chain,
                retained.words.capacity() * sizeof(std::uint32_t), retained.word_match};
  }
  return reconstructGoal(view, target_x, target_y, goal, record, work);
}

} // namespace RS_marching::query
