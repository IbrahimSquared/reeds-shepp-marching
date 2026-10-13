#include "marching/Marching.hpp"

#ifdef RS_INTERACTIVE_API

#include "thirdparty/nlohmann/json.hpp"
#include "query/ReconstructionQuery.hpp"
#include "query/RetainedView.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#ifdef RS_COUNT_WORK
extern "C" void rs_heap_checkpoint(const char *) __attribute__((weak));
#endif

namespace RS_marching {
namespace {

using json = nlohmann::json;
constexpr std::size_t kMaximumPolylinePoints = 200000;

const char *chainStatus(const audit::ResolvedChainStatus status) {
  switch (status) {
  case audit::ResolvedChainStatus::Clear: return "clear";
  case audit::ResolvedChainStatus::Indeterminate: return "indeterminate";
  case audit::ResolvedChainStatus::Blocked: return "blocked";
  case audit::ResolvedChainStatus::Incomplete: return "incomplete";
  case audit::ResolvedChainStatus::Unsupported: return "unsupported";
  case audit::ResolvedChainStatus::Invalid: return "invalid";
  }
  return "invalid";
}

const char *clearanceStatus(const audit::ContinuousClearanceStatus status) {
  switch (status) {
  case audit::ContinuousClearanceStatus::Clear: return "clear";
  case audit::ContinuousClearanceStatus::Blocked: return "blocked";
  case audit::ContinuousClearanceStatus::Indeterminate: return "indeterminate";
  case audit::ContinuousClearanceStatus::Incomplete: return "incomplete";
  case audit::ContinuousClearanceStatus::Invalid: return "invalid";
  }
  return "invalid";
}

const char *clearanceFeature(const audit::ContinuousClearanceFeature feature) {
  switch (feature) {
  case audit::ContinuousClearanceFeature::None: return "none";
  case audit::ContinuousClearanceFeature::InvalidInput: return "invalid_input";
  case audit::ContinuousClearanceFeature::MissingBranch: return "missing_branch";
  case audit::ContinuousClearanceFeature::NumericalBoundary: return "numerical_boundary";
  case audit::ContinuousClearanceFeature::MapExterior: return "map_exterior";
  case audit::ContinuousClearanceFeature::OccupiedCellInterior: return "occupied_cell_interior";
  case audit::ContinuousClearanceFeature::OccupiedSharedEdge: return "occupied_shared_edge";
  case audit::ContinuousClearanceFeature::OccupiedVertex: return "occupied_vertex";
  }
  return "invalid_input";
}

json poseJson(const State &pose) {
  return {{"x", pose.x}, {"y", pose.y}, {"heading_rad", pose.theta}};
}

State replayOne(const State source, const double radius,
                const audit::LegacyPrimitive &primitive) {
  std::array<audit::LegacyPrimitive, 5> one{};
  one[0] = primitive;
  return audit::replayLegacyPrimitives(source, radius, one, 1);
}

namespace interactive_detail {

audit::ResolvedChainStatus retainClearanceStatus(
    const audit::ResolvedChainStatus current,
    const audit::ContinuousClearanceStatus clearance) {
  constexpr std::array clearance_status{
      audit::ResolvedChainStatus::Invalid,
      audit::ResolvedChainStatus::Incomplete,
      audit::ResolvedChainStatus::Indeterminate,
      audit::ResolvedChainStatus::Blocked,
      audit::ResolvedChainStatus::Clear};
  return std::max(current, clearance_status.at(
      static_cast<std::size_t>(clearance)));
}

} // namespace interactive_detail

} // namespace

std::string Marching::geometricSnapshotJson() const {
  json distance = json::array();
  for (std::size_t y = 0; y < ny_; ++y) {
    for (std::size_t x = 0; x < nx_; ++x) {
      const double value = gScore_(x, y);
      if (sharedVisibilityField_->get(x, y) < 1.0 || !isUpdated_(x, y) ||
          !std::isfinite(value)) {
        distance.push_back(nullptr);
      } else {
        distance.push_back(value);
      }
    }
  }

  json pivots = json::array();
  for (std::size_t source = 0; source < nb_of_sources_; ++source) {
    const auto &prefix = pivot_prefixes_[source];
    const auto &light = lightSources_[source];
    const double heading = prefix.parent_source < 0
        ? light.theta + pi / 2.0 : prefix.birth_theta;
    const Point position = geometricPivotPosition(static_cast<int>(source));
    pivots.push_back({{"id", source}, {"x", position.x}, {"y", position.y},
                      {"heading_rad", heading}, {"parent", prefix.parent_source}});
  }

  const json metrics{
      {"iterations", nb_of_iterations_}, {"sources", nb_of_sources_},
      {"pivots", pivot_births_}, {"queue_pushes", queue_pushes_},
      {"queue_pops", queue_pops_}, {"max_queue_size", max_queue_size_},
      {"local_source_candidates", local_source_candidates_},
      {"geometric_solves", geometric_solves_},
      {"geometric_retargets", geometric_retargets_},
      {"geometric_fixed_continuations", geometric_fixed_continuations_},
      {"geometric_birth_attempts", geometric_birth_attempts_},
      {"geometric_cost_prunes", geometric_cost_prunes_},
      {"geometric_incumbent_prunes", geometric_incumbent_prunes_},
      {"geometric_incoming_cache_lookups", geometric_incoming_cache_lookups_},
      {"geometric_incoming_cache_hits", geometric_incoming_cache_hits_},
      {"geometric_leg_statuses", geometric_statuses_},
      {"clearance_queries", geometric_clearance_.queries},
      {"clearance_exact_checks", geometric_clearance_.exact_checks},
      {"clearance_filtered", geometric_clearance_.filtered},
      {"clearance_intervals", geometric_clearance_.intervals},
      {"clearance_events", geometric_clearance_.events},
      {"free_box_attempts", geometric_clearance_.free_box_attempts},
      {"free_box_cells", geometric_clearance_.free_box_cells},
      {"free_box_clears", geometric_clearance_.free_box_clears},
      {"first_arc_cache_lookups", geometric_clearance_.first_arc_cache_lookups},
      {"first_arc_cache_hits", geometric_clearance_.first_arc_cache_hits}};
  return json{{"width", nx_}, {"height", ny_}, {"distance", distance},
              {"pivots", pivots}, {"metrics", metrics}}.dump();
}

std::string Marching::queryGeometricPathJson(
    const int target_x, const int target_y, const bool check_collisions,
    const double sample_step, const std::optional<double> goal_heading,
    const query::GoalRoute *const prior) const {
#ifdef RS_COUNT_WORK
  struct HeapScope {
    HeapScope() { if (rs_heap_checkpoint) rs_heap_checkpoint("query_begin"); }
    ~HeapScope() { if (rs_heap_checkpoint) rs_heap_checkpoint("query_end"); }
  } heap_scope;
#endif
  const auto started = std::chrono::steady_clock::now();
  if (!std::isfinite(sample_step) || sample_step < 0.1 || sample_step > 5.0) {
    throw std::invalid_argument("sample_step must be finite and in [0.1,5]");
  }
  if (goal_heading && !std::isfinite(*goal_heading)) {
    throw std::invalid_argument("goal heading must be finite");
  }
  query::QueryWork query_work;
  double search_ms = 0.0;
  bool free_search = false;
  std::size_t ancestry_visits = 0, record_legs = 0;

  json result{{"ok", true}, {"target", {{"x", target_x}, {"y", target_y}}},
              {"stored_distance", nullptr}, {"reconstructed_length", nullptr},
              {"distance_residual", nullptr}, {"word_match", false},
              {"waypoints", json::array()}, {"polyline", json::array()},
              {"primitives", json::array()}, {"reversals", json::array()},
              {"reversal_count", 0}, {"collision_checked", false},
              {"collision_count", nullptr},
              {"collision_count_kind", "blocked_primitives"},
              {"collisions", json::array()}};
  if (goal_heading) {
    result["query_mode"] = "fixed_heading";
    result["target"]["heading_rad"] = *goal_heading;
    result["terminal_clearance_qualified"] = false;
  }
  const auto finish = [&](const char *status) {
    const auto now = std::chrono::steady_clock::now();
    result["status"] = status;
    result["timings_ms"] = {
        {"reconstruction", std::chrono::duration<double, std::milli>(now - started).count()},
        {"collision", 0.0},
        {"sampling", 0.0},
        {"total", std::chrono::duration<double, std::milli>(now - started).count()}};
    if (goal_heading || free_search) {
      result["timings_ms"]["search"] = search_ms;
      result["timings_ms"]["reconstruction"] =
          result["timings_ms"]["reconstruction"].get<double>() - search_ms;
    }
    return result.dump();
  };

  if (target_x < 0 || target_y < 0 || static_cast<std::size_t>(target_x) >= nx_ ||
      static_cast<std::size_t>(target_y) >= ny_) {
    return finish("invalid");
  }
  if (sharedVisibilityField_->get(target_x, target_y) < 1.0) {
    return finish("invalid");
  }
  const bool reached = isUpdated_(target_x, target_y) &&
      cameFrom_(target_x, target_y) >= 0 && std::isfinite(gScore_(target_x, target_y));
  if (!reached && !goal_heading) return finish("unreached");
  const int source = reached ? cameFrom_(target_x, target_y) : -1;
  std::vector<State> poses;
  std::vector<int> source_ids;
  std::vector<std::uint32_t> words;
  audit::ResolvedChainAuditResult checked;
  bool word_match = false;
  if (reached) result["stored_distance"] = gScore_(target_x, target_y);
  if (reached && (!goal_heading ||
      std::abs(std::remainder(thetas_(target_x, target_y) - *goal_heading,
                             2.0 * std::numbers::pi)) <= audit::kReplayTolerance)) {
    // A fixed-heading query drops a record with broken ancestry and keeps
    // searching, as answerFixedGoal does.
    const bool valid_source = static_cast<std::size_t>(source) < nb_of_sources_ &&
        static_cast<std::size_t>(source) < pivot_prefixes_.size();
    if (!valid_source && !goal_heading) return finish("incomplete");
    auto retained = valid_source
        ? query::replayRetainedRecord(query::RetainedView(*this), target_x, target_y,
                                      &query_work, &ancestry_visits)
        : query::RetainedRecord{};
    if (!retained.valid && !goal_heading) return finish("incomplete");
    if (retained.valid) {
      poses = std::move(retained.poses);
      source_ids = std::move(retained.source_ids);
      words = std::move(retained.words);
      checked = std::move(retained.chain);
      word_match = retained.word_match;
      record_legs = checked.segments.size();
    }
  }

  if (!goal_heading && word_match && checked.certified() && !poses.empty()) {
    // Keep the original record available even if a proposed route fails its
    // final checks. Free search adds no retained-state mutation.
    free_search = true;
    const query::RecordControl record{&poses, &source_ids, &checked,
                                      words.capacity() * sizeof(std::uint32_t), word_match};
    const State goal{static_cast<double>(target_x), static_cast<double>(target_y), 0.0};
    const auto search_started = std::chrono::steady_clock::now();
    auto route = query::reconstructGoal(query::RetainedView(*this), target_x, target_y,
        goal, record, query_work, query::GoalQueryOptions{.free_heading = true});
    search_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - search_started).count();
    // The stored terminal heading is also a valid free heading. Reuse the
    // existing bounded fixed-query families at that seed, with the original
    // record control. This adds pivot/alternative-word proposals without
    // claiming all terminal headings have been searched.
    query::GoalRoute seeded;
    const bool seed_searched = route.status == query::GoalStatus::Clear &&
                               route.chain.segment_sum > 0.0;
    if (seed_searched) {
      const auto seed_started = std::chrono::steady_clock::now();
      const State seed_goal{goal.x, goal.y, thetas_(target_x, target_y)};
      seeded = query::reconstructGoal(query::RetainedView(*this), target_x, target_y,
                                     seed_goal, record, query_work);
      search_ms += std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - seed_started).count();
    }
    const double tie = 1e-9 * std::max({1.0, r_, route.chain.segment_sum});
    const bool seed_used = seeded.status == query::GoalStatus::Clear &&
        (route.status != query::GoalStatus::Clear ||
         seeded.chain.segment_sum < route.chain.segment_sum - tie);
    auto &selected = seed_used ? seeded : route;
    result["query_mode"] = "free_heading";
    result["candidate_coverage"] = {
        {"source_policy", "retained record, direct/free record extensions, and existing bounded fixed-query families at the stored terminal heading"},
        {"stored_heading_search", {{"searched", seed_searched}, {"heading_rad", thetas_(target_x, target_y)},
            {"sources_considered", seeded.sources_considered}, {"source_limit", query::kSourceLimit},
            {"endpoint_candidates", seeded.endpoint_candidates}, {"endpoint_limit", query::kEndpointLimit},
            {"heading_samples", query::kHeadingSamples}, {"refinement_start_limit", query::kRefinementStarts},
            {"refinement_iteration_limit", query::kRefinementIterations}}},
        {"record_extension_window", static_cast<int>(std::min(
            static_cast<double>(query::kExtensionWindowCap), std::ceil(1.5 * r_) + 2.0))},
        {"record_extension_qualification_limit", query::kExtensionPriceLimit},
        {"ranking", "replayed URS candidate lengths rank surviving reached cells; all ranking solves/materializations counted; at most the qualification limit are repriced per search"},
        {"bounds", "replay-aware stored-prefix envelope and displacement prefilter; URS ranking is not a rejecting lower bound"},
        {"qualification", "shared ConnectorClearance; selected new legs confirmed by exact continuous checker"},
        {"omissions", "other extension cells, unrepresented words and other terminal-heading families; no global minimum guarantee"}};
    result["query_work"] = {
        {"solves", query_work.solves}, {"materializations", query_work.materializations},
        {"retained_replay_hits", query_work.retained_replay_hits},
        {"bound_solves", query_work.bound_solves},
        {"heading_evaluations", query_work.heading_evaluations},
        {"refinement_evaluations", query_work.refinement_evaluations},
        {"contact_evaluations", query_work.contact_evaluations},
        {"contact_scratch_bytes", query_work.contact_scratch_bytes},
        {"lazy_skips", query_work.lazy_skips}, {"witness_checks", query_work.witness_checks},
        {"clearance_checks", query_work.clearance_checks},
        {"confirmations", query_work.confirmations}, {"exact_checks", query_work.exact_checks},
        {"clearance_intervals", query_work.clearance_intervals},
        {"clearance_events", query_work.clearance_events},
        {"free_box_attempts", query_work.free_box_attempts},
        {"free_box_cells", query_work.free_box_cells}, {"free_box_clears", query_work.free_box_clears},
        {"replay_failures", query_work.replay_failures},
        {"blocked_legs", query_work.blocked}, {"indeterminate_legs", query_work.indeterminate},
        {"incomplete_legs", query_work.incomplete},
        {"ancestry_visits", ancestry_visits + route.ancestry_visits + seeded.ancestry_visits},
        {"prefix_legs_replayed", route.prefix_legs + seeded.prefix_legs},
        {"retained_record_legs_replayed", record_legs}, {"prefix_failures", route.prefix_failures + seeded.prefix_failures},
        {"extension_cells", route.extension_cells + seeded.extension_cells}, {"extensions_ranked", route.extensions_ranked},
        {"extensions_priced", route.extensions_priced + seeded.extensions_priced},
        {"extensions_qualified", route.extensions_qualified + seeded.extensions_qualified}, {"extension_rank", selected.extension_rank},
        {"authority_fallbacks", route.authority_fallbacks + seeded.authority_fallbacks},
        {"scratch_scope", "sum of search chain vector capacity peaks; excludes original control, helper stack, allocator and output"},
        {"peak_chain_payload_bytes", route.peak_chain_payload + seeded.peak_chain_payload}};
    const bool accepted = selected.status == query::GoalStatus::Clear;
    result["route_kind"] = accepted ? selected.route_kind : "retained_record";
    result["stored_heading_search_used"] = seed_used;
    result["free_search_record_fallback"] = !accepted;
    result["best_control_length"] = accepted ? selected.best_control : checked.segment_sum;
    if (accepted && std::string_view(selected.route_kind) != "retained_record") {
      poses = std::move(selected.poses);
      source_ids = std::move(selected.source_ids);
      checked = std::move(selected.chain);
      word_match = selected.prefix_word_match;
    }
    result["terminal_clearance_qualified"] = true;
    result["prefix_word_match"] = word_match;
    result["field_distance_delta"] = checked.segment_sum - gScore_(target_x, target_y);
  }

  if (goal_heading) {
    const State goal{static_cast<double>(target_x), static_cast<double>(target_y),
                     *goal_heading};
    const query::RecordControl record{&poses, &source_ids, &checked,
                                      words.capacity() * sizeof(std::uint32_t), word_match};
    const auto search_started = std::chrono::steady_clock::now();
    auto route = query::reconstructGoal(query::RetainedView(*this), target_x, target_y,
                                        goal, record, query_work);
    search_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - search_started).count();
    if (prior != nullptr) {
      const bool shorter = prior->status == query::GoalStatus::Clear &&
          (route.status != query::GoalStatus::Clear ||
           prior->chain.segment_sum < route.chain.segment_sum -
               1e-9 * std::max({1.0, r_, route.chain.segment_sum}));
      if (shorter) {
        // Only the geometry is taken: coverage, work and best_control_length
        // keep describing this final search; the prior's work is reported by
        // its caller.
        route.status = prior->status;
        route.poses = prior->poses;
        route.source_ids = prior->source_ids;
        route.chain = prior->chain;
        route.prefix_word_match = prior->prefix_word_match;
        route.route_kind = prior->route_kind;
      }
      result["prior_route_used"] = shorter;
    }
    result["candidate_coverage"] = {
        {"source_policy", "target and 3x3 owners, then target-owner ancestors; nearest retained pivots fill unused initial slots, with at most one overflow nearest pivot after a full initial pool fails; then at most eight reached-neighbor endpoint waypoints if those families fail (a pending record extension prunes them)"},
        {"source_limit", query::kSourceLimit}, {"sources_considered", route.sources_considered},
        {"spatial_fallback_used", route.spatial_fallback},
        {"endpoint_limit", query::kEndpointLimit},
        {"endpoint_candidates", route.endpoint_candidates},
        {"endpoint_fallback_used", route.endpoint_fallback},
        {"heading_samples", query::kHeadingSamples},
        {"refinement_start_limit", query::kRefinementStarts},
        {"refinement_iteration_limit", query::kRefinementIterations},
        {"qualification", "shared ConnectorClearance authority; selected new legs confirmed by the exact continuous checker"},
        {"record_extension_window", std::min(query::kExtensionWindowCap,
                                             static_cast<int>(std::ceil(1.5 * r_)) + 2)},
        {"record_extension_price_limit", query::kExtensionPriceLimit},
        {"bounds", "record extensions priced in order of g(c) + RS length lower bound, at most the price limit, and replayed before they become the incumbent; a candidate is skipped only when a certified lower bound on its physical length exceeds the physical incumbent: stored distances less the certified replay residual, legs by RS length bounds with endpoint slack; sources searched in URS-price order; refinement steps solved and qualified only if they can improve"},
        {"omissions", "other pivots, non-neighbor waypoints and unsampled feasible heading components"}};
    result["query_work"] = {
        {"solves", query_work.solves}, {"bound_solves", query_work.bound_solves},
        {"materializations", query_work.materializations},
        {"retained_replay_hits", query_work.retained_replay_hits},
        {"heading_evaluations", query_work.heading_evaluations},
        {"refinement_evaluations", query_work.refinement_evaluations},
        {"contact_evaluations", query_work.contact_evaluations},
        {"contact_scratch_bytes", query_work.contact_scratch_bytes},
        {"clearance_checks", query_work.clearance_checks}, {"lazy_skips", query_work.lazy_skips},
        {"witness_checks", query_work.witness_checks}, {"confirmations", query_work.confirmations},
        {"exact_checks", query_work.exact_checks},
        {"clearance_intervals", query_work.clearance_intervals},
        {"clearance_events", query_work.clearance_events},
        {"free_box_attempts", query_work.free_box_attempts},
        {"free_box_cells", query_work.free_box_cells},
        {"free_box_clears", query_work.free_box_clears},
        {"replay_failures", query_work.replay_failures}, {"blocked_legs", query_work.blocked},
        {"indeterminate_legs", query_work.indeterminate}, {"incomplete_legs", query_work.incomplete},
        {"ancestry_visits", ancestry_visits + route.ancestry_visits},
        {"prefix_legs_replayed", route.prefix_legs},
        {"retained_record_legs_replayed", record_legs}, {"prefix_failures", route.prefix_failures},
        {"sources_pruned", route.sources_pruned},
        {"sources_bound_pruned", route.sources_bound_pruned},
        {"extension_cells", route.extension_cells},
        {"extensions_priced", route.extensions_priced},
        {"extensions_qualified", route.extensions_qualified},
        {"extension_rank", route.extension_rank},
        {"authority_fallbacks", route.authority_fallbacks},
        {"spatial_source_visits", route.spatial_source_visits},
        {"spatial_sources_added", route.spatial_sources_added},
        {"endpoint_candidates", route.endpoint_candidates},
        {"scratch_scope", "chain vector element capacities; excludes helper stack, allocator and output"},
        {"peak_chain_payload_bytes", route.peak_chain_payload}};
    result["best_control_length"] = std::isfinite(route.best_control)
        ? json(route.best_control) : json(nullptr);
    result["route_kind"] = route.route_kind;
    if (route.status == query::GoalStatus::Unsupported) {
      result["unsupported_reason"] = "no clear path in bounded represented candidates";
      return finish("unsupported");
    }
    if (route.status == query::GoalStatus::Incomplete) return finish("incomplete");
    poses = std::move(route.poses);
    source_ids = std::move(route.source_ids);
    checked = std::move(route.chain);
    word_match = route.prefix_word_match;
    result["terminal_clearance_qualified"] = true;
    result["prefix_word_match"] = word_match;
    result["field_distance_delta"] = result["stored_distance"].is_number()
        ? json(checked.segment_sum - result["stored_distance"].get<double>()) : json(nullptr);
    const State end = checked.segments.empty() ? poses.front() : checked.segments.back().path.endpoint;
    result["endpoint_position_residual"] = std::hypot(end.x - goal.x, end.y - goal.y);
    result["endpoint_heading_residual"] =
        std::abs(std::remainder(end.theta - goal.theta, 2.0 * std::numbers::pi));
  }

  result["word_match"] = word_match;
  result["reconstructed_length"] = std::isfinite(checked.segment_sum)
      ? json(checked.segment_sum) : json(nullptr);
  result["distance_residual"] = std::isfinite(checked.complete_distance_residual)
      ? json(checked.complete_distance_residual) : json(nullptr);

  for (std::size_t i = 0; i < poses.size(); ++i) {
    const char *role = i == 0 ? "start" : (i + 1 == poses.size() ? "target" : "pivot");
    result["waypoints"].push_back(
        {{"x", poses[i].x}, {"y", poses[i].y}, {"heading_rad", poses[i].theta},
         {"source_id", source_ids[i]}, {"role", role}});
  }

  const auto reconstruction_done = std::chrono::steady_clock::now();

  std::size_t samples = 1 + (checked.segments.empty() ? 0 : checked.segments.size() - 1);
  for (const auto &segment : checked.segments) {
    for (std::size_t i = 0; i < segment.path.primitive_count; ++i) {
      const double length = segment.path.primitives[i].length;
      if (!std::isfinite(length) || length < 0.0) {
        return finish("incomplete");
      }
      if (length > 0.0) {
        const double count = std::ceil(length / sample_step);
        if (!std::isfinite(count) || count > kMaximumPolylinePoints ||
            samples > kMaximumPolylinePoints - static_cast<std::size_t>(count)) {
          throw std::runtime_error("query exceeds the 200000-point sampling limit");
        }
        samples += static_cast<std::size_t>(count);
      }
    }
  }

  json polyline = json::array();
  State cursor = poses.front();
  polyline.push_back({cursor.x, cursor.y, cursor.theta});
  int previous_gear = 0;
  double arclength = 0.0;
  std::size_t primitive_index = 0;
  for (std::size_t segment_index = 0; segment_index < checked.segments.size(); ++segment_index) {
    const auto &segment = checked.segments[segment_index];
    cursor = poses[segment_index];
    if (segment_index != 0) {
      polyline.push_back({cursor.x, cursor.y, cursor.theta});
    }
    for (std::size_t index = 0; index < segment.path.primitive_count; ++index, ++primitive_index) {
      const auto primitive = segment.path.primitives[index];
      const State primitive_start = cursor;
      const State primitive_end = replayOne(cursor, r_, primitive);
      const std::size_t sample_begin = polyline.size() - 1;
      if (primitive.length > 0.0) {
        const std::size_t count = static_cast<std::size_t>(std::ceil(primitive.length / sample_step));
        for (std::size_t step = 1; step <= count; ++step) {
          audit::LegacyPrimitive partial = primitive;
          partial.length = primitive.length * static_cast<double>(step) / static_cast<double>(count);
          const State point = replayOne(primitive_start, r_, partial);
          polyline.push_back({point.x, point.y, point.theta});
        }
      }
      const std::size_t sample_end = polyline.size() - 1;
      const char type[2] = {primitive.motion_type, '\0'};
      json primitive_json{{"index", primitive_index}, {"leg_index", segment_index},
                          {"type", type}, {"gear", primitive.direction},
                          {"length", primitive.length}, {"start", poseJson(primitive_start)},
                          {"end", poseJson(primitive_end)}, {"sample_begin", sample_begin},
                          {"sample_end", sample_end}, {"clearance", "not_checked"}};
      if (primitive.length != 0.0 && previous_gear != 0 && previous_gear != primitive.direction) {
        result["reversals"].push_back({{"x", primitive_start.x}, {"y", primitive_start.y},
                                        {"heading_rad", primitive_start.theta}, {"arclength", arclength},
                                        {"primitive_index", primitive_index}});
      }
      if (primitive.length != 0.0) previous_gear = primitive.direction;
      arclength += primitive.length;
      result["primitives"].push_back(std::move(primitive_json));
      cursor = primitive_end;
    }
  }
  result["polyline"] = std::move(polyline);
  result["reversal_count"] = result["reversals"].size();

  const auto collision_started = std::chrono::steady_clock::now();
  audit::ResolvedChainStatus output_status = checked.status;
  if (check_collisions) {
    result["collision_checked"] = true;
    std::uint64_t box_attempts = 0, box_cells = 0, box_clears = 0;
    primitive_index = 0;
    for (std::size_t segment_index = 0; segment_index < checked.segments.size(); ++segment_index) {
      const auto &segment = checked.segments[segment_index];
      for (std::size_t index = 0; index < segment.path.primitive_count; ++index, ++primitive_index) {
        const auto &primitive = segment.path.primitives[index];
        if (primitive.length == 0.0) {
          result["primitives"][primitive_index]["clearance"] = "clear";
          output_status = interactive_detail::retainClearanceStatus(
              output_status, audit::ContinuousClearanceStatus::Clear);
          continue;
        }
        const auto clearance = audit::continuousPathIntervalClearance(
            *sharedVisibilityField_, poses[segment_index], r_, segment.path.primitives,
            segment.path.primitive_count, {index, 0.0, primitive.length});
        box_attempts += clearance.free_box_attempts;
        box_cells += clearance.free_box_cells;
        box_clears += clearance.free_box_clears;
        result["primitives"][primitive_index]["clearance"] = clearanceStatus(clearance.status);
        if (clearance.status == audit::ContinuousClearanceStatus::Blocked) {
          result["collisions"].push_back({{"x", clearance.witness.x}, {"y", clearance.witness.y},
                                           {"primitive_index", primitive_index},
                                           {"feature", clearanceFeature(clearance.feature)}});
        }
        output_status = interactive_detail::retainClearanceStatus(
            output_status, clearance.status);
      }
    }
    result["collision_count"] = result["collisions"].size();
    result["collision_work"] = {{"free_box_attempts", box_attempts},
                                {"free_box_cells", box_cells},
                                {"free_box_clears", box_clears}};
  }
  const auto collision_done = std::chrono::steady_clock::now();
  const auto finished = collision_done;
  result["status"] = (check_collisions || goal_heading.has_value()) && output_status == audit::ResolvedChainStatus::Clear
      ? "clear" : (!check_collisions && output_status == audit::ResolvedChainStatus::Clear
                         ? "reconstructed" : chainStatus(output_status));
  result["timings_ms"] = {
      {"reconstruction", std::chrono::duration<double, std::milli>(reconstruction_done - started).count()},
      {"collision", std::chrono::duration<double, std::milli>(collision_done - collision_started).count()},
      {"sampling", std::chrono::duration<double, std::milli>(collision_started - reconstruction_done).count()},
      {"total", std::chrono::duration<double, std::milli>(finished - started).count()}};
  if (goal_heading || free_search) {
    result["timings_ms"]["search"] = search_ms;
    result["timings_ms"]["reconstruction"] =
        result["timings_ms"]["reconstruction"].get<double>() - search_ms;
  }
  return result.dump();
}

} // namespace RS_marching

#endif
