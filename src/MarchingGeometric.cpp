#include "marching/Marching.hpp"
#include "misc/AngleRemainder.hpp"

#include "thirdparty/nlohmann/json.hpp"
#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>

namespace RS_marching {

std::uint64_t Marching::geometricSelectorCacheHash(const std::uint64_t source,
                                                   const std::uint64_t target) {
  const auto mix = [](std::uint64_t value) {
    value = (value ^ (value >> 30)) * 0xbf58476d1ce4e5b9ULL;
    value = (value ^ (value >> 27)) * 0x94d049bb133111ebULL;
    return value ^ (value >> 31);
  };
  return mix(source ^ std::rotl(mix(target), 32));
}

void Marching::resetGeometricSelectorCache() {
  if (geometric_selector_cache_.size() != selector_memo_capacity_)
    geometric_selector_cache_.assign(selector_memo_capacity_, SelectorMemoEntry{});
  else
    std::fill(geometric_selector_cache_.begin(), geometric_selector_cache_.end(),
              SelectorMemoEntry{});
}

Point Marching::geometricPivotPosition(const int source) const {
  const auto &prefix = pivot_prefixes_[source];
  if (std::isfinite(prefix.pivot_x))
    return {prefix.pivot_x, prefix.pivot_y};
  return {static_cast<double>(lightSources_[source].x),
          static_cast<double>(lightSources_[source].y)};
}

bool Marching::selectGeometricTripletHeadings(
    const PivotPrefix &prefix, const int source, const int x, const int y,
    State &pivot, State &target, SelectorMemoEntry **const memo) {
  if (memo != nullptr) *memo = nullptr;
  const auto fresh = [&] {
    std::array<double, 2> ignored;
    return selectTripletHeadings(prefix.preceding_pose, {pivot.x, pivot.y},
                                 {target.x, target.y}, true, pivot, target,
                                 ignored, &prefix.prepared_triplet);
  };
  if (source < 0 || static_cast<std::size_t>(source) >= nb_of_sources_ || x < 0 ||
      y < 0 || static_cast<std::size_t>(x) >= nx_ || static_cast<std::size_t>(y) >= ny_)
    return fresh();

  if (geometric_selector_cache_.empty())
    resetGeometricSelectorCache();
  ++geometric_selector_cache_lookups_;
  const std::uint64_t source_key = static_cast<std::uint64_t>(source);
  const std::uint64_t target_key =
      static_cast<std::uint64_t>(y) * static_cast<std::uint64_t>(nx_) +
      static_cast<std::uint64_t>(x);
  auto &entry = geometric_selector_cache_[
      geometricSelectorCacheHash(source_key, target_key) &
      (selector_memo_capacity_ - 1)];
  if (memo != nullptr) *memo = &entry;
  if (entry.source == source && entry.target == target_key) {
    ++geometric_selector_cache_hits_;
    pivot = entry.pivot;
    target = entry.target_state;
    return entry.selected;
  }

  const bool selected = fresh();
  entry = {source, selected, false, target_key, pivot, target};
  return selected;
}

Marching::LegStatus Marching::geometricLeg(const State from, const State to, double &distance,
                            std::uint32_t &word, const double distance_limit,
                            const double complete_prefix, const double incumbent_cutoff,
                            geometry::FirstArcClear *const owned_arcs,
                            const Solver::PlanarPreparation *const incoming_planar) {
  ++geometric_solves_;
  int condition = 0, quadrant = 0;
  if (incoming_planar) {
    solver_.acceleratedRSPlanner(*incoming_planar, from.theta, to.theta,
                                 distance, r_, condition, quadrant);
  } else {
    solver_.acceleratedRSPlanner(from, to, distance, r_, condition, quadrant);
  }
  if (std::isfinite(distance) && distance > distance_limit) {
    ++geometric_cost_prunes_;
    return LegStatus::PriceRefused;
  }
  if (std::isfinite(distance) && std::isfinite(complete_prefix) &&
      std::isfinite(incumbent_cutoff) && complete_prefix + distance >= incumbent_cutoff) {
    ++geometric_incumbent_prunes_;
    return LegStatus::PriceRefused;
  }
  audit::ReplayPrimitiveBuffer prepared;
  const auto path = audit::replaySelectedLegacyPath(
      solver_, from, to, distance, r_, condition, quadrant, &prepared);
  word = audit::selectedPathWord(path);
  const auto status = geometric_clearance_.check(from, r_, path, owned_arcs, &prepared);
  if (!geometric_clearance_.filter_only)
    ++geometric_statuses_[static_cast<std::size_t>(status)];
  return static_cast<LegStatus>(status);
}

Marching::LegStatus Marching::geometricFreeHeadingLeg(
    const State from, State &to, double &distance, std::uint32_t &word,
    const double distance_limit, const double complete_prefix,
    const double incumbent_cutoff, const double fallback_orientation,
    geometry::FirstArcClear *const owned_arcs) {
  ++geometric_solves_;
  audit::ReplayPrimitiveBuffer prepared;
  auto direct = audit::solveFreeHeadingCandidate(
      from, to.x, to.y, r_, distance_limit, complete_prefix, incumbent_cutoff, &prepared);
  // Retargeting selected a terminal branch in its canonical chart. Across
  // a periodic-heading seam, world URS may select the other equal-cost branch.
  // Qualify the selected heading before using even a direct price rejection.
  if (!std::isfinite(fallback_orientation) &&
      !(std::abs(detail::boundedNearestAngleRemainder(
          direct.target_theta - to.theta, 2.0 * pi)) <=
        audit::kReplayTolerance))
    return geometricLeg(from, to, distance, word, distance_limit,
                        complete_prefix, incumbent_cutoff, owned_arcs);
  if (direct.status == audit::LegacyReplayStatus::CostPruned) {
    if (direct.distance > distance_limit)
      ++geometric_cost_prunes_;
    else
      ++geometric_incumbent_prunes_;
    return LegStatus::PriceRefused;
  }
  if (direct.replayed()) {
    if (!std::isfinite(fallback_orientation)) {
      if (!std::isfinite(to.theta))
        return LegStatus::Invalid;
      direct.condition = -2;
      direct.target_theta = to.theta;
      direct.endpoint_heading_residual =
          std::abs(detail::boundedNearestAngleRemainder(
              direct.endpoint.theta - to.theta, 2.0 * pi));
      if (!(direct.endpoint_heading_residual <= audit::kReplayTolerance))
        return geometricLeg(from, to, distance, word, distance_limit,
                            complete_prefix, incumbent_cutoff, owned_arcs);
    } else {
      to.theta = direct.target_theta;
    }
    distance = direct.distance;
    word = audit::selectedPathWord(direct);
    const auto status = geometric_clearance_.check(from, r_, direct, owned_arcs, &prepared);
    if (!geometric_clearance_.filter_only)
      ++geometric_statuses_[static_cast<std::size_t>(status)];
    return static_cast<LegStatus>(status);
  }
  if (std::isfinite(fallback_orientation)) {
    double omega = std::numeric_limits<double>::infinity();
    urs_.getOmega(from.x, from.y, to.x, to.y, r_, fallback_orientation, omega);
    to.theta = omega + fallback_orientation;
    if (!std::isfinite(to.theta))
      return LegStatus::Invalid;
  }
  return geometricLeg(from, to, distance, word, distance_limit,
                      complete_prefix, incumbent_cutoff, owned_arcs);
}

Marching::LegStatus Marching::geometricIncomingLeg(const int source, const double pivot_heading,
                                    double &distance, std::uint32_t &word,
                                    const double distance_limit) {
  auto &prefix = pivot_prefixes_[source];
  const State from = prefix.preceding_pose;
  const Point position = geometricPivotPosition(source);
  const State pivot{position.x, position.y, pivot_heading};
  if (prefix.incoming_leg_cache.eligible && std::isfinite(pivot_heading)) {
    const std::uint64_t key = std::bit_cast<std::uint64_t>(pivot_heading);
    ++geometric_incoming_cache_lookups_;
    auto &cache = prefix.incoming_leg_cache;
    if (cache.valid && cache.heading == key) {
      ++geometric_incoming_cache_hits_;
      distance = cache.distance;
      word = cache.word;
      if (std::isfinite(distance) && distance > distance_limit) {
        ++geometric_cost_prunes_;
        return LegStatus::PriceRefused;
      }
      return cache.status;
    }
    const auto status = geometricLeg(from, pivot, distance, word,
                                    distance_limit,
                                    std::numeric_limits<double>::infinity(),
                                    std::numeric_limits<double>::infinity(),
                                    &prefix.preceding_arcs,
                                    &prefix.incoming_planar);
    // A price refusal is not a blocked-geometry cache entry.
    if (std::isfinite(distance) && distance > distance_limit)
      return LegStatus::PriceRefused;
    cache.heading = key;
    cache.distance = distance;
    cache.word = word;
    cache.status = status;
    cache.valid = true;
    return status;
  }
  return geometricLeg(from, pivot, distance, word,
                      distance_limit,
                      std::numeric_limits<double>::infinity(),
                      std::numeric_limits<double>::infinity(),
                      &prefix.preceding_arcs);
}

// Same-source neighbour records with a valid accepted score, ordered by stored
// incoming length (clear selected heading: only the offering cell). Duplicates
// are dropped.
std::size_t Marching::geometricHeadingSeeds(
    const int source, const int x, const int y, const int offer_x, const int offer_y,
    const bool clear_selected, const double selected_heading,
    std::array<std::pair<double, int>, 8> &seeds) const {
  std::size_t count = 0;
  for (int k = 0; k < 16; k += 2) {
    const int nx = x + neighbours_[k], ny = y + neighbours_[k + 1];
    if ((clear_selected && (nx != offer_x || ny != offer_y)) || !isValid(nx, ny) ||
        sharedVisibilityField_->get(nx, ny) < 1.0 || !isUpdated_(nx, ny) ||
        cameFrom_(nx, ny) != source || !(gScore_(nx, ny) < std::numeric_limits<double>::max()))
      continue;
    const double heading = geometric_pivot_theta_(nx, ny);
    const double incoming = geometric_incoming_distance_(nx, ny);
    // The birth heading is already priced by the fixed continuation.
    if (!std::isfinite(heading) || !std::isfinite(incoming) || incoming < 0.0 ||
        heading == selected_heading || heading == pivot_prefixes_[source].birth_theta)
      continue;
    const std::pair<double, int> seed{incoming, k};
    std::size_t i = 0;
    while (i < count && geometric_pivot_theta_(x + neighbours_[seeds[i].second],
                                               y + neighbours_[seeds[i].second + 1]) != heading)
      ++i;
    if (i == count)
      seeds[count++] = seed;
    else
      seeds[i] = std::min(seeds[i], seed);
  }
  std::sort(seeds.begin(), seeds.begin() + count);
  return count;
}

// This bounded search probes between a Clear anchor and a refused heading.
// It uses geometric expansion and bisection at a fixed angular resolution;
// it does not establish a contact boundary or a constrained optimum. Every
// accepted heading is freshly qualified.
void Marching::geometricContactHeading(PivotPrefix &prefix, const int source,
                                       const State target,
                                       const double blocked_heading,
                                       const double incumbent_cutoff,
                                       triplet &best, const double nearest_blocked) {
  if (!std::isfinite(blocked_heading) || !std::isfinite(best.pivot_theta) ||
      !(best.distance < std::numeric_limits<double>::max()))
    return;
  // A same-source neighbour heading is one bounded stored proposal. The search
  // remains restricted to the birth-heading case used by this heuristic.
  if (best.pivot_theta != prefix.birth_theta)
    return;
  const Point position = geometricPivotPosition(source);
  double good = best.pivot_theta;
  double bad = good + detail::boundedNearestAngleRemainder(blocked_heading - good, 2.0 * pi);
  // A sampled heading between the anchor and the selected heading that was not
  // better than the anchor bounds the bracket more tightly.
  if (std::isfinite(nearest_blocked)) {
    const double tight = good + detail::boundedNearestAngleRemainder(nearest_blocked - good, 2.0 * pi);
    if ((tight - good) * (bad - good) > 0.0 && std::abs(tight - good) < std::abs(bad - good))
      bad = tight;
  }
  // Use a fixed angular resolution for this bounded heuristic.
  constexpr double resolution = 5e-6;
  const double direction = bad > good ? 1.0 : -1.0;
  ++geometric_contact_searches_;
  const double anchor = best.distance;
  // Probes only need a sound Clear; undecided probes stop the search at the
  // spatial filter's resolution instead of paying exact interval work.
  struct FilterOnly {
    geometry::ConnectorClearance &clearance;
    explicit FilterOnly(geometry::ConnectorClearance &c) : clearance(c) { clearance.filter_only = true; }
    ~FilterOnly() { clearance.filter_only = false; }
  } filter_only(geometric_clearance_);
  const auto improves = [&](const double heading) {
    ++geometric_contact_steps_;
    State pivot{position.x, position.y, heading};
    State terminal = target;
    double incoming = 0.0, outgoing = 0.0;
    std::uint32_t incoming_word = 0, outgoing_word = 0;
    // Price the incoming leg, then qualify the short outgoing leg before the
    // long incoming one.
    int condition = 0, quadrant = 0;
    ++geometric_solves_;
    if (prefix.incoming_leg_cache.eligible) {
      solver_.acceleratedRSPlanner(prefix.incoming_planar,
                                   prefix.preceding_pose.theta, pivot.theta,
                                   incoming, r_, condition, quadrant);
    } else {
      solver_.acceleratedRSPlanner(prefix.preceding_pose, pivot, incoming, r_,
                                   condition, quadrant);
    }
    if (!(prefix.distance + incoming < best.distance))
      return false;
    const bool outgoing_clear = geometricFreeHeadingLeg(
        pivot, terminal, outgoing, outgoing_word, std::numeric_limits<double>::max(),
        prefix.distance + incoming, std::min(best.distance, incumbent_cutoff),
        heading - pi / 2.0) == LegStatus::Clear;
    if (!outgoing_clear || !(prefix.distance + incoming + outgoing < best.distance))
      return false;
    if (geometricLeg(prefix.preceding_pose, pivot, incoming, incoming_word,
                      best.distance - prefix.distance - outgoing,
                      std::numeric_limits<double>::infinity(),
                      std::numeric_limits<double>::infinity(), &prefix.preceding_arcs,
                      prefix.incoming_leg_cache.eligible
                          ? &prefix.incoming_planar
                          : nullptr) != LegStatus::Clear)
      return false;
    best.distance = prefix.distance + incoming + outgoing;
    best.theta = terminal.theta;
    best.pivot_theta = heading;
    best.incoming_distance = incoming;
    best.incoming_word = incoming_word;
    best.outgoing_word = outgoing_word;
    return true;
  };
  for (double step = resolution; direction * (bad - good) > resolution; step *= 2.0) {
    const double heading = good + direction * step;
    if (direction * (bad - heading) <= 0.0)
      break;
    if (!improves(heading)) {
      bad = heading;
      break;
    }
    good = heading;
  }
  while (direction * (bad - good) > resolution) {
    const double middle = 0.5 * (good + bad);
    if (improves(middle))
      good = middle;
    else
      bad = middle;
  }
  geometric_contact_wins_ += best.distance < anchor;
}

triplet Marching::geometricAnalyticRescueCandidate(
    const int source, const int x, const int y, const double incumbent_cutoff) {
  triplet result{std::numeric_limits<double>::max(),
                 std::numeric_limits<double>::quiet_NaN(), source};
  if (source < 0 || static_cast<std::size_t>(source) >= nb_of_sources_ ||
      static_cast<std::size_t>(source) >= pivot_prefixes_.size())
    return result;
  auto &prefix = pivot_prefixes_[source];
  if (prefix.parent_source < 0 || !std::isfinite(prefix.distance) ||
      prefix.distance < 0.0 || !std::isfinite(r_) || r_ <= 0.0)
    return result;

  const Point position = geometricPivotPosition(source);
  const State pivot_position{position.x, position.y, 0.0};
  const State target{static_cast<double>(x), static_cast<double>(y), 0.0};
  if (pivot_position.x == target.x && pivot_position.y == target.y)
    return result;

  ++geometric_rescue_attempts_;
  const double psi = std::atan2(target.y - pivot_position.y,
                                target.x - pivot_position.x);
  for (const double heading : {psi + pi / 2.0, psi - pi / 2.0}) {
    State pivot = pivot_position;
    pivot.theta = heading;
    double incoming = 0.0;
    std::uint32_t incoming_word = 0;
    if (geometricIncomingLeg(source, heading, incoming, incoming_word) != LegStatus::Clear)
      continue;

    const double frame = pivot.theta - pi / 2.0;
    double omega = std::numeric_limits<double>::infinity();
    urs_.getOmega(pivot.x, pivot.y, target.x, target.y, r_, frame, omega);
    const double phi = omega + frame;
    if (!std::isfinite(phi))
      continue;

    const auto consider = [&](const double terminal_heading) {
      State terminal = target;
      terminal.theta = terminal_heading;
      double outgoing = 0.0;
      std::uint32_t outgoing_word = 0;
      const double cutoff = std::min(incumbent_cutoff, result.distance);
      if (geometricLeg(pivot, terminal, outgoing, outgoing_word,
                        std::numeric_limits<double>::max(),
                        prefix.distance + incoming, cutoff) != LegStatus::Clear)
        return;
      const double distance = prefix.distance + incoming + outgoing;
      if (!(distance < result.distance && distance < incumbent_cutoff))
        return;
      result.distance = distance;
      result.theta = terminal.theta;
      result.pivot_theta = pivot.theta;
      result.incoming_distance = incoming;
      result.incoming_word = incoming_word;
      result.outgoing_word = outgoing_word;
    };
    consider(phi);
    consider(detail::boundedNearestAngleRemainder(2.0 * psi - phi + pi, 2.0 * pi));
  }
  if (result.distance < incumbent_cutoff)
    ++geometric_rescue_wins_;
  return result;
}

triplet Marching::geometricHeadingSweep(const int source, const int x, const int y,
                                       const double incumbent_cutoff) {
  constexpr int kSamples = 720;
  const PivotPrefix &prefix = pivot_prefixes_[source];
  const Point position = geometricPivotPosition(source);
  triplet best{std::numeric_limits<double>::max(),
               std::numeric_limits<double>::quiet_NaN(), source};
  ++geometric_sweeps_;
  for (int i = 0; i < kSamples; ++i) {
    State pivot{position.x, position.y, -pi + 2.0 * pi * i / kSamples};
    State target{static_cast<double>(x), static_cast<double>(y), 0.0};
    const double limit = std::min(best.distance, incumbent_cutoff);
    double incoming = 0.0, outgoing = 0.0;
    std::uint32_t incoming_word = 0, outgoing_word = 0;
    if (prefix.distance + std::hypot(pivot.x - prefix.preceding_pose.x,
                                     pivot.y - prefix.preceding_pose.y) +
            std::hypot(target.x - pivot.x, target.y - pivot.y) >= limit)
      continue;
    ++geometric_sweep_samples_;
    if (geometricLeg(prefix.preceding_pose, pivot, incoming, incoming_word,
                      limit - prefix.distance) != LegStatus::Clear ||
        geometricFreeHeadingLeg(pivot, target, outgoing, outgoing_word,
                                 std::numeric_limits<double>::max(),
                                 prefix.distance + incoming, limit, pivot.theta - pi / 2.0) != LegStatus::Clear ||
        !(prefix.distance + incoming + outgoing < limit))
      continue;
    best.distance = prefix.distance + incoming + outgoing;
    best.theta = target.theta;
    best.pivot_theta = pivot.theta;
    best.incoming_distance = incoming;
    best.incoming_word = incoming_word;
    best.outgoing_word = outgoing_word;
  }
  return best;
}

// This bounded fallback reuses a child's immutable preceding pose for targets
// within r. A Clear proposal may compete through the ordinary writer; the
// radius bound is a heuristic scope and does not establish propagation coverage.
triplet Marching::geometricPoseCandidate(const int source, const int x, const int y,
                                         const double incumbent_cutoff) {
  const PivotPrefix &prefix = pivot_prefixes_[source];
  const int parent = prefix.parent_source;
  if (std::hypot(x - prefix.preceding_pose.x, y - prefix.preceding_pose.y) > r_)
    return {std::numeric_limits<double>::max(), std::numeric_limits<double>::quiet_NaN(), parent};
  if (pivot_prefixes_[parent].parent_source < 0) {
    ++geometric_pose_attempts_;
    return geometricCandidate(parent, x, y, incumbent_cutoff);
  }
  triplet result{std::numeric_limits<double>::max(),
                 std::numeric_limits<double>::quiet_NaN(), parent};
  State from = prefix.preceding_pose;
  State target{static_cast<double>(x), static_cast<double>(y), 0.0};
  const double bound = prefix.distance + std::hypot(target.x - from.x, target.y - from.y);
  if (!(bound < incumbent_cutoff - 1e-10 * std::max({1.0, r_, bound})))
    return result;
  ++geometric_pose_attempts_;
  double outgoing = 0.0;
  std::uint32_t outgoing_word = 0;
  if (geometricFreeHeadingLeg(from, target, outgoing, outgoing_word,
                               std::numeric_limits<double>::max(), prefix.distance,
                               incumbent_cutoff, from.theta - pi / 2.0) != LegStatus::Clear)
    return result;
  result.distance = prefix.distance + outgoing;
  result.theta = target.theta;
  result.pivot_theta = from.theta;
  result.incoming_distance = prefix.distance - pivot_prefixes_[parent].distance;
  result.incoming_word = prefix.incoming_word;
  result.outgoing_word = outgoing_word;
  return result;
}

triplet Marching::geometricCandidate(
    const int source, const int x, const int y, const double incumbent_cutoff,
    const Point *const point_target, const int offer_x, const int offer_y) {
  triplet result{std::numeric_limits<double>::max(),
                 std::numeric_limits<double>::quiet_NaN(), source};
  if (source < 0 || static_cast<std::size_t>(source) >= pivot_prefixes_.size())
    return result;
  auto &prefix = pivot_prefixes_[source];
  const auto &light = lightSources_[source];
  const bool root = prefix.parent_source < 0;
  const Point position = geometricPivotPosition(source);
  State pivot{position.x, position.y, light.theta + pi / 2.0};
  // A point target has no cell record: no selector memo, no neighbour headings.
  State target = point_target != nullptr
      ? State{point_target->x, point_target->y, 0.0}
      : State{static_cast<double>(x), static_cast<double>(y), 0.0};

  // Heading selection and qualification preserve these physical positions.
  const double outgoing_chord = std::hypot(target.x - pivot.x, target.y - pivot.y);

  // Numerical chord filter; qualification remains unchanged. Its empirical
  // 1e-9 allowance can reject a tiny price advantage within replay allowance.
  if (std::isfinite(incumbent_cutoff) &&
      incumbent_cutoff < std::numeric_limits<double>::max() &&
      std::isfinite(prefix.distance) && prefix.distance >= 0.0 &&
      std::isfinite(r_) && r_ > 0.0) {
    double incoming_chord = 0.0;
    if (!root) {
      incoming_chord = std::hypot(prefix.preceding_pose.x - pivot.x,
                                  prefix.preceding_pose.y - pivot.y);
      if (std::isfinite(prefix.birth_incoming_distance) &&
          prefix.birth_incoming_distance >= 0.0)
        incoming_chord = std::min(incoming_chord, prefix.birth_incoming_distance);
    }
    const double bound = prefix.distance + incoming_chord +
        outgoing_chord;
    const double scale = std::max({1.0, r_, std::abs(bound),
        std::abs(incumbent_cutoff), std::abs(prefix.preceding_pose.x),
        std::abs(prefix.preceding_pose.y), std::abs(pivot.x), std::abs(pivot.y),
        std::abs(target.x), std::abs(target.y)});
    if (std::isfinite(incoming_chord) && std::isfinite(bound) &&
        std::isfinite(scale) &&
        bound > incumbent_cutoff + 1e-9 * scale) {
      ++geometric_incumbent_prunes_;
      return result;
    }
  }
  triplet retarget = result;
  double blocked_heading = std::numeric_limits<double>::quiet_NaN();
  double nearest_blocked = std::numeric_limits<double>::quiet_NaN();
  const auto contact = [&](triplet best) {
    geometricContactHeading(prefix, source, target, blocked_heading,
                            incumbent_cutoff, best, nearest_blocked);
    return best;
  };

  if (!root) {
    ++geometric_retargets_;
    SelectorMemoEntry *selected_memo = nullptr;
    const bool selected =
        selectGeometricTripletHeadings(prefix, source, point_target ? -1 : x,
                                       point_target ? -1 : y, pivot, target, &selected_memo);
    bool selected_retarget = false;
    if (selected) {
      double incoming = 0.0, outgoing = 0.0;
      std::uint32_t incoming_word = 0, outgoing_word = 0;
      double incoming_limit = std::numeric_limits<double>::max();
      if (std::isfinite(incumbent_cutoff) &&
          incumbent_cutoff < std::numeric_limits<double>::max() &&
          std::isfinite(prefix.distance) && prefix.distance >= 0.0 &&
          std::isfinite(r_) && r_ > 0.0) {
        const double chord = outgoing_chord;
        // Match the existing numerical chord filter's empirical guard. This
        // prices the selected incoming leg, without qualifying it or constructing
        // a speculative outgoing leg. Price refusal is not a clearance result.
        const double scale = std::max({1.0, r_, std::abs(prefix.distance),
            std::abs(incumbent_cutoff), chord, std::abs(prefix.preceding_pose.x),
            std::abs(prefix.preceding_pose.y), std::abs(pivot.x), std::abs(pivot.y),
            std::abs(target.x), std::abs(target.y)});
        const double limit = incumbent_cutoff - prefix.distance - chord + 1e-9 * scale;
        if (std::isfinite(limit))
          incoming_limit = limit;
      }
      const auto selected_outgoing = [&] {
        if (selected_memo != nullptr && selected_memo->outgoing_blocked) {
          ++geometric_selected_blocked_reuses_;
          return false;
        }
        const auto arc_lookups = geometric_clearance_.first_arc_cache_lookups;
        const auto blocked = geometric_statuses_[static_cast<std::size_t>(
            audit::ContinuousClearanceStatus::Blocked)];
        const bool clear = geometricFreeHeadingLeg(
            pivot, target, outgoing, outgoing_word,
            std::numeric_limits<double>::max(), prefix.distance + incoming,
            incumbent_cutoff) == LegStatus::Clear;
        // This selected leg has no owned certificate. Without a global arc
        // lookup its qualified refusal depends only on immutable geometry/map.
        // Price refusals and other unsuccessful outcomes are never retained.
        if (selected_memo != nullptr && !clear &&
            geometric_clearance_.first_arc_cache_lookups == arc_lookups &&
            geometric_statuses_[static_cast<std::size_t>(
                audit::ContinuousClearanceStatus::Blocked)] > blocked)
          selected_memo->outgoing_blocked = true;
        return clear;
      };
      if (geometricIncomingLeg(source, pivot.theta, incoming, incoming_word, incoming_limit) == LegStatus::Clear &&
          selected_outgoing()) {
        retarget.distance = prefix.distance + incoming + outgoing;
        retarget.theta = target.theta;
        retarget.pivot_theta = pivot.theta;
        retarget.incoming_distance = incoming;
        retarget.incoming_word = incoming_word;
        retarget.outgoing_word = outgoing_word;
        selected_retarget = true;
      }
    }
    // Same-source neighbour records provide bounded heading proposals with
    // immutable incoming geometry. Each outgoing leg is solved and qualified
    // for this target; selected samples only seed the contact heuristic.
    if (selected && !selected_retarget)
      blocked_heading = pivot.theta;
    if (selected && point_target == nullptr) {
      std::array<std::pair<double, int>, 8> seeds{};
      const std::size_t count = geometricHeadingSeeds(
          source, x, y, selected_retarget ? offer_x : -1, selected_retarget ? offer_y : -1,
          selected_retarget, pivot.theta, seeds);
      const double chord = outgoing_chord;
      for (std::size_t i = 0; i < count; ++i) {
        const int nx = x + neighbours_[seeds[i].second];
        const int ny = y + neighbours_[seeds[i].second + 1];
        const double heading = geometric_pivot_theta_(nx, ny);
        const double incoming = geometric_incoming_distance_(nx, ny);
        const double limit = std::min(retarget.distance, incumbent_cutoff);
        // Bounded donor-selection heuristic: subtract the free-space rotation
        // price from the selected outgoing cost. A fixed-terminal fallback may
        // exceed the free-heading optimum, so this estimate is not an admissible
        // lower bound and may omit a better donor. Only the chord is a bound.
        double outgoing_estimate = chord;
        if (retarget.distance < std::numeric_limits<double>::max())
          outgoing_estimate = std::max(outgoing_estimate,
              retarget.distance - prefix.distance - retarget.incoming_distance -
              r_ * std::abs(detail::boundedNearestAngleRemainder(heading - retarget.pivot_theta, 2.0 * pi)));
        if (prefix.distance + incoming + outgoing_estimate >=
            limit - 1e-10 * std::max({1.0, r_, prefix.distance + incoming + outgoing_estimate}))
          continue;
        ++geometric_inherited_heading_attempts_;
        ++geometric_inherited_incoming_reused_;
        State seed_pivot{position.x, position.y, heading};
        State seed_target{target.x, target.y, 0.0};
        double outgoing = 0.0;
        std::uint32_t outgoing_word = 0;
        if (geometricFreeHeadingLeg(seed_pivot, seed_target, outgoing, outgoing_word,
                                     std::numeric_limits<double>::max(),
                                     prefix.distance + incoming, limit, heading - pi / 2.0) != LegStatus::Clear ||
            !(prefix.distance + incoming + outgoing < retarget.distance)) {
          if (!selected_retarget)
            nearest_blocked = heading;  // last refused seed (blocked or not cheaper): a bracket hint only
          continue;
        }
        retarget.distance = prefix.distance + incoming + outgoing;
        retarget.theta = seed_target.theta;
        retarget.pivot_theta = heading;
        retarget.incoming_distance = incoming;
        retarget.incoming_word = geometric_incoming_word_(nx, ny);
        retarget.outgoing_word = outgoing_word;
        ++geometric_inherited_heading_wins_;
      }
    }
    pivot = {position.x, position.y, prefix.birth_theta};
    if (!std::isfinite(pivot.theta))
      return contact(retarget);
  }

  double incoming = 0.0;
  std::uint32_t incoming_word = 0;
  if (!root) {
    incoming = prefix.birth_incoming_distance;
    incoming_word = prefix.birth_incoming_word;
    if (!std::isfinite(incoming) || incoming < 0.0) {
      if (geometricLeg(prefix.preceding_pose, pivot, incoming, incoming_word,
                        std::numeric_limits<double>::max(),
                        std::numeric_limits<double>::infinity(),
                        std::numeric_limits<double>::infinity(),
                        &prefix.preceding_arcs) != LegStatus::Clear)
        return contact(retarget);
    }
  }
  const double bound =
      prefix.distance + incoming + outgoing_chord;
  if (retarget.distance < std::numeric_limits<double>::max()) {
    const double tolerance = 1e-10 * std::max(
        {1.0, r_, std::abs(bound), std::abs(retarget.distance)});
    if (bound > retarget.distance + tolerance)
      return contact(retarget);
  }
  double outgoing = 0.0;
  std::uint32_t outgoing_word = 0;
  double outgoing_limit = std::numeric_limits<double>::max();
  if (retarget.distance < std::numeric_limits<double>::max()) {
    outgoing_limit = retarget.distance - prefix.distance - incoming + 1e-10 * std::max(
        {1.0, r_, std::abs(retarget.distance), std::abs(prefix.distance), std::abs(incoming)});
  }
  if (geometricFreeHeadingLeg(pivot, target, outgoing, outgoing_word,
                               outgoing_limit, prefix.distance + incoming,
                               incumbent_cutoff, light.theta, &prefix.birth_arcs) != LegStatus::Clear)
    return contact(retarget);
  result.distance = prefix.distance + incoming + outgoing;
  result.theta = target.theta;
  result.pivot_theta = pivot.theta;
  result.incoming_distance = incoming;
  result.incoming_word = incoming_word;
  result.outgoing_word = outgoing_word;
  geometric_fixed_continuations_ += !root;
  return contact(result.distance < retarget.distance ? result : retarget);
}

void Marching::geometricCandidates(const std::vector<size_t> &sources,
                                    std::vector<triplet> &distances,
                                    const int from_x, const int from_y,
                                    const int x, const int y,
                                    const bool allow_rescue) {
  distances.clear();
  bool clear = false;
  double best = std::numeric_limits<double>::max();
  for (const auto source : sources) {
    if (source < pivot_prefixes_.size() && pivot_prefixes_[source].parent_source >= 0) {
      distances.push_back(geometricPoseCandidate(static_cast<int>(source), x, y, best));
      if (distances.back().distance < best) {
        best = distances.back().distance;
        ++geometric_pose_wins_;
      }
      clear |= distances.back().distance < std::numeric_limits<double>::max();
    }
    distances.push_back(geometricCandidate(source, x, y, best, nullptr, from_x, from_y));
    if (distances.back().distance < best)
      best = distances.back().distance;
    clear |= distances.back().distance < std::numeric_limits<double>::max();
  }
  if (clear)
    return;
  const auto appendAnalyticRescues = [&] {
    for (const auto source : sources) {
      if (source >= nb_of_sources_ || source >= pivot_prefixes_.size() ||
          pivot_prefixes_[source].parent_source < 0)
        continue;
      const auto rescue = geometricAnalyticRescueCandidate(source, x, y, best);
      distances.push_back(rescue);
      if (rescue.distance < best)
        best = rescue.distance;
    }
  };
  const int parent = cameFrom_(from_x, from_y);
  if (parent < 0 || nb_of_sources_ >= nx_ * ny_ ||
      !isUpdated_(from_x, from_y) || sharedVisibilityField_->get(from_x, from_y) < 1.0) {
    if (allow_rescue)
      appendAnalyticRescues();
    return;
  }
  ++geometric_birth_attempts_;
  if (lightSources_.size() <= nb_of_sources_)
    lightSources_.resize(nb_of_sources_ + 1);
  // Keep an owning value before appending provisional prefixes: vector growth
  // may otherwise invalidate the parent reference during the two trials.
  const PivotPrefix old = pivot_prefixes_[parent];
  PivotPrefix prefix = old;
  prefix.parent_source = parent;
  prefix.birth_x = x;
  prefix.birth_y = y;
  prefix.pivot_x = prefix.pivot_y = std::numeric_limits<double>::quiet_NaN();
  const Point parent_position = geometricPivotPosition(parent);
  if (old.parent_source >= 0) {
    prefix.preceding_pose = {parent_position.x, parent_position.y,
                            geometric_pivot_theta_(from_x, from_y)};
    prefix.distance += geometric_incoming_distance_(from_x, from_y);
    prefix.incoming_word = geometric_incoming_word_(from_x, from_y);
  }
  prefix.birth_theta = thetas_(from_x, from_y);
  prefix.birth_incoming_distance = gScore_(from_x, from_y) - prefix.distance;
  prefix.birth_incoming_word = geometric_outgoing_word_(from_x, from_y);
  // The prefix is a subpath of the actual accepted record at (from_x,from_y).
  // Only the new open head changes; clearance of that head is never inherited.
  const auto canBeatChildIncumbent = [&](const PivotPrefix &candidate,
                                          const double incumbent) {
    if (!(std::isfinite(incumbent) &&
          incumbent < std::numeric_limits<double>::max()))
      return true;
    const State birth{static_cast<double>(from_x), static_cast<double>(from_y),
                      candidate.birth_theta};
    const State target{static_cast<double>(x), static_cast<double>(y), 0.0};
    const double bound = candidate.distance +
        std::hypot(candidate.preceding_pose.x - birth.x,
                   candidate.preceding_pose.y - birth.y) +
        std::hypot(birth.x - target.x, birth.y - target.y);
    const double scale = std::max(
        {1.0, r_, std::abs(bound), std::abs(incumbent),
         std::abs(candidate.distance), std::abs(candidate.preceding_pose.x),
         std::abs(candidate.preceding_pose.y), std::abs(birth.x), std::abs(birth.y),
         std::abs(target.x), std::abs(target.y)});
    return !(std::isfinite(candidate.distance) && candidate.distance >= 0.0 &&
             std::isfinite(candidate.preceding_pose.x) &&
             std::isfinite(candidate.preceding_pose.y) && std::isfinite(bound) &&
             std::isfinite(scale) && bound > incumbent + 1e-9 * scale);
  };
  // Every trial child, like the committed one, stores the heading it was born with.
  const auto trial = [&](const PivotPrefix &candidate, const double limit, const bool sweep) {
    lightSources_[nb_of_sources_] = {from_x, from_y, candidate.birth_theta - pi / 2.0};
    appendPivotPrefix(candidate);
    const int id = static_cast<int>(nb_of_sources_);
    triplet result = geometricCandidate(id, x, y, limit);
    // Completion only: when ordinary bounded proposals find no Clear result,
    // sample the child's heading circle as a heuristic fallback.
    if (sweep && !(result.distance < std::numeric_limits<double>::max()))
      result = geometricHeadingSweep(id, x, y, limit);
    pivot_prefixes_.pop_back();
    return result;
  };

  triplet normal_child{std::numeric_limits<double>::max(),
                       std::numeric_limits<double>::quiet_NaN(),
                       static_cast<int>(nb_of_sources_)};
  if (canBeatChildIncumbent(prefix, best)) {
    normal_child = trial(prefix, best, allow_rescue);
  } else {
    ++geometric_child_bound_skips_;
  }

  PivotPrefix bypass = old;
  bypass.pivot_x = bypass.pivot_y = std::numeric_limits<double>::quiet_NaN();
  triplet bypass_child{std::numeric_limits<double>::max(),
                       std::numeric_limits<double>::quiet_NaN(),
                       static_cast<int>(nb_of_sources_)};
  if (old.parent_source >= 0) {
    bypass.birth_x = x;
    bypass.birth_y = y;
    bypass.birth_theta = thetas_(from_x, from_y);
    const double bypass_limit = std::min(best, normal_child.distance);
    const State birth{static_cast<double>(from_x), static_cast<double>(from_y),
                      bypass.birth_theta};
    // Compare qualified direct prefixes from ancestor poses. At the parent
    // level, also scan nearby birth headings and rank them by incoming length
    // plus the free-space rotation price r*|delta|. This is a proposal score:
    // that rotation is not collision-qualified or committed, so it proves no
    // constrained-path dominance. Sample the first clear half-degree heading
    // per side and refine its boundary; narrow or disconnected windows can be
    // missed. The eventual child trial freshly qualifies its actual two legs.
    // Higher ancestors retain the fixed-heading proposal to bound scan work.
    double bypass_total = std::numeric_limits<double>::infinity();
    const double record_limit = gScore_(from_x, from_y) +
        audit::kReplayTolerance * std::max({1.0, r_, gScore_(from_x, from_y)});
    const double step = pi / 360.0;
    std::uint64_t depth = 0;
    for (int k = parent; k >= 0 && pivot_prefixes_[k].parent_source >= 0;
         k = pivot_prefixes_[k].parent_source) {
      geometric_bypass_max_depth_ = std::max(geometric_bypass_max_depth_, ++depth);
      PivotPrefix candidate = pivot_prefixes_[k];
      candidate.birth_x = x;
      candidate.birth_y = y;
      candidate.birth_theta = birth.theta;
      candidate.pivot_x = candidate.pivot_y = std::numeric_limits<double>::quiet_NaN();
      if (!canBeatChildIncumbent(candidate, bypass_limit)) {
        ++geometric_child_bound_skips_;
        continue;
      }
      const double chord = std::hypot(candidate.preceding_pose.x - birth.x,
                                      candidate.preceding_pose.y - birth.y);
      if (candidate.distance + chord >= bypass_total)
        continue;
      ++geometric_bypass_attempts_;
      bool rotate = false;
      for (const double side : {1.0, -1.0}) {
        // j counts half degrees; the half turn is probed once, on the first side.
        for (int j = side > 0.0 ? 0 : 1; j <= 360 - (side < 0.0) && (j == 0 || rotate); ++j) {
          const double limit = std::min(bypass_total, record_limit) - candidate.distance -
                               r_ * j * step;
          if (!(chord < limit))
            break;
          State pose = birth;
          pose.theta += side * j * step;
          geometric_bypass_rotation_probes_ += j > 0;
          double incoming = 0.0;
          std::uint32_t incoming_word = 0;
          if (geometricLeg(candidate.preceding_pose, pose, incoming, incoming_word, limit) != LegStatus::Clear) {
            if (!(incoming <= limit))
              break;
            rotate |= j == 0 && k == parent;
            continue;
          }
          for (double blocked = pose.theta - side * step;
               j > 0 && std::abs(pose.theta - blocked) > 5e-6;) {
            ++geometric_bypass_rotation_probes_;
            State middle{birth.x, birth.y, 0.5 * (pose.theta + blocked)};
            double middle_incoming = 0.0;
            std::uint32_t middle_word = 0;
            if (geometricLeg(candidate.preceding_pose, middle, middle_incoming, middle_word) == LegStatus::Clear) {
              pose = middle;
              incoming = middle_incoming;
              incoming_word = middle_word;
            } else {
              blocked = middle.theta;
            }
          }
          const double bound =
              candidate.distance + incoming + r_ * std::abs(pose.theta - birth.theta);
          if (bound < bypass_total) {
            geometric_bypass_rotations_ += j > 0;
            candidate.birth_theta = pose.theta;
            candidate.birth_incoming_distance = incoming;
            candidate.birth_incoming_word = incoming_word;
            bypass = candidate;
            bypass_total = bound;
          }
          break;
        }
      }
    }
    if (bypass_total < std::numeric_limits<double>::infinity()) {
      bypass_child = trial(bypass, bypass_limit, false);
      if (bypass_child.distance < bypass_limit)
        ++geometric_bypass_wins_;
    }
  }

  const triplet *child = &normal_child;
  const PivotPrefix *child_prefix = &prefix;
  if (bypass_child.distance < normal_child.distance) {
    child = &bypass_child;
    child_prefix = &bypass;
  }

  // Corner children are finite convex-corner proposals from the birth cell,
  // displaced into free space by the shared replay tolerance. The displacement
  // is proposal geometry, not collision semantics, and the finite set does not
  // establish corner completeness. Repeated (ancestor, position) proposals
  // reuse the stored prefix.
  triplet vertex_child = normal_child;
  vertex_child.distance = std::numeric_limits<double>::max();
  PivotPrefix vertex_prefix;
  triplet reused{std::numeric_limits<double>::max(),
                 std::numeric_limits<double>::quiet_NaN(), -1};
  const auto shortest_reach = [&](const std::array<int, 2> ancestors, const Point point,
                                  int &ancestor) {
    triplet best_reach{std::numeric_limits<double>::max(),
                       std::numeric_limits<double>::quiet_NaN(), -1};
    ancestor = -1;
    for (const int candidate : ancestors) {
      if (candidate < 0)
        continue;
      const triplet reach = geometricCandidate(
          candidate, -1, -1, std::numeric_limits<double>::infinity(), &point);
      if (reach.distance < best_reach.distance) {
        best_reach = reach;
        ancestor = candidate;
      }
    }
    return best_reach;
  };
  const auto offer = [&](const std::array<int, 2> ancestors, const Point point) {
    int ancestor = -1;
    ++geometric_vertex_trials_;
    const triplet reach = shortest_reach(ancestors, point, ancestor);
    if (!(reach.distance < std::numeric_limits<double>::max()))
      return;
    const std::array<int, 3> key{ancestor, static_cast<int>(std::lround(2.0 * point.x)),
                                 static_cast<int>(std::lround(2.0 * point.y))};
    if (const auto found = geometric_vertex_children_.find(key);
        found != geometric_vertex_children_.end()) {
      ++geometric_vertex_reuses_;
      const triplet candidate = geometricCandidate(found->second, x, y, best);
      if (candidate.distance < reused.distance)
        reused = candidate;
      return;
    }
    PivotPrefix candidate_prefix = pivot_prefixes_[ancestor];
    const bool ancestor_root = candidate_prefix.parent_source < 0;
    candidate_prefix.parent_source = ancestor;
    candidate_prefix.birth_x = x;
    candidate_prefix.birth_y = y;
    if (!ancestor_root) {
      const Point ancestor_position = geometricPivotPosition(ancestor);
      candidate_prefix.preceding_pose = {ancestor_position.x, ancestor_position.y,
                                         reach.pivot_theta};
      candidate_prefix.distance += reach.incoming_distance;
      candidate_prefix.incoming_word = reach.incoming_word;
    }
    candidate_prefix.birth_theta = reach.theta;
    candidate_prefix.birth_incoming_distance = reach.distance - candidate_prefix.distance;
    candidate_prefix.birth_incoming_word = reach.outgoing_word;
    candidate_prefix.pivot_x = point.x;
    candidate_prefix.pivot_y = point.y;
    const triplet candidate = trial(candidate_prefix, best, false);
    if (candidate.distance < vertex_child.distance) {
      vertex_child = candidate;
      vertex_prefix = candidate_prefix;
    }
  };
  const int grandparent = old.parent_source;
  const double offset = audit::kReplayTolerance * std::max(1.0, r_);
  for (int gx = from_x; gx <= from_x + 1; ++gx) {
    for (int gy = from_y; gy <= from_y + 1; ++gy) {
      int occupied = 0, ox = 0, oy = 0;
      for (int cx = gx - 1; cx <= gx; ++cx)
        for (int cy = gy - 1; cy <= gy; ++cy)
          if (!isValid(cx, cy) || sharedVisibilityField_->get(cx, cy) < 1.0) {
            ++occupied;
            ox = cx;
            oy = cy;
          }
      if (occupied != 1)
        continue;
      const Point corner{gx - 0.5 + (gx - 0.5 > ox ? offset : -offset),
                         gy - 0.5 + (gy - 0.5 > oy ? offset : -offset)};
      offer({parent, grandparent}, corner);
    }
  }
  if (reused.distance < std::min({best, child->distance, vertex_child.distance})) {
    lightSources_[nb_of_sources_] = pose{};
    distances.push_back(reused);
    return;
  }
  const bool vertex_won = vertex_child.distance < child->distance;
  if (vertex_won) {
    child = &vertex_child;
    child_prefix = &vertex_prefix;
  }
  if (!(child->distance < best)) {
    // This slot is outside the published source range; never read its old value.
    lightSources_[nb_of_sources_] = pose{};
    if (allow_rescue)
      appendAnalyticRescues();
    return;
  }
  // A child is its parent, position and immutable prefix: an identical child
  // already born is offered again through the ordinary writer, not duplicated.
  const Point position = std::isfinite(child_prefix->pivot_x)
      ? Point{child_prefix->pivot_x, child_prefix->pivot_y}
      : Point{static_cast<double>(from_x), static_cast<double>(from_y)};
  const std::array<int, 3> key{child_prefix->parent_source,
                               static_cast<int>(std::lround(2.0 * position.x)),
                               static_cast<int>(std::lround(2.0 * position.y))};
  if (const auto found = geometric_vertex_children_.find(key);
      found != geometric_vertex_children_.end()) {
    const auto &existing = pivot_prefixes_[found->second];
    const Point existing_position = geometricPivotPosition(found->second);
    // Quantized keys select a bucket; reuse requires the literal waypoint.
    if (existing_position.x == position.x && existing_position.y == position.y &&
        existing.preceding_pose.x == child_prefix->preceding_pose.x &&
        existing.preceding_pose.y == child_prefix->preceding_pose.y &&
        existing.preceding_pose.theta == child_prefix->preceding_pose.theta &&
        existing.distance == child_prefix->distance &&
        existing.birth_theta == child_prefix->birth_theta) {
      ++geometric_vertex_reuses_;
      lightSources_[nb_of_sources_] = pose{};
      // Identical prefix and position: the evaluated triplet (possibly from the
      // completion sweep) is the existing child's own geometry.
      triplet reuse = *child;
      reuse.source = found->second;
      const triplet again = geometricCandidate(found->second, x, y, best);
      distances.push_back(again.distance < reuse.distance ? again : reuse);
      return;
    }
  }
  geometric_vertex_children_.insert_or_assign(key, static_cast<int>(nb_of_sources_));
  geometric_vertex_births_ += vertex_won;
  lightSources_[nb_of_sources_] = {from_x, from_y, child_prefix->birth_theta - pi / 2.0};
  appendPivotPrefix(*child_prefix);
  distances.push_back(*child);
  ++pivot_births_;
  ++nb_of_sources_;
}

bool Marching::geometricPrefixChain(
    int source, std::vector<State> &poses, std::vector<std::uint32_t> &words,
    std::vector<int> *source_ids, std::size_t *visits) const {
  poses.clear();
  words.clear();
  if (source_ids) source_ids->clear();
  std::size_t hops = 0;
  for (;;) {
    if (source < 0 || static_cast<std::size_t>(source) >= nb_of_sources_ ||
        static_cast<std::size_t>(source) >= pivot_prefixes_.size()) return false;
    const auto &prefix = pivot_prefixes_[source];
    if (prefix.parent_source < 0) break;
    if (++hops > nb_of_sources_) return false;
    if (visits) ++*visits;
    poses.push_back(prefix.preceding_pose);
    words.push_back(prefix.incoming_word);
    if (source_ids) source_ids->push_back(prefix.parent_source);
    source = prefix.parent_source;
  }
  std::reverse(poses.begin(), poses.end());
  std::reverse(words.begin(), words.end());
  if (source_ids) std::reverse(source_ids->begin(), source_ids->end());
  if (!words.empty()) words.erase(words.begin());
  return true;
}

bool Marching::geometricRecordChain(
    int x, int y, std::vector<State> &poses, std::vector<std::uint32_t> &words,
    std::vector<int> *source_ids, std::size_t *visits) const {
  const int source = cameFrom_(x, y);
  if (!geometricPrefixChain(source, poses, words, source_ids, visits)) return false;
  if (!poses.empty()) words.push_back(geometric_incoming_word_(x, y));
  words.push_back(geometric_outgoing_word_(x, y));
  const Point position = geometricPivotPosition(source);
  poses.push_back({position.x, position.y, pivot_prefixes_[source].parent_source < 0
      ? lightSources_[source].theta + pi / 2.0 : geometric_pivot_theta_(x, y)});
  poses.push_back({static_cast<double>(x), static_cast<double>(y), thetas_(x, y)});
  if (source_ids) {
    source_ids->push_back(source);
    source_ids->push_back(source);
  }
  return true;
}

void Marching::runGeometricAudit() {
  const char *name = std::getenv("RS_G1_AUDIT");
  if (name == nullptr || name[0] == '\0')
    return;
  const auto started = std::chrono::steady_clock::now();
  const std::filesystem::path path(name);
  if (!path.parent_path().empty())
    std::filesystem::create_directories(path.parent_path());
  std::ofstream records(path.string() + ".csv");
  records << "x,y,source,status,distance,theta,pivot_theta,incoming,segments,gear_changes\n"
          << std::setprecision(17);
  std::array<std::uint64_t, 6> statuses{};
  std::uint64_t free = 0, missing = 0, fidelity_failures = 0, word_failures = 0;
  std::map<unsigned, std::uint64_t> gear_histogram, material_gear_histogram;
  for (std::size_t y = 0; y < ny_; ++y) {
    for (std::size_t x = 0; x < nx_; ++x) {
      if (sharedVisibilityField_->get(x, y) < 1.0)
        continue;
      ++free;
      const int source = cameFrom_(x, y);
      if (!isUpdated_(x, y) || source < 0) {
        ++missing;
        continue;
      }
      std::vector<State> poses;
      std::vector<std::uint32_t> words;
      if (!geometricRecordChain(x, y, poses, words))
        throw std::logic_error("G1 invalid ancestry");
      const auto &light = lightSources_[source];
      audit::ResolvedChainAuditResult checked;
      if (poses.size() == 2 && static_cast<int>(x) == light.x &&
          static_cast<int>(y) == light.y && gScore_(x, y) == 0.0) {
        checked.status = audit::ResolvedChainStatus::Clear;
        checked.complete_distance_residual = 0.0;
      } else {
        checked = audit::auditResolvedChain(*sharedVisibilityField_, poses,
                                            gScore_(x, y), r_, &words);
        bool matched = checked.segments.size() == words.size();
        for (std::size_t i = 0; i < checked.segments.size() && matched; ++i)
          matched = audit::selectedPathWord(checked.segments[i].path) == words[i];
        if (!matched) {
          ++word_failures;
          checked.status = audit::ResolvedChainStatus::Incomplete;
        }
      }
      ++statuses[static_cast<std::size_t>(checked.status)];
      fidelity_failures += !(checked.complete_distance_residual <=
          audit::kReplayTolerance * std::max(r_, gScore_(x, y)));
      int previous_gear = 0, previous_material = 0;
      unsigned gear_changes = 0, material_changes = 0;
      for (const auto &segment : checked.segments) {
        for (std::size_t i = 0; i < segment.path.primitive_count; ++i) {
          const auto &primitive = segment.path.primitives[i];
          if (primitive.length == 0.0) continue;
          gear_changes += previous_gear != 0 && previous_gear != primitive.direction;
          previous_gear = primitive.direction;
          // Material cusps ignore primitives within the shared replay tolerance.
          if (primitive.length > audit::kReplayTolerance * std::max(1.0, r_)) {
            material_changes += previous_material != 0 && previous_material != primitive.direction;
            previous_material = primitive.direction;
          }
        }
      }
      if (checked.certified()) {
        ++gear_histogram[gear_changes];
        ++material_gear_histogram[material_changes];
      }
      records << x << ',' << y << ',' << source << ',' << static_cast<int>(checked.status)
              << ',' << gScore_(x, y) << ',' << thetas_(x, y) << ','
              << geometric_pivot_theta_(x, y) << ',' << geometric_incoming_distance_(x, y)
              << ',' << checked.segments.size() << ',' << gear_changes << '\n';
    }
  }
  const auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now() - started).count();
  nlohmann::json prefixes = nlohmann::json::array();
  for (std::size_t source = 0; source < nb_of_sources_; ++source) {
    const auto &prefix = pivot_prefixes_[source];
    prefixes.push_back({{"source", source}, {"x", lightSources_[source].x},
                       {"y", lightSources_[source].y}, {"parent", prefix.parent_source},
                       {"birth_target", {prefix.birth_x, prefix.birth_y}},
                       {"preceding_pose", {prefix.preceding_pose.x, prefix.preceding_pose.y,
                                            prefix.preceding_pose.theta}},
                       {"distance", prefix.distance}, {"birth_theta", prefix.birth_theta},
                       {"birth_incoming_distance", prefix.birth_incoming_distance},
                       {"birth_incoming_word", prefix.birth_incoming_word},
                       {"pivot_position", {geometricPivotPosition(source).x,
                                           geometricPivotPosition(source).y}}});
  }
  const nlohmann::json summary{
      {"schema", "g1-experimental-record-audit-v1"}, {"free", free},
      {"missing", missing}, {"statuses", statuses}, {"fidelity_failures", fidelity_failures},
      {"word_failures", word_failures},
      {"pivots", pivot_births_},
      {"solves", geometric_solves_}, {"retargets", geometric_retargets_},
      {"fixed_continuations", geometric_fixed_continuations_},
      {"cost_prunes", geometric_cost_prunes_},
      {"incumbent_prunes", geometric_incumbent_prunes_},
      {"reoffers", geometric_reoffers_}, {"improvements", geometric_improvements_},
      {"stale_pops", geometric_stale_pops_},
      {"incoming_cache_lookups", geometric_incoming_cache_lookups_},
      {"incoming_cache_hits", geometric_incoming_cache_hits_},
      {"inherited_heading_attempts", geometric_inherited_heading_attempts_},
      {"inherited_heading_wins", geometric_inherited_heading_wins_},
      {"inherited_incoming_reused", geometric_inherited_incoming_reused_},
      {"vertex_trials", geometric_vertex_trials_}, {"vertex_births", geometric_vertex_births_},
      {"vertex_reuses", geometric_vertex_reuses_},
      {"bypass_attempts", geometric_bypass_attempts_},
      {"bypass_wins", geometric_bypass_wins_},
      {"bypass_rotation_probes", geometric_bypass_rotation_probes_},
      {"bypass_rotations", geometric_bypass_rotations_},
      {"child_bound_skips", geometric_child_bound_skips_},
      {"analytic_rescue_attempts", geometric_rescue_attempts_},
      {"analytic_rescue_wins", geometric_rescue_wins_},
      {"deferred_failed_frontier_count", geometric_failed_frontier_count_},
      {"deferred_failed_frontier_capacity_bytes", geometric_failed_frontier_capacity_bytes_},
      {"deferred_completion_unique_seeds", geometric_completion_unique_seeds_},
      {"deferred_completion_enqueued_seeds", geometric_completion_enqueued_seeds_},
      {"deferred_completion_pops", geometric_completion_pops_},
      {"deferred_completion_writes", geometric_completion_writes_},
      {"ordinary_sources", geometric_ordinary_sources_},
      {"bypass_max_depth", geometric_bypass_max_depth_},
      {"completion_sweeps", geometric_sweeps_}, {"completion_sweep_samples", geometric_sweep_samples_},
      {"contact_searches", geometric_contact_searches_}, {"contact_steps", geometric_contact_steps_},
      {"contact_wins", geometric_contact_wins_},
      {"pose_attempts", geometric_pose_attempts_}, {"pose_wins", geometric_pose_wins_},
      {"vertex_children", geometric_vertex_children_.size()},
      {"birth_attempts", geometric_birth_attempts_}, {"leg_statuses", geometric_statuses_},
      {"filtered", geometric_clearance_.filtered}, {"exact_checks", geometric_clearance_.exact_checks},
      {"first_arc_cache_lookups", geometric_clearance_.first_arc_cache_lookups},
      {"first_arc_cache_hits", geometric_clearance_.first_arc_cache_hits},
      {"prefixes", prefixes}, {"gear_histogram", gear_histogram},
      {"material_gear_histogram", material_gear_histogram}, {"audit_us", elapsed}};
  std::ofstream output(path);
  output << summary.dump(2) << '\n';
  std::cout << "G1 audit: " << summary.dump() << '\n';
}

} // namespace RS_marching
