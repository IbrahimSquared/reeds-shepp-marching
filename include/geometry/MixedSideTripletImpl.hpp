#pragma once

namespace RS_marching::geometry {
namespace {

[[nodiscard]] bool mixedTerminalHeading(
    const State pivot, const TripletPoint target, const double radius,
    State &selected_target) {
  double terminal = std::numeric_limits<double>::quiet_NaN();
  UnderSpecifiedRSPlanner urs;
  urs.getOmega(pivot.x, pivot.y, target.x, target.y, radius, pivot.theta,
               terminal, true);
  selected_target = {target.x, target.y,
                     terminal + pivot.theta - kPi / 2.0};
  return std::isfinite(selected_target.theta);
}

[[nodiscard]] bool mixedCondition14BoundaryHeading(
    const State pivot, const TripletPoint target, const double radius,
    Solver &solver, State &selected_target, double &distance) {
  const UnderSpecifiedRSPlanner urs;
  const IntersectionHeadingCandidates headings = urs.getIntersectionHeadings(
      pivot.x, pivot.y, target.x, target.y, radius, pivot.theta);
  if (headings.count == 0) {
    return false;
  }
  const auto terminal_arc_collapsed = [&](const double target_heading) {
    const float source_heading = static_cast<float>(pivot.theta);
    const float terminal_heading = static_cast<float>(target_heading);
    const double dx = target.x - pivot.x;
    const double dy = target.y - pivot.y;
    const double local_x =
        (dx * std::cos(source_heading) + dy * std::sin(source_heading)) /
        radius;
    double local_y =
        (-dx * std::sin(source_heading) + dy * std::cos(source_heading)) /
        radius;
    if (local_x <= 0.0 || local_y > 0.0) {
      return false;
    }
    local_y = -local_y;
    const double local_heading =
        wrapToPi(2.0 * kPi - (terminal_heading - source_heading));
    const double xi = -local_x - std::sin(local_heading);
    const double eta = -local_y - 1.0 + std::cos(local_heading);
    const double ratio = 0.25 * std::hypot(xi, eta);
    if (ratio > 1.0) {
      return false;
    }
    const double middle = -2.0 * std::asin(ratio);
    const double first =
        wrapToPi(std::atan2(eta, xi) + 0.5 * middle + kPi);
    return std::abs(wrapToPi(local_heading - first + middle)) <=
           64.0 * std::numeric_limits<float>::epsilon();
  };

  bool found = false;
  std::size_t best_ordinal = std::numeric_limits<std::size_t>::max();
  for (std::size_t index = 0; index < headings.count; ++index) {
    const State candidate{target.x, target.y, headings.values[index]};
    double candidate_distance = std::numeric_limits<double>::infinity();
    int condition = 0;
    int quadrant = 0;
    solver.acceleratedRSPlanner(pivot, candidate, candidate_distance, radius,
                                condition, quadrant);
    if (condition != 14 || quadrant != 4 ||
        !validDistance(candidate_distance) ||
        !terminal_arc_collapsed(candidate.theta)) {
      continue;
    }
    const double tolerance =
        64.0 * std::numeric_limits<float>::epsilon() *
        std::max({radius, distance, candidate_distance});
    if (!found || candidate_distance < distance - tolerance ||
        (std::abs(candidate_distance - distance) <= tolerance &&
         index < best_ordinal)) {
      found = true;
      distance = candidate_distance;
      best_ordinal = index;
      selected_target = candidate;
    }
  }
  return found;
}

[[nodiscard]] double mixedCollapseTolerance(const double radial) {
  return 128.0 * std::numeric_limits<double>::epsilon() *
         std::max(1.0, std::abs(radial));
}

[[nodiscard, gnu::noinline]] bool selectMixedIncomingCollapse(
    const State parent, const TripletPoint pivot, const TripletPoint target,
    const double radius, State &selected_pivot, State &selected_target) {
  const double cosine = std::cos(parent.theta);
  const double sine = std::sin(parent.theta);
  const double dx = pivot.x - parent.x;
  const double dy = pivot.y - parent.y;
  const double local_x = (cosine * dx + sine * dy) / radius;
  const double local_y = (-sine * dx + cosine * dy) / radius;
  const double coefficient_x = local_y + 1.0;
  const double coefficient_y = -local_x;
  const double radial = coefficient_x * coefficient_x +
                        coefficient_y * coefficient_y;
  if (std::abs(radial - 1.0) > mixedCollapseTolerance(radial)) {
    return false;
  }

  const double heading =
      parent.theta + std::atan2(coefficient_y, coefficient_x);
  const State collapse_pivot{pivot.x, pivot.y, heading};
  if (target.x == pivot.x && target.y == pivot.y) {
    selected_pivot = collapse_pivot;
    selected_target = collapse_pivot;
    return true;
  }
  const double center_x = pivot.x - radius * std::sin(heading);
  const double center_y = pivot.y + radius * std::cos(heading);
  if (std::hypot(target.x - center_x, target.y - center_y) < radius) {
    return false;
  }
  State collapse_target;
  if (!mixedTerminalHeading(collapse_pivot, target, radius, collapse_target)) {
    return false;
  }
  selected_pivot = collapse_pivot;
  selected_target = collapse_target;
  return true;
}

[[nodiscard, gnu::noinline]] bool selectMixedTargetContact(
    const State parent, const TripletPoint pivot, const TripletPoint target,
    const double radius, Solver &solver, State &selected_pivot,
    State &selected_target, std::array<double, 2> &selected_segments) {
  const double dx = target.x - pivot.x;
  const double dy = target.y - pivot.y;
  const double rho = std::hypot(dx, dy);
  if (rho == 0.0 || rho > 2.0 * radius) {
    return false;
  }
  const double beta = std::atan2(dy, dx);
  const double delta = std::asin(rho / (2.0 * radius));
  const std::array<double, 2> roots{beta - delta, beta + kPi + delta};
  struct Candidate {
    State pivot;
    State target;
    std::array<double, 2> segments{};
    double lower_bound = std::numeric_limits<double>::infinity();
    unsigned ordinal = 0;
    bool valid = false;
  };
  std::array<Candidate, 3> candidates{};
  candidates[0] = {selected_pivot, selected_target,
                   {},
                   std::numeric_limits<double>::infinity(), 0U,
                   std::isfinite(selected_target.theta)};
  if (candidates[0].valid) {
    candidates[0].lower_bound = tripletLowerBound(
        parent, selected_pivot, selected_target, radius);
  }
  for (std::size_t index = 0; index < roots.size(); ++index) {
    Candidate &candidate = candidates[index + 1U];
    candidate.pivot = {pivot.x, pivot.y, roots[index]};
    candidate.ordinal = static_cast<unsigned>(index + 1U);
    candidate.valid = mixedTerminalHeading(candidate.pivot, target, radius,
                                           candidate.target);
    if (candidate.valid) {
      candidate.lower_bound = tripletLowerBound(
          parent, candidate.pivot, candidate.target, radius);
    }
  }
  const auto out_of_order = [](const Candidate &lhs, const Candidate &rhs) {
    return lhs.lower_bound > rhs.lower_bound ||
           (lhs.lower_bound == rhs.lower_bound &&
            lhs.ordinal > rhs.ordinal);
  };
  if (out_of_order(candidates[0], candidates[1])) {
    std::swap(candidates[0], candidates[1]);
  }
  if (out_of_order(candidates[1], candidates[2])) {
    std::swap(candidates[1], candidates[2]);
  }
  if (out_of_order(candidates[0], candidates[1])) {
    std::swap(candidates[0], candidates[1]);
  }

  double best_distance = std::numeric_limits<double>::infinity();
  State best_pivot = selected_pivot;
  State best_target = selected_target;
  unsigned best_ordinal = std::numeric_limits<unsigned>::max();
  bool found = false;
  for (Candidate &candidate : candidates) {
    if (!candidate.valid) {
      continue;
    }
    double distance = std::numeric_limits<double>::infinity();
    const double pruning_tolerance =
        64.0 * std::numeric_limits<float>::epsilon() *
        std::max(radius, best_distance);
    if (found && candidate.lower_bound > best_distance + pruning_tolerance) {
      continue;
    }
    if (!tripletDistance(parent, candidate.pivot, candidate.target, radius,
                         solver, distance, &candidate.segments)) {
      continue;
    }
    const double selection_tolerance =
        64.0 * std::numeric_limits<float>::epsilon() *
        std::max({radius, best_distance, distance});
    if (!found || distance < best_distance - selection_tolerance ||
        (std::abs(distance - best_distance) <= selection_tolerance &&
         candidate.ordinal < best_ordinal)) {
      found = true;
      best_distance = distance;
      best_ordinal = candidate.ordinal;
      best_pivot = candidate.pivot;
      best_target = candidate.target;
      selected_segments = candidate.segments;
    }
  }
  if (found) {
    selected_pivot = best_pivot;
    selected_target = best_target;
  }
  return found;
}

[[nodiscard, gnu::noinline]] bool selectMixedIncomingMinimum(
    const State parent, const TripletPoint pivot, const TripletPoint target,
    const double radius, Solver &solver, State &selected_pivot,
    State &selected_target, std::array<double, 2> &selected_segments) {
  State boundary_pivot;
  State boundary_target;
  double incoming_distance = std::numeric_limits<double>::infinity();
  double outgoing_distance = std::numeric_limits<double>::infinity();
  if (!mixedCondition14BoundaryHeading(parent, pivot, radius, solver,
                                       boundary_pivot, incoming_distance) ||
      !mixedCondition14BoundaryHeading(boundary_pivot, target, radius, solver,
                                       boundary_target, outgoing_distance)) {
    return std::isfinite(selected_target.theta);
  }
  bool selected_valid = validDistance(selected_segments[0]) &&
                        validDistance(selected_segments[1]);
  double selected_distance = selected_segments[0] + selected_segments[1];
  if (!selected_valid) {
    selected_valid = tripletDistance(parent, selected_pivot, selected_target,
                                     radius, solver, selected_distance,
                                     &selected_segments);
  }
  const double boundary_distance = incoming_distance + outgoing_distance;
  const double tolerance =
      64.0 * std::numeric_limits<float>::epsilon() *
      std::max({radius, selected_distance, boundary_distance});
  if (!validDistance(boundary_distance) ||
      (selected_valid &&
       boundary_distance >= selected_distance - tolerance)) {
    return selected_valid;
  }
  selected_pivot = boundary_pivot;
  selected_target = boundary_target;
  selected_segments = {incoming_distance, outgoing_distance};
  return true;
}

} // namespace

[[gnu::always_inline]] inline MixedSideTripletStatus
selectMixedSideTripletChartHeadings(
    const State parent, const TripletPoint pivot, const TripletPoint target,
    const TripletPoint physical_chart_pivot,
    const TripletPoint physical_chart_target,
    const double physical_chart_seed_heading, const double radius,
    const SameSideTransform transform, Solver &solver, const bool select_target,
    State &selected_pivot, State &selected_target,
    std::array<double, 2> *const selected_segments_out) {
  if (selected_segments_out != nullptr) {
    selected_segments_out->fill(std::numeric_limits<double>::infinity());
  }
  selected_pivot = {pivot.x, pivot.y,
                    std::numeric_limits<double>::quiet_NaN()};
  selected_target = {target.x, target.y,
                     std::numeric_limits<double>::quiet_NaN()};
  std::array<double, 2> selected_segments{};
  selected_segments.fill(std::numeric_limits<double>::infinity());
  if (!std::isfinite(parent.x) || !std::isfinite(parent.y) ||
      !std::isfinite(parent.theta) || !std::isfinite(pivot.x) ||
      !std::isfinite(pivot.y) || !std::isfinite(target.x) ||
      !std::isfinite(target.y) || !std::isfinite(physical_chart_pivot.x) ||
      !std::isfinite(physical_chart_pivot.y) ||
      !std::isfinite(physical_chart_target.x) ||
      !std::isfinite(physical_chart_target.y) ||
      !std::isfinite(physical_chart_seed_heading) || !std::isfinite(radius) ||
      radius <= 0.0 || static_cast<unsigned>(transform) > 3U) {
    return MixedSideTripletStatus::InvalidInput;
  }

  const State canonical_parent = pullbackParent(parent, transform);
  const TripletPoint canonical_pivot_point = pullbackPoint(pivot, transform);
  const TripletPoint canonical_target = pullbackPoint(target, transform);
  const TripletPoint chart_pivot =
      pullbackChartPoint(parent, physical_chart_pivot, transform);
  const TripletPoint chart_target =
      pullbackChartPoint(parent, physical_chart_target, transform);

  double theta = wrapTo2Pi(reflected(transform)
                               ? kPi - physical_chart_seed_heading
                               : physical_chart_seed_heading);
  const double center_x = chart_pivot.x - radius * std::sin(theta);
  const double center_y = chart_pivot.y + radius * std::cos(theta);
  const double target_center_distance =
      std::hypot(chart_target.x - center_x, chart_target.y - center_y);
  const double parent_center_distance = std::hypot(
      center_x - (canonical_parent.x + radius),
      center_y - canonical_parent.y);
  const double geometry_tolerance =
      kEpsilon * std::max({1.0, radius, target_center_distance,
                           parent_center_distance});
  if (target_center_distance <= geometry_tolerance ||
      parent_center_distance <= geometry_tolerance) {
    return MixedSideTripletStatus::GeometryFailure;
  }
  const double target_angle = wrapToPi(
      std::atan2(chart_target.y - center_y, chart_target.x - center_x));
  const double target_tangent =
      target_center_distance <= radius
          ? kPi / 2.0
          : std::asin(clampUnit(radius / target_center_distance));
  const double internal_tangent =
      parent_center_distance <= 2.0 * radius
          ? kPi / 2.0
          : std::acos(clampUnit(std::sqrt(
                                   parent_center_distance *
                                       parent_center_distance -
                                   4.0 * radius * radius) /
                               parent_center_distance));
  const double parent_angle = wrapTo2Pi(std::atan2(
      center_y - canonical_parent.y,
      center_x - (canonical_parent.x + radius)));
  theta = 0.5 * target_angle + 0.5 * target_tangent -
          0.5 * internal_tangent + 0.5 * parent_angle;

  State canonical_pivot{canonical_pivot_point.x, canonical_pivot_point.y,
                        theta + canonical_parent.theta - kPi / 2.0};
  State canonical_selected_target{canonical_target.x, canonical_target.y,
                                  std::numeric_limits<double>::quiet_NaN()};
  const double target_dx = chart_target.x - chart_pivot.x;
  const double target_dy = chart_target.y - chart_pivot.y;
  const double target_distance2 =
      target_dx * target_dx + target_dy * target_dy;
  const double parent_dx = chart_pivot.x - (canonical_parent.x + radius);
  const double parent_dy = chart_pivot.y - canonical_parent.y;
  const double parent_pivot_distance2 =
      parent_dx * parent_dx + parent_dy * parent_dy;
  const double radius2 = radius * radius;
  const bool incoming_collapse =
      std::abs(parent_pivot_distance2 - radius2) <=
      128.0 * std::numeric_limits<double>::epsilon() *
          std::max(parent_pivot_distance2, radius2);
  const bool collapse_selected =
      incoming_collapse && selectMixedIncomingCollapse(
                               canonical_parent, canonical_pivot_point,
                               canonical_target, radius, canonical_pivot,
                               canonical_selected_target);
  if (!collapse_selected) {
    if (target_distance2 == 0.0) {
      canonical_selected_target.theta = canonical_pivot.theta;
    } else if (!mixedTerminalHeading(canonical_pivot, canonical_target, radius,
                                     canonical_selected_target)) {
      return MixedSideTripletStatus::UrsFailure;
    } else if (parent_pivot_distance2 < radius2 &&
               target_distance2 <= 4.0 * radius2 &&
               !selectMixedTargetContact(canonical_parent,
                                         canonical_pivot_point,
                                         canonical_target, radius, solver,
                                         canonical_pivot,
                                         canonical_selected_target,
                                         selected_segments)) {
      return MixedSideTripletStatus::DistanceFailure;
    }
  }

  if (select_target && parent_pivot_distance2 < radius2 &&
      !selectMixedIncomingMinimum(canonical_parent, canonical_pivot_point,
                                  canonical_target, radius, solver,
                                  canonical_pivot,
                                  canonical_selected_target,
                                  selected_segments)) {
    return MixedSideTripletStatus::DistanceFailure;
  }
  selected_pivot.theta = applyHeading(canonical_pivot.theta, transform);
  if (!select_target) {
    return MixedSideTripletStatus::PivotSelected;
  }
  selected_target.theta =
      applyHeading(canonical_selected_target.theta, transform);
  const double selected_distance = selected_segments[0] + selected_segments[1];
  if (selected_segments_out != nullptr && validDistance(selected_segments[0]) &&
      validDistance(selected_segments[1]) && validDistance(selected_distance)) {
    *selected_segments_out = selected_segments;
  }
  return MixedSideTripletStatus::HeadingsSelected;
}

[[gnu::always_inline]] inline MixedSideTripletStatus
selectMixedSideTripletHeadings(
    const State parent, const TripletPoint pivot, const TripletPoint target,
    const double radius, const SameSideTransform transform, Solver &solver,
    State &selected_pivot, State &selected_target,
    std::array<double, 2> *const selected_segments) {
  const TripletPoint chart_pivot = toQuarterTurnChart(parent, pivot);
  const TripletPoint chart_target = toQuarterTurnChart(parent, target);
  const double chart_seed =
      0.5 * wrapTo2Pi(std::atan2(chart_target.y - chart_pivot.y,
                                 chart_target.x - chart_pivot.x) +
                      std::atan2(chart_pivot.y - parent.y,
                                 chart_pivot.x - parent.x));
  return selectMixedSideTripletChartHeadings(
      parent, pivot, target, chart_pivot, chart_target, chart_seed, radius,
      transform, solver, true, selected_pivot, selected_target,
      selected_segments);
}

inline MixedSideTripletResult solveMixedSideTriplet(
    const State parent, const TripletPoint pivot, const TripletPoint target,
    const double radius, const SameSideTransform transform, Solver &solver) {
  MixedSideTripletResult result;
  std::array<double, 2> selected_segments{};
  result.status = selectMixedSideTripletHeadings(
      parent, pivot, target, radius, transform, solver, result.pivot,
      result.target, &selected_segments);
  if (result.status != MixedSideTripletStatus::HeadingsSelected) {
    return result;
  }
  result.incoming_distance = selected_segments[0];
  result.outgoing_distance = selected_segments[1];
  if (!validDistance(result.incoming_distance) ||
      !validDistance(result.outgoing_distance)) {
    int condition = 0;
    int quadrant = 0;
    solver.acceleratedRSPlanner(parent, result.pivot, result.incoming_distance,
                                radius, condition, quadrant);
    solver.acceleratedRSPlanner(result.pivot, result.target,
                                result.outgoing_distance, radius, condition,
                                quadrant);
  }
  result.distance = result.incoming_distance + result.outgoing_distance;
  if (!validDistance(result.incoming_distance) ||
      !validDistance(result.outgoing_distance) ||
      !validDistance(result.distance)) {
    result.status = MixedSideTripletStatus::DistanceFailure;
    return result;
  }
  result.status = MixedSideTripletStatus::Selected;
  return result;
}

} // namespace RS_marching::geometry
