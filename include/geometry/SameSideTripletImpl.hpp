#pragma once

#include "underspecified/UnderSpecifiedRSPlanner.hpp"
#include "misc/AngleRemainder.hpp"

#include <algorithm>
#include <cmath>

namespace RS_marching::geometry {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kEpsilon = 1e-9;

[[nodiscard]] double wrapToPi(double theta) {
  theta = detail::boundedAngleRemainder(theta + kPi, 2.0 * kPi);
  if (theta < 0.0) {
    theta += 2.0 * kPi;
  }
  return theta - kPi;
}

[[nodiscard]] double wrapTo2Pi(double theta) {
  theta = detail::boundedAngleRemainder(theta, 2.0 * kPi);
  return theta < 0.0 ? theta + 2.0 * kPi : theta;
}

[[nodiscard]] double clampUnit(const double value) {
  return std::clamp(value, -1.0, 1.0);
}

[[nodiscard]] bool reflected(const SameSideTransform transform) {
  return (static_cast<unsigned>(transform) & 1U) != 0U;
}

[[nodiscard]] bool headingGearDual(const SameSideTransform transform) {
  return (static_cast<unsigned>(transform) & 2U) != 0U;
}

[[nodiscard]] TripletPoint pullbackPoint(TripletPoint point,
                                         const SameSideTransform transform) {
  if (reflected(transform)) {
    point.y = -point.y;
  }
  return point;
}

[[nodiscard]] State pullbackParent(State parent,
                                   const SameSideTransform transform) {
  const TripletPoint point = pullbackPoint({parent.x, parent.y}, transform);
  parent.x = point.x;
  parent.y = point.y;
  if (reflected(transform)) {
    parent.theta = -parent.theta;
  }
  if (headingGearDual(transform)) {
    parent.theta -= kPi;
  }
  return parent;
}

[[nodiscard]] double applyHeading(double theta,
                                  const SameSideTransform transform) {
  if (headingGearDual(transform)) {
    theta += kPi;
  }
  if (reflected(transform)) {
    theta = -theta;
  }
  return wrapToPi(theta);
}

[[nodiscard]] TripletPoint toQuarterTurnChart(const State parent,
                                              const TripletPoint point) {
  const double angle = parent.theta - kPi / 2.0;
  const double dx = point.x - parent.x;
  const double dy = point.y - parent.y;
  return {parent.x + std::cos(angle) * dx + std::sin(angle) * dy,
          parent.y - std::sin(angle) * dx + std::cos(angle) * dy};
}

[[nodiscard]] TripletPoint
pullbackChartPoint(const State parent, const TripletPoint point,
                   const SameSideTransform transform) {
  const bool mirror = reflected(transform);
  const bool dual = headingGearDual(transform);
  return {parent.x + (mirror != dual ? -1.0 : 1.0) *
                         (point.x - parent.x),
          (mirror ? -parent.y : parent.y) +
              (dual ? -1.0 : 1.0) * (point.y - parent.y)};
}

void pivotCircle(const TripletPoint pivot, const double radius,
                 const double theta, double &center_x, double &center_y) {
  center_x = pivot.x - radius * std::sin(theta);
  center_y = pivot.y + radius * std::cos(theta);
}

[[nodiscard]] bool seedTheta(const double parent_center_x,
                             const double parent_center_y,
                             const TripletPoint pivot,
                             const TripletPoint target, const double radius,
                             const double theta_guess, double &theta_out,
                             double *alternate_out) {
  if (alternate_out != nullptr) {
    *alternate_out = std::numeric_limits<double>::quiet_NaN();
  }
  double center_x = 0.0;
  double center_y = 0.0;
  pivotCircle(pivot, radius, theta_guess, center_x, center_y);
  const double h = std::hypot(target.x - center_x, target.y - center_y);
  if (h <= kEpsilon) {
    return false;
  }
  const double target_phase =
      std::atan2(target.y - center_y, target.x - center_x);
  const double tangent_phase = std::asin(clampUnit(radius / h));
  const double parent_phase =
      std::atan2(center_y - parent_center_y, center_x - parent_center_x);
  theta_out = 0.5 * wrapTo2Pi(target_phase) + 0.5 * tangent_phase +
              0.5 * wrapTo2Pi(parent_phase);
  if (alternate_out != nullptr) {
    const double half_phase =
        0.5 * (target_phase + tangent_phase + parent_phase);
    const double phase_theta =
        theta_guess - detail::boundedNearestAngleRemainder(
                          theta_guess - half_phase, kPi);
    if (std::abs(wrapToPi(phase_theta - theta_out)) > kPi / 2.0) {
      *alternate_out = phase_theta;
    }
  }
  return std::isfinite(theta_out);
}

[[nodiscard]] bool objective(const double parent_center_x,
                             const double parent_center_y,
                             const TripletPoint pivot,
                             const TripletPoint target, const double radius,
                             const double theta, double &value_out) {
  double center_x = 0.0;
  double center_y = 0.0;
  pivotCircle(pivot, radius, theta, center_x, center_y);
  const double incoming =
      std::hypot(center_x - parent_center_x, center_y - parent_center_y);
  const double h2 = (target.x - center_x) * (target.x - center_x) +
                    (target.y - center_y) * (target.y - center_y);
  const double radius2 = radius * radius;
  if (incoming <= kEpsilon || h2 <= radius2 + kEpsilon) {
    return false;
  }
  const double h = std::sqrt(h2);
  const double tail = std::sqrt(h2 - radius2);
  const double beta = std::asin(clampUnit(radius / h));
  const double alpha1 = wrapTo2Pi(
      std::atan2(center_y - parent_center_y, center_x - parent_center_x));
  const double alpha3 =
      wrapTo2Pi(std::atan2(target.y - center_y, target.x - center_x));
  value_out = incoming + tail + radius * (alpha1 + beta + alpha3);
  return std::isfinite(value_out);
}

[[nodiscard]] bool gradient(const double parent_center_x,
                            const double parent_center_y,
                            const TripletPoint pivot,
                            const TripletPoint target, const double radius,
                            const double theta, double &gradient_out,
                            double *derivative_out = nullptr) {
  double center_x = 0.0;
  double center_y = 0.0;
  pivotCircle(pivot, radius, theta, center_x, center_y);
  const double dx1 = parent_center_x - pivot.x;
  const double dy1 = parent_center_y - pivot.y;
  const double dx2 = pivot.x - target.x;
  const double dy2 = pivot.y - target.y;
  const double incoming =
      std::hypot(center_x - parent_center_x, center_y - parent_center_y);
  const double h2 = (target.x - center_x) * (target.x - center_x) +
                    (target.y - center_y) * (target.y - center_y);
  const double radius2 = radius * radius;
  if (incoming <= kEpsilon || h2 <= radius2 + kEpsilon) {
    return false;
  }
  const double h = std::sqrt(h2);
  const double outgoing = std::sqrt(h2 - radius2);
  const double cosine = outgoing / h;
  if (outgoing <= kEpsilon || cosine <= kEpsilon) {
    return false;
  }
  const double sine = std::sin(theta);
  const double cosine_theta = std::cos(theta);
  gradient_out =
      radius * ((radius * (radius + cosine_theta * dy2 - sine * dx2)) /
                    h2 +
                (radius2 * (cosine_theta * dx2 + sine * dy2)) /
                    (cosine * h2 * h)) +
      radius * (cosine_theta * dx1 + sine * dy1) / incoming -
      radius * (cosine_theta * dx2 + sine * dy2) / outgoing;
  if (derivative_out != nullptr) {
    const double parent_dx = center_x - parent_center_x;
    const double parent_dy = center_y - parent_center_y;
    const double target_dx = target.x - center_x;
    const double target_dy = target.y - center_y;
    const double sine_alpha2 =
        (sine * parent_dx - cosine_theta * parent_dy) / incoming;
    const double sine_tau =
        (target_dy * outgoing + target_dx * radius) / h2;
    const double cosine_tau =
        (target_dx * outgoing - target_dy * radius) / h2;
    const double sine_alpha3 =
        sine_tau * cosine_theta - cosine_tau * sine;
    *derivative_out =
        radius *
        (sine_alpha3 * (1.0 + radius * sine_alpha3 / outgoing) +
         sine_alpha2 * (1.0 + radius * sine_alpha2 / incoming));
  }
  return std::isfinite(gradient_out);
}

void refineTheta(const double parent_center_x, const double parent_center_y,
                 const TripletPoint pivot, const TripletPoint target,
                 const double radius, const bool use_bisection,
                 const int gradient_iterations, double &theta) {
  if (use_bisection) {
    double left = theta - 0.25;
    double right = theta + 0.25;
    constexpr double tolerance = 0.001;
    for (int iteration = 0;
         std::abs(right - left) > tolerance && iteration < 100; ++iteration) {
      const double first = left + (right - left) / 3.0;
      const double second = right - (right - left) / 3.0;
      double first_value = 0.0;
      double second_value = 0.0;
      const bool first_valid =
          objective(parent_center_x, parent_center_y, pivot, target, radius,
                    first, first_value);
      const bool second_valid =
          objective(parent_center_x, parent_center_y, pivot, target, radius,
                    second, second_value);
      if (first_valid && (!second_valid || first_value < second_value)) {
        right = second;
      } else if (second_valid) {
        left = first;
      } else {
        break;
      }
    }
    theta = 0.5 * (left + right);
    return;
  }

  double value = 0.0;
  for (int iteration = 1;
       iteration <= gradient_iterations &&
       gradient(parent_center_x, parent_center_y, pivot, target, radius, theta,
                value) &&
       std::abs(value) > 0.1;
       ++iteration) {
    theta -= value * kPi / (180.0 * std::sqrt(iteration));
  }
}

[[nodiscard]] bool parentContactTheta(const State parent,
                                      const TripletPoint pivot,
                                      const double radius,
                                      double &theta_out) {
  const double distance = std::hypot(pivot.x - parent.x,
                                     pivot.y - parent.y);
  if (distance <= kEpsilon || distance > 2.0 * radius + kEpsilon) {
    return false;
  }
  const double discriminant = 4.0 * radius * radius - distance * distance;
  if (discriminant < -kEpsilon) {
    return false;
  }
  const double root = std::sqrt(std::max(0.0, discriminant));
  const double a = distance * root + 2.0 * radius * (parent.x - pivot.x);
  const double b = distance * distance + 2.0 * radius * (parent.y - pivot.y);
  theta_out = -2.0 * std::atan2(a, b);
  return std::isfinite(theta_out);
}

[[nodiscard]] bool validDistance(const double distance) {
  return std::isfinite(distance) && distance >= 0.0 &&
         distance != std::numeric_limits<double>::max();
}

enum class ShortContactRoute { Unavailable, Same, Mixed };

[[nodiscard]] ShortContactRoute shortContactRoute(
    const State parent, const TripletPoint pivot, const TripletPoint target,
    const double radius, const double contact_theta, double &theta_out) {
  const TripletPoint local_pivot{(pivot.y - parent.y) / radius,
                                 (parent.x - pivot.x) / radius};
  if (local_pivot.x <= 0.0 || local_pivot.y <= 0.0 ||
      local_pivot.y >= 2.0) {
    return ShortContactRoute::Unavailable;
  }

  const double contact = wrapTo2Pi(contact_theta - kPi / 2.0);
  const double seam = std::acos(clampUnit(1.0 - local_pivot.y));
  const double dx = (target.y - pivot.y) / radius;
  const double dy = (pivot.x - target.x) / radius;
  const double terminal_end =
      contact + wrapTo2Pi(std::atan2(dy, dx) - contact);
  if (contact >= seam || seam > terminal_end + kEpsilon ||
      local_pivot.x - std::sin(seam) <= 0.0) {
    return ShortContactRoute::Unavailable;
  }

  const double nominal_right = seam + kPi / 2.0;
  double left = contact_theta;
  double right = nominal_right +
                 std::round((contact_theta - nominal_right) /
                            (2.0 * kPi)) *
                     2.0 * kPi;
  double right_value = 0.0;
  if (right <= left ||
      !gradient(parent.x - radius, parent.y, pivot, target, radius, right,
                right_value)) {
    return ShortContactRoute::Unavailable;
  }
  const double tie_tolerance =
      64.0 * std::numeric_limits<float>::epsilon() * radius;
  if (right_value < -tie_tolerance) {
    return ShortContactRoute::Mixed;
  }
  if (right_value <= tie_tolerance) {
    theta_out = right;
    return ShortContactRoute::Same;
  }
  for (int iteration = 0; iteration < 40; ++iteration) {
    const double middle = 0.5 * (left + right);
    double middle_value = 0.0;
    if (!gradient(parent.x - radius, parent.y, pivot, target, radius, middle,
                  middle_value) || middle_value < 0.0) {
      left = middle;
    } else {
      right = middle;
    }
  }
  theta_out = 0.5 * (left + right);
  return std::isfinite(theta_out) ? ShortContactRoute::Same
                                  : ShortContactRoute::Unavailable;
}

[[nodiscard]] bool tripletDistance(
    const State parent, const State pivot, const State target,
    const double radius, Solver &solver, double &distance,
    std::array<double, 2> *selected_segments = nullptr) {
  std::array<double, 2> local_segments{
      std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity()};
  auto &segments = selected_segments == nullptr ? local_segments
                                                 : *selected_segments;
  int condition = 0;
  int quadrant = 0;
  solver.acceleratedRSPlanner(parent, pivot, segments[0], radius, condition,
                              quadrant);
  solver.acceleratedRSPlanner(pivot, target, segments[1], radius, condition,
                              quadrant);
  distance = segments[0] + segments[1];
  return validDistance(segments[0]) && validDistance(segments[1]) &&
         validDistance(distance);
}

[[nodiscard]] double tripletLowerBound(const State parent, const State pivot,
                                       const State target,
                                       const double radius) {
  const auto segment_bound = [radius](const State from, const State to) {
    return std::max(std::hypot(to.x - from.x, to.y - from.y),
                    radius * std::abs(wrapToPi(to.theta - from.theta)));
  };
  return segment_bound(parent, pivot) + segment_bound(pivot, target);
}

} // namespace

[[gnu::always_inline]] inline SameSideTripletPreparation
prepareSameSideTripletParent(
    const State parent, const TripletPoint pivot,
    const TripletPoint physical_chart_pivot, const double radius,
    const SameSideTransform transform) {
  const State canonical_parent = pullbackParent(parent, transform);
  const TripletPoint canonical_pivot = pullbackPoint(pivot, transform);
  const TripletPoint chart_pivot =
      pullbackChartPoint(parent, physical_chart_pivot, transform);
  const double parent_contact_distance = std::hypot(
      canonical_pivot.x - canonical_parent.x +
          radius * std::sin(canonical_parent.theta),
      canonical_pivot.y - canonical_parent.y -
          radius * std::cos(canonical_parent.theta));
  const bool parent_contact =
      parent_contact_distance <= radius * (1.0 + kEpsilon);
  if (parent_contact) {
    return {true, 0.0};
  }
  return {false, std::atan2(chart_pivot.y - canonical_parent.y,
                            chart_pivot.x - canonical_parent.x)};
}

[[gnu::always_inline]] inline SameSideTripletStatus
selectSameSideTripletChartHeadings(
    const State parent, const TripletPoint pivot, const TripletPoint target,
    const TripletPoint physical_chart_pivot,
    const TripletPoint physical_chart_target, const double radius,
    const SameSideTransform transform, const bool use_bisection,
    const int gradient_iterations, Solver &solver, const bool select_target,
    State &selected_pivot, State &selected_target,
    std::array<double, 2> &selected_segments,
    const SameSideTripletPreparation *prepared_parent) {
  selected_segments.fill(std::numeric_limits<double>::infinity());
  selected_pivot = {pivot.x, pivot.y,
                    std::numeric_limits<double>::quiet_NaN()};
  selected_target = {target.x, target.y,
                     std::numeric_limits<double>::quiet_NaN()};
  if (!std::isfinite(parent.x) || !std::isfinite(parent.y) ||
      !std::isfinite(parent.theta) || !std::isfinite(pivot.x) ||
      !std::isfinite(pivot.y) || !std::isfinite(target.x) ||
      !std::isfinite(target.y) || !std::isfinite(physical_chart_pivot.x) ||
      !std::isfinite(physical_chart_pivot.y) ||
      !std::isfinite(physical_chart_target.x) ||
      !std::isfinite(physical_chart_target.y) || !std::isfinite(radius) ||
      radius <= 0.0 ||
      static_cast<unsigned>(transform) > 3U || gradient_iterations < 0) {
    return SameSideTripletStatus::InvalidInput;
  }

  const State canonical_parent = pullbackParent(parent, transform);
  const TripletPoint canonical_pivot = pullbackPoint(pivot, transform);
  const TripletPoint canonical_target = pullbackPoint(target, transform);
  const TripletPoint chart_pivot =
      pullbackChartPoint(parent, physical_chart_pivot, transform);
  const TripletPoint chart_target =
      pullbackChartPoint(parent, physical_chart_target, transform);
  const double parent_center_x = canonical_parent.x - radius;
  const double parent_center_y = canonical_parent.y;

  double theta = 0.0;
  double alternate_theta = std::numeric_limits<double>::quiet_NaN();
  bool short_root_selected = false;
  bool seed_requires_target_contact = false;
  const SameSideTripletPreparation prepared_on_demand =
      prepared_parent == nullptr
          ? prepareSameSideTripletParent(parent, pivot, physical_chart_pivot,
                                         radius, transform)
          : SameSideTripletPreparation{};
  const SameSideTripletPreparation &parent_preparation =
      prepared_parent == nullptr ? prepared_on_demand : *prepared_parent;
  const bool parent_contact = parent_preparation.parent_contact;
  if (parent_contact) {
    if (!parentContactTheta(canonical_parent, chart_pivot, radius, theta)) {
      return SameSideTripletStatus::GeometryFailure;
    }
  } else {
    theta =
        0.5 * wrapTo2Pi(std::atan2(chart_target.y - chart_pivot.y,
                                   chart_target.x - chart_pivot.x) +
                        parent_preparation.parent_bearing);
    double seeded_theta = theta;
    if (seedTheta(parent_center_x, parent_center_y, chart_pivot, chart_target,
                  radius, theta, seeded_theta,
                  select_target ? &alternate_theta : nullptr)) {
      theta = seeded_theta;
    }
    double seed_center_x = 0.0;
    double seed_center_y = 0.0;
    pivotCircle(chart_pivot, radius, theta, seed_center_x, seed_center_y);
    seed_requires_target_contact =
        std::hypot(chart_target.x - seed_center_x,
                   chart_target.y - seed_center_y) < radius;

    refineTheta(parent_center_x, parent_center_y, chart_pivot, chart_target,
                radius, use_bisection, gradient_iterations, theta);
  }

  if (select_target && !parent_contact && seed_requires_target_contact) {
    double contact_theta = 0.0;
    if (sameSideTargetContactTheta(
            chart_pivot.x, chart_pivot.y, chart_target.x, chart_target.y,
            radius, SameSideTransform::Identity, contact_theta)) {
      double short_theta = 0.0;
      const ShortContactRoute route = shortContactRoute(
          canonical_parent, chart_pivot, chart_target, radius, contact_theta,
          short_theta);
      if (route == ShortContactRoute::Same) {
        theta = short_theta;
        short_root_selected = true;
      } else if (route == ShortContactRoute::Mixed) {
        return SameSideTripletStatus::AdjacentMixedRequired;
      }
    }
  }

  double center_x = 0.0;
  double center_y = 0.0;
  pivotCircle(chart_pivot, radius, theta, center_x, center_y);
  if (std::hypot(chart_target.x - center_x, chart_target.y - center_y) <
      radius) {
    if (!sameSideTargetContactTheta(
            chart_pivot.x, chart_pivot.y, chart_target.x, chart_target.y,
            radius, SameSideTransform::Identity, theta)) {
      return SameSideTripletStatus::GeometryFailure;
    }
  }

  double canonical_pivot_heading =
      theta + canonical_parent.theta - kPi / 2.0;
  selected_pivot.theta = applyHeading(canonical_pivot_heading, transform);
  if (!select_target) {
    return SameSideTripletStatus::PivotSelected;
  }
  double terminal_heading = std::numeric_limits<double>::quiet_NaN();
  UnderSpecifiedRSPlanner urs;
  urs.getOmega(canonical_pivot.x, canonical_pivot.y, canonical_target.x,
               canonical_target.y, radius, canonical_pivot_heading,
               terminal_heading, true);
  if (!std::isfinite(terminal_heading)) {
    return SameSideTripletStatus::UrsFailure;
  }
  double canonical_target_heading =
      terminal_heading + canonical_pivot_heading - kPi / 2.0;
  const State regular_pivot{canonical_pivot.x, canonical_pivot.y,
                            canonical_pivot_heading};
  const State regular_target{canonical_target.x, canonical_target.y,
                             canonical_target_heading};
  double regular_distance = std::numeric_limits<double>::infinity();
  if (short_root_selected || std::isfinite(alternate_theta)) {
    if (!tripletDistance(canonical_parent, regular_pivot, regular_target,
                         radius, solver, regular_distance,
                         &selected_segments)) {
      regular_distance = std::numeric_limits<double>::infinity();
    }
  }
  if (std::isfinite(alternate_theta)) {
    if (use_bisection) {
      refineTheta(parent_center_x, parent_center_y, chart_pivot, chart_target,
                  radius, true, gradient_iterations, alternate_theta);
    } else {
      for (int iteration = 0; iteration < gradient_iterations; ++iteration) {
        double value = 0.0;
        double derivative = std::numeric_limits<double>::quiet_NaN();
        if (!gradient(parent_center_x, parent_center_y, chart_pivot,
                      chart_target, radius, alternate_theta, value,
                      &derivative) ||
            std::abs(value) <= kEpsilon * radius ||
            !std::isfinite(derivative) ||
            std::abs(derivative) <= kEpsilon * radius) {
          break;
        }
        alternate_theta -= std::clamp(value / derivative, -0.35, 0.35);
      }
    }
    pivotCircle(chart_pivot, radius, alternate_theta, center_x, center_y);
    if (std::isfinite(alternate_theta) &&
        std::hypot(chart_target.x - center_x, chart_target.y - center_y) <
            radius &&
        !sameSideTargetContactTheta(
            chart_pivot.x, chart_pivot.y, chart_target.x, chart_target.y,
            radius, SameSideTransform::Identity, alternate_theta)) {
      alternate_theta = std::numeric_limits<double>::quiet_NaN();
    }
    if (std::isfinite(alternate_theta)) {
      const double alternate_pivot_heading =
          alternate_theta + canonical_parent.theta - kPi / 2.0;
      double alternate_terminal = std::numeric_limits<double>::quiet_NaN();
      urs.getOmega(canonical_pivot.x, canonical_pivot.y, canonical_target.x,
                   canonical_target.y, radius, alternate_pivot_heading,
                   alternate_terminal, true);
      if (std::isfinite(alternate_terminal)) {
        const double alternate_target_heading =
            alternate_terminal + alternate_pivot_heading - kPi / 2.0;
        const State alternate_pivot{canonical_pivot.x, canonical_pivot.y,
                                    alternate_pivot_heading};
        const State alternate_target{canonical_target.x, canonical_target.y,
                                     alternate_target_heading};
        const double pruning_tolerance =
            64.0 * std::numeric_limits<float>::epsilon() *
            std::max(radius, regular_distance);
        std::array<double, 2> alternate_segments{
            std::numeric_limits<double>::infinity(),
            std::numeric_limits<double>::infinity()};
        double alternate_distance = std::numeric_limits<double>::infinity();
        if ((!validDistance(regular_distance) ||
             tripletLowerBound(canonical_parent, alternate_pivot,
                               alternate_target, radius) <=
                 regular_distance + pruning_tolerance) &&
            tripletDistance(canonical_parent, alternate_pivot,
                            alternate_target, radius, solver,
                            alternate_distance, &alternate_segments)) {
          const double selection_tolerance =
              64.0 * std::numeric_limits<float>::epsilon() *
              std::max({radius, regular_distance, alternate_distance});
          if (!validDistance(regular_distance) ||
              alternate_distance < regular_distance - selection_tolerance) {
            canonical_pivot_heading = alternate_pivot_heading;
            canonical_target_heading = alternate_target_heading;
            selected_segments = alternate_segments;
          }
          selected_pivot.theta =
              applyHeading(canonical_pivot_heading, transform);
        }
      }
    }
  }
  selected_target.theta = applyHeading(canonical_target_heading, transform);
  return SameSideTripletStatus::HeadingsSelected;
}

[[gnu::always_inline]] inline SameSideTripletStatus
selectSameSideTripletHeadings(
    const State parent, const TripletPoint pivot, const TripletPoint target,
    const double radius, const SameSideTransform transform,
    const bool use_bisection, const int gradient_iterations,
    Solver &solver, State &selected_pivot, State &selected_target,
    std::array<double, 2> &selected_segments) {
  return selectSameSideTripletChartHeadings(
      parent, pivot, target, toQuarterTurnChart(parent, pivot),
      toQuarterTurnChart(parent, target), radius, transform, use_bisection,
      gradient_iterations, solver, true, selected_pivot, selected_target,
      selected_segments);
}

inline SameSideTripletResult solveSameSideTriplet(
    const State parent, const TripletPoint pivot, const TripletPoint target,
    const double radius, const SameSideTransform transform, Solver &solver,
    const bool use_bisection, const int gradient_iterations) {
  SameSideTripletResult result;
  std::array<double, 2> selected_segments{};
  result.status = selectSameSideTripletHeadings(
      parent, pivot, target, radius, transform, use_bisection,
      gradient_iterations, solver, result.pivot, result.target,
      selected_segments);
  if (result.status != SameSideTripletStatus::HeadingsSelected) {
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
  if (!validDistance(result.incoming_distance) ||
      !validDistance(result.outgoing_distance)) {
    result.status = SameSideTripletStatus::DistanceFailure;
    return result;
  }
  result.distance = result.incoming_distance + result.outgoing_distance;
  if (!validDistance(result.distance)) {
    result.status = SameSideTripletStatus::DistanceFailure;
    return result;
  }
  result.status = SameSideTripletStatus::Selected;
  return result;
}

} // namespace RS_marching::geometry
