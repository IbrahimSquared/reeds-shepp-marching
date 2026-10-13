#ifndef TRIPLET_DISTANCE_HPP
#define TRIPLET_DISTANCE_HPP

#include "accelerated/Solver.hpp"
#include "geometry/SameSideTripletPreparation.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

namespace RS_marching::geometry {

// Frozen same-side group bits: mirror = 1, heading/gear dual = 2.
enum class SameSideTransform : unsigned {
  Identity = 0,
  Mirror = 1,
  HeadingGearDual = 2,
  MirrorHeadingGearDual = 3,
};

inline bool sameSideTargetContactTheta(
    const double pivot_x, const double pivot_y, const double target_x,
    const double target_y, const double radius,
    const SameSideTransform transform, double &theta_out) {
  theta_out = std::numeric_limits<double>::quiet_NaN();
  if (!std::isfinite(pivot_x) || !std::isfinite(pivot_y) ||
      !std::isfinite(target_x) || !std::isfinite(target_y) ||
      !std::isfinite(radius) || radius <= 0.0) {
    return false;
  }
  const auto bits = static_cast<unsigned>(transform);
  if (bits > 3U) {
    return false;
  }
  const double dx = target_x - pivot_x;
  const double dy = (bits & 1U) != 0U ? pivot_y - target_y
                                      : target_y - pivot_y;
  const double rho = std::hypot(dx, dy);
  if (rho == 0.0 || rho > 2.0 * radius) {
    return false;
  }
  const double ratio = std::min(1.0, rho / (2.0 * radius));
  double theta = std::atan2(dy, dx) - std::asin(ratio);
  if ((bits & 2U) != 0U) {
    constexpr double pi = 3.14159265358979323846;
    theta += pi;
  }
  theta_out = (bits & 1U) != 0U ? -theta : theta;
  return std::isfinite(theta_out);
}

struct TripletPoint {
  double x;
  double y;
};

[[nodiscard, gnu::always_inline]] inline SameSideTripletPreparation
prepareSameSideTripletParent(
    State parent, TripletPoint pivot, TripletPoint physical_chart_pivot,
    double radius, SameSideTransform transform);

enum class SameSideTripletStatus : std::uint8_t {
  InvalidInput = 0,
  GeometryFailure = 1,
  UrsFailure = 2,
  DistanceFailure = 3,
  PivotSelected = 4,
  HeadingsSelected = 5,
  Selected = 6,
  AdjacentMixedRequired = 7,
};

struct SameSideTripletResult {
  SameSideTripletStatus status = SameSideTripletStatus::InvalidInput;
  State pivot{0.0, 0.0, std::numeric_limits<double>::quiet_NaN()};
  State target{0.0, 0.0, std::numeric_limits<double>::quiet_NaN()};
  double incoming_distance = std::numeric_limits<double>::infinity();
  double outgoing_distance = std::numeric_limits<double>::infinity();
  double distance = std::numeric_limits<double>::infinity();

  [[nodiscard]] bool selected() const {
    return status == SameSideTripletStatus::Selected;
  }

};

[[nodiscard, gnu::always_inline]] inline SameSideTripletStatus
selectSameSideTripletHeadings(
    State parent, TripletPoint pivot, TripletPoint target, double radius,
    SameSideTransform transform, bool use_bisection, int gradient_iterations,
    Solver &solver, State &selected_pivot, State &selected_target,
    std::array<double, 2> &selected_segments);

[[nodiscard, gnu::always_inline]] inline SameSideTripletStatus
selectSameSideTripletChartHeadings(
    State parent, TripletPoint pivot, TripletPoint target,
    TripletPoint chart_pivot, TripletPoint chart_target, double radius,
    SameSideTransform transform, bool use_bisection, int gradient_iterations,
    Solver &solver, bool select_target, State &selected_pivot,
    State &selected_target, std::array<double, 2> &selected_segments,
    const SameSideTripletPreparation *prepared_parent = nullptr);

[[nodiscard]] inline SameSideTripletResult solveSameSideTriplet(
    State parent, TripletPoint pivot, TripletPoint target, double radius,
    SameSideTransform transform, Solver &solver, bool use_bisection,
    int gradient_iterations);

using MixedSideTripletStatus = SameSideTripletStatus;
using MixedSideTripletResult = SameSideTripletResult;

[[nodiscard, gnu::always_inline]] inline MixedSideTripletStatus
selectMixedSideTripletChartHeadings(
    State parent, TripletPoint pivot, TripletPoint target,
    TripletPoint chart_pivot, TripletPoint chart_target,
    double chart_seed_heading, double radius, SameSideTransform transform,
    Solver &solver, bool select_target, State &selected_pivot,
    State &selected_target,
    std::array<double, 2> *selected_segments = nullptr);

[[nodiscard, gnu::always_inline]] inline MixedSideTripletStatus
selectMixedSideTripletHeadings(
    State parent, TripletPoint pivot, TripletPoint target, double radius,
    SameSideTransform transform, Solver &solver, State &selected_pivot,
    State &selected_target,
    std::array<double, 2> *selected_segments = nullptr);

[[nodiscard]] inline MixedSideTripletResult solveMixedSideTriplet(
    State parent, TripletPoint pivot, TripletPoint target, double radius,
    SameSideTransform transform, Solver &solver);

[[nodiscard, gnu::always_inline]] inline SameSideTripletStatus
selectAdjacentTripletChartHeadings(
    State parent, TripletPoint pivot, TripletPoint target,
    TripletPoint chart_pivot, TripletPoint chart_target,
    double chart_seed_heading, double radius, SameSideTransform transform,
    bool use_bisection, int gradient_iterations, Solver &solver,
    bool select_target, State &selected_pivot, State &selected_target,
    std::array<double, 2> &selected_segments,
    const SameSideTripletPreparation *prepared_parent = nullptr);

} // namespace RS_marching::geometry

#include "geometry/SameSideTripletImpl.hpp"
#include "geometry/MixedSideTripletImpl.hpp"

namespace RS_marching::geometry {

[[gnu::always_inline]] inline SameSideTripletStatus
selectAdjacentTripletChartHeadings(
    const State parent, const TripletPoint pivot, const TripletPoint target,
    const TripletPoint chart_pivot, const TripletPoint chart_target,
    const double chart_seed_heading, const double radius,
    const SameSideTransform transform, const bool use_bisection,
    const int gradient_iterations, Solver &solver, const bool select_target,
    State &selected_pivot, State &selected_target,
    std::array<double, 2> &selected_segments,
    const SameSideTripletPreparation *prepared_parent) {
  const SameSideTripletStatus status = selectSameSideTripletChartHeadings(
      parent, pivot, target, chart_pivot, chart_target, radius, transform,
      use_bisection, gradient_iterations, solver, select_target,
      selected_pivot, selected_target, selected_segments, prepared_parent);
  if (status != SameSideTripletStatus::AdjacentMixedRequired) {
    return status;
  }
  const SameSideTripletStatus mixed_status =
      selectMixedSideTripletChartHeadings(
      parent, pivot, target, chart_pivot, chart_target, chart_seed_heading,
      radius, transform, solver, select_target, selected_pivot,
      selected_target, &selected_segments);
  return mixed_status;
}

} // namespace RS_marching::geometry

#endif
