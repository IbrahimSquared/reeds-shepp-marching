#include "audit/DirectStartAudit.hpp"
#include "misc/AngleRemainder.hpp"
#include "underspecified/UnderSpecifiedRSPlanner.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <limits>
#include <numbers>

namespace RS_marching::audit {
namespace {

constexpr double kPi = std::numbers::pi;
constexpr double kTwoPi = 2.0 * kPi;

[[nodiscard]] double wrapPi(double angle) {
  angle = detail::boundedAngleRemainder(angle + kPi, kTwoPi);
  if (angle < 0.0) {
    angle += kTwoPi;
  }
  return angle - kPi;
}

[[nodiscard]] LegacyPathAuditResult materializeSelectedPath(
    const Solver &solver, const State selector_to, const State replay_source,
    const double selected_distance,
    const double radius, const LegacyUrsAuditStatus urs_status,
    const int condition, const int quadrant, ReplayPrimitiveBuffer *const prepared = nullptr) {
  if (prepared != nullptr) prepared->reset();
  LegacyPathAuditResult result;
  result.urs_status = urs_status;
  result.target_theta = selector_to.theta;
  if (!std::isfinite(selected_distance) || condition < 1 || condition > 20 ||
      quadrant < 1 || quadrant > 4) {
    result.status = LegacyReplayStatus::SolverInvalid;
    return result;
  }

  char motion_types[5];
  int gears[5];
  double lengths[5];
  if (!solver.selectedWord(condition, quadrant, radius, motion_types, gears,
                           lengths, result.primitive_count)) {
    result.status = LegacyReplayStatus::PrimitiveMaterializationFailed;
    return result;
  }

  result.distance = selected_distance;
  result.condition = condition;
  result.quadrant = quadrant;
  double materialized_distance = 0.0;
  for (std::size_t index = 0; index < result.primitive_count; ++index) {
    result.primitives[index] =
        LegacyPrimitive{motion_types[index], gears[index], lengths[index]};
    materialized_distance += lengths[index];
  }

  result.endpoint = replayLegacyPrimitives(
      replay_source, radius, result.primitives, result.primitive_count, prepared);
  result.endpoint_position_residual =
      std::hypot(result.endpoint.x - selector_to.x,
                 result.endpoint.y - selector_to.y);
  result.endpoint_heading_residual =
      std::abs(wrapPi(result.endpoint.theta - selector_to.theta));
  result.distance_residual =
      std::abs(materialized_distance - selected_distance);
  const double distance_scale =
      std::max(radius, std::abs(selected_distance));
  const bool replay_matches =
      result.endpoint_position_residual <= kReplayTolerance * radius &&
      result.endpoint_heading_residual <= kReplayTolerance &&
      result.distance_residual <= kReplayTolerance * distance_scale;
  result.status = replay_matches ? LegacyReplayStatus::Replayed
                                 : LegacyReplayStatus::ReplayMismatch;
  if (!replay_matches && prepared != nullptr) prepared->reset();
  return result;
}

[[nodiscard]] LegacyPathAuditResult materializeLegacyPath(
    const State from, const State to, const double radius,
    const LegacyUrsAuditStatus urs_status) {
  Solver solver;
  double selected_distance = std::numeric_limits<double>::infinity();
  int condition = 0;
  int quadrant = 0;
  solver.acceleratedRSPlanner(from, to, selected_distance, radius, condition, quadrant);
  return materializeSelectedPath(solver, to, from, selected_distance, radius, urs_status,
                                 condition, quadrant);
}

[[nodiscard]] LegacyReplayStatus replayStatusForUrs(
    const LegacyUrsAuditStatus status) {
  if (status == LegacyUrsAuditStatus::PartitionGap) {
    return LegacyReplayStatus::UrsPartitionGap;
  }
  if (status == LegacyUrsAuditStatus::Nonfinite) {
    return LegacyReplayStatus::UrsNonfinite;
  }
  return LegacyReplayStatus::InvalidInput;
}

} // namespace

LegacyUrsAuditResult legacyUrsAudit(
    const double source_x, const double source_y, const double target_x,
    const double target_y, const double radius, const double source_theta,
    const bool offset) {
  LegacyUrsAuditResult result;
  if (!std::isfinite(source_x) || !std::isfinite(source_y) ||
      !std::isfinite(target_x) || !std::isfinite(target_y) ||
      !std::isfinite(radius) || radius <= 0.0 ||
      !std::isfinite(source_theta)) {
    return result;
  }

  UnderSpecifiedRSPlanner planner;
  // Every matched current branch writes a finite angle or NaN, never +infinity;
  // an unmatched branch leaves the caller-provided output untouched.
  double omega = std::numeric_limits<double>::infinity();
  planner.getOmega(source_x, source_y, target_x, target_y, radius,
                   source_theta, omega, offset);
  if (omega == std::numeric_limits<double>::infinity()) {
    result.status = LegacyUrsAuditStatus::PartitionGap;
    return result;
  }
  if (!std::isfinite(omega)) {
    result.status = LegacyUrsAuditStatus::Nonfinite;
    return result;
  }
  result.status = LegacyUrsAuditStatus::Valid;
  result.omega = omega;
  return result;
}

template <bool Prepare>
[[nodiscard]] State replayLegacyPrimitivesImpl(
    State source, const double radius,
    const std::array<LegacyPrimitive, 5> &primitives,
    const std::size_t primitive_count, ReplayPrimitiveBuffer *const prepared) {
  if (!std::isfinite(source.x) || !std::isfinite(source.y) ||
      !std::isfinite(source.theta) || !std::isfinite(radius) ||
      radius <= 0.0 || primitive_count > primitives.size()) {
    return State{std::numeric_limits<double>::quiet_NaN(),
                 std::numeric_limits<double>::quiet_NaN(),
                 std::numeric_limits<double>::quiet_NaN()};
  }
  [[maybe_unused]] const State replay_source = source;
  bool heading_ready = false;
  double sine = 0.0, cosine = 0.0;
  for (std::size_t index = 0; index < primitive_count; ++index) {
    const auto &primitive = primitives[index];
    const double direction = static_cast<double>(primitive.direction);
    if (!std::isfinite(primitive.length) || primitive.length < 0.0 ||
        (primitive.direction != -1 && primitive.direction != 1) ||
        (primitive.motion_type != 'L' && primitive.motion_type != 'S' &&
         primitive.motion_type != 'R')) {
      return State{std::numeric_limits<double>::quiet_NaN(),
                   std::numeric_limits<double>::quiet_NaN(),
                   std::numeric_limits<double>::quiet_NaN()};
    }
    if constexpr (!Prepare) {
      if (primitive.length == 0.0) continue;
    }
    if (!heading_ready) {
      sine = std::sin(source.theta);
      cosine = std::cos(source.theta);
      heading_ready = true;
    }
    [[maybe_unused]] ReplayPrimitiveGeometry *geometry = nullptr;
    if constexpr (Prepare) {
      geometry = &prepared->primitives[index];
      geometry->start_sine = sine;
      geometry->start_cosine = cosine;
      geometry->literal_endpoint = source;
      prepared->literal_primitives[index] = primitive;
    }
    if (primitive.motion_type == 'S') {
      if constexpr (Prepare) {
        geometry->enclosure_endpoint = {
            source.x + direction * primitive.length * cosine,
            source.y + direction * primitive.length * sine,
            source.theta};
      }
      if (primitive.length != 0.0) {
        source.x += direction * primitive.length * cosine;
        source.y += direction * primitive.length * sine;
        if constexpr (Prepare) geometry->literal_endpoint = source;
      }
    } else {
      const double curvature = primitive.motion_type == 'L' ? 1.0 : -1.0;
      const double center_x = source.x - curvature * radius * sine;
      const double center_y = source.y + curvature * radius * cosine;
      const double end_theta = source.theta +
          direction * curvature * primitive.length / radius;
      const double end_sine = std::sin(end_theta);
      const double end_cosine = std::cos(end_theta);
      if constexpr (Prepare) {
        geometry->enclosure_endpoint = {
            source.x + curvature * radius * (end_sine - sine),
            source.y - curvature * radius * (end_cosine - cosine),
            end_theta};
      }
      if (primitive.length != 0.0) {
        source = {center_x + curvature * radius * end_sine,
                  center_y - curvature * radius * end_cosine, end_theta};
        // A zero-length arc may change the sign of an enclosure heading's
        // zero, but leaves the literal pose and its trigonometry untouched.
        sine = end_sine;
        cosine = end_cosine;
        if constexpr (Prepare) geometry->literal_endpoint = source;
      }
    }
  }
  if constexpr (Prepare) {
    if (std::isfinite(source.x) && std::isfinite(source.y) &&
        std::isfinite(source.theta)) {
    prepared->source = replay_source;
    prepared->radius = radius;
    prepared->primitive_count = primitive_count;
    prepared->valid = true;
    }
  }
  source.theta = wrapPi(source.theta);
  return source;
}

State replayLegacyPrimitives(
    State source, const double radius,
    const std::array<LegacyPrimitive, 5> &primitives,
    const std::size_t primitive_count, ReplayPrimitiveBuffer *const prepared) {
  if (prepared != nullptr) {
    prepared->reset();
    return replayLegacyPrimitivesImpl<true>(source, radius, primitives,
                                            primitive_count, prepared);
  }
  return replayLegacyPrimitivesImpl<false>(source, radius, primitives,
                                           primitive_count, nullptr);
}

std::uint32_t selectedPathWord(const LegacyPathAuditResult &path) {
  std::uint32_t word = path.condition == -1 ? kFreeHeadingWord :
                       path.condition == -2 ? kFreeHeadingWord | kSuppliedHeadingWord : 0U;
  unsigned shift = 0;
  for (std::size_t i = 0; i < path.primitive_count; ++i) {
    const auto &leg = path.primitives[i];
    if (leg.length == 0.0) continue;
    const unsigned turn = leg.motion_type == 'L' ? 1U :
                          leg.motion_type == 'R' ? 2U : 3U;
    word |= (turn + (leg.direction < 0 ? 4U : 0U)) << shift;
    shift += 4;
  }
  return word;
}

LegacyPathAuditResult solveFreeHeadingCandidate(
    const State from, const double target_x, const double target_y,
    const double radius, const double distance_limit,
    const double complete_prefix, const double incumbent_cutoff,
    ReplayPrimitiveBuffer *const prepared) {
  if (prepared != nullptr) prepared->reset();
  LegacyPathAuditResult result;
  if (!std::isfinite(from.x) || !std::isfinite(from.y) ||
      !std::isfinite(from.theta) || !std::isfinite(target_x) ||
      !std::isfinite(target_y) || !std::isfinite(radius) || radius <= 0.0)
    return result;
  const double frame = from.theta - kPi / 2.0;
  double omega = std::numeric_limits<double>::infinity();
  UrsConnectorGeometry geometry;
  UnderSpecifiedRSPlanner{}.getOmega(from.x, from.y, target_x, target_y,
                                     radius, frame, omega, false, &geometry);
  if (geometry.region == 0 || !std::isfinite(omega)) return result;
  result.target_theta = omega + frame;
  const bool right = geometry.quadrant == 1 || geometry.quadrant == 4;
  const char turn = right ? 'R' : 'L';
  const char other = right ? 'L' : 'R';
  const int gear = geometry.quadrant <= 2 ? 1 : -1;
  // In each selected region the two turns have the same signed angular
  // displacement and total bend at most pi. No modulo-to-full-circle repair.
  const double bend = std::abs(detail::boundedNearestAngleRemainder(
      omega - kPi / 2.0, kTwoPi));
  const auto append = [&](const char type, const int direction,
                          const double length) {
    result.primitives[result.primitive_count++] = {type, direction, length};
  };
  if (geometry.region == 1) {
    append(turn, gear, radius * bend);
    append('S', gear, geometry.straight);
  } else if (geometry.region == 2) {
    append(other, -gear, radius * (bend - kPi / 2.0));
    append(turn, gear, radius * (kPi / 2.0));
    append('S', gear, geometry.straight);
  } else {
    append(other, -gear, radius * geometry.first_arc);
    append(turn, gear, radius * (bend - geometry.first_arc));
  }
  result.distance = 0.0;
  for (std::size_t i = 0; i < result.primitive_count; ++i) {
    const double length = result.primitives[i].length;
    // Unresolved signed lengths retain the original selector path. A replay
    // tolerance is not permission to reinterpret a negative primitive.
    if (!(length >= 0.0) || !std::isfinite(length)) return result;
    result.distance += length;
  }
  if (!std::isfinite(result.distance)) return result;
  // Price the analytic word before trigonometric replay. A pruned result is
  // explicitly not Replayed and cannot enter clearance or retained records.
  if (result.distance > distance_limit ||
      (std::isfinite(complete_prefix) && std::isfinite(incumbent_cutoff) &&
       complete_prefix + result.distance >= incumbent_cutoff)) {
    result.status = LegacyReplayStatus::CostPruned;
    return result;
  }
  result.endpoint = replayLegacyPrimitives(from, radius, result.primitives,
                                           result.primitive_count, prepared);
  result.endpoint_position_residual = std::hypot(result.endpoint.x - target_x,
                                                result.endpoint.y - target_y);
  result.endpoint_heading_residual = std::abs(wrapPi(
      result.endpoint.theta - result.target_theta));
  result.distance_residual = 0.0;
  result.urs_status = LegacyUrsAuditStatus::Valid;
  result.condition = -1;
  result.quadrant = geometry.quadrant;
  result.status = result.endpoint_position_residual <= kReplayTolerance * radius &&
                          result.endpoint_heading_residual <= kReplayTolerance
                      ? LegacyReplayStatus::Replayed
                      : LegacyReplayStatus::ReplayMismatch;
  if (!result.replayed() && prepared != nullptr) prepared->reset();
  return result;
}

LegacyPathAuditResult reconstructRetainedLeg(
    const State from, const State to, const double radius,
    const std::uint32_t word) {
  if ((word & kFreeHeadingWord) != 0) {
    auto path = solveFreeHeadingCandidate(from, to.x, to.y, radius);
    if ((word & kSuppliedHeadingWord) != 0) {
      if (!std::isfinite(to.theta)) {
        path.status = LegacyReplayStatus::ReplayMismatch;
        return path;
      }
      if (path.replayed()) {
        path.condition = -2;
        path.target_theta = to.theta;
        path.endpoint_heading_residual =
            std::abs(wrapPi(path.endpoint.theta - to.theta));
        if (!(path.endpoint_heading_residual <= kReplayTolerance) ||
            selectedPathWord(path) != word)
          path.status = LegacyReplayStatus::ReplayMismatch;
      }
    } else if (path.replayed() &&
               (path.target_theta != to.theta || selectedPathWord(path) != word)) {
      path.status = LegacyReplayStatus::ReplayMismatch;
    }
    return path;
  }
  auto path = materializeLegacyPath(from, to, radius, LegacyUrsAuditStatus::Valid);
  if (path.replayed() && selectedPathWord(path) != word)
    path.status = LegacyReplayStatus::ReplayMismatch;
  return path;
}

LegacyPathAuditResult solveDirectLegacyCandidate(
    const double source_x, const double source_y,
    const double legacy_source_theta, const double target_x,
    const double target_y, const double radius) {
  const auto urs = legacyUrsAudit(source_x, source_y, target_x, target_y,
                                  radius, legacy_source_theta, false);
  if (!urs.valid()) {
    LegacyPathAuditResult result;
    result.urs_status = urs.status;
    result.status = replayStatusForUrs(urs.status);
    return result;
  }
  const State from{source_x, source_y, legacy_source_theta + kPi / 2.0};
  const State to{target_x, target_y, urs.omega + legacy_source_theta};
  return materializeLegacyPath(from, to, radius, urs.status);
}

LegacyPathAuditResult replaySelectedLegacyPath(
    const Solver &selected_solver, const State selector_from,
    const State selector_to, const double selected_distance,
    const double radius, const int condition, const int quadrant,
    ReplayPrimitiveBuffer *const prepared) {
  if (prepared != nullptr) prepared->reset();
  return materializeSelectedPath(
      selected_solver, selector_to, selector_from, selected_distance, radius,
      LegacyUrsAuditStatus::Valid, condition, quadrant, prepared);
}

std::uint64_t
legacyPrimitiveIdentityHash(const LegacyPathAuditResult &result) {
  constexpr std::uint64_t offset = 1469598103934665603ULL;
  constexpr std::uint64_t prime = 1099511628211ULL;
  std::uint64_t hash = offset;
  const auto append = [&](const std::uint64_t value) {
    for (unsigned int shift = 0; shift < 64; shift += 8) {
      hash ^= (value >> shift) & 0xffU;
      hash *= prime;
    }
  };
  append(static_cast<std::uint64_t>(result.condition));
  append(static_cast<std::uint64_t>(result.quadrant));
  append(static_cast<std::uint64_t>(result.primitive_count));
  for (std::size_t index = 0; index < result.primitive_count; ++index) {
    append(static_cast<std::uint64_t>(
        static_cast<unsigned char>(result.primitives[index].motion_type)));
    append(static_cast<std::uint64_t>(
        static_cast<std::int64_t>(result.primitives[index].direction)));
    append(std::bit_cast<std::uint64_t>(result.primitives[index].length));
  }
  return hash;
}

} // namespace RS_marching::audit
