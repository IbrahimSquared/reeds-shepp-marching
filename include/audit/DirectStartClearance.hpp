#ifndef DIRECT_START_CLEARANCE_HPP
#define DIRECT_START_CLEARANCE_HPP

#include "audit/DirectStartAudit.hpp"
#include "environment/Field.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace RS_marching::audit {

enum class ContinuousClearanceStatus : std::uint8_t {
  Invalid = 0,
  Incomplete = 1,
  Indeterminate = 2,
  Blocked = 3,
  Clear = 4
};

enum class ContinuousClearanceFeature : std::uint8_t {
  None = 0,
  InvalidInput = 1,
  MissingBranch = 2,
  NumericalBoundary = 3,
  MapExterior = 4,
  OccupiedCellInterior = 5,
  OccupiedSharedEdge = 6,
  OccupiedVertex = 7
};

struct ClearanceCell {
  int x = 0;
  int y = 0;
};

struct ContinuousClearanceResult {
  static constexpr std::uint8_t kNoPrimitive =
      std::numeric_limits<std::uint8_t>::max();

  ContinuousClearanceStatus status = ContinuousClearanceStatus::Invalid;
  ContinuousClearanceFeature feature =
      ContinuousClearanceFeature::InvalidInput;
  std::uint8_t primitive_index = kNoPrimitive;
  State witness{};
  std::array<ClearanceCell, 4> incident_cells{};
  std::uint8_t incident_cell_count = 0;
  double primitive_arclength_begin = 0.0;
  double primitive_arclength_end = 0.0;
  // Valid for Blocked: parameter of witness within primitive_index, distinct
  // from the bounds of the interval whose interior contains the witness.
  double witness_arclength = 0.0;
  std::size_t visited_intervals = 0;
  std::size_t visited_events = 0;
  std::size_t precision_refinements = 0;
  std::size_t refined_intervals = 0;
  std::size_t refined_events = 0;
  std::size_t free_box_attempts = 0;
  std::size_t free_box_cells = 0;
  std::size_t free_box_clears = 0;

  [[nodiscard]] bool clear() const {
    return status == ContinuousClearanceStatus::Clear;
  }
};

// Exact point-centerline clearance against the union of occupied centered unit
// squares. Exact external boundary contact is legal; shared occupied edges and
// four-occupied-cell vertices are interior and blocked.
[[nodiscard]] ContinuousClearanceResult continuousPathClearance(
    const Field<double> &occupancy, State source, double radius,
    const std::array<LegacyPrimitive, 5> &primitives,
    std::size_t primitive_count);

struct PrimitiveInterval {
  std::size_t primitive = 0;
  double begin = 0.0;
  double end = 0.0;
};

// Checks only this closed arclength interval of one literal primitive. Earlier
// primitives locate its exact starting geometry; their clearance is NOT
// implied by this result. Refinement restarts from the original input doubles.
[[nodiscard]] ContinuousClearanceResult continuousPathIntervalClearance(
    const Field<double> &occupancy, State source, double radius,
    const std::array<LegacyPrimitive, 5> &primitives,
    std::size_t primitive_count, PrimitiveInterval interval);

enum class ResolvedChainStatus : std::uint8_t {
  Clear = 0,
  Indeterminate = 1,
  Blocked = 2,
  Incomplete = 3,
  Unsupported = 4,
  Invalid = 5
};

struct ResolvedSegmentAudit {
  LegacyPathAuditResult path{};
  ContinuousClearanceResult clearance{};
};

struct ResolvedChainAuditResult {
  ResolvedChainStatus status = ResolvedChainStatus::Invalid;
  std::vector<ResolvedSegmentAudit> segments;
  double segment_sum = std::numeric_limits<double>::infinity();
  double complete_distance_residual = std::numeric_limits<double>::infinity();

  [[nodiscard]] bool certified() const {
    return status == ResolvedChainStatus::Clear;
  }
};

// Audits one already resolved target-conditioned pose chain. It selects and
// materializes the public ARS identity for every adjacent pair, independently
// replays it, checks the complete sum, and applies exact continuous clearance.
[[nodiscard]] ResolvedChainAuditResult auditResolvedChain(
    const Field<double> &occupancy, const std::vector<State> &poses,
    double accepted_distance, double radius,
    const std::vector<std::uint32_t> *words = nullptr);

} // namespace RS_marching::audit

#endif
