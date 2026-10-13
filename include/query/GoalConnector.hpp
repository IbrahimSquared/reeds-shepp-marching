#ifndef QUERY_GOAL_CONNECTOR_HPP
#define QUERY_GOAL_CONNECTOR_HPP

#include "audit/DirectStartClearance.hpp"
#include "geometry/ConnectorClearance.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <span>

namespace RS_marching::query {

// Bounded pivot-heading search. Supplied seeds beyond the first two extras are
// ignored, which keeps its work bound independent of caller input size.
inline constexpr std::size_t kHeadingSamples = 64;
inline constexpr std::size_t kRefinementStarts = 4;
inline constexpr std::size_t kRefinementIterations = 32;

struct QueryWork {
  std::uint64_t solves = 0;
  // Free-heading solves used only as lower bounds; they never build a leg.
  std::uint64_t bound_solves = 0;
  std::uint64_t materializations = 0;
  // Search-local retained replay copies; excluded from solves/materializations.
  std::uint64_t retained_replay_hits = 0;
  std::uint64_t heading_evaluations = 0;
  std::uint64_t refinement_evaluations = 0;
  // Analytic contact proposals are considered only after every supplied and
  // coarse heading fails to qualify. They consume the otherwise unavailable
  // refinement-start budget and are not local-refinement evaluations.
  std::uint64_t contact_evaluations = 0;
  // Maximum sizeof of the fixed contact scratch for any valid search call.
  std::uint64_t contact_scratch_bytes = 0;
  // Qualifications by the shared authority, and priced evaluations whose
  // length could not improve the search so needed none.
  std::uint64_t clearance_checks = 0;
  std::uint64_t lazy_skips = 0;
  // Exact continuous audits: contact witnesses and selected-route confirmation.
  std::uint64_t witness_checks = 0;
  std::uint64_t confirmations = 0;
  // Exact interval work of the authority's fallbacks and of the audits above.
  std::uint64_t exact_checks = 0;
  std::uint64_t clearance_intervals = 0;
  std::uint64_t clearance_events = 0;
  std::uint64_t free_box_attempts = 0, free_box_cells = 0, free_box_clears = 0;
  std::uint64_t replay_failures = 0;
  std::uint64_t blocked = 0;
  std::uint64_t indeterminate = 0;
  std::uint64_t incomplete = 0;
};

// One newly constructed leg: its fresh ARS or free-heading URS selection, literal replay (the
// price) and, once qualified, the shared authority's clearance status.
struct QualifiedLeg {
  audit::LegacyPathAuditResult path{};
  audit::ReplayPrimitiveBuffer prepared{};
  audit::ContinuousClearanceStatus clearance =
      audit::ContinuousClearanceStatus::Invalid;
  bool qualified = false;
  // First blocked primitive; meaningful only when clearance is Blocked.
  std::size_t blocked_primitive = 0;

  [[nodiscard]] bool priced() const { return path.replayed(); }
  [[nodiscard]] bool clear() const {
    return priced() && qualified &&
           clearance == audit::ContinuousClearanceStatus::Clear;
  }
};

// The qualification contract for new legs of every query mode: a fresh ARS/URS
// selection, literal replay within the shared tolerance, then the marcher's
// ConnectorClearance over the retained scene (sound filters, with the exact
// continuous checker when unresolved). The exact audit supplies contact
// witnesses and confirms a selected route; exact_authority makes it the
// qualification authority as well.
class LegQualifier {
public:
  LegQualifier(const Field<double> &map, geometry::ConnectorClearance checker,
               double radius, bool exact_authority = false);

  [[nodiscard]] QualifiedLeg price(State from, State to, QueryWork &work) const;
  // Existing URS selection and literal replay, with no terminal constraint.
  [[nodiscard]] QualifiedLeg priceFree(State from, double x, double y, QueryWork &work) const;
  bool qualify(QualifiedLeg &leg, State from, QueryWork &work);
  [[nodiscard]] QualifiedLeg leg(State from, State to, QueryWork &work);
  [[nodiscard]] audit::ContinuousClearanceResult exactAudit(
      const QualifiedLeg &leg, State from, QueryWork &work) const;
  // Adds the authority's exact fallback work since construction.
  void reportExactWork(QueryWork &work) const;

  [[nodiscard]] const Field<double> &map() const { return map_; }
  [[nodiscard]] double radius() const { return radius_; }

private:
  const Field<double> &map_;
  geometry::ConnectorClearance checker_;
  double radius_;
  bool exact_authority_;
};

// How far the exact endpoint of a replayed leg between pose a and point
// (bx, by) can lie from its target: the replay predicate's kReplayTolerance * r
// on the computed endpoint, plus the rounding of the replay and of distances
// and offsets, which the predicate does not see (at most five primitives, each
// rounding at the scale of the coordinates and the radius; headings are
// assumed moderate, see the bounds' domain). Every rejecting bound below
// subtracts it before inverting a lateral offset. Outside that scale/domain
// the slack is infinity, so displacement filters also reject nothing.
[[nodiscard]] double replayPositionSlack(State a, double bx, double by, double r);

// A lower bound on the length of every RS path from a to b with turning
// radius r, from displacement, heading change and lateral offsets. It is 0
// (rejecting nothing) for non-finite poses, r < 1e-100 or r > 1e100, and
// |a.theta| + |b.theta| > 1e3; rsConnectorLowerBound shares that domain.
[[nodiscard]] double rsLengthLowerBound(State a, State b, double r);
// A lower bound on the total length of the two RS legs a -> (p, psi) -> c over
// every heading psi at p, where the first leg starts exactly at a and the
// second exactly at (p, psi), and each may end within replayPositionSlack of
// its target. On each of 128 heading intervals it bounds every term of the two
// legs' rsLengthLowerBound from below, and takes the smallest sum; the scan
// starts at the interval centred on seed_heading. It returns as soon
// as one interval's bound is at most cutoff (a value <= cutoff then proves
// nothing); a result above cutoff is the full bound.
[[nodiscard]] double rsConnectorLowerBound(State a, double px, double py, State c, double r,
                                           double seed_heading, double cutoff);

struct PivotConnector {
  State pivot{std::numeric_limits<double>::quiet_NaN(),
              std::numeric_limits<double>::quiet_NaN(),
              std::numeric_limits<double>::quiet_NaN()};
  std::array<QualifiedLeg, 2> legs{};
  double distance = std::numeric_limits<double>::infinity();
  // The least-cost clear supplied seed (the pivot heading followed by at most
  // two extras), before periodic sampling or refinement.
  double retained_control_distance = std::numeric_limits<double>::infinity();

  [[nodiscard]] bool clear() const {
    return legs[0].clear() && legs[1].clear() && std::isfinite(distance) &&
           distance >= 0.0;
  }
};

// Searches only the represented fixed-pivot family A -> (pivot, psi) -> C.
// Coarse headings are A.theta + 2*pi*i/64. If those headings find no clear
// pair, a bounded CSCC last-cusp/occupied-edge contact proposal pass can test
// additional headings before one ordinary local refinement. A refinement step
// is qualified only when its literal length could improve the step, and a
// contact witness is audited only where an adjacent one can match it; neither
// changes a decision for a given qualification authority. This is neither an
// event partition nor a completeness, global-minimum, or convergence claim.
[[nodiscard]] PivotConnector searchPivotHeading(
    LegQualifier &qualifier, State A, State pivot, State C,
    std::span<const double> extra_seeds, QueryWork &work);

// The retained_control_distance searchPivotHeading would report, from the
// same seeds only. Used for a source skipped by a lower bound.
[[nodiscard]] double retainedControlDistance(
    LegQualifier &qualifier, State A, State pivot, State C,
    std::span<const double> extra_seeds, QueryWork &work);

} // namespace RS_marching::query

#endif
