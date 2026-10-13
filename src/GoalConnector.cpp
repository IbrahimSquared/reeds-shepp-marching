#include "query/GoalConnector.hpp"

#include "accelerated/Solver.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <bit>
#include <cstdint>
#include <limits>
#include <utility>

namespace RS_marching::query {
namespace {

constexpr double kTwoPi = 2.0 * std::numbers::pi;
constexpr double kRefinementStop = 1e-7;
constexpr std::size_t kContactRootCapacity = 2 + 2 * 4 * 4 * 2 * 2;
constexpr std::size_t kContactEvaluationLimit =
    (kRefinementStarts - 1) * kRefinementIterations * 2;

[[nodiscard]] bool validPose(const State pose) {
  return std::isfinite(pose.x) && std::isfinite(pose.y) &&
         std::isfinite(pose.theta);
}

[[nodiscard]] bool validInput(const Field<double> &map, const State from,
                              const State to, const double radius) {
  return map.nx() != 0 && map.ny() != 0 && validPose(from) && validPose(to) &&
         std::isfinite(radius) && radius > 0.0;
}

void countExact(const audit::ContinuousClearanceResult &result, QueryWork &work) {
  ++work.exact_checks;
  work.clearance_intervals += result.visited_intervals;
  work.clearance_events += result.visited_events;
  work.free_box_attempts += result.free_box_attempts;
  work.free_box_cells += result.free_box_cells;
  work.free_box_clears += result.free_box_clears;
}

void countStatus(const audit::ContinuousClearanceStatus status, QueryWork &work) {
  switch (status) {
  case audit::ContinuousClearanceStatus::Blocked: ++work.blocked; break;
  case audit::ContinuousClearanceStatus::Indeterminate: ++work.indeterminate; break;
  case audit::ContinuousClearanceStatus::Clear: break;
  case audit::ContinuousClearanceStatus::Invalid:
  case audit::ContinuousClearanceStatus::Incomplete: ++work.incomplete; break;
  }
}

struct Trial {
  double psi = std::numeric_limits<double>::quiet_NaN();
  double cost = std::numeric_limits<double>::infinity();
};

struct ContactWitness {
  char curve = 'N';
  int gear = 0;
  std::array<audit::ClearanceCell, 4> cells{};
  std::uint8_t cell_count = 0;
  bool valid = false;
};

struct StoredWord {
  std::array<audit::LegacyPrimitive, 5> primitives{};
  std::uint8_t count = 0;
};

struct ContactScratch {
  std::array<ContactWitness, kHeadingSamples> witnesses{};
  // Nonzero for a coarse heading whose incoming leg is clear and whose
  // outgoing word, first blocked at primitive 2 (the only index contactWitness
  // accepts), has the witness shape; encodes its curve and gear.
  std::array<std::uint8_t, kHeadingSamples> shapes{};
  std::array<bool, kHeadingSamples> audited{};
  // The blocked outgoing word of each witness-shaped coarse heading.
  std::array<StoredWord, kHeadingSamples> outgoing{};
  std::array<double, kContactRootCapacity> roots{};
};

[[nodiscard]] bool isCurve(const char motion) {
  return motion == 'L' || motion == 'R';
}

// C_s^g S^g C_s^g C_-s^-g, the only outgoing word that can carry a witness;
// the code encodes its curve and gear.
[[nodiscard]] std::uint8_t wordShape(const audit::LegacyPathAuditResult &path) {
  const auto &word = path.primitives;
  const char curve = word[0].motion_type;
  const int gear = word[0].direction;
  if (!path.replayed() || path.primitive_count != 4 || !isCurve(curve) ||
      word[1].motion_type != 'S' || word[2].motion_type != curve ||
      !isCurve(word[3].motion_type) || word[3].motion_type == curve ||
      (gear != 1 && gear != -1) || word[1].direction != gear ||
      word[2].direction != gear || word[3].direction != -gear)
    return 0;
  return static_cast<std::uint8_t>(1 + (curve == 'L' ? 2 : 0) + (gear == 1 ? 1 : 0));
}

// The witness needs the exact first blocking contact, so it comes from the
// exact audit of the outgoing word (the incoming leg is already clear).
[[nodiscard]] ContactWitness contactWitness(
    const QualifiedLeg &outgoing, const audit::ContinuousClearanceResult &clearance,
    const double radius) {
  ContactWitness result;
  if (wordShape(outgoing.path) == 0 ||
      clearance.status != audit::ContinuousClearanceStatus::Blocked ||
      clearance.primitive_index != 2 ||
      clearance.incident_cell_count > result.cells.size() ||
      !std::isfinite(radius) || radius <= 0.0)
    return result;
  const auto &word = outgoing.path.primitives;
  if (!std::isfinite(word[2].length) ||
      std::abs(word[2].length - radius * std::numbers::pi / 2.0) >
          audit::kReplayTolerance * std::max(radius, std::abs(word[2].length)))
    return result;
  result.curve = word[0].motion_type;
  result.gear = word[0].direction;
  result.cell_count = clearance.incident_cell_count;
  for (std::size_t index = 0; index < result.cell_count; ++index)
    result.cells[index] = clearance.incident_cells[index];
  result.valid = true;
  return result;
}

[[nodiscard]] bool matchingWitnesses(const ContactWitness &left,
                                     const ContactWitness &right) {
  return left.valid && right.valid && left.curve == right.curve &&
         left.gear == right.gear;
}

void appendContactRoots(std::array<double, kContactRootCapacity> &roots,
                        std::size_t &root_count, const ContactWitness &witness,
                        const State pivot, const State target,
                        const double radius) {
  const double turn = witness.curve == 'L' ? 1.0 : -1.0;
  const double gear = static_cast<double>(witness.gear);
  // For C_s^g S^g C_s^g C_-s^-g with a quarter-turn third arc,
  // O = C - s*r*n(theta_C), last cusp X = O - g*r*u(alpha), and
  // cos(psi-alpha) = n(alpha).(O-Q)/(s*r). Solve X on each cell
  // edge in Q-local, radius-one coordinates. These are proposals only:
  // branch membership and clearance still require two fresh qualified legs.
  const double dx = (target.x - pivot.x) / radius - turn * -std::sin(target.theta);
  const double dy = (target.y - pivot.y) / radius - turn * std::cos(target.theta);
  if (!std::isfinite(dx) || !std::isfinite(dy)) return;
  const auto append = [&](const double psi) {
    if (root_count < roots.size() && std::isfinite(psi)) roots[root_count++] = psi;
  };
  for (std::size_t cell_index = 0; cell_index < witness.cell_count; ++cell_index) {
    const auto &cell = witness.cells[cell_index];
    for (const bool x_axis : {true, false}) {
      const int coordinate = x_axis ? cell.x : cell.y;
      for (const double edge : {static_cast<double>(coordinate) - 0.5,
                                static_cast<double>(coordinate) + 0.5}) {
        const double q_coordinate = x_axis ? pivot.x : pivot.y;
        const double d_coordinate = x_axis ? dx : dy;
        const double z = (d_coordinate - (edge - q_coordinate) / radius) / gear;
        if (!std::isfinite(z) || z < -1.0 || z > 1.0) continue;
        const double first = x_axis ? std::acos(z) : std::asin(z);
        const double second = x_axis ? -first : std::numbers::pi - first;
        for (const double alpha : {first, second}) {
          const double b = ((-std::sin(alpha)) * dx + std::cos(alpha) * dy) / turn;
          if (!std::isfinite(b) || b < -1.0 || b > 1.0) continue;
          const double delta = std::acos(b);
          append(alpha + delta);
          append(alpha - delta);
        }
      }
    }
  }
}

void wrapRootsIntoBracket(std::array<double, kContactRootCapacity> &roots,
                          std::size_t &root_count, const double lo,
                          const double hi) {
  const std::size_t generated = root_count;
  root_count = 0;
  for (std::size_t index = 0; index < generated; ++index) {
    double root = roots[index];
    if (!std::isfinite(root)) continue;
    root += kTwoPi * std::ceil((lo - root) / kTwoPi);
    if (root >= lo && root <= hi && root_count < roots.size())
      roots[root_count++] = root;
  }
  roots[root_count++] = lo;
  roots[root_count++] = hi;
  std::sort(roots.begin(), roots.begin() + root_count);
  root_count = static_cast<std::size_t>(
      std::unique(roots.begin(), roots.begin() + root_count) - roots.begin());
}

// Prices the incoming leg, then the outgoing one unless the incoming length
// plus rsLengthLowerBound already reaches improve_below; qualifies them
// (incoming first) only when their literal length is below improve_below, the
// only case in which the caller can use the result. A finite improve_below
// therefore never changes a decision.
[[nodiscard]] PivotConnector evaluate(
    LegQualifier &qualifier, const State A, const State pivot, const State C,
    const double psi, QueryWork &work,
    const double improve_below = std::numeric_limits<double>::infinity(),
    std::uint8_t *const shape = nullptr,
    StoredWord *const outgoing = nullptr) {
  PivotConnector result;
  result.pivot = pivot;
  result.pivot.theta = psi;
  ++work.heading_evaluations;
  result.legs[0] = qualifier.price(A, result.pivot, work);
  if (!result.legs[0].priced()) return result;
  if (std::isfinite(improve_below) &&
      !(result.legs[0].path.distance +
            rsLengthLowerBound(result.pivot, C, qualifier.radius()) < improve_below)) {
    ++work.lazy_skips;
    return result;
  }
  result.legs[1] = qualifier.price(result.pivot, C, work);
  if (!result.legs[1].priced()) return result;
  const double total = result.legs[0].path.distance + result.legs[1].path.distance;
  if (std::isfinite(improve_below) && !(total < improve_below)) {
    ++work.lazy_skips;
    return result;
  }
  if (!qualifier.qualify(result.legs[0], A, work)) return result;
  if (!qualifier.qualify(result.legs[1], result.pivot, work)) {
    if (shape != nullptr &&
        result.legs[1].clearance == audit::ContinuousClearanceStatus::Blocked) {
      *shape = result.legs[1].blocked_primitive == 2 ? wordShape(result.legs[1].path) : 0;
      if (*shape != 0 && outgoing != nullptr)
        *outgoing = {result.legs[1].path.primitives,
                     static_cast<std::uint8_t>(result.legs[1].path.primitive_count)};
    }
    return result;
  }
  if (!std::isfinite(total) || total < 0.0) {
    ++work.incomplete;
    return result;
  }
  result.distance = total;
  return result;
}

void insertStart(std::array<Trial, kRefinementStarts> &starts, const Trial trial) {
  if (!std::isfinite(trial.cost)) return;
  for (std::size_t index = 0; index < starts.size(); ++index) {
    if (trial.cost < starts[index].cost) {
      for (std::size_t shift = starts.size() - 1; shift > index; --shift)
        starts[shift] = starts[shift - 1];
      starts[index] = trial;
      return;
    }
  }
}

void retain(PivotConnector &best, const PivotConnector &candidate) {
  if (candidate.clear() && candidate.distance < best.distance) best = candidate;
}

// The shortest length that reaches lateral offset e from a heading line with
// relative heading |sin| at most delta at one end (see rsLengthLowerBound).
// It grows with e and shrinks with delta, so delta = 1 bounds every heading.
double lateralLength(const double e, const double delta, const double r) {
  const double envelope = e <= 0.5 * r * delta * delta
      ? std::sqrt(2.0 * r * e)
      : -r * delta + std::sqrt(2.0 * r * r * delta * delta + 4.0 * r * e);
  return std::max(envelope, e > 0.5 * r ? e + 0.5 * r : 0.0);
}

// Offset of (x, y) from the heading line of pose.
double lineOffset(const State pose, const double x, const double y) {
  return std::abs(-std::sin(pose.theta) * (x - pose.x) + std::cos(pose.theta) * (y - pose.y));
}

// Rounded down so that a literal length equal to the exact bound (a straight
// leg) is never below it.
double roundedBound(const double bound, const double r) {
  // Overflow in a sufficient bound is uncertainty, never a rejecting infinity.
  if (!std::isfinite(bound)) return 0.0;
  return std::max(0.0, bound * (1.0 - 1e-12) - 1e-12 * r);
}

// Heading slack: the replay tolerance plus the rounding of the headings.
double headingSlack(const double heading_scale) {
  return audit::kReplayTolerance +
         256.0 * std::numeric_limits<double>::epsilon() * (8.0 + heading_scale);
}

bool finitePose(const State pose) {
  return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.theta);
}

// The slacks assume moderate headings: the replay rounds each heading, which
// changes an arc's effective length by up to r * ulp(theta) / 2, covered by
// roundedBound only while |theta| stays well below 1e3. Radii beyond 1e100
// could overflow r * turn or r^2 * delta^2; radii below 1e-100 can
// underflow the rounding allowances. Outside this domain the bounds are
// 0 and reject nothing.
bool supported(const double r, const double heading_scale) {
  return r >= 1e-100 && r <= 1e100 && heading_scale <= 1e3;
}

} // namespace

double replayPositionSlack(const State a, const double bx, const double by, const double r) {
  if (!finitePose(a) || !std::isfinite(bx) || !std::isfinite(by) ||
      !supported(r, std::abs(a.theta)))
    return std::numeric_limits<double>::infinity();
  const double scale = std::max({std::abs(a.x), std::abs(a.y), std::abs(bx), std::abs(by)});
  return audit::kReplayTolerance * r +
         256.0 * std::numeric_limits<double>::epsilon() * (scale + 2.0 * r);
}

// A lower bound on the length of any RS path between poses a and b. Besides
// |ab| and r * heading change: the heading phi relative to b's heading turns
// by at most 1/r per unit length, ends at zero and starts at a's relative
// heading, so |sin phi(s)| <= min(1, delta + s/r, (L - s)/r) with
// delta = |sin(a.theta - b.theta)|. A lateral offset e from b's heading line
// therefore needs e <= L^2/(4r) + delta*L/2 - r*delta^2/4 (or L^2/(2r) while
// L <= r*delta), and e <= L - r/2. The same holds from a by reversal.
double rsLengthLowerBound(const State a, const State b, const double r) {
  // A replayed leg's computed endpoint is within kReplayTolerance * r and
  // kReplayTolerance radians of b, and the exact endpoint within rounding of
  // that, so the bound holds for every such endpoint: each term below is
  // evaluated with that slack (the lateral bound grows with e and shrinks with
  // delta). Inputs outside the supported domain give 0, which rejects nothing.
  if (!finitePose(a) || !finitePose(b) || !supported(r, std::abs(a.theta) + std::abs(b.theta)))
    return 0.0;
  const double position_slack = replayPositionSlack(a, b.x, b.y, r);
  const double heading_slack = headingSlack(std::abs(a.theta) + std::abs(b.theta));
  const double distance = std::hypot(b.x - a.x, b.y - a.y);
  const double turn = std::abs(std::remainder(a.theta - b.theta, 2.0 * std::numbers::pi));
  const double delta = std::min(1.0, std::abs(std::sin(a.theta - b.theta)) + heading_slack);
  const double offset_b = std::max(0.0, lineOffset(b, a.x, a.y) - position_slack -
                                            heading_slack * (distance + position_slack));
  const double offset_a = std::max(0.0, lineOffset(a, b.x, b.y) - position_slack);
  return roundedBound(std::max({distance - position_slack,
                                r * std::max(0.0, turn - heading_slack),
                                lateralLength(offset_b, delta, r),
                                lateralLength(offset_a, delta, r)}),
                      r);
}

namespace {

constexpr int kConnectorIntervals = 128;

// sin and cos of k * 2pi / kConnectorIntervals, for k = 0..kConnectorIntervals.
struct IntervalTable {
  std::array<double, kConnectorIntervals + 1> s{}, c{};
  IntervalTable() {
    for (int k = 0; k <= kConnectorIntervals; ++k) {
      const double angle = 2.0 * std::numbers::pi * k / kConnectorIntervals;
      s[k] = std::sin(angle);
      c[k] = std::cos(angle);
    }
  }
};

// A boundary value this close to zero counts as a zero of the function inside
// the interval: the minimum of |sin| becomes 0 and its maximum 1, which only
// lowers the bound.
constexpr double kZeroTolerance = 1e-9;

bool crossesZero(const double v0, const double v1) {
  return (v0 <= kZeroTolerance && v1 >= -kZeroTolerance) ||
         (v0 >= -kZeroTolerance && v1 <= kZeroTolerance);
}

} // namespace

// Over a heading interval at p no wider than pi/2, |sin(u - psi)| has at most
// one extremum: a zero of sin (its minimum 0) or of cos (its maximum 1);
// otherwise it is monotone and its extremes lie at the boundaries. Each term of
// rsLengthLowerBound(a, (p, psi)) and rsLengthLowerBound((p, psi), c) is
// bounded below on the interval by evaluating it at the interval's smallest
// turn and offsets and largest delta (the lateral term grows with the offset
// and shrinks with delta). The boundary sines come from one exact sine and
// cosine per angle and the table, within a few ulp; offsets are lowered by
// 1e-11 * distance before the lateral term's square root to cover that error.
double rsConnectorLowerBound(const State a, const double px, const double py, const State c,
                             const double r, const double seed_heading, const double cutoff) {
  static const IntervalTable table;
  constexpr double kTwoPi = 2.0 * std::numbers::pi;
  constexpr double w = kTwoPi / kConnectorIntervals;
  if (!finitePose(a) || !finitePose(c) || !std::isfinite(px) || !std::isfinite(py) ||
      !supported(r, std::abs(a.theta) + std::abs(c.theta)))
    return 0.0;
  // One slack for both legs, at the scale of all three points; psi stays
  // within about 4 pi of zero.
  const double ps = std::max(replayPositionSlack(a, px, py, r), replayPositionSlack(c, px, py, r));
  const double hs = headingSlack(std::abs(a.theta) + std::abs(c.theta) + 4.0 * std::numbers::pi);
  const double d0 = std::hypot(px - a.x, py - a.y), d1 = std::hypot(c.x - px, c.y - py);
  // Offsets from a's and c's heading lines do not depend on psi.
  const double off_a0 = std::max(0.0, lineOffset(a, px, py) - ps);
  const double off_b1 = std::max(0.0, lineOffset(c, px, py) - ps - hs * (d1 + ps));
  // The first interval is centred on the seed heading, where a reachable
  // connector usually lies, so an unprunable source returns early.
  const double psi0 =
      (std::isfinite(seed_heading) ? std::remainder(seed_heading, kTwoPi) : 0.0) - 0.5 * w;
  // Angles u whose difference u - psi the bound needs: the headings of a and c,
  // and the directions from p to a and to c.
  const std::array<double, 4> u{a.theta, c.theta, std::atan2(a.y - py, a.x - px),
                                std::atan2(c.y - py, c.x - px)};
  std::array<double, 4> s0{}, c0{};
  for (int i = 0; i < 4; ++i) {
    s0[i] = std::sin(u[i] - psi0);
    c0[i] = std::cos(u[i] - psi0);
  }
  // sin and cos of u_i - (psi0 + k w) by angle subtraction.
  const auto sinAt = [&](const int i, const int k) { return s0[i] * table.c[k] - c0[i] * table.s[k]; };
  const auto cosAt = [&](const int i, const int k) { return c0[i] * table.c[k] + s0[i] * table.s[k]; };
  const double turn_a0 = std::remainder(a.theta - psi0, kTwoPi);
  const double turn_c0 = std::remainder(c.theta - psi0, kTwoPi);
  const auto wrapped = [&](const double t0, const int k) {
    const double x = t0 - k * w;
    return x < -std::numbers::pi ? x + kTwoPi : x;
  };
  // Smallest |remainder(theta - psi)| over interval k. The wrapped difference
  // decreases with psi except for one jump at +-pi, its maximum.
  const auto turnMin = [&](const double t0, const int k) {
    const double x0 = wrapped(t0, k), x1 = wrapped(t0, k + 1);
    if (x1 <= x0 && x0 >= -kZeroTolerance && x1 <= kZeroTolerance) return 0.0;
    return std::min(std::abs(x0), std::abs(x1));
  };
  const auto minAbsSin = [&](const int i, const int k) {
    const double v0 = sinAt(i, k), v1 = sinAt(i, k + 1);
    return crossesZero(v0, v1) ? 0.0 : std::min(std::abs(v0), std::abs(v1));
  };
  const auto delta = [&](const int i, const int k) {
    const double max_abs_sin = crossesZero(cosAt(i, k), cosAt(i, k + 1))
        ? 1.0 : std::max(std::abs(sinAt(i, k)), std::abs(sinAt(i, k + 1))) + 1e-12;
    return std::min(1.0, max_abs_sin + hs);
  };
  double best = std::numeric_limits<double>::infinity();
  for (int k = 0; k < kConnectorIntervals; ++k) {
    const double delta0 = delta(0, k), delta1 = delta(1, k);
    const double off_b0 = std::max(0.0, d0 * minAbsSin(2, k) - ps - hs * (d0 + ps) - 1e-11 * d0);
    const double off_a1 = std::max(0.0, d1 * minAbsSin(3, k) - ps - 1e-11 * d1);
    const double leg0 = std::max({d0 - ps, r * std::max(0.0, turnMin(turn_a0, k) - hs),
                                  lateralLength(off_b0, delta0, r),
                                  lateralLength(off_a0, delta0, r)});
    const double leg1 = std::max({d1 - ps, r * std::max(0.0, turnMin(turn_c0, k) - hs),
                                  lateralLength(off_b1, delta1, r),
                                  lateralLength(off_a1, delta1, r)});
    const double bound = roundedBound(leg0 + leg1, r);
    if (bound <= cutoff) return bound;
    best = std::min(best, bound);
  }
  return best;
}

LegQualifier::LegQualifier(const Field<double> &map,
                           geometry::ConnectorClearance checker,
                           const double radius, const bool exact_authority)
    : map_(map), checker_(std::move(checker)), radius_(radius),
      exact_authority_(exact_authority) {}

QualifiedLeg LegQualifier::price(const State from, const State to,
                                 QueryWork &work) const {
  QualifiedLeg result;
  if (!validInput(map_, from, to, radius_)) {
    ++work.incomplete;
    return result;
  }
  Solver solver;
  double selected_distance = std::numeric_limits<double>::infinity();
  int condition = 0;
  int quadrant = 0;
  ++work.solves;
  solver.acceleratedRSPlanner(from, to, selected_distance, radius_, condition,
                              quadrant);
  if (!std::isfinite(selected_distance) || selected_distance < 0.0) {
    ++work.incomplete;
    return result;
  }
  ++work.materializations;
  result.path = audit::replaySelectedLegacyPath(solver, from, to,
                                                selected_distance, radius_,
                                                condition, quadrant,
                                                &result.prepared);
  if (!result.path.replayed()) {
    ++work.replay_failures;
    ++work.incomplete;
    return result;
  }
  double literal_length = 0.0;
  for (std::size_t index = 0; index < result.path.primitive_count; ++index)
    literal_length += result.path.primitives[index].length;
  if (!std::isfinite(literal_length) || literal_length < 0.0) {
    ++work.incomplete;
    result.path.status = audit::LegacyReplayStatus::ReplayMismatch;
    return result;
  }
  // replayed() has already checked this literal sum against selector price.
  result.path.distance = literal_length;
  return result;
}

QualifiedLeg LegQualifier::priceFree(const State from, const double x, const double y,
                                     QueryWork &work) const {
  QualifiedLeg result;
  if (!validInput(map_, from, {x, y, from.theta}, radius_)) {
    ++work.incomplete;
    return result;
  }
  ++work.solves;
  ++work.materializations;
  result.path = audit::solveFreeHeadingCandidate(
      from, x, y, radius_, std::numeric_limits<double>::infinity(), 0.0,
      std::numeric_limits<double>::infinity(), &result.prepared);
  if (!result.path.replayed()) {
    ++work.replay_failures;
    ++work.incomplete;
    return result;
  }
  double length = 0.0;
  for (std::size_t index = 0; index < result.path.primitive_count; ++index)
    length += result.path.primitives[index].length;
  if (!std::isfinite(length) || length < 0.0) {
    result.path.status = audit::LegacyReplayStatus::ReplayMismatch;
    ++work.incomplete;
    return result;
  }
  result.path.distance = length;
  return result;
}

bool LegQualifier::qualify(QualifiedLeg &leg, const State from, QueryWork &work) {
  if (!leg.priced()) return false;
  ++work.clearance_checks;
  if (exact_authority_) {
    const auto exact = audit::continuousPathClearance(
        map_, from, radius_, leg.path.primitives, leg.path.primitive_count);
    countExact(exact, work);
    leg.clearance = exact.status;
    leg.blocked_primitive = exact.primitive_index;
  } else {
    leg.clearance = checker_.check(from, radius_, leg.path, nullptr, &leg.prepared);
    leg.blocked_primitive = checker_.blocked_primitive;
  }
  leg.qualified = true;
  countStatus(leg.clearance, work);
  return leg.clear();
}

QualifiedLeg LegQualifier::leg(const State from, const State to, QueryWork &work) {
  auto result = price(from, to, work);
  qualify(result, from, work);
  return result;
}

audit::ContinuousClearanceResult LegQualifier::exactAudit(
    const QualifiedLeg &leg, const State from, QueryWork &work) const {
  auto result = audit::continuousPathClearance(
      map_, from, radius_, leg.path.primitives, leg.path.primitive_count);
  countExact(result, work);
  return result;
}

void LegQualifier::reportExactWork(QueryWork &work) const {
  work.exact_checks += checker_.exact_checks;
  work.clearance_intervals += checker_.intervals;
  work.clearance_events += checker_.events;
  work.free_box_attempts += checker_.free_box_attempts;
  work.free_box_cells += checker_.free_box_cells;
  work.free_box_clears += checker_.free_box_clears;
}

PivotConnector searchPivotHeading(LegQualifier &qualifier, const State A,
                                  const State pivot, const State C,
                                  const std::span<const double> extra_seeds,
                                  QueryWork &work) {
  PivotConnector best;
  best.pivot = pivot;
  if (!validInput(qualifier.map(), A, pivot, qualifier.radius()) || !validPose(C)) {
    ++work.incomplete;
    return best;
  }

  std::array<Trial, kHeadingSamples> coarse{};
  std::array<Trial, kRefinementStarts> starts{};
  ContactScratch contact{};
  work.contact_scratch_bytes = std::max<std::uint64_t>(
      work.contact_scratch_bytes, sizeof(contact));
  PivotConnector retained_best;
  retained_best.pivot = pivot;

  // A repeated seed heading (an ancestor's extra seed is its own pivot
  // heading) reuses its evaluation; its start is still inserted, so the start
  // list is unchanged.
  std::array<std::pair<double, PivotConnector>, 3> included{};
  std::size_t included_count = 0;
  const auto includeSeed = [&](const double psi) {
    if (!std::isfinite(psi)) {
      ++work.incomplete;
      return;
    }
    const PivotConnector *repeated = nullptr;
    for (std::size_t i = 0; i < included_count; ++i)
      if (std::bit_cast<std::uint64_t>(included[i].first) == std::bit_cast<std::uint64_t>(psi))
        repeated = &included[i].second;
    const auto candidate = repeated != nullptr ? *repeated
                                               : evaluate(qualifier, A, pivot, C, psi, work);
    if (repeated == nullptr && included_count < included.size())
      included[included_count++] = {psi, candidate};
    if (candidate.clear()) {
      retain(best, candidate);
      retain(retained_best, candidate);
      insertStart(starts, {psi, candidate.distance});
    }
  };
  includeSeed(pivot.theta);
  const std::size_t extras = std::min<std::size_t>(extra_seeds.size(), 2);
  for (std::size_t index = 0; index < extras; ++index) includeSeed(extra_seeds[index]);
  best.retained_control_distance = retained_best.distance;

  const auto coarseHeading = [&](const std::size_t index) {
    return A.theta + kTwoPi * static_cast<double>(index) /
                         static_cast<double>(kHeadingSamples);
  };
  for (std::size_t index = 0; index < coarse.size(); ++index) {
    const double psi = coarseHeading(index);
    const auto candidate = evaluate(qualifier, A, pivot, C, psi, work,
                                    std::numeric_limits<double>::infinity(),
                                    &contact.shapes[index], &contact.outgoing[index]);
    coarse[index] = {psi, candidate.distance};
    retain(best, candidate);
  }
  for (std::size_t index = 0; index < coarse.size(); ++index) {
    const Trial &previous = coarse[(index + coarse.size() - 1) % coarse.size()];
    const Trial &current = coarse[index];
    const Trial &next = coarse[(index + 1) % coarse.size()];
    if (std::isfinite(current.cost) && current.cost <= previous.cost &&
        current.cost <= next.cost)
      insertStart(starts, current);
  }

  // A witness is used only with an adjacent one of the same curve and gear,
  // so only such coarse headings receive the exact audit of their outgoing leg.
  const auto witness = [&](const std::size_t index) -> const ContactWitness & {
    if (contact.audited[index]) return contact.witnesses[index];
    contact.audited[index] = true;
    const std::size_t previous = (index + kHeadingSamples - 1) % kHeadingSamples;
    const std::size_t next = (index + 1) % kHeadingSamples;
    const auto shape = contact.shapes[index];
    if (shape == 0 || (contact.shapes[previous] != shape && contact.shapes[next] != shape))
      return contact.witnesses[index];
    State heading_pivot = pivot;
    heading_pivot.theta = coarseHeading(index);
    QualifiedLeg outgoing;
    outgoing.path.status = audit::LegacyReplayStatus::Replayed;
    outgoing.path.primitives = contact.outgoing[index].primitives;
    outgoing.path.primitive_count = contact.outgoing[index].count;
    ++work.witness_checks;
    const auto exact = qualifier.exactAudit(outgoing, heading_pivot, work);
    contact.witnesses[index] = contactWitness(outgoing, exact, qualifier.radius());
    return contact.witnesses[index];
  };

  std::size_t contact_evaluations = 0;
  const auto evaluateContact = [&](const double psi) {
    ++contact_evaluations;
    ++work.contact_evaluations;
    return evaluate(qualifier, A, pivot, C, psi, work);
  };
  const auto hasStart = [&] {
    return std::any_of(starts.begin(), starts.end(), [](const Trial start) {
      return std::isfinite(start.cost);
    });
  };
  if (!best.clear() && !hasStart()) {
    bool discovered = false;
    for (std::size_t index = 0;
         index < coarse.size() && !discovered &&
         contact_evaluations < kContactEvaluationLimit;
         ++index) {
      const std::size_t next = (index + 1) % coarse.size();
      const auto &left = witness(index);
      const auto &right = witness(next);
      if (!matchingWitnesses(left, right)) continue;
      const double lo = coarse[index].psi;
      const double hi = index + 1 == coarse.size()
          ? coarse[0].psi + kTwoPi : coarse[next].psi;
      std::size_t root_count = 0;
      appendContactRoots(contact.roots, root_count, left, pivot, C, qualifier.radius());
      appendContactRoots(contact.roots, root_count, right, pivot, C, qualifier.radius());
      wrapRootsIntoBracket(contact.roots, root_count, lo, hi);
      const bool interior = std::any_of(
          contact.roots.begin(), contact.roots.begin() + root_count,
          [lo, hi](const double root) { return root > lo && root < hi; });
      if (!interior) continue;
      for (std::size_t root = 1;
           root < root_count && !discovered &&
           contact_evaluations < kContactEvaluationLimit;
           ++root) {
        const auto candidate = evaluateContact(
            (contact.roots[root - 1] + contact.roots[root]) / 2.0);
        if (!candidate.clear()) continue;
        retain(best, candidate);
        insertStart(starts, {candidate.pivot.theta, candidate.distance});
        discovered = true;
      }
      for (std::size_t root = 1;
           root + 1 < root_count && !discovered &&
           contact_evaluations < kContactEvaluationLimit;
           ++root) {
        const auto candidate = evaluateContact(contact.roots[root]);
        if (!candidate.clear()) continue;
        retain(best, candidate);
        insertStart(starts, {candidate.pivot.theta, candidate.distance});
        discovered = true;
      }
    }
  }

  for (std::size_t start_index = 0; start_index < starts.size(); ++start_index) {
    const Trial start = starts[start_index];
    if (!std::isfinite(start.cost)) continue;
    // A repeated start (a repeated seed) would repeat the same walk: a walk
    // depends only on its start and the qualifier (whose first-arc
    // certificates can only have grown, never turning Clear into Blocked).
    bool walked = false;
    for (std::size_t earlier = 0; earlier < start_index; ++earlier)
      walked |= starts[earlier].psi == start.psi && starts[earlier].cost == start.cost;
    if (walked) continue;
    Trial current = start;
    double step = kTwoPi / static_cast<double>(kHeadingSamples);
    // After a move, the probe that lands exactly on the heading just left is
    // known to be worse and is skipped (-1: moved left, +1: moved right).
    int moved = 0;
    double previous_psi = std::numeric_limits<double>::quiet_NaN();
    for (std::size_t iteration = 0;
         iteration < kRefinementIterations && step >= kRefinementStop;
         ++iteration) {
      // best.distance <= current.cost, so a step that cannot beat the walker
      // cannot be retained either.
      PivotConnector left, right;
      const double left_psi = current.psi - step, right_psi = current.psi + step;
      if (!(moved == 1 && left_psi == previous_psi)) {
        left = evaluate(qualifier, A, pivot, C, left_psi, work, current.cost);
        ++work.refinement_evaluations;
      }
      if (!(moved == -1 && right_psi == previous_psi)) {
        right = evaluate(qualifier, A, pivot, C, right_psi, work, current.cost);
        ++work.refinement_evaluations;
      }
      retain(best, left);
      retain(best, right);
      const bool left_better = left.clear() && left.distance < current.cost;
      const bool right_better = right.clear() && right.distance < current.cost;
      if (left_better || right_better) {
        const bool go_left = left_better && (!right_better || left.distance < right.distance);
        const auto &next = go_left ? left : right;
        previous_psi = current.psi;
        current = {next.pivot.theta, next.distance};
        moved = go_left ? -1 : 1;
      } else {
        step *= 0.5;
        moved = 0;
      }
    }
  }
  best.retained_control_distance = retained_best.distance;
  return best;
}

double retainedControlDistance(LegQualifier &qualifier, const State A,
                               const State pivot, const State C,
                               const std::span<const double> extra_seeds,
                               QueryWork &work) {
  double best = std::numeric_limits<double>::infinity();
  if (!validInput(qualifier.map(), A, pivot, qualifier.radius()) || !validPose(C)) {
    ++work.incomplete;
    return best;
  }
  const auto includeSeed = [&](const double psi) {
    if (!std::isfinite(psi)) {
      ++work.incomplete;
      return;
    }
    const auto candidate = evaluate(qualifier, A, pivot, C, psi, work);
    if (candidate.clear() && candidate.distance < best) best = candidate.distance;
  };
  includeSeed(pivot.theta);
  const std::size_t extras = std::min<std::size_t>(extra_seeds.size(), 2);
  for (std::size_t index = 0; index < extras; ++index) includeSeed(extra_seeds[index]);
  return best;
}

} // namespace RS_marching::query
