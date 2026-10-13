#ifndef DIRECT_START_AUDIT_HPP
#define DIRECT_START_AUDIT_HPP

#include "accelerated/Solver.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace RS_marching::audit {

inline constexpr double kReplayTolerance =
    64.0 * static_cast<double>(std::numeric_limits<float>::epsilon());

enum class LegacyUrsAuditStatus : std::uint8_t {
  InvalidInput = 0,
  Valid = 1,
  PartitionGap = 2,
  Nonfinite = 3
};

struct LegacyUrsAuditResult {
  LegacyUrsAuditStatus status = LegacyUrsAuditStatus::InvalidInput;
  double omega = std::numeric_limits<double>::quiet_NaN();

  [[nodiscard]] bool valid() const {
    return status == LegacyUrsAuditStatus::Valid;
  }
};

// "Legacy" identifies the inherited direct-start conventions and result
// envelope. Terminal-heading authority is the current shared URS planner.
// A native output left at +infinity is an unmatched partition; another
// nonfinite output is a native arithmetic failure. Nonvalid results retain
// their default NaN omega.
[[nodiscard]] LegacyUrsAuditResult legacyUrsAudit(
    double source_x, double source_y, double target_x, double target_y,
    double radius, double source_theta, bool offset = false);

struct LegacyPrimitive {
  char motion_type = 'N';
  int direction = 0;
  double length = 0.0;
};

// Call-local replay geometry for one exact literal path, source pose, and
// radius. It is valid only after replayLegacyPrimitives succeeds; callers must
// retain it only through the immediately following clearance check.
struct ReplayPrimitiveGeometry {
  double start_sine;
  double start_cosine;
  State literal_endpoint;
  State enclosure_endpoint;
};

struct ReplayPrimitiveBuffer {
  std::array<ReplayPrimitiveGeometry, 5> primitives;
  State source;
  double radius;
  std::array<LegacyPrimitive, 5> literal_primitives;
  std::size_t primitive_count;
  bool valid;

  ReplayPrimitiveBuffer() : primitive_count(0), valid(false) {}

  void reset() {
    primitive_count = 0;
    valid = false;
  }
};

enum class LegacyReplayStatus : std::uint8_t {
  InvalidInput = 0,
  UrsPartitionGap = 1,
  UrsNonfinite = 2,
  SolverInvalid = 3,
  PrimitiveMaterializationFailed = 4,
  ReplayMismatch = 5,
  Replayed = 6,
  CostPruned = 7
};

struct LegacyPathAuditResult {
  LegacyReplayStatus status = LegacyReplayStatus::InvalidInput;
  LegacyUrsAuditStatus urs_status = LegacyUrsAuditStatus::InvalidInput;
  double distance = std::numeric_limits<double>::infinity();
  double target_theta = std::numeric_limits<double>::quiet_NaN();
  int condition = 0;
  int quadrant = 0;
  std::array<LegacyPrimitive, 5> primitives{};
  std::size_t primitive_count = 0;
  State endpoint{std::numeric_limits<double>::quiet_NaN(),
                 std::numeric_limits<double>::quiet_NaN(),
                 std::numeric_limits<double>::quiet_NaN()};
  double endpoint_position_residual =
      std::numeric_limits<double>::infinity();
  double endpoint_heading_residual =
      std::numeric_limits<double>::infinity();
  double distance_residual = std::numeric_limits<double>::infinity();

  [[nodiscard]] bool replayed() const {
    return status == LegacyReplayStatus::Replayed;
  }
};

// A retained free-heading leg reconstructs with the same direct URS constructor.
// Existing packed words occupy at most twenty bits.
inline constexpr std::uint32_t kFreeHeadingWord = 0x80000000U;
inline constexpr std::uint32_t kSuppliedHeadingWord = 0x40000000U;
[[nodiscard]] std::uint32_t selectedPathWord(const LegacyPathAuditResult &path);
[[nodiscard]] LegacyPathAuditResult solveFreeHeadingCandidate(
    State from, double target_x, double target_y, double radius,
    double distance_limit = std::numeric_limits<double>::infinity(),
    double complete_prefix = 0.0,
    double incumbent_cutoff = std::numeric_limits<double>::infinity(),
    ReplayPrimitiveBuffer *prepared = nullptr);
[[nodiscard]] LegacyPathAuditResult reconstructRetainedLeg(
    State from, State to, double radius, std::uint32_t word);

// Uses the shared URS terminal heading followed by the public ARS selector at
// the true world query. The source_theta argument is the public marcher's
// stored light-source heading (world start heading minus pi/2).
[[nodiscard]] LegacyPathAuditResult solveDirectLegacyCandidate(
    double source_x, double source_y, double legacy_source_theta,
    double target_x, double target_y, double radius);

// Materializes and independently replays an already completed selector state.
// Stack word export reads the supplied Solver without mutating it or allocating
// vector storage.
[[nodiscard]] LegacyPathAuditResult replaySelectedLegacyPath(
    const Solver &selected_solver, State selector_from, State selector_to,
    double selected_distance, double radius, int condition, int quadrant,
    ReplayPrimitiveBuffer *prepared = nullptr);

[[nodiscard]] std::uint64_t
legacyPrimitiveIdentityHash(const LegacyPathAuditResult &result);

[[nodiscard]] State replayLegacyPrimitives(
    State source, double radius,
    const std::array<LegacyPrimitive, 5> &primitives,
    std::size_t primitive_count, ReplayPrimitiveBuffer *prepared = nullptr);

} // namespace RS_marching::audit

#endif
