#ifndef QUERY_RECONSTRUCTION_QUERY_HPP
#define QUERY_RECONSTRUCTION_QUERY_HPP

#include "audit/DirectStartClearance.hpp"
#include "query/GoalConnector.hpp"
#include "query/RetainedView.hpp"

#include <cstdint>
#include <limits>
#include <vector>

namespace RS_marching::query {

// Replays one retained pose chain from its stored words (or fresh selections
// when words is null) and checks the complete sum against accepted_distance.
// It never requalifies clearance: retained legs keep their march certificate.
[[nodiscard]] audit::ResolvedChainAuditResult replayRetainedChain(
    const std::vector<State> &poses, double accepted_distance, double radius,
    QueryWork *work = nullptr, const std::vector<std::uint32_t> *words = nullptr);

// The accepted record of one reached cell, rebuilt from retained ancestry and
// replayed from its stored words. valid is false for broken ancestry.
struct RetainedRecord {
  std::vector<State> poses;
  std::vector<int> source_ids;
  std::vector<std::uint32_t> words;
  audit::ResolvedChainAuditResult chain;
  bool word_match = false;
  bool valid = false;
};

// How far the physical length of a certified replay of a retained chain with
// `legs` legs can differ from its stored distance g: the pruning bound for
// retained scalars.
[[nodiscard]] double replayResidualBound(double g, std::size_t legs, double r);
// Conservative physical prefix envelope obtained before adding another leg.
[[nodiscard]] double replayPrefixLowerBound(double g, std::size_t legs, double r);
// Physical lower bound for a retained prefix plus a new leg's lower bound.
// Nonfinite intermediate arithmetic yields zero and cannot reject a candidate.
[[nodiscard]] double replayExtensionLowerBound(double g, std::size_t legs, double r,
                                              double leg_bound);

[[nodiscard]] RetainedRecord replayRetainedRecord(
    const RetainedView &view, int x, int y, QueryWork *work = nullptr,
    std::size_t *ancestry_visits = nullptr);

// A retained record whose terminal heading already equals the goal heading.
struct RecordControl {
  const std::vector<State> *poses = nullptr;
  const std::vector<int> *source_ids = nullptr;
  const audit::ResolvedChainAuditResult *chain = nullptr;
  std::size_t word_bytes = 0;
  bool word_match = false;
};

enum class GoalStatus : std::uint8_t { Clear, Unsupported, Incomplete };

struct GoalRoute {
  GoalStatus status = GoalStatus::Unsupported;
  std::vector<State> poses;
  std::vector<int> source_ids;
  audit::ResolvedChainAuditResult chain;
  bool prefix_word_match = false;
  const char *route_kind = "none";
  double best_control = std::numeric_limits<double>::infinity();
  // Candidate coverage and work.
  std::size_t sources_considered = 0;
  bool spatial_fallback = false;
  std::size_t spatial_source_visits = 0, spatial_sources_added = 0;
  std::size_t ancestry_visits = 0, prefix_legs = 0, prefix_failures = 0;
  std::size_t sources_pruned = 0, sources_bound_pruned = 0;
  std::size_t extension_cells = 0, extensions_priced = 0, extensions_qualified = 0;
  std::size_t extensions_ranked = 0; // Free URS ranking solves, included in QueryWork.
  std::size_t extension_rank = 0; // 1-based bound-order rank of an accepted extension
  std::size_t authority_fallbacks = 0;
  std::size_t peak_chain_payload = 0;
  bool endpoint_fallback = false;
  std::size_t endpoint_candidates = 0;
};

inline constexpr std::size_t kInitialSourceLimit = 12;
inline constexpr std::size_t kSourceLimit = kInitialSourceLimit + 1;
// Endpoint proposals: at most one per reached free neighbour of the goal cell.
inline constexpr std::size_t kEndpointLimit = 8;
// Record extensions consider reached cells within this Chebyshev radius of the
// goal, min(cap, ceil(1.5 r) + 2), and price at most kExtensionPriceLimit of
// them in lower-bound order.
inline constexpr int kExtensionWindowCap = 60;
inline constexpr std::size_t kExtensionPriceLimit = 64;

struct GoalQueryOptions {
  bool record_extensions = true;
  // Test seam: treat the Shared route's new legs as unconfirmed, forcing the
  // exact-authority repeat.
  bool reject_confirmation = false;
  // Free terminal heading: record/direct/record-extension families only.
  bool free_heading = false;
};

// Shared: the marcher's ConnectorClearance qualifies new legs and the exact
// continuous checker confirms the selected ones. Exact: the exact checker
// qualifies every new leg.
enum class QueryAuthority : std::uint8_t { Shared, Exact };

// True when the exact continuous checker finds each of the last new_legs
// segments of chain Clear; segment i starts at poses[i].
[[nodiscard]] bool confirmNewLegs(const Field<double> &map, double radius,
                                  const std::vector<State> &poses,
                                  const audit::ResolvedChainAuditResult &chain,
                                  std::size_t new_legs, QueryWork &work);

// Reconstruction query after one-to-all marching. free_heading ignores the
// goal heading and searches only record/direct/record-extension families;
// its bounded pricing order is not a minimum/completeness guarantee.
// Otherwise the goal is fully specified.
// Candidates: the matching retained record, the direct start leg, record
// extensions (a nearby cell's retained record + one new leg), and pivot
// connectors (retained prefix + two new legs through a searched heading) for
// the bounded source policy. When those families find none, endpoint
// proposals keep a reached neighbour's record up to its owner pivot and
// replace its last leg by two new legs through the neighbour cell with a
// searched heading. Under the Shared authority a route whose new legs
// fail confirmation is recomputed under the Exact authority; work counters
// then sum both passes and coverage describes the repeat. Nothing retained is
// written. Unsupported means no represented candidate qualified, never
// unreachability.
[[nodiscard]] GoalRoute reconstructGoal(const RetainedView &view, int target_x,
                                        int target_y, State goal,
                                        const RecordControl &record,
                                        QueryWork &work,
                                        const GoalQueryOptions &options = {},
                                        QueryAuthority authority = QueryAuthority::Shared);

// The fixed-goal answer on a retained state, with the goal cell's own
// retained record as a candidate when its heading equals the goal heading.
[[nodiscard]] GoalRoute answerFixedGoal(const RetainedView &view, int target_x,
                                        int target_y, State goal, QueryWork &work);

} // namespace RS_marching::query

#endif
