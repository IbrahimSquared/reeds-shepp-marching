#ifndef MARCHING_HPP
#define MARCHING_HPP

#include <array>
#include <functional>
#include <map>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <queue>
#include <string>
#include <vector>

#include "accelerated/Solver.hpp"
#include "audit/DirectStartAudit.hpp"
#include "audit/DirectStartClearance.hpp"
#include "environment/Environment.hpp"
#include "geometry/SameSideTripletPreparation.hpp"
#include "geometry/ConnectorClearance.hpp"
#include "underspecified/UnderSpecifiedRSPlanner.hpp"

const double pi = 3.1415926535897932;

namespace RS_marching {

namespace query {
class RetainedView;
struct GoalRoute;
}

struct triplet {
  double distance;
  double theta;
  int source;
  double pivot_theta = 0.0;
  double incoming_distance = 0.0;
  std::uint32_t incoming_word = 0, outgoing_word = 0;
};

struct Point {
  double x;
  double y;
};

class Marching {
public:
  explicit Marching(Environment &env);

  // Deconstructor
  ~Marching() = default;

  // A Marching instance owns one run; construct a fresh instance to restart.
  // Repeated calls are rejected before any retained state is changed.
  void runMarching();

#ifdef RS_INTERACTIVE_API
  // Retained field state only: neither call changes marching decisions or fields.
  [[nodiscard]] std::string geometricSnapshotJson() const;
  // prior: a clear fixed-goal route found earlier on this state (a direct
  // query's stop evaluation), returned when shorter than the final search's.
  [[nodiscard]] std::string queryGeometricPathJson(
      int target_x, int target_y, bool check_collisions,
      double sample_step, std::optional<double> goal_heading = std::nullopt,
      const query::GoalRoute *prior = nullptr) const;
  // Direct queries only: called with each settled pop key; returning true ends
  // the march there, without deferred completion. Unset for one-to-all runs.
  void setMarchStop(std::function<bool(double)> stop) { march_stop_ = std::move(stop); }
  [[nodiscard]] bool marchStopped() const { return march_stopped_; }
  // Direct queries only: queue keys become g + heuristic(x, y) (A* order).
  // Unset, keys are g as in the one-to-all march.
  void setMarchHeuristic(std::function<double(int, int)> heuristic) {
    march_heuristic_ = std::move(heuristic);
  }
  [[nodiscard]] double queueKey(const int x, const int y, const double g) const {
    return march_heuristic_ ? g + march_heuristic_(x, y) : g;
  }
#endif

  friend struct TripletDispatchContractAccess;
  friend struct GeometricIncomingReuseContractAccess;
  friend class query::RetainedView;
#ifdef RS_CUSTOM_QUEUE_POP
  friend struct MarchingQueueContractAccess;
#endif

private:
  bool run_started_ = false;
#ifdef RS_INTERACTIVE_API
  std::function<bool(double)> march_stop_;
  std::function<double(int, int)> march_heuristic_;
  bool march_stopped_ = false;
#endif
  // use gradient descent by default instead of bisection method
  int GD_bisection_ = 1;
  void reset();
  void saveLightSources() const;

  struct PreparedTriplet {
    double theta1;
    double pivot_x;
    double pivot_y;
    double incoming_cos;
    double incoming_sin;
    double pivot_bearing;
    double outgoing_cos;
    double outgoing_sin;
    geometry::SameSideTripletPreparation same_side_parent;
  };

  // Keep uncertain clearance distinct from a geometric obstruction and a price cutoff.
  enum class LegStatus : std::uint8_t {
    Invalid = static_cast<std::uint8_t>(audit::ContinuousClearanceStatus::Invalid),
    Incomplete = static_cast<std::uint8_t>(audit::ContinuousClearanceStatus::Incomplete),
    Indeterminate = static_cast<std::uint8_t>(audit::ContinuousClearanceStatus::Indeterminate),
    Blocked = static_cast<std::uint8_t>(audit::ContinuousClearanceStatus::Blocked),
    Clear = static_cast<std::uint8_t>(audit::ContinuousClearanceStatus::Clear),
    PriceRefused
  };

  struct PivotPrefix {
    State preceding_pose{};
    double distance = std::numeric_limits<double>::infinity();
    audit::ResolvedChainStatus clearance = audit::ResolvedChainStatus::Invalid;
    bool reconstructed = false;
    geometry::FirstArcClear preceding_arcs{}, birth_arcs{};
    // Fixed geometry belongs to the installed prefix; append resets reuse.
    // Only the exact pivot heading varies within this immutable-scene march.
    struct IncomingLegCache {
      std::uint64_t heading = 0;
      double distance = std::numeric_limits<double>::infinity();
      std::uint32_t word = 0;
      bool eligible = false;
      LegStatus status = LegStatus::Invalid;
      bool valid = false;
    } incoming_leg_cache{};
    Solver::PlanarPreparation incoming_planar{};
    int parent_source = -1;
    int birth_x = -1, birth_y = -1;
    std::uint32_t incoming_word = 0;
    double birth_theta = std::numeric_limits<double>::quiet_NaN();
    double birth_incoming_distance = std::numeric_limits<double>::infinity();
    std::uint32_t birth_incoming_word = 0;
    // Geometric pivot position when it differs from the owning cell centre.
    double pivot_x = std::numeric_limits<double>::quiet_NaN();
    double pivot_y = std::numeric_limits<double>::quiet_NaN();
    PreparedTriplet prepared_triplet{};
  };

  struct SelectorMemoEntry {
    int source = -1;
    bool selected = false;
    bool outgoing_blocked = false;
    std::uint64_t target = 0;
    State pivot{};
    State target_state{};
  };
  static constexpr std::size_t selector_memo_capacity_ = 8192;

  /*!
   * @brief queues sources
   */
  inline void queuePotentialSources(std::vector<size_t> &potentialSources,
                                    const int neighbour_x,
                                    const int neighbour_y);

  /*!
   * @brief gets distances
   */
  void getPotentialDistances(const std::vector<size_t> &potentialSources,
                                    std::vector<triplet> &potentialDistances,
                                    const int from_x, const int from_y,
                                    const int neighbour_x,
                                    const int neighbour_y,
                                    bool allow_rescue = false);

  std::shared_ptr<Field<double>> sharedVisibilityField_;
  std::shared_ptr<Config> sharedConfig_;

  Field<double> gScore_;
  Field<double> thetas_;
  Field<int> cameFrom_;
  Field<bool> isUpdated_;
  // Installed sources plus at most one reusable provisional slot.
  std::vector<pose> lightSources_;
  int nb_of_iterations_ = 0;
  size_t nb_of_sources_ = 0;
  std::uint64_t pivot_births_ = 0;
  std::uint64_t queue_pushes_ = 0;
  std::uint64_t queue_pops_ = 0;
  std::uint64_t max_queue_size_ = 0;
  std::uint64_t local_source_candidates_ = 0;
  std::vector<PivotPrefix> pivot_prefixes_;
  std::uint64_t prefix_vector_allocations_ = 0;

  // Neighbours
  // [1 0; 0 1; -1 0; 0 -1; 1 1; -1 1; -1 -1; 1 -1] flattened out
  const int neighbours_[16] = {1, 0, 0,  1, -1, 0,  0, -1,
                               1, 1, -1, 1, -1, -1, 1, -1};

  [[nodiscard]] bool selectTripletHeadings(
      const State &start, const Point &pivot_position,
      const Point &target_position, bool final_triplet, State &pivot,
      State &target, std::array<double, 2> &final_segments,
      const PreparedTriplet *prepared_triplet = nullptr);
  [[nodiscard]] PreparedTriplet prepareTripletFrame(
      const State &start, const Point &pivot_position);
  void appendPivotPrefix(const PivotPrefix &prefix);
  geometry::ConnectorClearance geometric_clearance_;
  Field<double> geometric_pivot_theta_, geometric_incoming_distance_;
  Field<std::uint32_t> geometric_incoming_word_, geometric_outgoing_word_;
  std::uint64_t geometric_solves_ = 0, geometric_retargets_ = 0;
  std::uint64_t geometric_fixed_continuations_ = 0, geometric_birth_attempts_ = 0;
  std::uint64_t geometric_cost_prunes_ = 0, geometric_incumbent_prunes_ = 0;
  std::uint64_t geometric_reoffers_ = 0, geometric_improvements_ = 0;
  std::uint64_t geometric_stale_pops_ = 0;
  std::uint64_t geometric_incoming_cache_lookups_ = 0;
  std::uint64_t geometric_incoming_cache_hits_ = 0;
  std::uint64_t geometric_inherited_heading_attempts_ = 0;
  std::uint64_t geometric_inherited_heading_wins_ = 0;
  std::uint64_t geometric_inherited_incoming_reused_ = 0;
  std::uint64_t geometric_vertex_trials_ = 0, geometric_vertex_births_ = 0;
  std::uint64_t geometric_vertex_reuses_ = 0;
  std::uint64_t geometric_contact_searches_ = 0, geometric_contact_steps_ = 0;
  std::uint64_t geometric_contact_wins_ = 0;
  std::uint64_t geometric_pose_attempts_ = 0, geometric_pose_wins_ = 0;
  std::uint64_t geometric_bypass_max_depth_ = 0;
  std::uint64_t geometric_sweeps_ = 0, geometric_sweep_samples_ = 0;
  std::map<std::array<int, 3>, int> geometric_vertex_children_;
  std::uint64_t geometric_bypass_attempts_ = 0;
  std::uint64_t geometric_bypass_wins_ = 0;
  std::uint64_t geometric_bypass_rotation_probes_ = 0, geometric_bypass_rotations_ = 0;
  std::uint64_t geometric_child_bound_skips_ = 0;
  std::uint64_t geometric_rescue_attempts_ = 0;
  std::uint64_t geometric_rescue_wins_ = 0;
  std::uint64_t geometric_failed_frontier_count_ = 0;
  std::uint64_t geometric_ordinary_sources_ = 0;
  std::uint64_t geometric_failed_frontier_capacity_bytes_ = 0;
  std::uint64_t geometric_completion_unique_seeds_ = 0;
  std::uint64_t geometric_completion_enqueued_seeds_ = 0;
  std::uint64_t geometric_completion_pops_ = 0;
  std::uint64_t geometric_completion_writes_ = 0;
  std::uint64_t geometric_selector_cache_lookups_ = 0;
  std::uint64_t geometric_selector_cache_hits_ = 0;
  std::uint64_t geometric_selected_blocked_reuses_ = 0;
  std::vector<SelectorMemoEntry> geometric_selector_cache_;
  std::array<std::uint64_t, 5> geometric_statuses_{};
  void resetGeometricSelectorCache();
  [[nodiscard]] static std::uint64_t geometricSelectorCacheHash(
      std::uint64_t source, std::uint64_t target);
  [[nodiscard]] bool selectGeometricTripletHeadings(
      const PivotPrefix &prefix, int source, int x, int y, State &pivot,
      State &target, SelectorMemoEntry **memo = nullptr);
  [[nodiscard]] LegStatus geometricLeg(
      State from, State to, double &distance, std::uint32_t &word,
      double distance_limit = std::numeric_limits<double>::max(),
      double complete_prefix = std::numeric_limits<double>::infinity(),
      double incumbent_cutoff = std::numeric_limits<double>::infinity(),
      geometry::FirstArcClear *owned_arcs = nullptr,
      const Solver::PlanarPreparation *incoming_planar = nullptr);
  [[nodiscard]] LegStatus geometricFreeHeadingLeg(
      State from, State &to, double &distance, std::uint32_t &word,
      double distance_limit = std::numeric_limits<double>::max(),
      double complete_prefix = std::numeric_limits<double>::infinity(),
      double incumbent_cutoff = std::numeric_limits<double>::infinity(),
      double fallback_orientation = std::numeric_limits<double>::quiet_NaN(),
      geometry::FirstArcClear *owned_arcs = nullptr);
  [[nodiscard]] LegStatus geometricIncomingLeg(
      int source, double pivot_heading, double &distance, std::uint32_t &word,
      double distance_limit = std::numeric_limits<double>::max());
  [[nodiscard]] triplet geometricAnalyticRescueCandidate(
      int source, int x, int y,
      double incumbent_cutoff = std::numeric_limits<double>::infinity());
  [[nodiscard]] Point geometricPivotPosition(int source) const;
  // Immutable ancestry, in forward order; the root has no incoming leg.
  [[nodiscard]] bool geometricPrefixChain(
      int source, std::vector<State> &poses, std::vector<std::uint32_t> &words,
      std::vector<int> *source_ids = nullptr, std::size_t *visits = nullptr) const;
  [[nodiscard]] bool geometricRecordChain(
      int x, int y, std::vector<State> &poses, std::vector<std::uint32_t> &words,
      std::vector<int> *source_ids = nullptr, std::size_t *visits = nullptr) const;
  std::size_t geometricHeadingSeeds(int source, int x, int y, int offer_x, int offer_y,
                                    bool clear_selected, double selected_heading,
                                    std::array<std::pair<double, int>, 8> &seeds) const;
  void geometricContactHeading(PivotPrefix &prefix, int source, State target,
                               double blocked_heading, double incumbent_cutoff,
                               triplet &best, double nearest_blocked);
  [[nodiscard]] triplet geometricPoseCandidate(int source, int x, int y,
                                               double incumbent_cutoff);
  [[nodiscard]] triplet geometricHeadingSweep(int source, int x, int y,
                                              double incumbent_cutoff);
  [[nodiscard]] triplet geometricCandidate(
      int source, int x, int y,
      double incumbent_cutoff = std::numeric_limits<double>::infinity(),
      const Point *point_target = nullptr, int offer_x = -1, int offer_y = -1);
  void geometricCandidates(const std::vector<size_t> &sources,
                            std::vector<triplet> &distances,
                            int from_x, int from_y, int x, int y,
                            bool allow_rescue = false);
  void runGeometricAudit();
  // Dimensions.
  size_t ny_;
  size_t nx_;

  /*!
   * @brief Checks if requested cell is in grid
   * @param [in] x x position of the cell
   * @param [in] y y position of the cell
   * @return true if cell is in grid, false otherwise
   */
  inline bool isValid(const size_t x, const size_t y) const {
    return ((x < nx_) && (y < ny_));
  }

#ifdef RS_CUSTOM_QUEUE_POP
  // The march's open set: std::priority_queue's binary heap with a pop whose
  // child selection is written branch-free. The pop order, including ties,
  // is exactly libstdc++'s (GCC 13) pop_heap, so fields stay bit-identical;
  // only the data-dependent comparison no longer compiles to a mispredicted
  // jump. Other standard libraries may order ties differently.
  struct OpenSet : std::priority_queue<Node> {
    Node popTop(); // Requires nonempty queue; NaN keys are outside the comparator domain.
  };
  std::unique_ptr<OpenSet> openSet_;
#else
  std::unique_ptr<std::priority_queue<Node>> openSet_;
#endif

  // RS related
  UnderSpecifiedRSPlanner urs_;
  double r_ = 1;
  Solver solver_;

  int exit_it = 10;
};

} // namespace RS_marching
#endif
