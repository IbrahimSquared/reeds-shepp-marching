#include "marching/Marching.hpp"
#include "marching/MarchingInternal.hpp"
#include "geometry/TripletDistance.hpp"
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <ostream>

// Queue key of a record: g, or g + heuristic in an API direct query.
#ifdef RS_INTERACTIVE_API
#define RS_QUEUE_KEY(x, y, g) queueKey((x), (y), (g))
#else
#define RS_QUEUE_KEY(x, y, g) (g)
#endif

namespace RS_marching {

Marching::PreparedTriplet Marching::prepareTripletFrame(
    const State &start, const Point &pivot_position) {
  const double x1 = start.x;
  const double y1 = start.y;
  const double theta1 = wrapTo2Pi(start.theta);
  double omega_1;
  urs_.getOmega(x1, y1, pivot_position.x, pivot_position.y, r_, theta1,
                omega_1, true);
  omega_1 += theta1 - pi / 2;
  const double incoming_cos = cos(theta1 - pi / 2);
  const double incoming_sin = sin(theta1 - pi / 2);
  const double dx2 = x1 - pivot_position.x;
  const double dy2 = y1 - pivot_position.y;
  const double pivot_x = x1 - incoming_cos * dx2 - incoming_sin * dy2;
  const double pivot_y = y1 - incoming_cos * dy2 + incoming_sin * dx2;
  const geometry::SameSideTransform same_side_transform =
      pivot_x >= x1
          ? (pivot_y <= y1 ? geometry::SameSideTransform::HeadingGearDual
                           : geometry::SameSideTransform::Mirror)
          : (pivot_y <= y1
                 ? geometry::SameSideTransform::MirrorHeadingGearDual
                 : geometry::SameSideTransform::Identity);
  const geometry::SameSideTripletPreparation same_side_parent =
      geometry::prepareSameSideTripletParent(
          {start.x, start.y, theta1}, {pivot_position.x, pivot_position.y},
          {pivot_x, pivot_y}, r_, same_side_transform);
  return {theta1, pivot_x, pivot_y, incoming_cos, incoming_sin,
          atan2(pivot_y - y1, pivot_x - x1), cos(omega_1 - pi / 2),
          sin(omega_1 - pi / 2), same_side_parent};
}

bool Marching::selectTripletHeadings(
    const State &start, const Point &pivot_position,
    const Point &target_position, const bool final_triplet, State &pivot,
    State &target, std::array<double, 2> &final_segments,
    const PreparedTriplet *prepared_triplet) {
  const double x1 = start.x;
  const double y1 = start.y;
  const PreparedTriplet prepared_on_demand =
      prepared_triplet == nullptr ? prepareTripletFrame(start, pivot_position)
                                  : PreparedTriplet{};
  const PreparedTriplet &prepared =
      prepared_triplet == nullptr ? prepared_on_demand : *prepared_triplet;
  const double theta1 = prepared.theta1;
  double x2 = prepared.pivot_x;
  double y2 = prepared.pivot_y;
  double x3 = target_position.x;
  double y3 = target_position.y;

  const double cos_theta1_minus_pi_2 = prepared.incoming_cos;
  const double sin_theta1_minus_pi_2 = prepared.incoming_sin;
  const double dx3 = x1 - x3;
  const double dy3 = y1 - y3;

  x3 = x1 - cos_theta1_minus_pi_2 * dx3 - sin_theta1_minus_pi_2 * dy3;
  y3 = y1 - cos_theta1_minus_pi_2 * dy3 + sin_theta1_minus_pi_2 * dx3;

  const double theta2 =
      0.5 * wrapTo2Pi(atan2(y3 - y2, x3 - x2) + prepared.pivot_bearing);
  const State s1{x1, y1, theta1};
  pivot = State{x2, y2, theta2};
  target = State{x3, y3, 0};
  final_segments.fill(std::numeric_limits<double>::infinity());

  // x3/y3 are chart-rotated; dot the physical URS orientation with the
  // physical target coordinates.
  const double x3_t =
      -prepared.outgoing_cos * (pivot_position.x - target_position.x) -
      prepared.outgoing_sin * (pivot_position.y - target_position.y);
  const auto solve_mixed_side =
      [&](const geometry::SameSideTransform transform) {
        const auto status = geometry::selectMixedSideTripletChartHeadings(
            s1, {pivot_position.x, pivot_position.y},
            {target_position.x, target_position.y}, {pivot.x, pivot.y},
            {target.x, target.y}, pivot.theta, r_, transform, solver_,
            final_triplet, pivot, target, &final_segments);
        return status ==
               (final_triplet
                    ? geometry::MixedSideTripletStatus::HeadingsSelected
                    : geometry::MixedSideTripletStatus::PivotSelected);
      };
  const auto solve_same_side =
      [&](const geometry::SameSideTransform transform) {
        const auto status = geometry::selectAdjacentTripletChartHeadings(
            s1, {pivot_position.x, pivot_position.y},
            {target_position.x, target_position.y}, {pivot.x, pivot.y},
            {target.x, target.y}, pivot.theta, r_, transform,
            GD_bisection_ == 2, exit_it, solver_, final_triplet, pivot, target,
            final_segments, &prepared.same_side_parent);
        return status ==
               (final_triplet
                    ? geometry::SameSideTripletStatus::HeadingsSelected
                    : geometry::SameSideTripletStatus::PivotSelected);
      };

  if (x2 >= x1) {
    if (y2 <= y1) {
      return x3_t >= 0
                 ? solve_same_side(
                       geometry::SameSideTransform::HeadingGearDual)
                 : solve_mixed_side(
                       geometry::SameSideTransform::MirrorHeadingGearDual);
    }
    return x3_t >= 0
               ? solve_same_side(geometry::SameSideTransform::Mirror)
               : solve_mixed_side(geometry::SameSideTransform::Identity);
  }
  if (y2 <= y1) {
    return x3_t >= 0
               ? solve_mixed_side(geometry::SameSideTransform::HeadingGearDual)
               : solve_same_side(
                     geometry::SameSideTransform::MirrorHeadingGearDual);
  }
  return x3_t >= 0
             ? solve_mixed_side(geometry::SameSideTransform::Mirror)
             : solve_same_side(geometry::SameSideTransform::Identity);
}

void Marching::appendPivotPrefix(const PivotPrefix &prefix) {
  if (pivot_prefixes_.size() == pivot_prefixes_.capacity()) {
    ++prefix_vector_allocations_;
  }
  PivotPrefix appended = prefix;
  appended.incoming_leg_cache.valid = false;
  appended.incoming_leg_cache.eligible = false;
  appended.incoming_planar = {};
  appended.preceding_arcs = {};
  appended.birth_arcs = {};
  if (appended.parent_source >= 0) {
    const auto &light = lightSources_[pivot_prefixes_.size()];
    const Point position = std::isfinite(appended.pivot_x)
        ? Point{appended.pivot_x, appended.pivot_y}
        : Point{static_cast<double>(light.x), static_cast<double>(light.y)};
    appended.prepared_triplet = prepareTripletFrame(appended.preceding_pose, position);
    appended.incoming_leg_cache.eligible =
        std::isfinite(appended.preceding_pose.x) &&
        std::isfinite(appended.preceding_pose.y) &&
        std::isfinite(appended.preceding_pose.theta) &&
        std::isfinite(position.x) && std::isfinite(position.y) && std::isfinite(r_);
    if (appended.incoming_leg_cache.eligible)
      appended.incoming_planar = Solver::preparePlanar(
          appended.preceding_pose.x, appended.preceding_pose.y, position.x,
          position.y, appended.preceding_pose.theta, r_);
  }
  pivot_prefixes_.push_back(std::move(appended));
}

void Marching::saveLightSources() const {
  std::string outputFilePath = "./output/lightSources.txt";
  // Check if the directory exists, and create it if it doesn't
  namespace fs = std::filesystem;
  fs::path directory = fs::path(outputFilePath).parent_path();
  if (!fs::exists(directory)) {
    if (!fs::create_directories(directory)) {
      std::cerr << "Failed to create directory " << directory.string()
                << std::endl;
      return;
    }
  }

  std::fstream of(outputFilePath, std::ios::out | std::ios::trunc);
  if (!of.is_open()) {
    std::cerr << "Failed to open output file " << outputFilePath << std::endl;
    return;
  }
  std::ostream &os = of;
  os << std::setprecision(std::numeric_limits<double>::max_digits10);
  for (size_t i = 0; i < nb_of_sources_; ++i) {
    // The actual pivot position: corner (vertex) children pivot off-cell. The
    // third column remains the legacy physical-heading-minus-pi/2 value.
    const Point position = geometricPivotPosition(static_cast<int>(i));
    os << position.x << " " << position.y << " " << lightSources_[i].theta << std::endl;
  }
  of.close();
  if (!sharedConfig_->silent) {
    std::cout << "\nSaved light sources" << std::endl;
  }
}

Marching::Marching(Environment &env)
    : sharedVisibilityField_(env.getVisibilityField()),
      sharedConfig_(env.getConfig()) {
  nx_ = sharedVisibilityField_->nx();
  ny_ = sharedVisibilityField_->ny();
  r_ = sharedConfig_->radius;
  exit_it = sharedConfig_->exit_it;
  GD_bisection_ = sharedConfig_->GD_bisection;
  reset();
  const auto started = std::chrono::high_resolution_clock::now();
  geometric_clearance_.reset(*sharedVisibilityField_);
  geometric_pivot_theta_.reset(nx_, ny_, 0.0);
  geometric_incoming_distance_.reset(nx_, ny_, 0.0);
  geometric_incoming_word_.reset(nx_, ny_, 0);
  geometric_outgoing_word_.reset(nx_, ny_, 0);
  std::cout << "Memoization time in us: "
            << durationInMicroseconds(started, std::chrono::high_resolution_clock::now())
            << "us" << std::endl;
}

void Marching::reset() {
  gScore_.reset(nx_, ny_, std::numeric_limits<double>::max());
  thetas_.reset(nx_, ny_, 0);
  cameFrom_.reset(nx_, ny_, -1);
  isUpdated_.reset(nx_, ny_, false);

  lightSources_.clear();
  lightSources_.resize(1);

  openSet_.reset();

#ifdef RS_CUSTOM_QUEUE_POP
  openSet_ = std::make_unique<OpenSet>();
#else
  openSet_ = std::make_unique<std::priority_queue<Node>>();
#endif

  nb_of_sources_ = 0;
  nb_of_iterations_ = 0;
  pivot_births_ = 0;
  queue_pushes_ = 0;
  queue_pops_ = 0;
  geometric_reoffers_ = geometric_improvements_ = geometric_stale_pops_ = 0;
  geometric_incumbent_prunes_ = 0;
  geometric_inherited_heading_attempts_ = geometric_inherited_heading_wins_ = 0;
  geometric_vertex_trials_ = geometric_vertex_births_ = geometric_vertex_reuses_ = 0;
  geometric_contact_searches_ = geometric_contact_steps_ = geometric_contact_wins_ = 0;
  geometric_pose_attempts_ = geometric_pose_wins_ = 0;
  geometric_bypass_max_depth_ = 0;
  geometric_sweeps_ = geometric_sweep_samples_ = 0;
  geometric_inherited_incoming_reused_ = 0;
  geometric_bypass_attempts_ = geometric_bypass_wins_ = 0;
  geometric_bypass_rotation_probes_ = geometric_bypass_rotations_ = 0;
  geometric_child_bound_skips_ = 0;
  geometric_rescue_attempts_ = geometric_rescue_wins_ = 0;
  geometric_failed_frontier_count_ = geometric_ordinary_sources_ = 0;
  geometric_failed_frontier_capacity_bytes_ = 0;
  geometric_completion_unique_seeds_ = 0;
  geometric_completion_enqueued_seeds_ = 0;
  geometric_completion_pops_ = 0;
  geometric_completion_writes_ = 0;
  geometric_selector_cache_lookups_ = 0;
  geometric_selector_cache_hits_ = 0;
  geometric_selected_blocked_reuses_ = 0;
  resetGeometricSelectorCache();
  max_queue_size_ = 0;
  local_source_candidates_ = 0;
  prefix_vector_allocations_ = 0;
  geometric_incoming_cache_lookups_ = 0;
  geometric_incoming_cache_hits_ = 0;
  pivot_prefixes_.clear();
  geometric_vertex_children_.clear();
}

#ifdef RS_CUSTOM_QUEUE_POP
// libstdc++'s (GCC 13) __pop_heap/__adjust_heap/__push_heap for Node's order,
// with the child comparison as arithmetic. The comparisons spell out
// Node::operator< (a node is "less" when its key is larger), which push still
// uses through std::less: change both together. Requires a non-empty queue.
Node Marching::OpenSet::popTop() {
  auto &heap = c;
  const Node top = heap.front();
  const std::ptrdiff_t len = static_cast<std::ptrdiff_t>(heap.size()) - 1;
  if (len > 0) {
    const Node value = heap.back();
    std::ptrdiff_t hole = 0, child = 0;
    while (child < (len - 1) / 2) {
      child = 2 * (child + 1);
      child -= static_cast<std::ptrdiff_t>(heap[child].f > heap[child - 1].f);
      heap[hole] = heap[child];
      hole = child;
    }
    if ((len & 1) == 0 && child == (len - 2) / 2) {
      child = 2 * (child + 1);
      heap[hole] = heap[child - 1];
      hole = child - 1;
    }
    for (std::ptrdiff_t parent = (hole - 1) / 2;
         hole > 0 && heap[parent].f > value.f; parent = (hole - 1) / 2) {
      heap[hole] = heap[parent];
      hole = parent;
    }
    heap[hole] = value;
  }
  heap.pop_back();
  return top;
}
#endif

void Marching::runMarching() {
  if (run_started_) throw std::logic_error("Marching instances support one run");
  run_started_ = true;
#ifdef RS_INTERACTIVE_API
  march_stopped_ = false;
#endif
  // Validate every caller-supplied heading before randomization or root writes.
  // This applies equally to Config users that do not pass through the parser.
  for (const Node &initial : sharedConfig_->initialFrontline) {
    if (!std::isfinite(initial.f))
      throw std::invalid_argument("initial pose headings must be finite");
  }
  // Validate before randomization or any root/source writes.
  if (sharedConfig_->pose_randomness || sharedConfig_->initialFrontline.size() != 1)
    throw std::invalid_argument("geometric marcher requires exactly one initial pose");
  auto startTime = std::chrono::high_resolution_clock::now();
  // Init
  int x = 0, y = 0, neighbour_x = 0, neighbour_y = 0;
  double theta = 0;

  // Large finite degree values are reduced before conversion to radians.
  std::vector<Node> initial_frontline = sharedConfig_->initialFrontline;
  for (Node &initial : initial_frontline)
    initial.f = std::remainder(initial.f, 360.0);
  double g = 0;
  if (initial_frontline.size() > nx_ * ny_)
    throw std::invalid_argument("initial poses exceed source capacity");

  for (size_t i = 0; i < initial_frontline.size(); ++i) {
    x = initial_frontline[i].x;
    y = initial_frontline[i].y;
    theta = wrapToPi(initial_frontline[i].f * pi / 180 - pi / 2);
    std::cout << "Initial position: " << x << ", " << y << ", "
              << (theta + pi / 2) * 180 / pi << std::endl;

    // check if starting positions are inside the map
    if (!isValid(x, y)) {
      throw std::invalid_argument("initial position is outside the map");
    }

    if (sharedVisibilityField_->get(x, y) < 1) {
      throw std::invalid_argument("initial position is occupied");
    }

    g = 0;
    gScore_(x, y) = g;
    thetas_(x, y) = theta + pi / 2;
    isUpdated_(x, y) = true;
    cameFrom_(x, y) = nb_of_sources_;
    lightSources_[nb_of_sources_] = {x, y, theta};
    appendPivotPrefix(PivotPrefix{State{static_cast<double>(x),
                                       static_cast<double>(y), theta + pi / 2},
                                  0.0, audit::ResolvedChainStatus::Clear,
                                  true});
    openSet_->push(Node{x, y, RS_QUEUE_KEY(x, y, g)});
    ++queue_pushes_;
    max_queue_size_ =
        std::max<std::uint64_t>(max_queue_size_, openSet_->size());

    ++nb_of_sources_;
    ++nb_of_iterations_;
  }

  // For queing unique sources from neighbours of neighbour
  std::vector<size_t> potentialSources;
  potentialSources.reserve(10);
  std::vector<triplet> potentialDistances;
  potentialDistances.reserve(10);

  double distance = 0;

  std::vector<std::pair<int, int>> failed_frontier;
  bool completion_phase = false;
  while (true) {
    while (openSet_->size() > 0) {
#ifdef RS_CUSTOM_QUEUE_POP
    const auto current = openSet_->popTop();
#else
    const auto current = openSet_->top();
    openSet_->pop();
#endif
    x = current.x;
    y = current.y;
    ++queue_pops_;
    if (completion_phase)
      ++geometric_completion_pops_;
    if (current.f != RS_QUEUE_KEY(x, y, gScore_(x, y))) {
      ++geometric_stale_pops_;
      continue;
    }
#ifdef RS_INTERACTIVE_API
    if (march_stop_ && march_stop_(current.f)) {
      march_stopped_ = true;
      break;
    }
#endif

    // Expand frontline at current & update neighbours
    for (size_t j = 0; j < 16; j += 2) {
      // NOTE those are always positive
      neighbour_x = x + neighbours_[j];
      neighbour_y = y + neighbours_[j + 1];

      // Box check
      if (!isValid(neighbour_x, neighbour_y)) {
        continue;
      };
      if (completion_phase && isUpdated_(neighbour_x, neighbour_y))
        continue;
      const bool reoffer = isUpdated_(neighbour_x, neighbour_y);
      if (reoffer && (sharedVisibilityField_->get(neighbour_x, neighbour_y) < 1.0 ||
                      cameFrom_(neighbour_x, neighbour_y) == cameFrom_(x, y) ||
                      gScore_(neighbour_x, neighbour_y) == 0.0))
        continue;
      if (sharedVisibilityField_->get(neighbour_x, neighbour_y) < 1) {
        cameFrom_(neighbour_x, neighbour_y) = cameFrom_(x, y);
        isUpdated_(neighbour_x, neighbour_y) = true;
        continue;
      }

      if (reoffer) {
        ++geometric_reoffers_;
        ++local_source_candidates_;
        potentialDistances.clear();
        const double incumbent = gScore_(neighbour_x, neighbour_y);
        const double cutoff = incumbent - 1e-10 * std::max({1.0, r_, incumbent});
        potentialDistances.push_back(
            geometricCandidate(cameFrom_(x, y), neighbour_x, neighbour_y, cutoff,
                               nullptr, x, y));
      } else {
      queuePotentialSources(potentialSources, neighbour_x, neighbour_y);
      local_source_candidates_ += potentialSources.size();
      getPotentialDistances(potentialSources, potentialDistances, x, y,
                            neighbour_x, neighbour_y,
                            completion_phase
      );
      }
      auto minimum_element =
          std::min_element(potentialDistances.begin(), potentialDistances.end(),
                           [](const auto &lhs, const auto &rhs) {
                             return lhs.distance < rhs.distance;
                           });
      distance = minimum_element->distance;
      if (completion_phase &&
          (!std::isfinite(distance) || distance < 0.0))
        continue;
      if (reoffer) {
        const double old_distance = gScore_(neighbour_x, neighbour_y);
        const double tolerance = 1e-10 * std::max({1.0, r_, old_distance});
        if (!(distance < old_distance - tolerance))
          continue;
        ++geometric_improvements_;
      }
      const bool cond = (distance == std::numeric_limits<double>::max());
      if (cond) {
        if (!completion_phase && !reoffer)
          failed_frontier.emplace_back(x, y);
        continue;
      } else {
        // use source giving least distance
        g = minimum_element->distance;
        gScore_(neighbour_x, neighbour_y) = g;
        cameFrom_(neighbour_x, neighbour_y) = minimum_element->source;
        // Preserve the actual selected pose, avoiding a second selector query
        // with a numerically different, wrapped terminal angle during replay.
        thetas_(neighbour_x, neighbour_y) = minimum_element->theta;
        geometric_pivot_theta_(neighbour_x, neighbour_y) = minimum_element->pivot_theta;
        geometric_incoming_distance_(neighbour_x, neighbour_y) = minimum_element->incoming_distance;
        geometric_incoming_word_(neighbour_x, neighbour_y) = minimum_element->incoming_word;
        geometric_outgoing_word_(neighbour_x, neighbour_y) = minimum_element->outgoing_word;
        openSet_->push(Node{neighbour_x, neighbour_y, RS_QUEUE_KEY(neighbour_x, neighbour_y, g)});
        ++queue_pushes_;
        max_queue_size_ =
            std::max<std::uint64_t>(max_queue_size_, openSet_->size());
        isUpdated_(neighbour_x, neighbour_y) = true;
        if (completion_phase)
          ++geometric_completion_writes_;
        ++nb_of_iterations_;
      }
    }
    } // drain the ordinary or completion queue
    if (completion_phase)
      break;
#ifdef RS_INTERACTIVE_API
    if (march_stopped_)
      break;
#endif
    geometric_ordinary_sources_ = nb_of_sources_;
    geometric_failed_frontier_count_ = failed_frontier.size();
    geometric_failed_frontier_capacity_bytes_ =
        failed_frontier.capacity() * sizeof(failed_frontier.front());
    std::sort(failed_frontier.begin(), failed_frontier.end());
    failed_frontier.erase(
        std::unique(failed_frontier.begin(), failed_frontier.end()),
        failed_frontier.end());
    geometric_completion_unique_seeds_ = failed_frontier.size();
    for (const auto &[seed_x, seed_y] : failed_frontier) {
      if (!isValid(seed_x, seed_y) || !isUpdated_(seed_x, seed_y) ||
          sharedVisibilityField_->get(seed_x, seed_y) < 1.0 ||
          !std::isfinite(gScore_(seed_x, seed_y)) ||
          gScore_(seed_x, seed_y) >= std::numeric_limits<double>::max())
        continue;
      bool adjacent_unupdated_free = false;
      for (std::size_t j = 0; j < 16; j += 2) {
        const int candidate_x = seed_x + neighbours_[j];
        const int candidate_y = seed_y + neighbours_[j + 1];
        if (isValid(candidate_x, candidate_y) &&
            sharedVisibilityField_->get(candidate_x, candidate_y) >= 1.0 &&
            !isUpdated_(candidate_x, candidate_y)) {
          adjacent_unupdated_free = true;
          break;
        }
      }
      if (!adjacent_unupdated_free)
        continue;
      openSet_->push(Node{seed_x, seed_y, RS_QUEUE_KEY(seed_x, seed_y, gScore_(seed_x, seed_y))});
      ++queue_pushes_;
      max_queue_size_ = std::max<std::uint64_t>(max_queue_size_, openSet_->size());
      ++geometric_completion_enqueued_seeds_;
    }
    completion_phase = true;
    if (openSet_->empty())
      break;
  };

  auto stopTime = std::chrono::high_resolution_clock::now();
  auto executionDuration = durationInMicroseconds(startTime, stopTime);

  std::cout << "G3 local reoffers/improvements/stale-pops: " << geometric_reoffers_ << '/'
            << geometric_improvements_ << '/' << geometric_stale_pops_ << '\n';
  std::cout << "G1 solves/retargets/fixed/birth-attempts/cost-prunes: " << geometric_solves_ << '/'
            << geometric_retargets_ << '/' << geometric_fixed_continuations_ << '/'
            << geometric_birth_attempts_ << '/' << geometric_cost_prunes_ << '\n';
  std::cout << "G1 incoming-leg cache lookups/hits: "
            << geometric_incoming_cache_lookups_ << '/'
            << geometric_incoming_cache_hits_ << '\n';
  std::cout << "G3 inherited-heading attempts/wins/reused-incoming: "
            << geometric_inherited_heading_attempts_ << '/'
            << geometric_inherited_heading_wins_ << '/'
            << geometric_inherited_incoming_reused_ << '\n';
  std::cout << "G3 vertex-pivot trials/births/reuses: " << geometric_vertex_trials_
            << '/' << geometric_vertex_births_ << '/' << geometric_vertex_reuses_ << '\n';
  std::cout << "G3 contact-heading searches/steps/wins: " << geometric_contact_searches_
            << '/' << geometric_contact_steps_ << '/' << geometric_contact_wins_ << '\n';
  std::cout << "G3 pose-source attempts/wins: " << geometric_pose_attempts_ << '/'
            << geometric_pose_wins_ << '\n';
  std::cout << "G3 bypass-max-depth/completion-sweeps/samples: "
            << geometric_bypass_max_depth_ << '/'
            << geometric_sweeps_ << '/' << geometric_sweep_samples_ << '\n';
  std::cout << "G3 ancestry-bypass attempts/wins/bound-skips/rotation-probes/rotations: "
            << geometric_bypass_attempts_ << '/' << geometric_bypass_wins_ << '/'
            << geometric_child_bound_skips_ << '/' << geometric_bypass_rotation_probes_
            << '/' << geometric_bypass_rotations_ << '\n';
  std::cout << "G3 analytic-rescue attempts/wins: "
            << geometric_rescue_attempts_ << '/' << geometric_rescue_wins_ << '\n';
  std::cout << "G3 deferred-completion failed/capacity-bytes/unique/enqueued/pops/writes: "
            << geometric_failed_frontier_count_ << '/'
            << geometric_failed_frontier_capacity_bytes_ << '/'
            << geometric_completion_unique_seeds_ << '/'
            << geometric_completion_enqueued_seeds_ << '/'
            << geometric_completion_pops_ << '/'
            << geometric_completion_writes_ << '\n';
  std::cout << "G3 selector-cache lookups/hits: "
            << geometric_selector_cache_lookups_ << '/'
            << geometric_selector_cache_hits_ << '\n';
  std::cout << "G3 selected blocked reuses: " << geometric_selected_blocked_reuses_ << '\n';
  std::cout << "G3 selector-cache capacity/storage: "
            << selector_memo_capacity_ << '/'
            << selector_memo_capacity_ * sizeof(SelectorMemoEntry) << '\n';
  std::cout << "G1 incumbent prunes: " << geometric_incumbent_prunes_ << '\n';
  std::cout << "G1 clearance queries/filtered/exact/boxes/subdivisions: "
            << geometric_clearance_.queries << '/' << geometric_clearance_.filtered
            << '/' << geometric_clearance_.exact_checks << '/' << geometric_clearance_.box_queries
            << '/' << geometric_clearance_.subdivisions << '\n';
  std::cout << "G1 free box attempts/cells/clears: " << geometric_clearance_.free_box_attempts
            << '/' << geometric_clearance_.free_box_cells << '/' << geometric_clearance_.free_box_clears << '\n';
  std::cout << "G1 exact intervals/events/refinements: " << geometric_clearance_.intervals
            << '/' << geometric_clearance_.events << '/' << geometric_clearance_.refinements << '\n';
  std::cout << "G1 exact statuses:";
  for (const auto count : geometric_clearance_.exact_statuses) std::cout << ' ' << count;
  std::cout << '\n';
  std::cout << "G1 filtered blocked: " << geometric_clearance_.filtered_blocked << '\n';
  std::cout << "G1 range/whole checks: " << geometric_clearance_.range_checks
            << '/' << geometric_clearance_.whole_checks << '\n';
  std::cout << "G1 first-arc cache lookups/hits: " << geometric_clearance_.first_arc_cache_lookups
            << '/' << geometric_clearance_.first_arc_cache_hits << '\n';
  std::cout << "G1 owned first-arc lookups/hits: "
            << geometric_clearance_.owned_first_arc_lookups << '/'
            << geometric_clearance_.owned_first_arc_hits << '\n';
#ifndef RS_INTERACTIVE_API
  runGeometricAudit();
#endif
  if (!sharedConfig_->silent) {
    std::cout << "###################### Visibility-based solver output "
                 "######################"
              << std::endl;
    if (sharedConfig_->timer) {
      std::cout << "Marching execution time in us: " << executionDuration
                << "us" << std::endl;
      std::cout << "Iterations: " << nb_of_iterations_ << std::endl;
      std::cout << "Nb of sources: " << nb_of_sources_ << std::endl;
      std::cout << "Pivot births: " << pivot_births_ << std::endl;
      std::cout << "Queue pushes: " << queue_pushes_ << std::endl;
      std::cout << "Queue pops: " << queue_pops_ << std::endl;
      std::cout << "Maximum queue size: " << max_queue_size_ << std::endl;
      std::cout << "Local source candidates: " << local_source_candidates_
                << std::endl;
      std::cout << "Prefix records: " << pivot_prefixes_.size() << std::endl;
      std::cout << "Prefix record size: " << sizeof(PivotPrefix) << std::endl;
      std::cout << "Prefix storage capacity bytes: "
                << pivot_prefixes_.capacity() * sizeof(PivotPrefix)
                << std::endl;
      std::cout << "Prefix vector allocations: " << prefix_vector_allocations_
                << std::endl;
    }
  }


#ifndef RS_INTERACTIVE_API
  const auto outputStartTime = std::chrono::high_resolution_clock::now();
  if (sharedConfig_->saveResults) {
    std::cout << "\n------- Saving -------" << std::endl;
    saveField(gScore_, "distance");
    saveField(*sharedVisibilityField_, "occupancy");
    // saveField(distance_map_, "distance");
    saveField(cameFrom_, "cameFrom");
    // One root: every updated label (including occupied routing cells) has
    // origin 0; untouched cells retain -1. No origin grid is needed online.
    saveField(isUpdated_, "origin", [](bool updated) { return updated ? 0 : -1; });
    saveField(thetas_, "thetas_");
    saveLightSources();
  }
  const auto outputStopTime = std::chrono::high_resolution_clock::now();
  std::cout << "Output writing time in us: "
            << durationInMicroseconds(outputStartTime, outputStopTime) << "us"
            << std::endl;
#endif
}

inline void
Marching::queuePotentialSources(std::vector<size_t> &potentialSources,
                                const int neighbour_x, const int neighbour_y) {
  size_t potentialSource_x = 0, potentialSource_y = 0, lightSource_num = 0;
  potentialSources.clear();
  // Queue sources from updated neighbours of neighbour
  for (size_t k = 0; k < 16; k += 2) {
    // NOTE those are always positive
    potentialSource_x = neighbour_x + neighbours_[k];
    potentialSource_y = neighbour_y + neighbours_[k + 1];
    // Box check
    if (!isValid(potentialSource_x, potentialSource_y)) {
      continue;
    };
    if (!isUpdated_(potentialSource_x, potentialSource_y)) {
      continue;
    };

    lightSource_num = cameFrom_(potentialSource_x, potentialSource_y);
    // Pick only unique sources (no repitition in potentialSources)
    if (std::find(potentialSources.begin(), potentialSources.end(),
                  lightSource_num) == potentialSources.end()) {
      potentialSources.push_back(lightSource_num);
    }
  }
}

void Marching::getPotentialDistances(
    const std::vector<size_t> &potentialSources,
    std::vector<triplet> &potentialDistances, const int from_x,
    const int from_y, const int neighbour_x, const int neighbour_y,
    const bool allow_rescue) {
  geometricCandidates(potentialSources, potentialDistances, from_x, from_y,
                       neighbour_x, neighbour_y, allow_rescue);
  return;
}

} // namespace RS_marching
