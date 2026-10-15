// Copyright (c) 2026, RS-MoE research
// SPDX-License-Identifier: Apache-2.0

#include <png.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <sys/resource.h>
#include <type_traits>
#include <vector>

#include <nlohmann/json.hpp>

#include "ompl/base/spaces/ReedsSheppStateSpace.h"

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_smac_planner/a_star.hpp"
#include "nav2_smac_planner/collision_checker.hpp"
#include "nav2_smac_planner/node_hybrid.hpp"
#include "nav2_smac_planner/node_lattice.hpp"
#include "nav2_smac_planner/types.hpp"
#include "nav2_smac_planner/prefix_observer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace
{
using Clock = std::chrono::steady_clock;
using Json = nlohmann::json;
using nav2_smac_planner::AStarAlgorithm;
using nav2_smac_planner::GridCollisionChecker;
using nav2_smac_planner::MotionModel;
using nav2_smac_planner::NodeHybrid;
using nav2_smac_planner::NodeLattice;
using nav2_smac_planner::SearchInfo;

constexpr unsigned int kSearchScale = 2;
constexpr double kPublicResolutionMeters = 0.05;
constexpr double kSearchResolutionMeters = kPublicResolutionMeters / kSearchScale;

double ms(Clock::time_point begin, Clock::time_point end)
{
  return std::chrono::duration<double, std::milli>(end - begin).count();
}

double wrap(double angle)
{
  while (angle >= M_PI) {angle -= 2.0 * M_PI;}
  while (angle < -M_PI) {angle += 2.0 * M_PI;}
  return angle;
}

unsigned int odd_lookup(double radius_cells)
{
  int size = static_cast<int>(std::ceil(2.0 * radius_cells)) + 1;
  if ((size & 1) == 0) {++size;}
  return static_cast<unsigned int>(size);
}

struct Map
{
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap;
  unsigned int width;
  unsigned int height;
};

Map load_map(const std::string & path)
{
  png_image image{};
  image.version = PNG_IMAGE_VERSION;
  if (!png_image_begin_read_from_file(&image, path.c_str())) {
    throw std::runtime_error("cannot read map PNG: " + path);
  }
  image.format = PNG_FORMAT_GRAY;
  std::vector<png_byte> pixels(PNG_IMAGE_SIZE(image));
  if (!png_image_finish_read(&image, nullptr, pixels.data(), 0, nullptr)) {
    const std::string reason = image.message;
    png_image_free(&image);
    throw std::runtime_error("cannot decode map PNG: " + reason);
  }
  const unsigned int width = image.width;
  const unsigned int height = image.height;
  const unsigned int search_width = width * kSearchScale;
  const unsigned int search_height = height * kSearchScale;
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2D>(
    search_width, search_height, kSearchResolutionMeters, 0.0, 0.0,
    nav2_costmap_2d::FREE_SPACE);
  for (unsigned int y = 0; y < height; ++y) {
    for (unsigned int x = 0; x < width; ++x) {
      const auto value = pixels[y * width + x];
      const auto cost = value > 128 ? nav2_costmap_2d::FREE_SPACE :
        nav2_costmap_2d::LETHAL_OBSTACLE;
      // Expand each public pixel into 2x2 equal cells. No inflation is applied.
      for (unsigned int dy = 0; dy < kSearchScale; ++dy) {
        for (unsigned int dx = 0; dx < kSearchScale; ++dx) {
          costmap->setCost(x * kSearchScale + dx, y * kSearchScale + dy, cost);
        }
      }
    }
  }
  png_image_free(&image);
  return {costmap, width, height};
}

struct PlannerContext
{
  bool setup_initialized{false};
  std::string method;
  std::string map_path;
  std::string primitive_file;
  double requested_radius{0.0};
  double max_time{0.0};
  unsigned int requested_bins{0};
  Map map{nullptr, 0, 0};
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node;
  std::unique_ptr<GridCollisionChecker> collision;
  std::unique_ptr<AStarAlgorithm<NodeHybrid>> hybrid;
  std::unique_ptr<AStarAlgorithm<NodeLattice>> lattice;
  unsigned int planner_bins{0};
  double metadata_radius_cells{0.0};
  float lattice_resolution{0.0f};
  unsigned int lattice_heading_count{0};
};

std::vector<double> pose_value(const Json & value, const char * name)
{
  auto pose = value.get<std::vector<double>>();
  if (pose.size() != 3) {
    throw std::runtime_error(std::string(name) + " must be [x, y, yaw_radians]");
  }
  return pose;
}

void require_cell_pose(const std::vector<double> & pose, unsigned int width,
  unsigned int height, const char * name)
{
  if (!std::isfinite(pose[0]) || !std::isfinite(pose[1]) || !std::isfinite(pose[2]) ||
    pose[0] < 0 || pose[1] < 0 || pose[0] >= width || pose[1] >= height ||
    std::floor(pose[0]) != pose[0] || std::floor(pose[1]) != pose[1])
  {
    throw std::runtime_error(std::string(name) + " must use an in-bounds integer cell center");
  }
}

unsigned int angle_bin(double theta, unsigned int bins)
{
  const auto n = static_cast<long>(bins);
  long index = std::lround(theta * static_cast<double>(bins) / (2.0 * M_PI)) % n;
  if (index < 0) {index += n;}
  return static_cast<unsigned int>(index);
}

Json path_json(const NodeHybrid::CoordinateVector & path)
{
  Json out = Json::array();
  for (const auto & p : path) {
    out.push_back({p.x / kSearchScale - 0.5, p.y / kSearchScale - 0.5, wrap(p.theta)});
  }
  return out;
}

Json primitive_json(const nav2_smac_planner::MotionPrimitive * primitive)
{
  Json poses = Json::array();
  for (const auto & p : primitive->poses) {
    poses.push_back({p._x, p._y, p._theta});
  }
  return {
    {"trajectory_id", primitive->trajectory_id}, {"start_angle", primitive->start_angle},
    {"end_angle", primitive->end_angle}, {"turning_radius", primitive->turning_radius},
    {"trajectory_length", primitive->trajectory_length}, {"arc_length", primitive->arc_length},
    {"straight_length", primitive->straight_length}, {"left_turn", primitive->left_turn},
    {"poses", std::move(poses)}};
}

template<typename NodeT>
Json parent_chain(typename AStarAlgorithm<NodeT>::NodePtr goal, std::size_t limit)
{
  Json chain = Json::array();
  auto * node = goal;
  std::size_t count = 0;
  while (node && count++ <= limit) {
    double theta;
    if constexpr (std::is_same<NodeT, NodeHybrid>::value) {
      theta = NodeHybrid::motion_table.getAngleFromBin(
        static_cast<unsigned int>(node->pose.theta));
    } else {
      theta = NodeLattice::motion_table.getAngleFromBin(
        static_cast<unsigned int>(node->pose.theta));
    }
    Json entry = {{"x", node->pose.x / kSearchScale - 0.5},
      {"y", node->pose.y / kSearchScale - 0.5},
      {"theta_bin", static_cast<unsigned int>(node->pose.theta)},
      {"theta_rad", wrap(theta)}, {"native_theta_rad", theta}};
    if constexpr (std::is_same<NodeT, NodeHybrid>::value) {
      entry["primitive_index"] = node->getMotionPrimitiveIndex();
    } else {
      entry["backwards"] = node->isBackward();
      const auto * primitive = node->getMotionPrimitive();
      if (primitive) {
        entry["primitive"] = primitive_json(primitive);
      } else {
        entry["primitive"] = nullptr;
        if (const auto * saved = rs_benchmark_observer::surviving_prefix(node)) {
          entry["saved_prefix_primitive"] = primitive_json(saved->primitive);
          entry["saved_prefix_backwards"] = saved->backwards;
          entry["saved_prefix_source"] = "before_analytic_clear";
        }
      }
    }
    chain.push_back(std::move(entry));
    node = node->parent;
  }
  if (node) {throw std::runtime_error("parent chain exceeded graph-size bound");}
  std::reverse(chain.begin(), chain.end());
  return chain;
}

Json run(const Json & input, std::unique_ptr<PlannerContext> & context)
{
  const auto complete_begin = Clock::now();
  rs_benchmark_observer::reset();
  Json result = {{"method", input.value("method", std::string())},
    {"nav2_version", "1.1.20"}, {"status", "error"}, {"setup_ms", 0.0},
    {"search_ms", 0.0}, {"map_load_ms", 0.0}, {"setup_reused", false},
    {"iterations", 0}, {"path", Json::array()}, {"parent_chain", Json::array()}};
  try {
    const std::string method = input.at("method").get<std::string>();
    if (method != "hybrid" && method != "lattice") {
      throw std::runtime_error("method must be hybrid or lattice");
    }
    const auto start = pose_value(input.at("start"), "start");
    const auto goal = pose_value(input.at("goal"), "goal");
    const double requested_radius = input.at("radius_cells").get<double>();
    if (!std::isfinite(requested_radius) || requested_radius <= 0) {
      throw std::runtime_error("radius_cells must be positive and finite");
    }
    const unsigned int bins = input.value("bins", 72u);
    const double max_time = input.value("max_time_s", 10.0);
    const std::string map_path = input.at("map").get<std::string>();
    const std::string primitive_file = input.value("primitive_file", std::string());
    if (bins < 3 || !std::isfinite(max_time) || max_time <= 0) {
      throw std::runtime_error("bins must be >= 3 and max_time_s must be positive");
    }
    if (method == "lattice" && primitive_file.empty()) {
      throw std::runtime_error("lattice requires primitive_file");
    }

    const bool setup_reused = context && context->setup_initialized;
    if (context && (method != context->method || map_path != context->map_path ||
      requested_radius != context->requested_radius || bins != context->requested_bins ||
      max_time != context->max_time || primitive_file != context->primitive_file))
    {
      throw std::runtime_error("persistent process is pinned to one method, map, radius_cells, bins, max_time_s, and primitive_file; start a new process to change them");
    }

    const auto setup_begin = Clock::now();
    double map_load_ms = 0.0;
    if (!context) {
      auto created = std::make_unique<PlannerContext>();
      created->method = method;
      created->map_path = map_path;
      created->primitive_file = primitive_file;
      created->requested_radius = requested_radius;
      created->requested_bins = bins;
      created->max_time = max_time;
      const auto map_begin = Clock::now();
      created->map = load_map(map_path);
      map_load_ms = ms(map_begin, Clock::now());
      context = std::move(created);
    }
    auto & state = *context;
    require_cell_pose(start, state.map.width, state.map.height, "start");
    require_cell_pose(goal, state.map.width, state.map.height, "goal");
    const auto sx = static_cast<unsigned int>(kSearchScale * start[0] + 1);
    const auto sy = static_cast<unsigned int>(kSearchScale * start[1] + 1);
    const auto gx = static_cast<unsigned int>(kSearchScale * goal[0] + 1);
    const auto gy = static_cast<unsigned int>(kSearchScale * goal[1] + 1);
    if (state.map.costmap->getCost(sx, sy) != nav2_costmap_2d::FREE_SPACE ||
      state.map.costmap->getCost(gx, gy) != nav2_costmap_2d::FREE_SPACE)
    {
      throw std::runtime_error("start and goal cells must be free (gray > 128)");
    }

    int max_iterations = 10000000;
    constexpr int max_on_approach = 0;
    constexpr float tolerance = 0.0f;
    constexpr bool allow_unknown = false;
    const double search_radius_cells = kSearchScale * requested_radius;
    const float lookup = static_cast<float>(odd_lookup(search_radius_cells));
    double effective_start_yaw = 0.0;
    double effective_goal_yaw = 0.0;
    int iterations = 0;
    bool succeeded = false;
    double search_ms = 0.0;
    Json path = Json::array();
    Json chain = Json::array();
    Json effective_config = {
      {"method", method}, {"radius_cells", requested_radius},
      {"reverse_penalty", 1.0}, {"non_straight_penalty", 1.0}, {"cost_penalty", 1.0},
      {"change_penalty", 0.0}, {"retrospective_penalty", 0.0}, {"rotation_penalty", 5.0},
      {"allow_reverse_expansion", true}, {"allow_unknown", false},
      {"cache_obstacle_heuristic", false}, {"lookup_table_size_cells", lookup},
      {"analytic_expansion_max_length_search_cells", 5.0 * search_radius_cells},
      {"analytic_expansion_ratio", 3.5}, {"goal_position_tolerance_cells", 0.0},
      {"max_iterations", max_iterations}, {"angle_bins", bins}, {"max_time_s", max_time},
      {"footprint", "point_cell"}, {"search_scale", kSearchScale},
      {"public_resolution_m_per_cell", kPublicResolutionMeters},
      {"search_resolution_m_per_cell", kSearchResolutionMeters}};

    if (!state.setup_initialized) {
      state.hybrid.reset();
      state.lattice.reset();
      state.collision.reset();
      state.node.reset();
      SearchInfo info{};
      info.non_straight_penalty = 1.0f;
      info.change_penalty = 0.0f;
      info.reverse_penalty = 1.0f;
      info.cost_penalty = 1.0f;
      info.retrospective_penalty = 0.0f;
      info.rotation_penalty = 5.0f;
      info.analytic_expansion_ratio = 3.5f;
      info.cache_obstacle_heuristic = false;
      info.allow_reverse_expansion = true;
      info.analytic_expansion_max_length = static_cast<float>(5.0 * search_radius_cells);
      state.node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("nav2_benchmark_runner");
      unsigned int planner_bins = bins;
      double min_radius = search_radius_cells;
      if (method == "lattice") {
        info.lattice_filepath = primitive_file;
        const auto metadata = nav2_smac_planner::LatticeMotionTable::getLatticeMetadata(primitive_file);
        if (metadata.grid_resolution <= 0.0f) {
          throw std::runtime_error("lattice metadata grid_resolution must be positive");
        }
        min_radius = metadata.min_turning_radius / metadata.grid_resolution;
        if (std::abs(min_radius - search_radius_cells) > 1e-4) {
          throw std::runtime_error("primitive file radius does not match radius_cells");
        }
        info.minimum_turning_radius = static_cast<float>(min_radius);
        info.analytic_expansion_max_length = static_cast<float>(5.0 * min_radius);
        planner_bins = metadata.number_of_headings;
        state.metadata_radius_cells = min_radius;
        state.lattice_resolution = metadata.grid_resolution;
        state.lattice_heading_count = metadata.number_of_headings;
      } else {
        info.minimum_turning_radius = static_cast<float>(search_radius_cells);
      }
      state.planner_bins = planner_bins;
      state.collision = std::make_unique<GridCollisionChecker>(state.map.costmap.get(), planner_bins, state.node);
      state.collision->setFootprint(nav2_costmap_2d::Footprint(), true, 0.0);
      if (method == "hybrid") {
        state.hybrid = std::make_unique<AStarAlgorithm<NodeHybrid>>(MotionModel::REEDS_SHEPP, info);
        state.hybrid->initialize(allow_unknown, max_iterations, max_on_approach, max_time, lookup, planner_bins);
      } else {
        state.lattice = std::make_unique<AStarAlgorithm<NodeLattice>>(MotionModel::STATE_LATTICE, info);
        state.lattice->initialize(allow_unknown, max_iterations, max_on_approach, max_time, lookup, planner_bins);
      }
      state.setup_initialized = true;
    }

    if (method == "hybrid") {
      auto & planner = *state.hybrid;
      const unsigned int sb = angle_bin(start[2], bins);
      const unsigned int gb = angle_bin(goal[2], bins);
      planner.setCollisionChecker(state.collision.get());
      planner.setStart(sx, sy, sb);
      planner.setGoal(gx, gy, gb);
      effective_start_yaw = NodeHybrid::motion_table.getAngleFromBin(sb);
      effective_goal_yaw = NodeHybrid::motion_table.getAngleFromBin(gb);
      effective_config["minimum_turning_radius_search_cells"] = search_radius_cells;
      effective_config["angle_bins"] = state.planner_bins;
      const auto setup_ms = ms(setup_begin, Clock::now());
      NodeHybrid::CoordinateVector reverse_path;
      const auto search_begin = Clock::now();
      succeeded = planner.createPath(reverse_path, iterations, tolerance);
      search_ms = ms(search_begin, Clock::now());
      if (succeeded) {
        std::reverse(reverse_path.begin(), reverse_path.end());
        path = path_json(reverse_path);
        const std::size_t bound = static_cast<std::size_t>(state.map.width) * kSearchScale *
          state.map.height * kSearchScale * state.planner_bins + 1;
        chain = parent_chain<NodeHybrid>(planner.getGoal(), bound);
      }
      result["setup_ms"] = setup_ms;
    } else {
      auto & planner = *state.lattice;
      const unsigned int sb = NodeLattice::motion_table.getClosestAngularBin(start[2]);
      const unsigned int gb = NodeLattice::motion_table.getClosestAngularBin(goal[2]);
      planner.setCollisionChecker(state.collision.get());
      planner.setStart(sx, sy, sb);
      planner.setGoal(gx, gy, gb);
      effective_start_yaw = NodeLattice::motion_table.getAngleFromBin(sb);
      effective_goal_yaw = NodeLattice::motion_table.getAngleFromBin(gb);
      effective_config["minimum_turning_radius_search_cells"] = state.metadata_radius_cells;
      effective_config["lattice_grid_resolution"] = state.lattice_resolution;
      effective_config["lattice_heading_count"] = state.lattice_heading_count;
      effective_config["primitive_file"] = primitive_file;
      effective_config["angle_bins"] = state.planner_bins;
      const auto setup_ms = ms(setup_begin, Clock::now());
      NodeLattice::CoordinateVector reverse_path;
      const auto search_begin = Clock::now();
      succeeded = planner.createPath(reverse_path, iterations, tolerance);
      search_ms = ms(search_begin, Clock::now());
      if (succeeded) {
        std::reverse(reverse_path.begin(), reverse_path.end());
        path = path_json(reverse_path);
        const std::size_t bound = static_cast<std::size_t>(state.map.width) * kSearchScale *
          state.map.height * kSearchScale * state.planner_bins + 1;
        chain = parent_chain<NodeLattice>(planner.getGoal(), bound);
      }
      std::cerr << "prefix_observer captured_records=" << rs_benchmark_observer::captured_count()
                << " record_bytes=" << rs_benchmark_observer::record_bytes() << '\n';
      result["setup_ms"] = setup_ms;
    }
    struct rusage usage{};
    getrusage(RUSAGE_SELF, &usage);
    const bool timed_out = !succeeded && search_ms >= max_time * 1000.0;
    const bool maxed_iterations = !succeeded && iterations >= max_iterations;
    result["status"] = succeeded ? "success" : (timed_out ? "timeout" :
      (maxed_iterations ? "max_iterations" : "no_path"));
    if (!succeeded) {
      result["error"] = result["status"] == "no_path" ? "no path found" :
        result["status"].get<std::string>();
    }
    result["iterations"] = iterations;
    result["search_ms"] = search_ms;
    result["map_load_ms"] = map_load_ms;
    result["setup_reused"] = setup_reused;
    result["complete_ms"] = ms(complete_begin, Clock::now());
    result["peak_rss_kib"] = usage.ru_maxrss;
    result["effective_start_yaw_rad"] = effective_start_yaw;
    result["effective_goal_yaw_rad"] = effective_goal_yaw;
    result["effective_config"] = effective_config;
    result["coordinate_mapping"] = {
      {"public_to_search", "search_index = 2 * public_integer_center + 1"},
      {"search_to_public", "public_center = search_coordinate / 2 - 0.5"},
      {"search_scale", kSearchScale}, {"inflation", false}};
    result["path"] = std::move(path);
    result["parent_chain"] = std::move(chain);
  } catch (const std::exception & error) {
    result["error"] = error.what();
    result["complete_ms"] = ms(complete_begin, Clock::now());
    struct rusage usage{};
    getrusage(RUSAGE_SELF, &usage);
    result["peak_rss_kib"] = usage.ru_maxrss;
  }
  return result;
}

Json run_rs_path(const Json & input)
{
  const auto start = pose_value(input.at("start"), "start");
  const auto goal = pose_value(input.at("goal"), "goal");
  const double radius = input.at("radius_cells").get<double>();
  if (!std::isfinite(radius) || radius <= 0.0) {
    throw std::runtime_error("radius_cells must be positive and finite");
  }
  for (const auto value : start) {
    if (!std::isfinite(value)) {throw std::runtime_error("start values must be finite");}
  }
  for (const auto value : goal) {
    if (!std::isfinite(value)) {throw std::runtime_error("goal values must be finite");}
  }

  ompl::base::ReedsSheppStateSpace space(radius);
  auto * from = space.allocState();
  auto * to = space.allocState();
  auto * from_se2 = from->as<ompl::base::SE2StateSpace::StateType>();
  auto * to_se2 = to->as<ompl::base::SE2StateSpace::StateType>();
  from_se2->setXY(start[0], start[1]);
  from_se2->setYaw(start[2]);
  to_se2->setXY(goal[0], goal[1]);
  to_se2->setYaw(goal[2]);
  const auto path = space.reedsShepp(from, to);

  Json segments = Json::array();
  for (std::size_t i = 0; i < 5; ++i) {
    const double normalized_length = path.length_[i];
    if (std::abs(normalized_length) <= 1e-12 ||
      path.type_[i] == ompl::base::ReedsSheppStateSpace::RS_NOP)
    {
      continue;
    }
    const char * type = "S";
    if (path.type_[i] == ompl::base::ReedsSheppStateSpace::RS_LEFT) {type = "L";}
    if (path.type_[i] == ompl::base::ReedsSheppStateSpace::RS_RIGHT) {type = "R";}
    segments.push_back({{"type", type}, {"gear", normalized_length > 0.0 ? 1 : -1},
      {"length", std::abs(normalized_length) * radius}});
  }
  space.freeState(from);
  space.freeState(to);
  return {{"operation", "rs_path"}, {"status", "success"}, {"reference_only", true},
    {"source", "OMPL ReedsSheppStateSpace"}, {"radius_cells", radius},
    {"segments", std::move(segments)}, {"total_length", path.length() * radius}};
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    rclcpp::init(argc, argv);
    std::string line;
    std::string fixed_method;
    double fixed_radius = 0.0;
    std::string fixed_primitive;
    std::unique_ptr<PlannerContext> planner_context;
    while (std::getline(std::cin, line)) {
      if (line.empty()) {continue;}
      Json input;
      Json result;
      try {
        input = Json::parse(line);
        if (input.value("op", std::string()) == "rs_path") {
          result = run_rs_path(input);
        } else {
          const auto method = input.at("method").get<std::string>();
          const auto radius = input.at("radius_cells").get<double>();
          const auto primitive = input.value("primitive_file", std::string());
          if (fixed_method.empty()) {
            fixed_method = method;
            fixed_radius = radius;
            fixed_primitive = primitive;
          } else if (method != fixed_method || radius != fixed_radius || primitive != fixed_primitive) {
            throw std::runtime_error("persistent process is fixed to one method, radius_cells, and primitive_file; start a new process to change them");
          }
          result = run(input, planner_context);
        }
      } catch (const std::exception & error) {
        struct rusage usage{};
        getrusage(RUSAGE_SELF, &usage);
        result = {{"status", "error"}, {"error", error.what()},
          {"nav2_version", "1.1.20"}, {"peak_rss_kib", usage.ru_maxrss}};
      }
      std::cout << result.dump() << '\n' << std::flush;
    }
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "fatal: " << error.what() << '\n';
    if (rclcpp::ok()) {rclcpp::shutdown();}
    return 2;
  }
}
