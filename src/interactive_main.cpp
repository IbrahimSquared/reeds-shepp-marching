#include "environment/Environment.hpp"
#include "marching/Marching.hpp"
#include "query/DirectQuery.hpp"
#include "query/RetainedView.hpp"
#include "thirdparty/nlohmann/json.hpp"

#include <SFML/Graphics.hpp>



#include <chrono>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace {

using json = nlohmann::json;
constexpr std::size_t kMaximumMapCells = 1048576;
constexpr unsigned kMaximumMapDimension = 2048;

using PreparedMap = RS_marching::Field<std::uint8_t>;

// One private binary occupancy template; run Fields and Config remain owned.
// Identity is the actual bounded byte read, independent of path or timestamps.
class MapSnapshot {
public:
  const PreparedMap *load(const RS_marching::Config &config,
                          bool &dimension_error) {
    uncached_.reset();
    std::ifstream input(config.imagePath, std::ios::binary);
    if (!input) return nullptr;
    std::vector<char> bytes;
    std::error_code ec;
    const auto size = std::filesystem::file_size(config.imagePath, ec);
    if (!ec) bytes.reserve(static_cast<std::size_t>(
        std::min<std::uintmax_t>(size, kEncodedLimit + 1)));
    std::array<char, 65536> chunk;
    while (input && bytes.size() <= kEncodedLimit) {
      const auto count = std::min(chunk.size(), kEncodedLimit + 1 - bytes.size());
      input.read(chunk.data(), static_cast<std::streamsize>(count));
      const auto read = static_cast<std::size_t>(input.gcount());
      if (bytes.size() + read > bytes.capacity())
        bytes.reserve(std::min(kEncodedLimit + 1,
            std::max(bytes.size() + read, bytes.capacity() * 2)));
      bytes.insert(bytes.end(), chunk.data(), chunk.data() + read);
    }
    if (input.bad() || (input.fail() && !input.eof())) return nullptr;
    maximum_read_capacity_ = std::max(maximum_read_capacity_, bytes.capacity());
    const bool bounded = bytes.size() <= kEncodedLimit;
    if (bounded && cached_ && bytes == encoded_) {
      ++hits_;
      return cached_.get();
    }
    ++misses_;
    sf::Image image;
    ++decodes_;
    // Oversized files remain accepted. Decode once from file, then validate and
    // construct from that same image; the bounded prefix is not an identity key.
    if (!(bounded ? image.loadFromMemory(bytes.data(), bytes.size())
                  : image.loadFromFile(config.imagePath))) return nullptr;
    const auto dimensions = image.getSize();
    if (!dimensions.x || !dimensions.y || dimensions.x > kMaximumMapDimension ||
        dimensions.y > kMaximumMapDimension ||
        static_cast<std::size_t>(dimensions.x) * dimensions.y > kMaximumMapCells) {
      dimension_error = true;
      return nullptr;
    }
    ++preparations_;
    auto scene = std::make_unique<PreparedMap>(dimensions.x, dimensions.y, 0);
    for (unsigned y = 0; y < dimensions.y; ++y) {
      for (unsigned x = 0; x < dimensions.x; ++x) {
        const auto pixel = image.getPixel(x, y);
        const int gray = 0.3 * pixel.r + 0.59 * pixel.g + 0.11 * pixel.b;
        scene->set(x, y, gray > 128);
      }
    }
    const auto payload = bytes.capacity() +
        static_cast<std::size_t>(dimensions.x) * dimensions.y;
    if (bounded && payload <= kRetainedLimit) {
      encoded_ = std::move(bytes);
      cached_ = std::move(scene);
      retained_payload_ = payload;
      return cached_.get();
    }
    uncached_ = std::move(scene);
    return uncached_.get();
  }

  std::unique_ptr<RS_marching::Environment> instantiate(
      RS_marching::Config &config, const PreparedMap &scene) {
    ++instances_;
    return std::make_unique<RS_marching::Environment>(config, scene);
  }

  void finishRequest() { uncached_.reset(); }

  void audit() const {
    const char *enabled = std::getenv("RS_API_CACHE_AUDIT");
    if (!enabled || std::string(enabled) != "1") return;
    std::cerr << "map_cache_audit " << json({
        {"hits", hits_}, {"misses", misses_}, {"decodes", decodes_},
        {"preparations", preparations_}, {"instances", instances_},
        {"entries", cached_ ? 1 : 0}, {"retained_payload_bytes", retained_payload_},
        {"maximum_read_capacity", maximum_read_capacity_},
        {"encoded_limit", kEncodedLimit}, {"retained_limit", kRetainedLimit}
    }).dump() << '\n';
  }

private:
  static constexpr std::size_t kEncodedLimit = 8 * 1024 * 1024;
  static constexpr std::size_t kRetainedLimit = 9 * 1024 * 1024;
  std::vector<char> encoded_;
  std::unique_ptr<PreparedMap> cached_, uncached_;
  std::size_t hits_ = 0, misses_ = 0, decodes_ = 0, preparations_ = 0, instances_ = 0;
  std::size_t retained_payload_ = 0, maximum_read_capacity_ = 0;
};

json error(const std::string &code, const std::string &message) {
  return {{"ok", false}, {"error", {{"code", code}, {"message", message}}}};
}

bool finiteNumber(const json &value, double &output) {
  if (!value.is_number()) return false;
  output = value.get<double>();
  return std::isfinite(output);
}

bool integer(const json &value, int &output) {
  if (value.is_number_unsigned()) {
    const auto candidate = value.get<unsigned long long>();
    if (candidate > static_cast<unsigned long long>(
                        std::numeric_limits<int>::max())) return false;
    output = static_cast<int>(candidate);
    return true;
  }
  if (!value.is_number_integer()) return false;
  const auto candidate = value.get<long long>();
  if (candidate < std::numeric_limits<int>::min() ||
      candidate > std::numeric_limits<int>::max()) return false;
  output = static_cast<int>(candidate);
  return true;
}

bool objectMember(const json &object, const char *name, const json *&value) {
  if (!object.is_object() || !object.contains(name)) return false;
  value = &object.at(name);
  return true;
}

bool parseStart(const json &command, int &x, int &y, double &heading,
                std::string &message) {
  const json *start = nullptr;
  const json *x_value = nullptr;
  const json *y_value = nullptr;
  const json *heading_value = nullptr;
  if (!objectMember(command, "start", start) || !start->is_object() ||
      !objectMember(*start, "x", x_value) || !objectMember(*start, "y", y_value) ||
      !objectMember(*start, "heading_deg", heading_value) ||
      !integer(*x_value, x) || !integer(*y_value, y) ||
      !finiteNumber(*heading_value, heading)) {
    message = "start requires integer x/y and finite heading_deg";
    return false;
  }
  return true;
}

// The path options shared by query and direct_query; throws on invalid values.
void pathOptions(const json &command, bool &check_collisions, double &sample_step) {
  if (command.contains("check_collisions")) {
    if (!command.at("check_collisions").is_boolean())
      throw std::invalid_argument("check_collisions must be boolean");
    check_collisions = command.at("check_collisions").get<bool>();
  }
  if (command.contains("sample_step") &&
      (!finiteNumber(command.at("sample_step"), sample_step) || sample_step < 0.1 ||
       sample_step > 5.0))
    throw std::invalid_argument("sample_step must be finite and in [0.1,5]");
}

double normalizedHeadingDegrees(double heading) {
  heading = std::remainder(heading, 360.0);
  return heading >= 180.0 ? heading - 360.0 : heading;
}

// Validates a run's map, start and radius, and fills the fixed deterministic settings
// (GD=1, exit_it=5, randomness disabled, one start). Returns an
// error reply, or null on success.
json runConfig(const json &command, RS_marching::Config &config, json &start,
               MapSnapshot &maps, const PreparedMap *&scene) {
  const json *map_path_value = nullptr;
  const json *radius_value = nullptr;
  int start_x = 0, start_y = 0;
  double heading = 0.0, requested_radius = 0.0;
  std::string message;
  if (!objectMember(command, "map_path", map_path_value) || !map_path_value->is_string() ||
      !objectMember(command, "radius", radius_value) ||
      !finiteNumber(*radius_value, requested_radius) ||
      !parseStart(command, start_x, start_y, heading, message)) {
    return error("invalid_argument", message.empty()
        ? "a run requires absolute map_path, start, and finite radius" : message);
  }
  if (requested_radius < 0.05 || requested_radius > 10000.0)
    return error("invalid_argument", "radius must be in [0.05,10000]");
  const std::filesystem::path map_path(map_path_value->get<std::string>());
  if (!map_path.is_absolute() || !std::filesystem::is_regular_file(map_path))
    return error("invalid_map", "map_path must be an absolute readable image path");
  heading = normalizedHeadingDegrees(heading);
  config.mapSource = RS_marching::MapSource::Image;
  config.imagePath = map_path.string();
  config.initialFrontline = {{start_x, start_y, heading}};
  config.radius = requested_radius;
  config.GD_bisection = 1;
  config.exit_it = 5;
  config.pose_randomness = false;
  config.saveResults = false;
  config.silent = true;
  config.timer = false;
  bool dimension_error = false;
  scene = maps.load(config, dimension_error);
  if (!scene) return error("invalid_map", dimension_error
      ? "map dimensions exceed the interactive limit"
      : "map_path must be an absolute readable image path");
  const auto &visibility = *scene;
  if (start_x < 0 || start_y < 0 || static_cast<unsigned>(start_x) >= visibility.nx() ||
      static_cast<unsigned>(start_y) >= visibility.ny())
    return error("invalid_argument", "start is outside the map");
  if (visibility.get(start_x, start_y) == 0.0)
    return error("invalid_argument", "start is occupied");
  start = {{"x", start_x}, {"y", start_y}, {"heading_deg", heading}};
  return nullptr;
}

} // namespace

int main() {

  std::streambuf *const stdout_buffer = std::cout.rdbuf();
  std::cout.rdbuf(std::cerr.rdbuf());
  std::ostream response(stdout_buffer);
  std::unique_ptr<RS_marching::Environment> environment;
  std::unique_ptr<RS_marching::Marching> marching;
  MapSnapshot maps;

  for (std::string line; std::getline(std::cin, line);) {
    json reply;
    // A query reply is already serialized JSON; it is written as is.
    std::optional<std::string> raw_reply;
    try {
      const json command = json::parse(line);
      const json *command_name = nullptr;
      if (!objectMember(command, "command", command_name) || !command_name->is_string()) {
        reply = error("malformed_command", "command must be a JSON object with a string command");
      } else if (*command_name == "march") {
        const auto initialization_start = std::chrono::steady_clock::now();
        RS_marching::Config config;
        json start;
        const PreparedMap *scene = nullptr;
        reply = runConfig(command, config, start, maps, scene);
        if (reply.is_null()) {
          auto loaded = maps.instantiate(config, *scene);
          if (!loaded->getVisibilityField())
            throw std::runtime_error("map_path could not be loaded");
          environment = std::move(loaded);
          marching = std::make_unique<RS_marching::Marching>(*environment);
          const auto initialized = std::chrono::steady_clock::now();
          marching->runMarching();
          const auto marched = std::chrono::steady_clock::now();
          reply = json::parse(marching->geometricSnapshotJson());
          const auto snapshotted = std::chrono::steady_clock::now();
          reply["ok"] = true;
          reply["radius"] = config.radius;
          reply["start"] = start;
          reply["timings_ms"] = {
              {"initialization", std::chrono::duration<double, std::milli>(initialized - initialization_start).count()},
              {"march", std::chrono::duration<double, std::milli>(marched - initialized).count()},
              {"snapshot", std::chrono::duration<double, std::milli>(snapshotted - marched).count()},
              {"total", std::chrono::duration<double, std::milli>(snapshotted - initialization_start).count()}};
        }
      } else if (*command_name == "direct_query") {
        // One start-to-goal query without a retained one-to-all run: a local
        // start wave stopped by query::DirectStop, then the same fixed-goal
        // reconstruction. The active run, if any, is left untouched.
        const auto initialization_start = std::chrono::steady_clock::now();
        RS_marching::Config config;
        json start;
        const PreparedMap *scene = nullptr;
        reply = runConfig(command, config, start, maps, scene);
        const json *goal_value = nullptr;
        const json *goal_x = nullptr;
        const json *goal_y = nullptr;
        const json *goal_heading = nullptr;
        int target_x = 0, target_y = 0;
        double heading = 0.0, sample_step = 0.5;
        bool check_collisions = true;
        if (reply.is_null() &&
            (!objectMember(command, "goal", goal_value) || !goal_value->is_object() ||
             !objectMember(*goal_value, "x", goal_x) || !objectMember(*goal_value, "y", goal_y) ||
             !objectMember(*goal_value, "heading_deg", goal_heading) ||
             !integer(*goal_x, target_x) || !integer(*goal_y, target_y) ||
             !finiteNumber(*goal_heading, heading))) {
          reply = error("invalid_argument", "goal requires integer x/y and finite heading_deg");
        }
        if (reply.is_null()) pathOptions(command, check_collisions, sample_step);
        std::string order = "dijkstra";
        if (reply.is_null() && command.contains("order")) {
          if (!command.at("order").is_string() ||
              (command.at("order") != "dijkstra" && command.at("order") != "astar"))
            throw std::invalid_argument("order must be \"dijkstra\" or \"astar\"");
          order = command.at("order").get<std::string>();
        }
        if (reply.is_null()) {
          const double goal_rad = normalizedHeadingDegrees(heading) * pi / 180.0;
          auto direct_environment = maps.instantiate(config, *scene);
          if (!direct_environment->getVisibilityField())
            throw std::runtime_error("map_path could not be loaded");
          auto run = std::make_unique<RS_marching::Marching>(*direct_environment);
          const RS_marching::query::RetainedView view(*run);
          RS_marching::query::DirectStop stop(
              view, target_x, target_y,
              {static_cast<double>(target_x), static_cast<double>(target_y), goal_rad});
          const bool valid_goal = view.inside(target_x, target_y) && view.free(target_x, target_y);
          const auto initialized = std::chrono::steady_clock::now();
          if (valid_goal) {
            run->setMarchStop([&stop](const double key) { return stop(key); });
            // A* order: the Euclidean distance to the goal bounds every
            // remaining RS path length from any heading.
            if (order == "astar")
              run->setMarchHeuristic([target_x, target_y](const int x, const int y) {
                return std::hypot(static_cast<double>(x - target_x),
                                  static_cast<double>(y - target_y));
              });
            run->runMarching();
          }
          const auto marched = std::chrono::steady_clock::now();
          reply = json::parse(run->queryGeometricPathJson(
              target_x, target_y, check_collisions, sample_step, goal_rad, &stop.best));
          const auto queried = std::chrono::steady_clock::now();
          std::size_t reached = 0, free_cells = 0;
          for (int y = 0; y < view.height(); ++y)
            for (int x = 0; x < view.width(); ++x) {
              if (!view.free(x, y)) continue;
              ++free_cells;
              reached += view.reached(x, y);
            }
          reply["start"] = start;
          reply["direct"] = {
              {"stop_rule", "start wave stopped once its popped key exceeds the best route found on its partial retained state"},
              {"order", order},
              {"pops", stop.pops}, {"evaluations", stop.evaluations},
              {"evaluation_solves", stop.work.solves},
              {"evaluation_bound_solves", stop.work.bound_solves},
              {"evaluation_clearance_checks", stop.work.clearance_checks},
              {"evaluation_free_box_attempts", stop.work.free_box_attempts},
              {"evaluation_free_box_cells", stop.work.free_box_cells},
              {"evaluation_free_box_clears", stop.work.free_box_clears},
              {"route_from_evaluation", reply.value("prior_route_used", false)},
              {"stop_key", std::isfinite(stop.stop_key) ? json(stop.stop_key) : json(nullptr)},
              {"stopped_early", run->marchStopped()}, {"cells_reached", reached},
              {"free_cells", free_cells}};
          reply["timings_ms"]["initialization"] =
              std::chrono::duration<double, std::milli>(initialized - initialization_start).count();
          reply["timings_ms"]["march"] =
              std::chrono::duration<double, std::milli>(marched - initialized).count();
          reply.erase("prior_route_used");
          reply["timings_ms"]["stop_evaluations"] = stop.evaluation_ms;
          reply["timings_ms"]["direct_total"] =
              std::chrono::duration<double, std::milli>(queried - initialization_start).count();
        }
      } else if (*command_name == "query") {
        const json *x_value = nullptr;
        const json *y_value = nullptr;
        int x = 0, y = 0;
        bool check_collisions = true;
        double sample_step = 0.5;
        std::optional<double> goal_heading;
        if (!marching) {
          reply = error("no_active_run", "march must succeed before query");
        } else if (!objectMember(command, "x", x_value) || !objectMember(command, "y", y_value) ||
                   !integer(*x_value, x) || !integer(*y_value, y)) {
          reply = error("invalid_argument", "query requires integer x and y");
        } else {
          pathOptions(command, check_collisions, sample_step);
          if (command.contains("heading_deg")) {
            double heading = 0.0;
            if (!finiteNumber(command.at("heading_deg"), heading)) {
              throw std::invalid_argument("heading_deg must be finite");
            }
            goal_heading = normalizedHeadingDegrees(heading) * pi / 180.0;
          }
          raw_reply = marching->queryGeometricPathJson(
              x, y, check_collisions, sample_step, goal_heading);
        }
      } else {
        reply = error("unknown_command", "command must be march, query or direct_query");
      }
    } catch (const nlohmann::json::exception &exception) {
      reply = error("malformed_json", exception.what());
    } catch (const std::exception &exception) {
      reply = error("invalid_argument", exception.what());
    }
    maps.finishRequest();
    maps.audit();
    response << (raw_reply ? *raw_reply : reply.dump()) << '\n' << std::flush;
  }
  return 0;
}
