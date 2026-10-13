#include "parser/Parser.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <stdexcept>
#include <string_view>
#include <utility>

namespace RS_marching {
namespace {
std::string trim(const std::string &value) {
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) return {};
  return value.substr(first, value.find_last_not_of(" \t\r\n") - first + 1);
}

template <typename T> T integer(const std::string &value) {
  T result{};
  const char *first = value.data();
  const char *last = first + value.size();
  // Preserve the previously accepted explicit plus sign.
  if (first != last && *first == '+') {
    ++first;
    if (first != last && (*first == '+' || *first == '-'))
      throw std::invalid_argument("expected a complete integer");
  }
  const auto parsed = std::from_chars(first, last, result);
  if (parsed.ec != std::errc{} || parsed.ptr != last)
    throw std::invalid_argument("expected a complete integer");
  return result;
}

double real(const std::string &value) {
  std::size_t end = 0;
  const double result = std::stod(value, &end);
  if (end != value.size())
    throw std::invalid_argument("expected a complete number");
  return result;
}

bool boolean(const std::string &value) {
  if (value == "1" || value == "true") return true;
  if (value == "0" || value == "false") return false;
  throw std::invalid_argument("expected boolean 0, 1, false or true");
}

std::vector<Node> poses(const std::string &value) {
  if (value.size() < 2 || value.front() != '{' || value.back() != '}')
    throw std::invalid_argument("expected {x,y,heading,...} with complete pose triples");
  const auto body = trim(value.substr(1, value.size() - 2));
  if (body.empty()) return {};
  std::vector<std::string> tokens;
  std::size_t begin = 0;
  for (;;) {
    const auto comma = body.find(',', begin);
    tokens.push_back(trim(body.substr(begin, comma - begin)));
    if (comma == std::string::npos) break;
    begin = comma + 1;
  }
  if (tokens.size() % 3 != 0)
    throw std::invalid_argument("expected complete x,y,heading pose triples");
  std::vector<Node> result;
  for (std::size_t i = 0; i < tokens.size(); i += 3) {
    const double heading = real(tokens[i + 2]);
    if (!std::isfinite(heading))
      throw std::invalid_argument("initial pose headings must be finite");
    result.push_back({integer<int>(tokens[i]), integer<int>(tokens[i + 1]), heading});
  }
  return result;
}

bool deprecated(const std::string &key) {
  static constexpr std::string_view keys[] = {
      "target_x", "target_y", "visibilityThreshold", "angleThreshold", "greedy",
      "visibilityBasedSolver", "expandInObstacles", "vstar", "astar",
      "distanceFunction", "saveCameFrom", "saveGScore", "saveFScore",
      "saveLightSources", "savePath", "saveDistanceFunction",
      "saveVisibilityBasedSolverImage", "saveDistanceFunctionImage", "ballRadius",
      "number_of_contour_lines"};
  for (const auto legacy : keys) if (key == legacy) return true;
  return false;
}
} // namespace

void validateConfig(const Config &config) {
  const auto fail = [](const char *key, const char *message) {
    throw std::invalid_argument(std::string(key) + ": " + message);
  };
  const auto intMax = static_cast<size_t>(std::numeric_limits<int>::max());
  if (config.mapSource != MapSource::Random && config.mapSource != MapSource::Image)
    fail("mapSource", "must be random or image");
  if (config.ncols == 0 || config.ncols > intMax)
    fail("ncols", "must be a positive dimension representable as int");
  if (config.nrows == 0 || config.nrows > intMax)
    fail("nrows", "must be a positive dimension representable as int");
  if (config.ncols > std::numeric_limits<size_t>::max() / sizeof(double) / config.nrows)
    fail("ncols/nrows", "field dimensions overflow storage size");
  if (config.nb_of_obstacles < 0) fail("nb_of_obstacles", "must be nonnegative");
  if (config.minWidth > config.maxWidth || config.maxWidth > intMax)
    fail("minWidth/maxWidth", "require ordered nonnegative widths representable as int");
  if (config.minHeight > config.maxHeight || config.maxHeight > intMax)
    fail("minHeight/maxHeight", "require ordered nonnegative heights representable as int");
  if (!std::isfinite(config.speedValue)) fail("speedValue", "must be finite");
  if (!std::isfinite(config.radius) || config.radius <= 0)
    fail("radius", "must be a finite positive double");
  if (config.GD_bisection != 1 && config.GD_bisection != 2)
    fail("GD_bisection", "must be 1 or 2");
  if (config.exit_it < 0) fail("exit_it", "must be nonnegative");
  // Parsed poses are validated above; direct C++ poses are checked by Marching
  // before root mutation, preserving that public rejection boundary.
}

bool ConfigParser::parse(const std::string &filename) {
  std::ifstream file(filename);
  if (!file) {
    std::cerr << "Failed to open " << filename << '\n';
    return false;
  }
  Config candidate;
  std::string line;
  size_t lineNumber = 0;
  std::map<std::string, size_t> keyLines;
  while (std::getline(file, line)) {
    ++lineNumber;
    line = trim(line);
    if (line.empty() || line.front() == '#') continue;
    const auto equals = line.find('=');
    const auto key = trim(line.substr(0, equals));
    const auto location = filename + ":" + std::to_string(lineNumber) + " [" + key + "]: ";
    if (equals == std::string::npos || key.empty()) {
      std::cerr << location << "expected key=value\n";
      return false;
    }
    keyLines[key] = lineNumber;
    const auto rawValue = line.substr(equals + 1);
    // Paths are literal: '#' is a valid filename character. Inline comments
    // apply to typed settings, while path comments belong on a separate line.
    const auto value = trim(key == "imagePath" ? rawValue
                                              : rawValue.substr(0, rawValue.find('#')));
    try {
      if (key == "mapSource") {
        if (value == "random") candidate.mapSource = MapSource::Random;
        else if (value == "image") candidate.mapSource = MapSource::Image;
        else throw std::invalid_argument("must be random or image");
      } else if (key == "mode") {
        throw std::invalid_argument(
            "obsolete setting; replace mode=1 with mapSource=random or mode=2 with mapSource=image");
      } else if (key == "ncols") candidate.ncols = integer<size_t>(value);
      else if (key == "nrows") candidate.nrows = integer<size_t>(value);
      else if (key == "nb_of_obstacles") candidate.nb_of_obstacles = integer<int>(value);
      else if (key == "minWidth") candidate.minWidth = integer<size_t>(value);
      else if (key == "maxWidth") candidate.maxWidth = integer<size_t>(value);
      else if (key == "minHeight") candidate.minHeight = integer<size_t>(value);
      else if (key == "maxHeight") candidate.maxHeight = integer<size_t>(value);
      else if (key == "speedValue") {
        const double speed = real(value);
        if (!std::isfinite(speed) || std::abs(speed) > std::numeric_limits<float>::max())
          throw std::invalid_argument("speedValue must be finite and representable as float");
        candidate.speedValue = static_cast<float>(speed);
      } else if (key == "randomSeed") candidate.randomSeed = boolean(value);
      else if (key == "seedValue") candidate.seedValue = integer<int>(value);
      else if (key == "imagePath") candidate.imagePath = value;
      else if (key == "initialFrontline") candidate.initialFrontline = poses(value);
      else if (key == "radius") candidate.radius = real(value);
      else if (key == "GD_bisection") candidate.GD_bisection = integer<int>(value);
      else if (key == "exit_it") candidate.exit_it = integer<int>(value);
      else if (key == "bumps") std::cerr << location << "obsolete setting; ignored (no effect)\n";
      else if (key == "pose_randomness") candidate.pose_randomness = boolean(value);
      else if (key == "timer") candidate.timer = boolean(value);
      else if (key == "saveResults") candidate.saveResults = boolean(value);
      else if (key == "saveVisibilityField") candidate.saveVisibilityField = boolean(value);
      else if (key == "saveSpeedField") candidate.saveSpeedField = boolean(value);
      else if (key == "silent") candidate.silent = boolean(value);
      else if (deprecated(key)) std::cerr << location << "deprecated key; ignored\n";
      else std::cerr << location << "unknown key; ignored\n";
    } catch (const std::exception &error) {
      std::cerr << location << error.what() << '\n';
      return false;
    }
  }
  if (file.bad()) {
    std::cerr << filename << ": failed reading config\n";
    return false;
  }
  try {
    validateConfig(candidate);
  } catch (const std::exception &error) {
    const std::string message = error.what();
    const auto keys = message.substr(0, message.find(':'));
    size_t sourceLine = 0;
    size_t begin = 0;
    for (;;) {
      const auto slash = keys.find('/', begin);
      const auto key = keys.substr(begin, slash - begin);
      if (const auto found = keyLines.find(key); found != keyLines.end())
        sourceLine = std::max(sourceLine, found->second);
      if (slash == std::string::npos) break;
      begin = slash + 1;
    }
    std::cerr << filename << ":" << sourceLine << " [" << keys << "]: "
              << message.substr(message.find(':') + 2) << '\n';
    return false;
  }
  config_ = std::move(candidate);
  if (!config_.silent)
    std::cout << "Parsed " << filename << ": mapSource="
              << (config_.mapSource == MapSource::Image ? "image" : "random")
              << '\n' << "Solver radius: " << config_.radius << '\n';
  return true;
}

const Config &ConfigParser::getConfig() const { return config_; }
} // namespace RS_marching
