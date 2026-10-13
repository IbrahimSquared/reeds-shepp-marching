#ifndef CONFIG_PARSER_HPP
#define CONFIG_PARSER_HPP

#include <string>
#include <vector>

namespace RS_marching {

using size_t = std::size_t;

struct Node {
  int x, y;
  double f;
  bool operator<(const Node &other) const { return f > other.f; }
};

enum class MapSource { Random, Image };

struct Config {
  bool pose_randomness = true;
  MapSource mapSource = MapSource::Random;
  int GD_bisection = 1;
  size_t ncols = 100;
  size_t nrows = 100;
  int nb_of_obstacles = 10;
  size_t minWidth = 10;
  size_t maxWidth = 20;
  size_t minHeight = 10;
  size_t maxHeight = 20;
  float speedValue = 2.0;
  bool randomSeed = true;
  int seedValue = 0;
  std::string imagePath = "C:\\...";
  std::vector<Node> initialFrontline;
  double radius = 1.0;
  bool timer = true;
  bool saveResults = true;
  bool saveVisibilityField = true;
  bool saveSpeedField = true;
  bool silent = false;
  int exit_it = 10;
};

// Validate settings at the environment boundary, including direct C++ callers.
// Throws std::invalid_argument with the offending key.
void validateConfig(const Config &config);

class ConfigParser {
public:
  static ConfigParser &getInstance() {
    static ConfigParser instance;
    return instance;
  }

  bool parse(const std::string &filename);
  const Config &getConfig() const;

  ConfigParser(ConfigParser const &) = delete;
  void operator=(ConfigParser const &) = delete;

private:
  ConfigParser() = default;
  Config config_;
};

} // namespace RS_marching
#endif
