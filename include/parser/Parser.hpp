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

struct Config {
  bool bumps = true;
  bool pose_randomness = true;
  int mode = 1;
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
  std::string imagePath = "C:\\..."; // or /home/...
  std::vector<Node> initialFrontline;
  int target_x;
  int target_y;
  float visibilityThreshold = 0.5;
  float angleThreshold = 45; // degrees
  float radius = 1;
  bool timer = true;
  bool greedy = true;
  bool visibilityBasedSolver = true;
  bool expandInObstacles = true;
  bool vstar = true;
  bool astar = true;
  bool distanceFunction = true;
  bool saveResults = true;
  bool saveCameFrom = true;
  bool saveGScore = true;
  bool saveFScore = true;
  bool saveLightSources = true;
  bool saveVisibilityField = true;
  bool saveSpeedField = true;
  bool savePath = true;
  bool saveDistanceFunction = true;
  bool silent = false;
  bool saveVisibilityBasedSolverImage = true;
  bool saveDistanceFunctionImage = true;
  int ballRadius = 5;
  int number_of_contour_lines = 50;
  int exit_it = 10;
};

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
  std::vector<Node> parseVectorString(const std::string &str);
};

} // namespace RS_marching
#endif
