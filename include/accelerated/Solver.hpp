#ifndef SOLVER_HPP
#define SOLVER_HPP

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace RS_marching {

struct State {
  double x, y, theta;
};
struct pose {
  int x;
  int y;
  double theta;
};
struct primitives {
  std::vector<char> motionTypes;
  std::vector<double> motionLengths;
  std::vector<int> motionDirections;
  std::vector<pose> waypoints;
  double turning_radius = 1.0;
  void clear() {
    motionTypes.clear();
    motionLengths.clear();
    motionDirections.clear();
    waypoints.clear();
    turning_radius = 1.0;
  }
};

class Solver {
public:
  // Reusable for one fixed source/target-position pair. The evaluated source
  // heading and radius must match those used by preparePlanar; only the target
  // heading may vary between solves.
  struct PlanarPreparation {
    double xn = 0.0;
    double yn = 0.0;
    double x = 0.0;
    double y = 0.0;
    double beta0 = 0.0;
    int quadrant = 1;
  };
  static_assert(sizeof(PlanarPreparation) == 48);
#ifdef RS_COUNT_WORK
  inline static thread_local std::uint64_t query_count = 0;
#endif
  Solver() = default;
  // Proposed method
  void acceleratedRSPlanner(const State &from, const State &to, double &d,
                            const double r, int &cond, int &Q,
                            bool offset = false);
  [[nodiscard]] static PlanarPreparation preparePlanar(
      double from_x, double from_y, double to_x, double to_y,
      double evaluated_source_heading, double radius);
  void acceleratedRSPlanner(const PlanarPreparation &preparation,
                            double evaluated_source_heading,
                            double target_heading, double &d, double radius,
                            int &cond, int &Q);

  void getMotionTypes(const int condition, char motionTypes[5]) const;
  // Allocation-free, non-mutating export of the already selected word.
  // Physical lengths and the legacy sign-of-zero gear convention match getPath.
  [[nodiscard]] bool selectedWord(int condition, int quadrant, double radius,
                                  char types[5], int gears[5],
                                  double lengths[5], std::size_t &count) const;
  void backProjectMotion(const int condition, const int Q, char motionTypes[5]);
  void getPath(const int condition, char motionTypes[5],
               std::vector<State> &path, const double r,
               primitives &primitives_) const;

private:
  // Main partitions of the algorithm
  void setA(const double x, const double y, const double thetaf, const double r,
            const double beta0, int &cond, double &d, const double xn,
            const double x0, const double yn);
  void setB(const double x, const double y, const double thetaf, const double r,
            int &cond, double &d, const double beta0);

  inline double wrapToPi(const double angle) const;
  inline double wrapTo2Pi(const double angle) const;
  inline double euclideanDistance(const double x1, const double y1,
                                  const double x2, const double y2) const {
    return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
  }

  /*!
   * @brief Compute paths.
   */
  inline void P1(const double x, const double y, const double thetaf,
                 double &d);
  inline void P2(const double x, const double y, const double thetaf,
                 double &d);
  inline void P3(const double x, const double y, const double thetaf,
                 double &d);
  inline void P4(const double x, const double y, const double thetaf,
                 double &d);
  inline void P5(const double x, const double y, const double thetaf,
                 double &d);
  inline void P6(const double x, const double y, const double thetaf,
                 double &d);
  inline void P7(const double x, const double y, const double thetaf,
                 double &d);
  inline void P8(const double x, const double y, const double thetaf,
                 double &d);
  inline void P9(const double x, const double y, const double thetaf,
                 double &d);
  inline void P10(const double x, const double y, const double thetaf,
                  double &d);
  inline void P11(const double x, const double y, const double thetaf,
                  double &d);
  inline void P12(const double x, const double y, const double thetaf,
                  double &d);
  inline void P13(const double x, const double y, const double thetaf,
                  double &d);
  inline void P14(const double x, const double y, const double thetaf,
                  double &d);
  inline void P15(const double x, const double y, const double thetaf,
                  double &d);
  inline void P16(const double x, const double y, const double thetaf,
                  double &d);
  inline void P17(const double x, const double y, const double thetaf,
                  double &d);
  inline void P18(const double x, const double y, const double thetaf,
                  double &d);
  inline void P19(const double x, const double y, const double thetaf,
                  double &d);
  inline void P20(const double x, const double y, const double thetaf,
                  double &d);

  // 21x5 matrix, holds segments lengths
  double lengths_[21][5] = {
      {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}};
  // Centers of the circles
  double c0Lx_ = 0, c0Rx_ = 0, c0Ly_ = 0, c0Ry_ = 0;
  double cfLx_ = 0, cfRx_ = 0, cfLy_ = 0, cfRy_ = 0;
  // Distances between circles & threshold
  double RR = 0, LL = 0, RL = 0, LR = 0, limit = 0;
};

} // namespace RS_marching

#endif // SOLVER_HPP
