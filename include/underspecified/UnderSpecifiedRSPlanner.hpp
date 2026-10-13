#ifndef UNDERSPECIFIED_RS_PLANNER_HPP
#define UNDERSPECIFIED_RS_PLANNER_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <numbers>

namespace RS_marching {

struct IntersectionHeadingCandidates {
  std::array<double, 4> values{};
  std::size_t count = 0;
};

// Geometry of the selected URS endpoint region, in the existing pi/2 frame.
// This is a construction descriptor, not a clearance/optimality certificate.
struct UrsConnectorGeometry {
  int region = 0;
  int quadrant = 0;
  double first_arc = 0.0; // normalized; used by the two-arc region
  double straight = 0.0; // physical length; used by tangent regions
};

class UnderSpecifiedRSPlanner {
public:
#ifdef RS_COUNT_WORK
  inline static thread_local std::uint64_t query_count = 0;
#endif
  UnderSpecifiedRSPlanner() = default;
  [[nodiscard]] IntersectionHeadingCandidates getIntersectionHeadings(
      double x0, double y0, double xn, double yn, double r,
      double theta) const;
  void getOmega(const double x0, const double y0, const double xn,
                const double yn, const double r, const double theta,
                double &omega, const bool offset = false,
                UrsConnectorGeometry *geometry = nullptr) const;
  inline void case1(const double x0, const double y0, const double xn,
                    const double yn, const double r, double &omega) const;
  inline void case2(const double x0, const double y0, const double xn,
                    const double yn, const double r, double &omega) const;
  inline void case3(const double x0, const double y0, const double xn,
                    const double yn, const double r, double &gamma) const;

private:
  inline double get2norm(const double x1, const double y1, const double x2,
                         const double y2) const;

  // Constant expression, fixed theta0 for the solution
  static constexpr double theta0 = std::numbers::pi / 2;

};

} // namespace RS_marching
#endif
