#include "underspecified/UnderSpecifiedRSPlanner.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

using namespace std::numbers;

namespace RS_marching {
namespace {

constexpr double kGeometryTolerance = 1e-12;
constexpr double kHeadingTolerance =
    64.0 * std::numeric_limits<double>::epsilon();

double wrapHeading(double theta) {
  theta = std::fmod(theta + pi, 2.0 * pi);
  if (theta < 0.0) {
    theta += 2.0 * pi;
  }
  return theta - pi;
}

} // namespace

IntersectionHeadingCandidates UnderSpecifiedRSPlanner::getIntersectionHeadings(
    const double x0, const double y0, const double xn, const double yn,
    const double r, const double theta) const {
  IntersectionHeadingCandidates result;
  if (!std::isfinite(x0) || !std::isfinite(y0) || !std::isfinite(xn) ||
      !std::isfinite(yn) || !std::isfinite(r) || r <= 0.0 ||
      !std::isfinite(theta)) {
    return result;
  }
  const double dx = (xn - x0) / r;
  const double dy = (yn - y0) / r;
  const double target_x = std::sin(theta) * dx - std::cos(theta) * dy;
  const double target_y = std::cos(theta) * dx + std::sin(theta) * dy;
  const auto append = [&](const double canonical_heading) {
    const double heading =
        wrapHeading(canonical_heading + theta - pi / 2.0);
    for (std::size_t index = 0; index < result.count; ++index) {
      if (std::abs(wrapHeading(heading - result.values[index])) <=
          kHeadingTolerance) {
        return;
      }
    }
    result.values[result.count++] = heading;
  };
  for (const double side : {-1.0, 1.0}) {
    const double center_dx = target_x - side;
    const double distance = std::hypot(center_dx, target_y);
    if (distance <= kGeometryTolerance ||
        distance > 3.0 + kGeometryTolerance ||
        distance < 1.0 - kGeometryTolerance) {
      continue;
    }
    const double along = (distance * distance + 3.0) / (2.0 * distance);
    const double transverse =
        std::sqrt(std::max(0.0, 4.0 - along * along));
    const double ux = center_dx / distance;
    const double uy = target_y / distance;
    for (const double sign : {-1.0, 1.0}) {
      const double center_x = side + along * ux - sign * transverse * uy;
      const double center_y = along * uy + sign * transverse * ux;
      append(std::atan2(target_y - center_y, target_x - center_x) +
             side * pi / 2.0);
      if (transverse <= kGeometryTolerance) {
        break;
      }
    }
  }
  return result;
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void UnderSpecifiedRSPlanner::getOmega(const double x0, const double y0,
                                       const double xf, const double yf,
                                       const double r, const double theta,
                                       double &omega, const bool offset,
                                       UrsConnectorGeometry *const geometry) const {
#ifdef RS_COUNT_WORK
  ++query_count;
#endif
  int region = 0, quadrant = 0;
  double first_arc = 0.0;
  if (geometry) *geometry = {};
  // rotate to the local frame
  const double dx = xf - x0;
  const double dy = yf - y0;
  double xn, yn;
  if (offset) {
    xn = dx * cos(theta - pi / 2) + dy * sin(theta - pi / 2);
    yn = -dx * sin(theta - pi / 2) + dy * cos(theta - pi / 2);
  } else {
    xn = dx * cos(theta) + dy * sin(theta);
    yn = -dx * sin(theta) + dy * cos(theta);
  }

  // centers of left and right side circles
  // theta0 is fixed at pi/2 in this source-local frame.
  const double cxr0 = r;
  const double cyr0 = 0;
  const double cxl0 = -r;
  const double cyl0 = 0;

  const double d1 = get2norm(xn, yn, cxr0, cyr0);
  const double d2 = get2norm(xn, yn, cxl0, cyl0);

  if (xn >= 0 && yn >= 0) {
    // Q1
    if ((d1 >= r) && ((yn >= r) | (xn - cxr0 < 0))) {
      region = 1; quadrant = 1;
      omega = atan2(yn - cyr0, xn - cxr0) + acos(r / d1) - pi / 2;
    } else if ((d2 >= sqrt(5) * r) && ((yn < r) && (xn - cxr0 > 0))) {
      region = 2; quadrant = 1;
      case2(0, 0, xn, yn, r, omega);
      omega = theta0 - (pi - omega);
    } else if ((d2 <= sqrt(5) * r) && (d1 <= r)) {
      region = 3; quadrant = 1;
      double gamma;
      case3(0, 0, xn, yn, r, gamma);

      first_arc = gamma;
      const double px = cxl0 + 2 * r * cos(gamma);
      const double py = cyl0 - 2 * r * sin(gamma);

      omega = theta0 - (pi - atan2(yn - py, xn - px));
    }
    // Q2
  } else if (xn <= 0 && yn >= 0) {
    if ((d2 >= r) && ((yn >= r) | (xn - cxl0 > 0))) {
      region = 1; quadrant = 2;
      omega = pi / 2 + atan2(yn - cyl0, xn - cxl0) - acos(r / d2);
    } else if ((d1 >= sqrt(5) * r) && ((yn <= r) && (xn - cxl0 < 0))) {
      region = 2; quadrant = 2;
      case2(xn, 0, 0, yn, r, omega);
      omega = theta0 + (pi - omega);
    } else if ((d1 <= sqrt(5) * r) && (d2 <= r)) {
      region = 3; quadrant = 2;
      double gamma;
      case3(xn, 0, 0, yn, r, gamma);

      first_arc = gamma;
      const double px = cxr0 + 2 * r * cos(pi - gamma);
      const double py = cyr0 - 2 * r * sin(pi - gamma);

      omega = theta0 + atan2(yn - py, xn - px);
    }
    // Q3
  } else if (xn <= 0 && yn <= 0) {
    if ((d2 >= r) && ((yn <= -r) | (xn - cxl0 > 0))) {
      region = 1; quadrant = 3;
      omega = pi / 2 + atan2(yn - cyl0, xn - cxl0) + acos(r / d2);
    } else if ((d1 >= sqrt(5) * r) && ((yn >= -r) && (xn - cxl0 < 0))) {
      region = 2; quadrant = 3;
      case2(xn, yn, 0, 0, r, omega);
      omega = theta0 - (pi - omega);
    } else if ((d1 <= sqrt(5) * r) && (d2 <= r)) {
      region = 3; quadrant = 3;

      double gamma;
      case3(xn, yn, 0, 0, r, gamma);

      first_arc = gamma;
      const double px = cxr0 + 2 * r * cos(pi - gamma);
      const double py = cyr0 + 2 * r * sin(pi - gamma);

      omega = theta0 + atan2(yn - py, xn - px);
    }
    // Q4
  } else if (xn >= 0 && yn <= 0) {
    if ((d1 >= r) && ((yn <= -r) | (xn - cxr0 < 0))) {
      region = 1; quadrant = 4;
      omega = atan2(yn - cyr0, xn - cxr0) - acos(r / d1) + 3 * pi / 2.0;
    } else if ((d2 >= sqrt(5) * r) && ((yn >= -r) && (xn - cxr0 > 0))) {
      region = 2; quadrant = 4;
      case2(0, yn, xn, 0, r, omega);
      omega = theta0 + (pi - omega);
    } else if ((d2 <= sqrt(5) * r) && (d1 <= r)) {
      region = 3; quadrant = 4;
      double gamma;
      case3(0, yn, xn, 0, r, gamma);

      first_arc = gamma;
      const double px = cxl0 + 2 * r * cos(gamma);
      const double py = cyl0 + 2 * r * sin(gamma);
      omega = theta0 + (pi + atan2(yn - py, xn - px));
    }
  }
  if (geometry && region != 0 && std::isfinite(omega)) {
    geometry->region = region;
    geometry->quadrant = quadrant;
    geometry->first_arc = first_arc;
    if (region != 3) {
      const double side = quadrant == 1 || quadrant == 4 ? -1.0 : 1.0;
      const double gear = quadrant <= 2 ? 1.0 : -1.0;
      // Project onto the selected tangent. Unlike sqrt(d*d-r*r), this
      // remains well-conditioned at a zero straight tail.
      const double center_shift = region == 1 ? side * r : -side * r;
      bool algebraic_straight = false;
      if (region == 1) {
        const double tangent_distance = quadrant == 1 || quadrant == 4 ? d1 : d2;
        if (r > 0.0 && tangent_distance > 1.01 * r) {
          const double q = (tangent_distance - r) * (tangent_distance + r);
          if (q >= std::numeric_limits<double>::min() && q <= std::numeric_limits<double>::max()) {
            geometry->straight = std::sqrt(q);
            algebraic_straight = true;
          }
        }
      }
      if (!algebraic_straight)
        geometry->straight = gear * ((xn + center_shift) * std::cos(omega) +
                                     yn * std::sin(omega));
      if (region == 2) geometry->straight -= 2.0 * r;
    }
  }
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
inline void UnderSpecifiedRSPlanner::case1(const double x0, const double y0,
                                           const double xn, const double yn,
                                           const double r,
                                           double &omega) const {
  omega = std::numeric_limits<double>::quiet_NaN();
  if (!std::isfinite(x0) || !std::isfinite(y0) || !std::isfinite(xn) ||
      !std::isfinite(yn) || !std::isfinite(r) || r <= 0.0) {
    return;
  }
  const double x = (xn - x0) / r;
  const double y = (yn - y0) / r;
  if (!std::isfinite(x) || !std::isfinite(y) || x < 0.0 || y < 0.0) {
    return;
  }
  if (x == 0.0 && y == 0.0) {
    omega = 0.0;
    return;
  }
  const double q = x * x + y * y;
  const double radicand = q - 2.0 * x;
  const double domain_tolerance =
      64.0 * std::numeric_limits<double>::epsilon() * (1.0 + 2.0 * x + q);
  if (!std::isfinite(q) || !std::isfinite(radicand) ||
      !std::isfinite(domain_tolerance) || radicand < -domain_tolerance) {
    return;
  }
  const double root = std::sqrt(radicand < 0.0 ? 0.0 : radicand);
  const double value = std::atan2(root, -1.0) - std::atan2(y, x - 1.0);
  if (std::isfinite(root) && std::isfinite(value)) {
    omega = value;
  }
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
inline void UnderSpecifiedRSPlanner::case2(const double x0, const double y0,
                                           const double xn, const double yn,
                                           const double r,
                                           double &omega) const {
  omega = std::numeric_limits<double>::quiet_NaN();
  if (!std::isfinite(x0) || !std::isfinite(y0) || !std::isfinite(xn) ||
      !std::isfinite(yn) || !std::isfinite(r) || r <= 0.0) {
    return;
  }
  const double x = (xn - x0) / r;
  const double y = (yn - y0) / r;
  if (!std::isfinite(x) || !std::isfinite(y) || x < 0.0 || y < 0.0) {
    return;
  }
  if (x == 0.0 && y == 0.0) {
    omega = 0.0;
    return;
  }
  const double q = x * x + y * y;
  const double radicand = q + 2.0 * x;
  if (!std::isfinite(q) || !std::isfinite(radicand) || radicand < 0.0) {
    return;
  }
  const double root = std::sqrt(radicand);
  const double value = std::atan2(root, 1.0) + std::atan2(y, 1.0 + x);
  if (std::isfinite(root) && std::isfinite(value)) {
    omega = value;
  }
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
inline void UnderSpecifiedRSPlanner::case3(const double x0, const double y0,
                                           const double xn, const double yn,
                                           const double r,
                                           double &gamma) const {
  gamma = std::numeric_limits<double>::quiet_NaN();
  if (!std::isfinite(x0) || !std::isfinite(y0) || !std::isfinite(xn) ||
      !std::isfinite(yn) || !std::isfinite(r) || r <= 0.0) {
    return;
  }

  const double x = (xn - x0) / r;
  const double y = (yn - y0) / r;
  if (!std::isfinite(x) || !std::isfinite(y) || x < 0.0 || y < 0.0) {
    return;
  }
  if (x == 0.0 && y == 0.0) {
    gamma = 0.0;
    return;
  }

  const double q = x * x + y * y;
  const double z = 2.0 * x + q;
  const double e = 2.0 * x - q;
  const double domain_tolerance =
      64.0 * std::numeric_limits<double>::epsilon() * (1.0 + 2.0 * x + q);
  if (!std::isfinite(q) || !std::isfinite(z) || !std::isfinite(e) ||
      z < 0.0 || z > 4.0 + domain_tolerance || e < -domain_tolerance) {
    return;
  }

  const double radicand = z * (8.0 - z);
  if (!std::isfinite(radicand) || radicand < 0.0) {
    return;
  }
  const double root = std::sqrt(radicand);
  const double denominator = root + 4.0 * y;
  if (!std::isfinite(root) || !std::isfinite(denominator)) {
    return;
  }
  gamma = 2.0 * std::atan2(e, denominator);
  if (!std::isfinite(gamma)) {
    gamma = std::numeric_limits<double>::quiet_NaN();
  }
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
inline double UnderSpecifiedRSPlanner::get2norm(const double x1,
                                                const double y1,
                                                const double x2,
                                                const double y2) const {
  return std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

} // namespace RS_marching
