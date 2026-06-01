#include "marching/Marching.hpp"
#include "thirdparty/nlohmann/json.hpp"
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <misc/Colormod.hpp>
#include <ostream>
#include <random>

namespace RS_marching {

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
template <typename T>
void saveField(const RS_marching::Field<T> &field, const std::string &name) {
  std::string outputFilePath = "./output/" + name + ".txt";
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
  try {
    std::fstream of(outputFilePath, std::ios::out | std::ios::trunc);
    std::ofstream file(outputFilePath);
    for (size_t j = 0; j < field.ny(); ++j) {
      for (size_t i = 0; i < field.nx(); ++i) {
        file << field(i, j) << " ";
      }
      file << "\n";
    }
    of.close();
    std::cout << "Saved " << name << std::endl;
  } catch (const std::exception &e) {
    std::cerr << "Failed to open output file " << outputFilePath << std::endl;
    return;
  }
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
template <typename T> auto durationInMicroseconds(T start, T end) {
  return std::chrono::duration_cast<std::chrono::microseconds>(end - start)
      .count();
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
inline double wrapToPi(double angle) {
  angle = fmod(angle + pi, 2 * pi);
  if (angle < 0)
    angle += 2 * pi;
  return angle - pi;
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void savePrimitivesToJson(const primitives &p, const double r,
                          const std::string &filename) {
  nlohmann::json j;

  std::vector<std::string> motionTypesStr;
  for (char c : p.motionTypes) {
    std::string s(1, c);
    motionTypesStr.push_back(s);
  }

  j["motionTypes"] = motionTypesStr;
  j["motionLengths"] = p.motionLengths;
  j["motionDirections"] = p.motionDirections;

  std::vector<nlohmann::json> waypointsJson;
  for (const pose &p : p.waypoints) {
    nlohmann::json jp;
    jp["x"] = p.x;
    jp["y"] = p.y;
    jp["theta"] = p.theta;
    waypointsJson.push_back(jp);
  }
  j["waypoints"] = waypointsJson;
  j["MinTurningRadius"] = r;

  std::ofstream file(filename);
  file << j.dump(4);
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
struct pose_d {
  double x;
  double y;
  double theta;
};
std::vector<pose_d> waypoints_d_;

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
int generatePathForwardSimOptimized(
    const primitives &primitives_,
    std::shared_ptr<Field<double>> sharedVisibilityField_,
    const double visibilityThreshold, const std::string &filename) {
  int count = 0;
  double r_ = primitives_.turning_radius;
  waypoints_d_.clear();
  double currentTheta = primitives_.waypoints[0].theta;
  const double x1 = primitives_.waypoints[0].x;
  const double y1 = primitives_.waypoints[0].y;
  double xx = x1, yy = y1, t = currentTheta;
  waypoints_d_.push_back({x1, y1, currentTheta});

  for (size_t i = 0; i < primitives_.motionTypes.size(); ++i) {
    const char motionType = primitives_.motionTypes[i];
    const double motionLength = primitives_.motionLengths[i];
    const int motionDirection =
        primitives_.motionDirections[i]; // Use int for direction

    if (motionType == 'N') {
      break;
    } else if (motionType == 'L') {
      // Simulate left arc - Optimized
      const double cxl =
          xx -
          r_ * cos(currentTheta - pi / 2.0); // Corrected pi/2.0 for clarity
      const double cyl =
          yy -
          r_ * sin(currentTheta - pi / 2.0); // Corrected pi/2.0 for clarity
      const double arcAngle = motionLength / r_;
      const int nb_of_steps = 10;
      const double step_ = arcAngle / nb_of_steps;

      for (int j = 1; j <= nb_of_steps; ++j) {
        t = currentTheta + motionDirection * j * step_;
        xx = cxl + r_ * cos(t - pi / 2.0); // Corrected pi/2.0 for clarity
        yy = cyl + r_ * sin(t - pi / 2.0); // Corrected pi/2.0 for clarity
        if (sharedVisibilityField_->get(static_cast<int>(xx),
                                        static_cast<int>(yy)) <
            visibilityThreshold) {
          count++;
        }
        waypoints_d_.push_back({xx, yy, t});
      }
      currentTheta = t; // Moved currentTheta update outside inner loop for
                        // clarity, though it was already correct in original
    } else if (motionType == 'R') {
      // Simulate right arc - Optimized
      const double cxr =
          xx +
          r_ * cos(currentTheta - pi / 2.0); // Corrected pi/2.0 for clarity
      const double cyr =
          yy +
          r_ * sin(currentTheta - pi / 2.0); // Corrected pi/2.0 for clarity
      const double arcAngle = motionLength / r_;
      const int nb_of_steps = 10;
      const double step_ = arcAngle / nb_of_steps;

      for (int j = 1; j <= nb_of_steps; ++j) {
        t = currentTheta - motionDirection * j * step_;
        xx = cxr - r_ * cos(t - pi / 2.0); // Corrected pi/2.0 for clarity
        yy = cyr - r_ * sin(t - pi / 2.0); // Corrected pi/2.0 for clarity
        if (sharedVisibilityField_->get(static_cast<int>(xx),
                                        static_cast<int>(yy)) <
            visibilityThreshold) {
          count++;
        }
        waypoints_d_.push_back({xx, yy, t});
      }
      currentTheta =
          t; // Moved currentTheta update outside inner loop for clarity
    } else if (motionType == 'S') {
      // Simulate straight - Corrected to use motionDirection for
      // forward/reverse
      const int nb_of_steps = 10;
      const double step_ = motionLength / nb_of_steps;
      const double segment_dx =
          step_ * cos(currentTheta) *
          motionDirection; // dx per step, using motionDirection
      const double segment_dy =
          step_ * sin(currentTheta) *
          motionDirection; // dy per step, using motionDirection

      for (int j = 1; j <= nb_of_steps; ++j) {
        xx += segment_dx;
        yy += segment_dy;
        if (sharedVisibilityField_->get(static_cast<int>(xx),
                                        static_cast<int>(yy)) <
            visibilityThreshold) {
          count++;
        }
        waypoints_d_.push_back(
            {xx, yy, currentTheta}); // currentTheta remains constant for
                                     // straight motion
      }
    }
  }

  // serialize path to .txt given file name
  std::ofstream file(filename);
  for (const auto &p : waypoints_d_) {
    file << p.x << " " << p.y << " " << p.theta << std::endl;
  }
  return count;
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
sf::Color getColor(double value) {
  const int color_index = 255 * value;
  double r, g, b;
  if (color_index < 32) {
    r = 0;
    g = 0;
    b = 0.5156 + 0.0156 * color_index;
  } else if (color_index < 96) {
    r = 0;
    g = 0.0156 + 0.9844 * (color_index - 32.0) / 64;
    b = 1;
  } else if (color_index < 158) {
    r = 0.0156 + (color_index - 96.0) / 64;
    g = 1;
    b = 0.9844 - (color_index - 96.0) / 64;
  } else if (color_index < 223) {
    r = 1;
    g = 1 - (color_index - 158.0) / 65;
    b = 0;
  } else {
    r = (2 - (color_index - 223.0) / 32) / 2.0;
    g = 0;
    b = 0;
  }
  return sf::Color(static_cast<sf::Uint8>(r * 255),
                   static_cast<sf::Uint8>(g * 255),
                   static_cast<sf::Uint8>(b * 255));
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
double wrapTo2Pi(double angle) {
  while (angle < 0) {
    angle += 2 * pi;
  }
  while (angle >= 2 * pi) {
    angle -= 2 * pi;
  }
  return angle;
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
namespace {

constexpr double kSameSideEps = 1e-9;

inline double clampUnit(const double value) {
  return std::max(-1.0, std::min(1.0, value));
}

inline void llPositivePivotCircle(const double x, const double y,
                                  const double r, const double theta,
                                  double &cx, double &cy) {
  cx = x - r * std::sin(theta);
  cy = y + r * std::cos(theta);
}

inline double llPositiveParentCircleX(const double x1, const double r) {
  return x1 - r;
}

inline double llPositiveParentCircleY(const double y1) { return y1; }

inline bool llPositiveSeedTheta(const double c1x, const double c1y,
                                const double x2, const double y2,
                                const double x3, const double y3,
                                const double r, const double theta_guess,
                                double &theta_out) {
  double c2x = 0.0;
  double c2y = 0.0;
  llPositivePivotCircle(x2, y2, r, theta_guess, c2x, c2y);
  const double h = std::hypot(x3 - c2x, y3 - c2y);
  if (h <= kSameSideEps) {
    return false;
  }
  const double beta = std::asin(clampUnit(r / h));
  theta_out = 0.5 * wrapTo2Pi(std::atan2(y3 - c2y, x3 - c2x)) +
              0.5 * beta +
              0.5 * wrapTo2Pi(std::atan2(c2y - c1y, c2x - c1x));
  return std::isfinite(theta_out);
}

inline bool llPositiveObjective(const double c1x, const double c1y,
                                const double x2, const double y2,
                                const double x3, const double y3,
                                const double r, const double theta,
                                double &value_out) {
  double c2x = 0.0;
  double c2y = 0.0;
  llPositivePivotCircle(x2, y2, r, theta, c2x, c2y);
  const double d1 = std::hypot(c2x - c1x, c2y - c1y);
  const double h2 = (x3 - c2x) * (x3 - c2x) + (y3 - c2y) * (y3 - c2y);
  const double r2 = r * r;
  if (d1 <= kSameSideEps || h2 <= r2 + kSameSideEps) {
    return false;
  }
  const double h = std::sqrt(h2);
  const double tail = std::sqrt(h2 - r2);
  const double beta = std::asin(clampUnit(r / h));
  const double alpha1 = wrapTo2Pi(std::atan2(c2y - c1y, c2x - c1x));
  const double alpha3 = wrapTo2Pi(std::atan2(y3 - c2y, x3 - c2x));
  value_out = d1 + tail + r * (alpha1 + beta + alpha3);
  return std::isfinite(value_out);
}

inline bool llPositiveGradient(const double c1x, const double c1y,
                               const double x2, const double y2,
                               const double x3, const double y3,
                               const double r, const double theta,
                               double &gradient_out) {
  double c2x = 0.0;
  double c2y = 0.0;
  llPositivePivotCircle(x2, y2, r, theta, c2x, c2y);
  const double dx_1 = c1x - x2;
  const double dy_1 = c1y - y2;
  const double dx_2 = x2 - x3;
  const double dy_2 = y2 - y3;
  const double d1 = std::hypot(c2x - c1x, c2y - c1y);
  const double h2 = (x3 - c2x) * (x3 - c2x) + (y3 - c2y) * (y3 - c2y);
  const double r2 = r * r;
  if (d1 <= kSameSideEps || h2 <= r2 + kSameSideEps) {
    return false;
  }
  const double h = std::sqrt(h2);
  const double d2 = std::sqrt(h2 - r2);
  const double c2 = d2 / h;
  if (d2 <= kSameSideEps || c2 <= kSameSideEps) {
    return false;
  }
  const double st2 = std::sin(theta);
  const double ct2 = std::cos(theta);
  gradient_out =
      r * ((r * (r + ct2 * dy_2 - st2 * dx_2)) / h2 +
           (r * r * (ct2 * dx_2 + st2 * dy_2)) / (c2 * h2 * h)) +
      (r * (ct2 * dx_1 + st2 * dy_1)) / d1 -
      r * (ct2 * dx_2 + st2 * dy_2) / d2;
  return std::isfinite(gradient_out);
}

inline bool llPositiveContactParentTheta(const double x1, const double y1,
                                         const double x2, const double y2,
                                         const double r,
                                         double &theta_out) {
  const double c = std::hypot(x2 - x1, y2 - y1);
  if (c <= kSameSideEps || c > 2.0 * r + kSameSideEps) {
    return false;
  }
  const double discriminant = 4.0 * r * r - c * c;
  if (discriminant < -kSameSideEps) {
    return false;
  }
  const double root = std::sqrt(std::max(0.0, discriminant));
  const double a = c * root + 2.0 * r * (x1 - x2);
  const double b = c * c + 2.0 * r * (y1 - y2);
  theta_out = -2.0 * std::atan2(a, b);
  return std::isfinite(theta_out);
}

inline bool llPositiveContactTargetTheta(const double x2, const double y2,
                                         const double x3, const double y3,
                                         const double r,
                                         double &theta_out) {
  const double c = std::hypot(x3 - x2, y3 - y2);
  if (c <= kSameSideEps || c > 2.0 * r + kSameSideEps) {
    return false;
  }
  const double discriminant = 4.0 * r * r - c * c;
  if (discriminant < -kSameSideEps) {
    return false;
  }
  const double root = std::sqrt(std::max(0.0, discriminant));
  const double a = c * root + 2.0 * r * (x2 - x3);
  const double b = c * c + 2.0 * r * (y3 - y2);
  theta_out = 2.0 * std::atan2(a, b);
  return std::isfinite(theta_out);
}

} // namespace

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
inline void Marching::RLPositive(const State s1, State &s2, State &s3,
                                 const Point p2_0, const Point p3_0) const {
  const double x1 = s1.x;
  const double y1 = s1.y;
  const double theta1 = s1.theta;
  const double r = r_;
  double x2 = s2.x;
  double y2 = s2.y;
  double theta2 = s2.theta;
  double x3 = s3.x;
  double y3 = s3.y;

  const double st1 = sin(theta1);
  const double ct1 = cos(theta1);
  const double cxr1 = x1 + r * st1 * st1 + r * ct1 * ct1;
  const double cyr1 = y1 - r * st1 * ct1 + r * ct1 * st1;
  double cxl2 = x2 - r * sin(theta2);
  double cyl2 = y2 + r * cos(theta2);

  // TODO: optimize fractions - inverse square root
  // check if modern compilers automatically optimize this
  const double frac = r / norm(cxl2, cyl2, x3, y3);
  double B;
  if (frac >= 1) {
    B = pi / 2;
  } else {
    B = asin(frac);
  }
  const double c1 = norm(cxr1, cyr1, cxl2, cyl2);
  const double c = c1 * c1 - 4 * r * r;
  double C;
  if (c <= 0) {
    C = pi / 2;
  } else {
    C = acos(sqrt(c) / c1);
  }
  theta2 = 0.5 * wrapToPi(atan2((y3 - cyl2), x3 - cxl2)) + 0.5 * B - 0.5 * C +
           0.5 * wrapTo2Pi(atan2(cyl2 - cyr1, cxl2 - cxr1));

  x2 = p2_0.x;
  y2 = p2_0.y;
  x3 = p3_0.x;
  y3 = p3_0.y;
  theta2 = theta2 + theta1 - pi / 2;
  cxl2 = x2 - r * sin(theta2);
  cyl2 = y2 + r * cos(theta2);

  const float D_norm = norm(cxl2, cyl2, x3, y3);
  if (D_norm < r) {
    const float c1 = norm(x2, y2, x3, y3);
    double sqrt_term = 4 * r * r - c1 * c1;
    if (sqrt_term < 0) {
      sqrt_term = 0; // Prevent NaN
    }
    const float A = c1 * sqrt(sqrt_term) + 2 * r * (x2 - x3);
    const float B = c1 * c1 + 2 * r * (y3 - y2);
    if (B == 0) {
      theta2 = (A > 0) ? pi : -pi;
    } else {
      theta2 = 2 * atan(A / B);
    }
    cxl2 = x2 - r * sin(theta2);
    cyl2 = y2 + r * cos(theta2);
  }

  double t2 = 0;
  urs_.getOmega(x2, y2, x3, y3, r, theta2, t2, true);
  t2 = t2 + theta2 - pi / 2;

  s2.theta = theta2;
  s3.theta = t2;
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
inline void Marching::RLNegative(const State s1, State &s2, State &s3,
                                 const Point p2_0, const Point p3_0) const {
  const double x1 = s1.x;
  const double y1 = s1.y;
  const double theta1 = s1.theta;
  const double r = r_;
  double x2 = s2.x;
  double y2 = s2.y;
  double x3 = s3.x;
  double y3 = s3.y;
  double theta2 = s2.theta;

  const double st1 = sin(theta1);
  const double ct1 = cos(theta1);
  double cxr1 = x1 + r * st1 * st1 + r * ct1 * ct1;
  double cyr1 = y1 - r * st1 * ct1 + r * ct1 * st1;
  double cxl2 = x2 - r * sin(theta2);
  double cyl2 = y2 + r * cos(theta2);

  // TODO: optimize fractions - inverse square root
  // check if modern compilers automatically optimize this
  const double frac = r / norm(cxl2, cyl2, x3, y3);
  double B;
  if (frac >= 1) {
    B = pi / 2;
  } else {
    B = asin(frac);
  }
  double C;
  const double c1 = norm(cxr1, cyr1, cxl2, cyl2);
  const double c = c1 * c1 - 4 * r * r;
  if (c <= 0) {
    C = pi / 2;
  } else {
    C = acos(sqrt(c) / c1);
  }
  theta2 = 0.5 * wrapTo2Pi(atan2((cyl2 - y3), cxl2 - x3)) - 0.5 * B + 0.5 * C +
           0.5 * wrapTo2Pi(atan2(cyr1 - cyl2, cxr1 - cxl2));

  x2 = p2_0.x;
  y2 = p2_0.y;
  x3 = p3_0.x;
  y3 = p3_0.y;
  theta2 = theta2 + theta1 - pi / 2;
  cxl2 = x2 - r * sin(theta2);
  cyl2 = y2 + r * cos(theta2);
  cxr1 = x1 + r * sin(theta1);
  cyr1 = y1 - r * cos(theta1);

  double cxr2 = x2 + r * sin(theta2);
  double cyr2 = y2 - r * cos(theta2);
  const float r2r1 =
      (cxr2 - cxr1) * (cxr2 - cxr1) + (cyr2 - cyr1) * (cyr2 - cyr1);
  const float r1l2 = norm(cxr1, cyr1, cxl2, cyl2);
  if (r2r1 < r * r) {
    urs_.getOmega(x1, y1, x2, y2, r_, theta1, theta2, true);
    theta2 += theta1 - pi / 2;
  } else if (r1l2 < 2 * r) {
    float sqrt_term = -(c1 * c1 - r * r) * (c1 * c1 - 9 * r * r);
    if (sqrt_term < 0) {
      // Handle the case where sqrt is not valid
      // Possible fixes:
      // 1. Clamp it to zero (forces a degenerate case)
      // 2. Skip the computation
      // 3. Throw an error if this case should never occur
      sqrt_term = 0; // or handle differently
    }
    const float A = sqrt(sqrt_term) - 2 * r * (cxr1 - x2);
    const float B = c1 * c1 - 3 * r * r + 2 * r * (cyr1 - y2);
    if (B == 0) {
      theta2 = (A > 0) ? pi : -pi;
    } else {
      theta2 = 2 * atan(A / B);
    }
    // const float c1 = norm(cxr1, cyr1, x2, y2);
    // const float A = sqrt(std::fabs((c1 * c1 - r * r) * (c1 * c1 - 9 * r *
    // r))) -
    //                 2 * r * (cxr1 - x2);
    // // const float A =
    // //     sqrt(-(c1 * c1 - r * r) * (c1 * c1 - 9 * r * r)) - 2 * r *
    // (cxr1 -
    // //     x2);
    // const float B = c1 * c1 - 3 * r * r + 2 * r * (cyr1 - y2);
    // theta2 = 2 * atan(A / B);
    cxl2 = x2 - r * sin(theta2);
    cyl2 = y2 + r * cos(theta2);
  }

  const float D_norm = norm(cxl2, cyl2, x3, y3);
  if (D_norm < r) {
    const float c1 = norm(x2, y2, x3, y3);
    double sqrt_term = 4 * r * r - c1 * c1;
    if (sqrt_term < 0) {
      sqrt_term = 0; // Prevent NaN
    }
    const float A = c1 * sqrt(sqrt_term) + 2 * r * (x2 - x3);
    const float B = c1 * c1 + 2 * r * (y2 - y3);
    // const float A = c1 * sqrt(4 * r * r - c1 * c1) + 2 * r * (x2 - x3);
    // const float B = c1 * c1 + 2 * r * (y2 - y3);
    if (B == 0) {
      theta2 = (A > 0) ? 0 : 2 * pi;
    } else {
      theta2 = pi - 2 * atan(A / B);
    }

    cxl2 = x2 - r * sin(theta2);
    cyl2 = y2 + r * cos(theta2);
  }

  double t2 = 0;
  urs_.getOmega(x2, y2, x3, y3, r, theta2, t2, true);
  t2 = t2 + theta2 - pi / 2;

  s2.theta = theta2;
  s3.theta = t2;
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
inline void Marching::LRPositive(const State s1, State &s2, State &s3,
                                 const Point p2_0, const Point p3_0) const {
  const double x1 = s1.x;
  const double y1 = s1.y;
  const double theta1 = s1.theta;
  const double r = r_;
  double x2 = s2.x;
  double y2 = s2.y;
  double theta2 = s2.theta;
  double x3 = s3.x;
  double y3 = s3.y;

  const double st1 = sin(theta1);
  const double ct1 = cos(theta1);
  const double cxl1 = x1 - r * st1 * st1 - r * ct1 * ct1;
  const double cyl1 = y1 + r * st1 * ct1 - r * ct1 * st1;
  double cxr2 = x2 + r * sin(theta2);
  double cyr2 = y2 - r * cos(theta2);

  const double frac = r / norm(cxr2, cyr2, x3, y3);
  double B;
  if (frac >= 1) {
    B = pi / 2;
  } else {
    B = asin(frac);
  }
  const double c1 = norm(cxl1, cyl1, cxr2, cyr2);
  const double c = c1 * c1 - 4 * r * r;
  double C;
  if (c <= 0) {
    C = pi / 2;
  } else {
    C = acos(sqrt(c) / c1);
  }

  // TODO: missing -norm compared to MATLAB code
  theta2 = 0.5 * wrapTo2Pi(atan2(y3 - cyr2, x3 - cxr2)) - 0.5 * B + 0.5 * C +
           0.5 * wrapTo2Pi(atan2(cyr2 - cyl1, cxr2 - cxl1));

  x2 = p2_0.x;
  y2 = p2_0.y;
  x3 = p3_0.x;
  y3 = p3_0.y;
  theta2 = theta2 + theta1 - pi / 2;
  cxr2 = x2 + r * sin(theta2);
  cyr2 = y2 - r * cos(theta2);

  double cxl2 = x2 - r * sin(theta2);
  double cyl2 = y2 + r * cos(theta2);
  double cxr1 = x1 + r * sin(theta1);
  double cyr1 = y1 - r * cos(theta1);

  const float l2r1 = norm(cxl2, cyl2, cxr1, cyr1);
  if (l2r1 < 2 * r) {
    // const float c1 = norm(cxr1, cyr1, x2, y2);
    // const float A =
    //     sqrt(-(c1 * c1 - r * r) * (c1 * c1 - 9 * r * r)) + 2 * r * (cxr1 -
    //     x2);
    // const float B = c1 * c1 - 3 * r * r - 2 * r * (y2 - cyr1);
    // theta2 = -2 * atan(A / B);

    const float c1 = norm(cxr1, cyr1, x2, y2);
    float sqrt_term = -(c1 * c1 - r * r) * (c1 * c1 - 9 * r * r);
    if (sqrt_term < 0) {
      sqrt_term = 0; // Prevent NaN
    }
    const float A = sqrt(sqrt_term) + 2 * r * (cxr1 - x2);
    const float B = c1 * c1 - 3 * r * r - 2 * r * (y2 - cyr1);
    // theta2 = -2 * atan(A / B);
    if (B == 0) {
      theta2 = (A > 0) ? -pi : pi;
    } else {
      theta2 = -2 * atan(A / B);
    }

    cxr2 = x2 + r * sin(theta2);
    cyr2 = y2 - r * cos(theta2);
  }

  const float D_norm = norm(cxr2, cyr2, x3, y3);
  if (D_norm < r) {
    const float c1 = norm(x2, y2, x3, y3);
    double sqrt_term = 4 * r * r - c1 * c1;
    if (sqrt_term < 0) {
      sqrt_term = 0; // Prevent NaN
    }
    const float A = c1 * sqrt(sqrt_term) + 2 * r * (x2 - x3);
    // const float A = c1 * sqrt(4 * r * r - c1 * c1) + 2 * r * (x2 - x3);
    const float B = c1 * c1 + 2 * r * (y2 - y3);
    // theta2 = -2 * atan(A / B);
    if (B == 0) {
      theta2 = (A > 0) ? -pi : pi;
    } else {
      theta2 = -2 * atan(A / B);
    }
    cxr2 = x2 + r * sin(theta2);
    cyr2 = y2 - r * cos(theta2);
  }

  double t2 = 0;
  urs_.getOmega(x2, y2, x3, y3, r, theta2, t2, true);
  t2 = t2 + theta2 - pi / 2;

  s2.theta = theta2;
  s3.theta = t2;
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
inline void Marching::LRNegative(const State s1, State &s2, State &s3,
                                 const Point p2_0, const Point p3_0) const {
  const double x1 = s1.x;
  const double y1 = s1.y;
  double theta1 = s1.theta;
  const double r = r_;
  double x2 = s2.x;
  double y2 = s2.y;
  double theta2 = s2.theta;
  double x3 = s3.x;
  double y3 = s3.y;

  const double st1 = sin(theta1);
  const double ct1 = cos(theta1);
  double cxl1 = x1 - r * st1 * st1 - r * ct1 * ct1;
  double cyl1 = y1 + r * st1 * ct1 - r * ct1 * st1;
  double cxr2 = x2 + r * sin(theta2);
  double cyr2 = y2 - r * cos(theta2);

  const double frac = r / norm(cxr2, cyr2, x3, y3);
  double B;
  if (frac >= 1) {
    B = pi / 2;
  } else {
    B = asin(frac);
  }
  const double c1 = norm(cxl1, cyl1, cxr2, cyr2);
  const double c = c1 * c1 - 4 * r * r;
  double C;
  if (c <= 0) {
    C = pi / 2;
  } else {
    C = acos(sqrt(c) / c1);
  }

  // TODO: missing -norm compared to MATLAB code
  theta2 = 0.5 * wrapTo2Pi(atan2((y3 - cyr2), x3 - cxr2)) + 0.5 * B - 0.5 * C +
           0.5 * wrapTo2Pi(atan2(cyr2 - cyl1, cxr2 - cxl1)) + pi;

  x2 = p2_0.x;
  y2 = p2_0.y;
  x3 = p3_0.x;
  y3 = p3_0.y;
  theta2 = theta2 + theta1 - pi / 2;
  cxl1 = x1 - r * sin(theta1);
  cyl1 = y1 + r * cos(theta1);

  double cxl2 = x2 - r * sin(theta2);
  double cyl2 = y2 + r * cos(theta2);
  cxr2 = x2 + r * sin(theta2);
  cyr2 = y2 - r * cos(theta2);

  const float l2l1 =
      (cxl2 - cxl1) * (cxl2 - cxl1) + (cyl2 - cyl1) * (cyl2 - cyl1);
  const float l1r2 = norm(cxl1, cyl1, cxr2, cyr2);
  if (l2l1 < r * r) {
    urs_.getOmega(x1, y1, x2, y2, r_, theta1, theta2, true);
    // theta1 = thetas_(x1, y1) - pi / 2;
    theta2 += theta1 - pi / 2;
  } else if (l1r2 < 2 * r) {
    // const float c1 = norm(cxl1, cyl1, x2, y2);
    // const float A =
    //     sqrt(-(c1 * c1 - r * r) * (c1 * c1 - 9 * r * r)) - 2 * r * (cxl1 -
    //     x2);
    // const float B = c1 * c1 - 3 * r * r + 2 * r * (y2 - cyl1);
    // theta2 = -2 * atan(A / B);

    const float c1 = norm(cxl1, cyl1, x2, y2);
    float sqrt_term = -(c1 * c1 - r * r) * (c1 * c1 - 9 * r * r);
    if (sqrt_term < 0) {
      sqrt_term = 0; // Prevent NaN
    }
    const float A = sqrt(sqrt_term) - 2 * r * (cxl1 - x2);
    const float B = c1 * c1 - 3 * r * r + 2 * r * (y2 - cyl1);
    if (B == 0) {
      theta2 = (A > 0) ? -pi : pi; // theta2 = -π or π
    } else {
      theta2 = -2 * atan(A / B);
    }
    // theta2 = -2 * atan(A / B);
  }
  cxr2 = x2 + r * sin(theta2);
  cyr2 = y2 - r * cos(theta2);

  const float D_norm = norm(cxr2, cyr2, x3, y3);
  if (D_norm < r) {
    const float c1 = norm(x2, y2, x3, y3);
    double sqrt_term = 4 * r * r - c1 * c1;
    if (sqrt_term < 0) {
      sqrt_term = 0; // Prevent NaN
    }
    const float A = c1 * sqrt(sqrt_term) + 2 * r * (x2 - x3);
    // const float A = c1 * sqrt(4 * r * r - c1 * c1) + 2 * r * (x2 - x3);
    const float B = c1 * c1 + 2 * r * (y3 - y2);
    // theta2 = 2 * atan(A / B) + pi;
    if (B == 0) {
      theta2 = (A > 0) ? 2 * pi : 0; // theta2 = 2π or 0
    } else {
      theta2 = 2 * atan(A / B) + pi;
    }

    cxr2 = x2 + r * sin(theta2);
    cyr2 = y2 - r * cos(theta2);
  }

  double t2 = 0;
  theta2 = wrapToPi(theta2);
  urs_.getOmega(x2, y2, x3, y3, r, theta2, t2, true);
  t2 = t2 + theta2 - pi / 2;
  t2 = wrapToPi(t2);

  // const double H2 = sqrt((x3 - cxr2) * (x3 - cxr2) + (y3 - cyr2) * (y3 -
  // cyr2)); if (H2 <= r) {
  //   t2 = atan2(y3 - cyr2, x3 - cxr2) + pi / 2;
  // } else {
  //   t2 = atan2(y3 - cyr2, x3 - cxr2) + asin(r / H2);
  // }
  // s3.theta = t2 - pi;

  s2.theta = theta2;
  s3.theta = t2;
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
inline void Marching::RRPositive(const State s1, State &s2, State &s3,
                                 const Point p2_0, const Point p3_0) const {
  const double x1 = s1.x;
  const double y1 = s1.y;
  const double theta1 = s1.theta;
  const double r = r_;
  double x2 = s2.x;
  double y2 = s2.y;
  double theta2 = s2.theta;
  double x3 = s3.x;
  double y3 = s3.y;

  const double st1 = sin(theta1);
  const double ct1 = cos(theta1);
  const double cxr1 = x1 + r * st1 * st1 + r * ct1 * ct1;
  const double cyr1 = y1 - r * st1 * ct1 + r * ct1 * st1;
  double cxr2 = x2 + r * sin(theta2);
  double cyr2 = y2 - r * cos(theta2);

  double frac = r / norm(cxr2, cyr2, x3, y3);
  double B;
  if (frac >= 1) {
    B = pi / 2;
  } else {
    B = asin(frac);
  }
  theta2 = 0.5 * (atan2(y3 - cyr2, x3 - cxr2)) - 0.5 * B +
           0.5 * (atan2(cyr2 - cyr1, cxr2 - cxr1));

  cxr2 = x2 + r * sin(theta2);
  cyr2 = y2 - r * cos(theta2);

  float H2;
  const double dx_1 = (cxr1 - x2);
  const double dy_1 = (cyr1 - y2);
  if (GD_bisection_ == 2) {
    // Use bisection method to find the correct theta2 around the initial
    // guess
    double a = theta2 - 0.25;
    double b = theta2 + 0.25;
    double c = (b - a) / 3;
    const double eps = 0.001;
    const int exit_it = 100;
    int it = 1;
    while (fabs(b - a) > eps && it < exit_it) {
      // two interior points
      double t1 = a + c;
      double t2 = b - c;
      double f1, f2;
      cxr2 = x2 + r * sin(t1);
      cyr2 = y2 - r * cos(t1);
      frac = r / norm(cxr2, cyr2, x3, y3);
      if (frac >= 1) {
        B = pi / 2;
      } else {
        B = asin(frac);
      }
      f1 = 0.5 * (atan2(y3 - cyr2, x3 - cxr2)) - 0.5 * B +
           0.5 * (atan2(cyr2 - cyr1, cxr2 - cxr1));
      cxr2 = x2 + r * sin(t2);
      cyr2 = y2 - r * cos(t2);
      frac = r / norm(cxr2, cyr2, x3, y3);
      if (frac >= 1) {
        B = pi / 2;
      } else {
        B = asin(frac);
      }
      f2 = 0.5 * (atan2(y3 - cyr2, x3 - cxr2)) - 0.5 * B +
           0.5 * (atan2(cyr2 - cyr1, cxr2 - cxr1));
      if (f1 < f2) {
        b = t2;
      } else {
        a = t1;
      }
      it++;
    }
    theta2 = 0.5 * (a + b);
  } else {
    const double dx_2 = x2 - x3;
    const double dy_2 = y2 - y3;
    double D1 = norm(cxr1, cyr1, cxr2, cyr2);
    H2 = (x3 - cxr2) * (x3 - cxr2) + (y3 - cyr2) * (y3 - cyr2);
    double D2 = sqrt(fabs(H2 - r * r));
    double C2 = sqrt(fabs(1 - r * r / H2));
    double st2 = sin(theta2);
    double ct2 = cos(theta2);
    double g = r * (ct2 * dx_2 + st2 * dy_2) / D2 -
               r * (ct2 * dx_1 + st2 * dy_1) / D1 -
               r * (r * (-ct2 * dy_2 + st2 * dx_2 + r) / (H2) +
                    r * r * (ct2 * dx_2 + st2 * dy_2) / (C2 * H2 * sqrt(H2)));

    int iter = 1;
    while (fabs(g) > 0.1) {
      theta2 = theta2 - g * pi / (180 * sqrt(iter));
      cxr2 = x2 + r * sin(theta2);
      cyr2 = y2 - r * cos(theta2);
      D1 = sqrt((cxr2 - cxr1) * (cxr2 - cxr1) + (cyr2 - cyr1) * (cyr2 - cyr1));
      H2 = (x3 - cxr2) * (x3 - cxr2) + (y3 - cyr2) * (y3 - cyr2);
      D2 = sqrt(fabs(H2 - r * r));
      C2 = sqrt(fabs(1 - r * r / H2));
      st2 = sin(theta2);
      ct2 = cos(theta2);
      g = r * (ct2 * dx_2 + st2 * dy_2) / D2 -
          r * (ct2 * dx_1 + st2 * dy_1) / D1 -
          r * (r * (-ct2 * dy_2 + st2 * dx_2 + r) / (H2) +
               r * r * (ct2 * dx_2 + st2 * dy_2) / (C2 * H2 * sqrt(H2)));
      iter++;
      if (iter > exit_it) {
        break;
      }
    }
  }

  x2 = p2_0.x;
  y2 = p2_0.y;
  x3 = p3_0.x;
  y3 = p3_0.y;
  theta2 = theta2 + theta1 - pi / 2;

  cxr2 = x2 + r * sin(theta2);
  cyr2 = y2 - r * cos(theta2);
  const float r1p2 = sqrt(dx_1 * dx_1 + dy_1 * dy_1);
  if (r1p2 < r) {
    const float c1 = norm(x1, y1, x2, y2);
    double sqrt_term = 4 * r * r - c1 * c1;
    if (sqrt_term < 0) {
      sqrt_term = 0; // Prevent NaN
    }
    const float A = c1 * sqrt(sqrt_term) + 2 * r * (x1 - x2);
    const float B = c1 * c1 + 2 * r * (y2 - y1);
    if (B == 0) {
      theta2 = (A > 0) ? pi : -pi;
    } else {
      theta2 = 2 * atan(A / B);
    }
    // theta2 = 2 * atan(A / B);
    // urs_.getOmega(x1, y1, x2, y2, r, theta1, theta2, true);
    // theta2 = theta2 + theta1 - pi / 2;
    cxr2 = x2 + r * sin(theta2);
    cyr2 = y2 - r * cos(theta2);
  }

  H2 = norm(cxr2, cyr2, x3, y3);
  if (H2 < r) {
    // const double c1 = norm(x2, y2, x3, y3);
    // const double A = c1 * sqrt(4 * r * r - c1 * c1) + 2 * r * (x2 - x3);
    // const double B = c1 * c1 + 2 * r * (y2 - y3);
    // theta2 = -2 * atan(A / B);

    const double c1 = norm(x2, y2, x3, y3);
    double sqrt_term = 4 * r * r - c1 * c1;
    if (sqrt_term < 0) {
      sqrt_term = 0; // Prevent NaN
    }
    const double A = c1 * sqrt(sqrt_term) + 2 * r * (x2 - x3);
    const double B = c1 * c1 + 2 * r * (y2 - y3);
    if (B == 0) {
      theta2 = (A > 0) ? -pi : pi; // theta2 = -π or π
    } else {
      theta2 = -2 * atan(A / B);
    }

    // theta2 = -2 * atan(A / B);
    cxr2 = x2 + r * sin(theta2);
    cyr2 = y2 - r * cos(theta2);
    H2 = norm(cxr2, cyr2, x3, y3);
  }

  double t2 = 0;
  if (H2 < r) {
    t2 = atan2(y3 - cyr2, x3 - cxr2) - pi / 2;
  } else {
    t2 = atan2(y3 - cyr2, x3 - cxr2) - asin(r / H2);
  }
  // urs_.getOmega(x2, y2, x3, y3, r, theta2, t2, true);
  // t2 = t2 + theta2 - pi / 2;

  s2.theta = theta2;
  s3.theta = t2;
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
inline void Marching::RRNegative(const State s1, State &s2, State &s3,
                                 const Point p2_0, const Point p3_0) const {
  const double x1 = s1.x;
  const double y1 = s1.y;
  const double theta1 = s1.theta;
  const double r = r_;
  double x2 = s2.x;
  double y2 = s2.y;
  double theta2 = s2.theta;
  double x3 = s3.x;
  double y3 = s3.y;

  const double st1 = sin(theta1);
  const double ct1 = cos(theta1);
  const double cxr1 = x1 + r * st1 * st1 + r * ct1 * ct1;
  const double cyr1 = y1 - r * st1 * ct1 + r * ct1 * st1;
  double cxr2 = x2 + r * sin(theta2);
  double cyr2 = y2 - r * cos(theta2);

  double frac = r / norm(cxr2, cyr2, x3, y3);
  double B;
  if (frac >= 1) {
    B = pi / 2;
  } else {
    B = asin(frac);
  }
  theta2 = 0.5 * (atan2(y3 - cyr2, x3 - cxr2)) + 0.5 * B +
           0.5 * (atan2(cyr2 - cyr1, cxr2 - cxr1)) - pi;

  cxr2 = x2 + r * sin(theta2);
  cyr2 = y2 - r * cos(theta2);

  const double dx_1 = (cxr1 - x2);
  const double dy_1 = (cyr1 - y2);
  float H2;
  if (GD_bisection_ == 2) {
    // Use bisection method to find the correct theta2 around the initial
    // guess
    double a = theta2 - 0.25;
    double b = theta2 + 0.25;
    double c = (b - a) / 3;
    const double eps = 0.001;
    const int exit_it = 100;
    int it = 1;
    while (fabs(b - a) > eps && it < exit_it) {
      // two interior points
      double t1 = a + c;
      double t2 = b - c;
      double f1, f2;
      cxr2 = x2 + r * sin(t1);
      cyr2 = y2 - r * cos(t1);
      frac = r / norm(cxr2, cyr2, x3, y3);
      if (frac >= 1) {
        B = pi / 2;
      } else {
        B = asin(frac);
      }
      f1 = 0.5 * (atan2(y3 - cyr2, x3 - cxr2)) + 0.5 * B +
           0.5 * (atan2(cyr2 - cyr1, cxr2 - cxr1)) - pi;
      cxr2 = x2 + r * sin(t2);
      cyr2 = y2 - r * cos(t2);
      frac = r / norm(cxr2, cyr2, x3, y3);
      if (frac >= 1) {
        B = pi / 2;
      } else {
        B = asin(frac);
      }
      f2 = 0.5 * (atan2(y3 - cyr2, x3 - cxr2)) + 0.5 * B +
           0.5 * (atan2(cyr2 - cyr1, cxr2 - cxr1)) - pi;
      if (f1 < f2) {
        b = t2;
      } else {
        a = t1;
      }
      it++;
    }
    theta2 = 0.5 * (a + b);
  } else {
    const double dx_2 = x2 - x3;
    const double dy_2 = y2 - y3;

    double D1 = norm(cxr1, cyr1, cxr2, cyr2);
    H2 = (x3 - cxr2) * (x3 - cxr2) + (y3 - cyr2) * (y3 - cyr2);
    double D2 = sqrt(fabs(H2 - r * r));
    double C2 = sqrt(fabs(1 - r * r / H2));
    double st2 = sin(theta2);
    double ct2 = cos(theta2);
    double g =
        r * (r * (r + ct2 * -dy_2 + st2 * dx_2) / H2 -
             (r * r * (ct2 * dx_2 + st2 * dy_2)) / (C2 * H2 * sqrt(H2))) -
        r * (ct2 * dx_1 + st2 * dy_1) / D1 + r * (ct2 * dx_2 + st2 * dy_2) / D2;
    int iter = 1;
    while (fabs(g) > 0.1) {
      theta2 = theta2 - g * pi / (180 * sqrt(iter));
      cxr2 = x2 + r * sin(theta2);
      cyr2 = y2 - r * cos(theta2);
      D1 = norm(cxr1, cyr1, cxr2, cyr2);
      H2 = (x3 - cxr2) * (x3 - cxr2) + (y3 - cyr2) * (y3 - cyr2);
      D2 = sqrt(fabs(H2 - r * r));
      C2 = sqrt(fabs(1 - r * r / H2));
      st2 = sin(theta2);
      ct2 = cos(theta2);
      g = r * (r * (r + ct2 * -dy_2 + st2 * dx_2) / H2 -
               (r * r * (ct2 * dx_2 + st2 * dy_2)) / (C2 * H2 * sqrt(H2))) -
          r * (ct2 * dx_1 + st2 * dy_1) / D1 +
          r * (ct2 * dx_2 + st2 * dy_2) / D2;
      iter++;
      if (iter > exit_it) {
        break;
      }
    }
  }

  x2 = p2_0.x;
  y2 = p2_0.y;
  x3 = p3_0.x;
  y3 = p3_0.y;
  theta2 = theta2 + theta1 - pi / 2;

  cxr2 = x2 + r * sin(theta2);
  cyr2 = y2 - r * cos(theta2);
  const float r1p2 = sqrt(dx_1 * dx_1 + dy_1 * dy_1);
  if (r1p2 < r) {
    const double c1 = norm(x1, y1, x2, y2);
    double sqrt_term = 4 * r * r - c1 * c1;
    if (sqrt_term < 0) {
      sqrt_term = 0; // Prevent NaN
    }
    const double A = c1 * sqrt(sqrt_term) + 2 * r * (x1 - x2);
    // const double A = c1 * sqrt(4 * r * r - c1 * c1) + 2 * r * (x1 - x2);
    const double B = c1 * c1 + 2 * r * (y1 - y2);
    // theta2 = pi - 2 * atan(A / B);
    if (B == 0) {
      theta2 = (A > 0) ? 0 : 2 * pi; // theta2 = 0 or 2π
    } else {
      theta2 = pi - 2 * atan(A / B);
    }

    cxr2 = x2 + r * sin(theta2);
    cyr2 = y2 - r * cos(theta2);
  }

  H2 = norm(cxr2, cyr2, x3, y3);
  if (H2 < r) {
    const double c1 = norm(x2, y2, x3, y3);
    double sqrt_term = 4 * r * r - c1 * c1;
    if (sqrt_term < 0) {
      sqrt_term = 0; // Prevent NaN
    }
    const double A = c1 * sqrt(sqrt_term) + 2 * r * (x3 - x2);
    // const double A = c1 * sqrt(4 * r * r - c1 * c1) + 2 * r * (x3 - x2);
    const double B = c1 * c1 + 2 * r * (y2 - y3);
    if (B == 0) {
      theta2 = (A > 0) ? pi : -pi;
    } else {
      theta2 = 2 * atan(A / B);
    }
    cxr2 = x2 + r * sin(theta2);
    cyr2 = y2 - r * cos(theta2);
  }

  double t2 = 0;
  urs_.getOmega(x2, y2, x3, y3, r, theta2, t2, true);
  t2 = t2 + theta2 - pi / 2;

  s2.theta = theta2;
  s3.theta = t2;
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
inline void Marching::LLPositive(const State s1, State &s2, State &s3,
                                 const Point p2_0, const Point p3_0) const {
  const double x1 = s1.x;
  const double y1 = s1.y;
  const double theta1 = s1.theta;
  const double r = r_;
  double x2 = s2.x;
  double y2 = s2.y;
  double theta2 = s2.theta;
  double x3 = s3.x;
  double y3 = s3.y;

  const double cxl1 = llPositiveParentCircleX(x1, r);
  const double cyl1 = llPositiveParentCircleY(y1);

  double seeded_theta = theta2;
  if (llPositiveSeedTheta(cxl1, cyl1, x2, y2, x3, y3, r, theta2,
                          seeded_theta)) {
    theta2 = seeded_theta;
  }

  if (GD_bisection_ == 2) {
    double left = theta2 - 0.25;
    double right = theta2 + 0.25;
    constexpr double eps = 0.001;
    for (int it = 0; std::abs(right - left) > eps && it < 100; ++it) {
      const double t1 = left + (right - left) / 3.0;
      const double t2 = right - (right - left) / 3.0;
      double f1 = 0.0;
      double f2 = 0.0;
      const bool ok1 =
          llPositiveObjective(cxl1, cyl1, x2, y2, x3, y3, r, t1, f1);
      const bool ok2 =
          llPositiveObjective(cxl1, cyl1, x2, y2, x3, y3, r, t2, f2);
      if (ok1 && (!ok2 || f1 < f2)) {
        right = t2;
      } else if (ok2) {
        left = t1;
      } else {
        break;
      }
    }
    theta2 = 0.5 * (left + right);
  } else {
    double g = 0.0;
    for (int iter = 1;
         iter <= exit_it &&
         llPositiveGradient(cxl1, cyl1, x2, y2, x3, y3, r, theta2, g) &&
         std::abs(g) > 0.1;
         ++iter) {
      theta2 = theta2 - g * pi / (180.0 * std::sqrt(iter));
    }
  }

  const double l1p2 = std::hypot(cxl1 - x2, cyl1 - y2);
  double contact_theta = theta2;
  if (l1p2 < r &&
      llPositiveContactParentTheta(x1, y1, x2, y2, r, contact_theta)) {
    theta2 = contact_theta;
  }

  double cxl2 = 0.0;
  double cyl2 = 0.0;
  llPositivePivotCircle(x2, y2, r, theta2, cxl2, cyl2);
  const double h = std::hypot(x3 - cxl2, y3 - cyl2);
  if (h < r &&
      llPositiveContactTargetTheta(x2, y2, x3, y3, r, contact_theta)) {
    theta2 = contact_theta;
  }

  x2 = p2_0.x;
  y2 = p2_0.y;
  x3 = p3_0.x;
  y3 = p3_0.y;
  theta2 = theta2 + theta1 - pi / 2;

  double t2 = 0.0;
  urs_.getOmega(x2, y2, x3, y3, r, theta2, t2, true);
  t2 = t2 + theta2 - pi / 2;

  s2.theta = theta2;
  s3.theta = t2;
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
inline void Marching::LLNegative(const State s1, State &s2, State &s3,
                                 const Point p2_0, const Point p3_0) const {
  const double x1 = s1.x;
  const double y1 = s1.y;
  const double theta1 = s1.theta;
  const double r = r_;
  double x2 = s2.x;
  double y2 = s2.y;
  double theta2 = s2.theta;
  double x3 = s3.x;
  double y3 = s3.y;
  // double theta2 = 0.5 * (wrapTo2Pi(atan2(y2 - y3, x2 - x3)) +
  //                        wrapTo2Pi(atan2(y1 - y2, x1 - x2)));

  const double st1 = sin(theta1);
  const double ct1 = cos(theta1);
  double cxl1 = x1 - r * st1 * st1 - r * ct1 * ct1;
  double cyl1 = y1 + r * st1 * ct1 - r * ct1 * st1;
  double cxl2 = x2 - r * sin(theta2);
  double cyl2 = y2 + r * cos(theta2);

  double frac = r / norm(cxl2, cyl2, x3, y3);
  double B;
  if (frac >= 1) {
    B = pi / 2;
  } else {
    B = asin(frac);
  }
  theta2 = 0.5 * (atan2(cyl2 - y3, cxl2 - x3)) - 0.5 * B +
           0.5 * (atan2(cyl1 - cyl2, cxl1 - cxl2));
  theta2 = wrapToPi(theta2);

  cxl2 = x2 - r * sin(theta2);
  cyl2 = y2 + r * cos(theta2);

  const double dx_1 = (cxl1 - x2);
  const double dy_1 = (cyl1 - y2);
  float H2;

  if (GD_bisection_ == 2) {
    // Use bisection method to find the correct theta2 around the initial
    // guess
    double a = theta2 - 0.25;
    double b = theta2 + 0.25;
    double c = (b - a) / 3;
    const double eps = 0.001;
    const int exit_it = 100;
    int it = 1;
    while (fabs(b - a) > eps && it < exit_it) {
      // two interior points
      double t1 = a + c;
      double t2 = b - c;
      double f1, f2;
      cxl2 = x2 - r * sin(t1);
      cyl2 = y2 + r * cos(t1);
      frac = r / norm(cxl2, cyl2, x3, y3);
      if (frac >= 1) {
        B = pi / 2;
      } else {
        B = asin(frac);
      }
      f1 = 0.5 * (atan2(cyl2 - y3, cxl2 - x3)) - 0.5 * B +
           0.5 * (atan2(cyl1 - cyl2, cxl1 - cxl2));
      cxl2 = x2 - r * sin(t2);
      cyl2 = y2 + r * cos(t2);
      frac = r / norm(cxl2, cyl2, x3, y3);
      if (frac >= 1) {
        B = pi / 2;
      } else {
        B = asin(frac);
      }
      f2 = 0.5 * (atan2(cyl2 - y3, cxl2 - x3)) - 0.5 * B +
           0.5 * (atan2(cyl1 - cyl2, cxl1 - cxl2));
      if (f1 < f2) {
        b = t2;
      } else {
        a = t1;
      }
      it++;
    }
    theta2 = 0.5 * (a + b);
  } else {
    const double dx_2 = (x2 - x3);
    const double dy_2 = (y2 - y3);
    double D1 = norm(cxl1, cyl1, cxl2, cyl2);
    H2 = (x3 - cxl2) * (x3 - cxl2) + (y3 - cyl2) * (y3 - cyl2);
    double D2 = sqrt(fabs(H2 - r * r));
    double C2 = sqrt(fabs(1 - r * r / H2));
    double st2 = sin(theta2);
    double ct2 = cos(theta2);
    double g = r * (ct2 * dx_1 + st2 * dy_1) / D1 -
               r * (r * (r + ct2 * dy_2 - st2 * dx_2) / H2 -
                    r * r * (ct2 * dx_2 + st2 * dy_2) / (C2 * H2 * sqrt(H2))) -
               r * (ct2 * dx_2 + st2 * dy_2) / D2;

    int iter = 1;
    while (fabs(g) > 0.1) {
      theta2 = theta2 - g * pi / (180 * sqrt(iter));
      cxl2 = x2 - r * sin(theta2);
      cyl2 = y2 + r * cos(theta2);
      D1 = norm(cxl1, cyl1, cxl2, cyl2);
      H2 = (x3 - cxl2) * (x3 - cxl2) + (y3 - cyl2) * (y3 - cyl2);
      D2 = sqrt(fabs(H2 - r * r));
      C2 = sqrt(fabs(1 - r * r / H2));
      st2 = sin(theta2);
      ct2 = cos(theta2);
      g = r * (ct2 * dx_1 + st2 * dy_1) / D1 -
          r * (r * (r + ct2 * dy_2 - st2 * dx_2) / H2 -
               r * r * (ct2 * dx_2 + st2 * dy_2) / (C2 * H2 * sqrt(H2))) -
          r * (ct2 * dx_2 + st2 * dy_2) / D2;
      iter++;
      if (iter > exit_it) {
        break;
      }
    }
  }

  x2 = p2_0.x;
  y2 = p2_0.y;
  x3 = p3_0.x;
  y3 = p3_0.y;
  theta2 = theta2 + theta1 - pi / 2;
  theta2 = wrapToPi(theta2);

  cxl2 = x2 - r * sin(theta2);
  cyl2 = y2 + r * cos(theta2);
  const float l1p2 = sqrt(dx_1 * dx_1 + dy_1 * dy_1);
  if (l1p2 < r) {
    const float c1 = norm(x1, y1, x2, y2);
    double sqrt_term = 4 * r * r - c1 * c1;
    if (sqrt_term < 0) {
      sqrt_term = 0; // Prevent NaN
    }
    const float A = c1 * sqrt(sqrt_term) + 2 * r * (x2 - x1);
    // const float A = c1 * sqrt(4 * r * r - c1 * c1) + 2 * r * (x2 - x1);
    const float B = c1 * c1 + 2 * r * (y1 - y2);
    // theta2 = 2 * atan(A / B);
    if (B == 0) {
      theta2 = (A > 0) ? pi : -pi;
    } else {
      theta2 = 2 * atan(A / B);
    }

    cxl2 = x2 - r * sin(theta2);
    cyl2 = y2 + r * cos(theta2);
  }

  H2 = norm(cxl2, cyl2, x3, y3);
  if (H2 < r) {
    const float c1 = norm(x2, y2, x3, y3);
    double sqrt_term = 4 * r * r - c1 * c1;
    if (sqrt_term < 0) {
      sqrt_term = 0; // Prevent NaN
    }
    const float A = c1 * sqrt(sqrt_term) + 2 * r * (x3 - x2);
    // const float A = c1 * sqrt(4 * r * r - c1 * c1) + 2 * r * (x3 - x2);
    const float B = c1 * c1 + 2 * r * (y3 - y2);
    // theta2 = -2 * atan2(A, B);
    if (B == 0) {
      theta2 = (A > 0) ? -pi : pi; // theta2 = -π or π
    } else {
      theta2 = -2 * atan(A / B);
    }
    cxl2 = x2 - r * sin(theta2);
    cyl2 = y2 + r * cos(theta2);
  }

  double t2 = 0;
  theta2 = wrapToPi(theta2);
  urs_.getOmega(x2, y2, x3, y3, r, theta2, t2, true);
  t2 = wrapToPi(t2 + theta2 - pi / 2);
  // if (H2 <= r) {
  //   t2 = atan2(cyl2 - y3, cxl2 - x3) - pi / 2;
  // } else {
  //   t2 = atan2(cyl2 - y3, cxl2 - x3) - asin(r / H2);
  // }

  s2.theta = theta2;
  s3.theta = t2;
}

// /*****************************************************************************/
// /*****************************************************************************/
// /*****************************************************************************/
// inline void Marching::RLPositive(const State s1, State &s2, State &s3,
//                                  const Point p2_0, const Point p3_0) const {
//   const double x1 = s1.x;
//   const double y1 = s1.y;
//   const double theta1 = s1.theta;
//   const double r = r_;
//   double x2 = s2.x;
//   double y2 = s2.y;
//   double theta2 = s2.theta;
//   double x3 = s3.x;
//   double y3 = s3.y;
//
//   const double st1 = sin(theta1);
//   const double ct1 = cos(theta1);
//   const double cxr1 = x1 + r * st1 * st1 + r * ct1 * ct1;
//   const double cyr1 = y1 - r * st1 * ct1 + r * ct1 * st1;
//   double cxl2 = x2 - r * sin(theta2);
//   double cyl2 = y2 + r * cos(theta2);
//
//   // TODO: optimize fractions - inverse square root
//   // check if modern compilers automatically optimize this
//   const double frac = r / norm(cxl2, cyl2, x3, y3);
//   double B;
//   if (frac >= 1) {
//     B = pi / 2;
//   } else {
//     B = asin(frac);
//   }
//   const double c1 = norm(cxr1, cyr1, cxl2, cyl2);
//   const double c = c1 * c1 - 4 * r * r;
//   double C;
//   if (c <= 0) {
//     C = pi / 2;
//   } else {
//     C = acos(sqrt(c) / c1);
//   }
//   theta2 = 0.5 * wrapToPi(atan2((y3 - cyl2), x3 - cxl2)) + 0.5 * B - 0.5 * C
//   +
//            0.5 * wrapTo2Pi(atan2(cyl2 - cyr1, cxl2 - cxr1));
//
//   x2 = p2_0.x;
//   y2 = p2_0.y;
//   x3 = p3_0.x;
//   y3 = p3_0.y;
//   theta2 = theta2 + theta1 - pi / 2;
//   cxl2 = x2 - r * sin(theta2);
//   cyl2 = y2 + r * cos(theta2);
//
//   const double D_norm = norm(cxl2, cyl2, x3, y3);
//   if (D_norm <= r) {
//     const double c1 = norm(x2, y2, x3, y3);
//     const double A = c1 * sqrt(4 * r * r - c1 * c1) + 2 * r * (x2 - x3);
//     const double B = c1 * c1 + 2 * r * (y3 - y2);
//     theta2 = 2 * atan(A / B);
//     cxl2 = x2 - r * sin(theta2);
//     cyl2 = y2 + r * cos(theta2);
//   }
//
//   double t2 = 0;
//   urs_.getOmega(x2, y2, x3, y3, r, theta2, t2, true);
//   t2 = t2 + theta2 - pi / 2;
//
//   s2.theta = theta2;
//   s3.theta = t2;
// }
//
// /*****************************************************************************/
// /*****************************************************************************/
// /*****************************************************************************/
// inline void Marching::RLNegative(const State s1, State &s2, State &s3,
//                                  const Point p2_0, const Point p3_0) const {
//   const double x1 = s1.x;
//   const double y1 = s1.y;
//   const double theta1 = s1.theta;
//   const double r = r_;
//   double x2 = s2.x;
//   double y2 = s2.y;
//   double x3 = s3.x;
//   double y3 = s3.y;
//   double theta2 = s2.theta;
//
//   const double st1 = sin(theta1);
//   const double ct1 = cos(theta1);
//   double cxr1 = x1 + r * st1 * st1 + r * ct1 * ct1;
//   double cyr1 = y1 - r * st1 * ct1 + r * ct1 * st1;
//   double cxl2 = x2 - r * sin(theta2);
//   double cyl2 = y2 + r * cos(theta2);
//
//   // TODO: optimize fractions - inverse square root
//   // check if modern compilers automatically optimize this
//   const double frac = r / norm(cxl2, cyl2, x3, y3);
//   double B;
//   if (frac >= 1) {
//     B = pi / 2;
//   } else {
//     B = asin(frac);
//   }
//   double C;
//   const double c1 = norm(cxr1, cyr1, cxl2, cyl2);
//   const double c = c1 * c1 - 4 * r * r;
//   if (c <= 0) {
//     C = pi / 2;
//   } else {
//     C = acos(sqrt(c) / c1);
//   }
//   theta2 = 0.5 * wrapTo2Pi(atan2((cyl2 - y3), cxl2 - x3)) - 0.5 * B + 0.5 * C
//   +
//            0.5 * wrapTo2Pi(atan2(cyr1 - cyl2, cxr1 - cxl2));
//
//   x2 = p2_0.x;
//   y2 = p2_0.y;
//   x3 = p3_0.x;
//   y3 = p3_0.y;
//   theta2 = theta2 + theta1 - pi / 2;
//   cxl2 = x2 - r * sin(theta2);
//   cyl2 = y2 + r * cos(theta2);
//   cxr1 = x1 + r * sin(theta1);
//   cyr1 = y1 - r * cos(theta1);
//
//   double cxr2 = x2 + r * sin(theta2);
//   double cyr2 = y2 - r * cos(theta2);
//   if ((cxr2 - cxr1) * (cxr2 - cxr1) + (cyr2 - cyr1) * (cyr2 - cyr1) <= r * r)
//   {
//     urs_.getOmega(x1, y1, x2, y2, r_, theta1, theta2, true);
//     theta2 += theta1 - pi / 2;
//   } else if (norm(cxr1, cyr1, cxl2, cyl2) <= 2 * r) {
//     const double c1 = norm(cxr1, cyr1, x2, y2);
//     const double A =
//         sqrt(-(c1 * c1 - r * r) * (c1 * c1 - 9 * r * r)) - 2 * r * (cxr1 -
//         x2);
//     const double B = c1 * c1 - 3 * r * r + 2 * r * (cyr1 - y2);
//     theta2 = 2 * atan(A / B);
//     cxl2 = x2 - r * sin(theta2);
//     cyl2 = y2 + r * cos(theta2);
//   }
//
//   const double D_norm = norm(cxl2, cyl2, x3, y3);
//   if (D_norm <= r) {
//     const double c1 = norm(x2, y2, x3, y3);
//     const double A = c1 * sqrt(4 * r * r - c1 * c1) + 2 * r * (x2 - x3);
//     const double B = c1 * c1 + 2 * r * (y2 - y3);
//     theta2 = pi - 2 * atan(A / B);
//     cxl2 = x2 - r * sin(theta2);
//     cyl2 = y2 + r * cos(theta2);
//   }
//
//   double t2 = 0;
//   urs_.getOmega(x2, y2, x3, y3, r, theta2, t2, true);
//   t2 = t2 + theta2 - pi / 2;
//
//   s2.theta = theta2;
//   s3.theta = t2;
// }
//
// /*****************************************************************************/
// /*****************************************************************************/
// /*****************************************************************************/
// inline void Marching::LRPositive(const State s1, State &s2, State &s3,
//                                  const Point p2_0, const Point p3_0) const {
//   const double x1 = s1.x;
//   const double y1 = s1.y;
//   const double theta1 = s1.theta;
//   const double r = r_;
//   double x2 = s2.x;
//   double y2 = s2.y;
//   double theta2 = s2.theta;
//   double x3 = s3.x;
//   double y3 = s3.y;
//
//   const double st1 = sin(theta1);
//   const double ct1 = cos(theta1);
//   const double cxl1 = x1 - r * st1 * st1 - r * ct1 * ct1;
//   const double cyl1 = y1 + r * st1 * ct1 - r * ct1 * st1;
//   double cxr2 = x2 + r * sin(theta2);
//   double cyr2 = y2 - r * cos(theta2);
//
//   const double frac = r / norm(cxr2, cyr2, x3, y3);
//   double B;
//   if (frac >= 1) {
//     B = pi / 2;
//   } else {
//     B = asin(frac);
//   }
//   const double c1 = norm(cxl1, cyl1, cxr2, cyr2);
//   const double c = c1 * c1 - 4 * r * r;
//   double C;
//   if (c <= 0) {
//     C = pi / 2;
//   } else {
//     C = acos(sqrt(c) / c1);
//   }
//
//   // TODO: missing -norm compared to MATLAB code
//   theta2 = 0.5 * wrapTo2Pi(atan2(y3 - cyr2, x3 - cxr2)) - 0.5 * B + 0.5 * C +
//            0.5 * wrapTo2Pi(atan2(cyr2 - cyl1, cxr2 - cxl1));
//
//   x2 = p2_0.x;
//   y2 = p2_0.y;
//   x3 = p3_0.x;
//   y3 = p3_0.y;
//   theta2 = theta2 + theta1 - pi / 2;
//   cxr2 = x2 + r * sin(theta2);
//   cyr2 = y2 - r * cos(theta2);
//
//   double cxl2 = x2 - r * sin(theta2);
//   double cyl2 = y2 + r * cos(theta2);
//   double cxr1 = x1 + r * sin(theta1);
//   double cyr1 = y1 - r * cos(theta1);
//
//   if (norm(cxl2, cyl2, cxr1, cyr1) <= 2 * r) {
//     const double c1 = norm(cxr1, cyr1, x2, y2);
//     const double A =
//         sqrt(-(c1 * c1 - r * r) * (c1 * c1 - 9 * r * r)) + 2 * r * (cxr1 -
//         x2);
//     const double B = c1 * c1 - 3 * r * r - 2 * r * (y2 - cyr1);
//     theta2 = -2 * atan(A / B);
//     cxr2 = x2 + r * sin(theta2);
//     cyr2 = y2 - r * cos(theta2);
//   }
//
//   const double D_norm = norm(cxr2, cyr2, x3, y3);
//   if (D_norm <= r) {
//     const double c1 = norm(x2, y2, x3, y3);
//     const double A = c1 * sqrt(4 * r * r - c1 * c1) + 2 * r * (x2 - x3);
//     const double B = c1 * c1 + 2 * r * (y2 - y3);
//     theta2 = -2 * atan(A / B);
//     cxr2 = x2 + r * sin(theta2);
//     cyr2 = y2 - r * cos(theta2);
//   }
//
//   double t2 = 0;
//   urs_.getOmega(x2, y2, x3, y3, r, theta2, t2, true);
//   t2 = t2 + theta2 - pi / 2;
//
//   s2.theta = theta2;
//   s3.theta = t2;
// }
//
// /*****************************************************************************/
// /*****************************************************************************/
// /*****************************************************************************/
// inline void Marching::LRNegative(const State s1, State &s2, State &s3,
//                                  const Point p2_0, const Point p3_0) const {
//   const double x1 = s1.x;
//   const double y1 = s1.y;
//   double theta1 = s1.theta;
//   const double r = r_;
//   double x2 = s2.x;
//   double y2 = s2.y;
//   double theta2 = s2.theta;
//   double x3 = s3.x;
//   double y3 = s3.y;
//
//   const double st1 = sin(theta1);
//   const double ct1 = cos(theta1);
//   double cxl1 = x1 - r * st1 * st1 - r * ct1 * ct1;
//   double cyl1 = y1 + r * st1 * ct1 - r * ct1 * st1;
//   double cxr2 = x2 + r * sin(theta2);
//   double cyr2 = y2 - r * cos(theta2);
//
//   const double frac = r / norm(cxr2, cyr2, x3, y3);
//   double B;
//   if (frac >= 1) {
//     B = pi / 2;
//   } else {
//     B = asin(frac);
//   }
//   const double c1 = norm(cxl1, cyl1, cxr2, cyr2);
//   const double c = c1 * c1 - 4 * r * r;
//   double C;
//   if (c <= 0) {
//     C = pi / 2;
//   } else {
//     C = acos(sqrt(c) / c1);
//   }
//
//   // TODO: missing -norm compared to MATLAB code
//   theta2 = 0.5 * wrapTo2Pi(atan2((y3 - cyr2), x3 - cxr2)) + 0.5 * B - 0.5 * C
//   +
//            0.5 * wrapTo2Pi(atan2(cyr2 - cyl1, cxr2 - cxl1)) + pi;
//
//   x2 = p2_0.x;
//   y2 = p2_0.y;
//   x3 = p3_0.x;
//   y3 = p3_0.y;
//   theta2 = theta2 + theta1 - pi / 2;
//   cxl1 = x1 - r * sin(theta1);
//   cyl1 = y1 + r * cos(theta1);
//
//   double cxl2 = x2 - r * sin(theta2);
//   double cyl2 = y2 + r * cos(theta2);
//   cxr2 = x2 + r * sin(theta2);
//   cyr2 = y2 - r * cos(theta2);
//
//   if ((cxl2 - cxl1) * (cxl2 - cxl1) + (cyl2 - cyl1) * (cyl2 - cyl1) <= r * r)
//   {
//     urs_.getOmega(x1, y1, x2, y2, r_, theta1, theta2, true);
//     // theta1 = thetas_(x1, y1) - pi / 2;
//     theta2 += theta1 - pi / 2;
//   } else if (norm(cxl1, cyl1, cxr2, cyr2) <= 2 * r) {
//     const double c1 = norm(cxl1, cyl1, x2, y2);
//     const double A =
//         sqrt(-(c1 * c1 - r * r) * (c1 * c1 - 9 * r * r)) - 2 * r * (cxl1 -
//         x2);
//     const double B = c1 * c1 - 3 * r * r + 2 * r * (y2 - cyl1);
//     theta2 = -2 * atan(A / B);
//   }
//   cxr2 = x2 + r * sin(theta2);
//   cyr2 = y2 - r * cos(theta2);
//
//   const double D_norm = norm(cxr2, cyr2, x3, y3);
//   if (D_norm <= r) {
//     const double c1 = norm(x2, y2, x3, y3);
//     const double A = c1 * sqrt(4 * r * r - c1 * c1) + 2 * r * (x2 - x3);
//     const double B = c1 * c1 + 2 * r * (y3 - y2);
//     theta2 = 2 * atan(A / B) + pi;
//     cxr2 = x2 + r * sin(theta2);
//     cyr2 = y2 - r * cos(theta2);
//   }
//
//   double t2 = 0;
//   theta2 = wrapToPi(theta2);
//   urs_.getOmega(x2, y2, x3, y3, r, theta2, t2, true);
//   t2 = t2 + theta2 - pi / 2;
//   t2 = wrapToPi(t2);
//
//   // const double H2 = sqrt((x3 - cxr2) * (x3 - cxr2) + (y3 - cyr2) * (y3 -
//   // cyr2)); if (H2 <= r) {
//   //   t2 = atan2(y3 - cyr2, x3 - cxr2) + pi / 2;
//   // } else {
//   //   t2 = atan2(y3 - cyr2, x3 - cxr2) + asin(r / H2);
//   // }
//   // s3.theta = t2 - pi;
//
//   s2.theta = theta2;
//   s3.theta = t2;
// }
//
// /*****************************************************************************/
// /*****************************************************************************/
// /*****************************************************************************/
// inline void Marching::RRPositive(const State s1, State &s2, State &s3,
//                                  const Point p2_0, const Point p3_0) const {
//   const double x1 = s1.x;
//   const double y1 = s1.y;
//   const double theta1 = s1.theta;
//   const double r = r_;
//   double x2 = s2.x;
//   double y2 = s2.y;
//   double theta2 = s2.theta;
//   double x3 = s3.x;
//   double y3 = s3.y;
//
//   const double st1 = sin(theta1);
//   const double ct1 = cos(theta1);
//   const double cxr1 = x1 + r * st1 * st1 + r * ct1 * ct1;
//   const double cyr1 = y1 - r * st1 * ct1 + r * ct1 * st1;
//   double cxr2 = x2 + r * sin(theta2);
//   double cyr2 = y2 - r * cos(theta2);
//
//   double frac = r / norm(cxr2, cyr2, x3, y3);
//   double B;
//   if (frac >= 1) {
//     B = pi / 2;
//   } else {
//     B = asin(frac);
//   }
//   theta2 = 0.5 * (atan2(y3 - cyr2, x3 - cxr2)) - 0.5 * B +
//            0.5 * (atan2(cyr2 - cyr1, cxr2 - cxr1));
//
//   cxr2 = x2 + r * sin(theta2);
//   cyr2 = y2 - r * cos(theta2);
//
//   double H2;
//   const double dx_1 = (cxr1 - x2);
//   const double dy_1 = (cyr1 - y2);
//   if (GD_bisection_ == 2) {
//     // Use bisection method to find the correct theta2 around the initial
//     guess double a = theta2 - 0.25; double b = theta2 + 0.25; double c = (b -
//     a) / 3; const double eps = 0.001; const int exit_it = 100; int it = 1;
//     while (fabs(b - a) > eps && it < exit_it) {
//       // two interior points
//       double t1 = a + c;
//       double t2 = b - c;
//       double f1, f2;
//       cxr2 = x2 + r * sin(t1);
//       cyr2 = y2 - r * cos(t1);
//       frac = r / norm(cxr2, cyr2, x3, y3);
//       if (frac >= 1) {
//         B = pi / 2;
//       } else {
//         B = asin(frac);
//       }
//       f1 = 0.5 * (atan2(y3 - cyr2, x3 - cxr2)) - 0.5 * B +
//            0.5 * (atan2(cyr2 - cyr1, cxr2 - cxr1));
//       cxr2 = x2 + r * sin(t2);
//       cyr2 = y2 - r * cos(t2);
//       frac = r / norm(cxr2, cyr2, x3, y3);
//       if (frac >= 1) {
//         B = pi / 2;
//       } else {
//         B = asin(frac);
//       }
//       f2 = 0.5 * (atan2(y3 - cyr2, x3 - cxr2)) - 0.5 * B +
//            0.5 * (atan2(cyr2 - cyr1, cxr2 - cxr1));
//       if (f1 < f2) {
//         b = t2;
//       } else {
//         a = t1;
//       }
//       it++;
//     }
//     theta2 = 0.5 * (a + b);
//   } else {
//     const double dx_2 = x2 - x3;
//     const double dy_2 = y2 - y3;
//     double D1 = norm(cxr1, cyr1, cxr2, cyr2);
//     H2 = (x3 - cxr2) * (x3 - cxr2) + (y3 - cyr2) * (y3 - cyr2);
//     double D2 = sqrt(fabs(H2 - r * r));
//     double C2 = sqrt(fabs(1 - r * r / H2));
//     double st2 = sin(theta2);
//     double ct2 = cos(theta2);
//     double g = r * (ct2 * dx_2 + st2 * dy_2) / D2 -
//                r * (ct2 * dx_1 + st2 * dy_1) / D1 -
//                r * (r * (-ct2 * dy_2 + st2 * dx_2 + r) / (H2) +
//                     r * r * (ct2 * dx_2 + st2 * dy_2) / (C2 * H2 *
//                     sqrt(H2)));
//
//     int iter = 1;
//     while (fabs(g) > 0.1) {
//       theta2 = theta2 - g * pi / (180 * sqrt(iter));
//       cxr2 = x2 + r * sin(theta2);
//       cyr2 = y2 - r * cos(theta2);
//       D1 = sqrt((cxr2 - cxr1) * (cxr2 - cxr1) + (cyr2 - cyr1) * (cyr2 -
//       cyr1)); H2 = (x3 - cxr2) * (x3 - cxr2) + (y3 - cyr2) * (y3 - cyr2); D2
//       = sqrt(fabs(H2 - r * r)); C2 = sqrt(fabs(1 - r * r / H2)); st2 =
//       sin(theta2); ct2 = cos(theta2); g = r * (ct2 * dx_2 + st2 * dy_2) / D2
//       -
//           r * (ct2 * dx_1 + st2 * dy_1) / D1 -
//           r * (r * (-ct2 * dy_2 + st2 * dx_2 + r) / (H2) +
//                r * r * (ct2 * dx_2 + st2 * dy_2) / (C2 * H2 * sqrt(H2)));
//       iter++;
//       if (iter > exit_it) {
//         break;
//       }
//     }
//   }
//
//   x2 = p2_0.x;
//   y2 = p2_0.y;
//   x3 = p3_0.x;
//   y3 = p3_0.y;
//   theta2 = theta2 + theta1 - pi / 2;
//
//   cxr2 = x2 + r * sin(theta2);
//   cyr2 = y2 - r * cos(theta2);
//   if (sqrt(dx_1 * dx_1 + dy_1 * dy_1) <= r) {
//     const double c1 = norm(x1, y1, x2, y2);
//     const double A = c1 * sqrt(4 * r * r - c1 * c1) + 2 * r * (x1 - x2);
//     const double B = c1 * c1 + 2 * r * (y2 - y1);
//     theta2 = 2 * atan(A / B);
//     // urs_.getOmega(x1, y1, x2, y2, r, theta1, theta2, true);
//     // theta2 = theta2 + theta1 - pi / 2;
//     cxr2 = x2 + r * sin(theta2);
//     cyr2 = y2 - r * cos(theta2);
//   }
//
//   H2 = norm(cxr2, cyr2, x3, y3);
//   if (H2 <= r) {
//     const double c1 = norm(x2, y2, x3, y3);
//     const double A = c1 * sqrt(4 * r * r - c1 * c1) + 2 * r * (x2 - x3);
//     const double B = c1 * c1 + 2 * r * (y2 - y3);
//     theta2 = -2 * atan(A / B);
//     cxr2 = x2 + r * sin(theta2);
//     cyr2 = y2 - r * cos(theta2);
//     H2 = norm(cxr2, cyr2, x3, y3);
//   }
//
//   double t2 = 0;
//   if (H2 <= r) {
//     t2 = atan2(y3 - cyr2, x3 - cxr2) - pi / 2;
//   } else {
//     t2 = atan2(y3 - cyr2, x3 - cxr2) - asin(r / H2);
//   }
//   // urs_.getOmega(x2, y2, x3, y3, r, theta2, t2, true);
//   // t2 = t2 + theta2 - pi / 2;
//
//   s2.theta = theta2;
//   s3.theta = t2;
// }
//
// /*****************************************************************************/
// /*****************************************************************************/
// /*****************************************************************************/
// inline void Marching::RRNegative(const State s1, State &s2, State &s3,
//                                  const Point p2_0, const Point p3_0) const {
//   const double x1 = s1.x;
//   const double y1 = s1.y;
//   const double theta1 = s1.theta;
//   const double r = r_;
//   double x2 = s2.x;
//   double y2 = s2.y;
//   double theta2 = s2.theta;
//   double x3 = s3.x;
//   double y3 = s3.y;
//
//   const double st1 = sin(theta1);
//   const double ct1 = cos(theta1);
//   const double cxr1 = x1 + r * st1 * st1 + r * ct1 * ct1;
//   const double cyr1 = y1 - r * st1 * ct1 + r * ct1 * st1;
//   double cxr2 = x2 + r * sin(theta2);
//   double cyr2 = y2 - r * cos(theta2);
//
//   double frac = r / norm(cxr2, cyr2, x3, y3);
//   double B;
//   if (frac >= 1) {
//     B = pi / 2;
//   } else {
//     B = asin(frac);
//   }
//   theta2 = 0.5 * (atan2(y3 - cyr2, x3 - cxr2)) + 0.5 * B +
//            0.5 * (atan2(cyr2 - cyr1, cxr2 - cxr1)) - pi;
//
//   cxr2 = x2 + r * sin(theta2);
//   cyr2 = y2 - r * cos(theta2);
//
//   const double dx_1 = (cxr1 - x2);
//   const double dy_1 = (cyr1 - y2);
//   double H2;
//   if (GD_bisection_ == 2) {
//     // Use bisection method to find the correct theta2 around the initial
//     guess double a = theta2 - 0.25; double b = theta2 + 0.25; double c = (b -
//     a) / 3; const double eps = 0.001; const int exit_it = 100; int it = 1;
//     while (fabs(b - a) > eps && it < exit_it) {
//       // two interior points
//       double t1 = a + c;
//       double t2 = b - c;
//       double f1, f2;
//       cxr2 = x2 + r * sin(t1);
//       cyr2 = y2 - r * cos(t1);
//       frac = r / norm(cxr2, cyr2, x3, y3);
//       if (frac >= 1) {
//         B = pi / 2;
//       } else {
//         B = asin(frac);
//       }
//       f1 = 0.5 * (atan2(y3 - cyr2, x3 - cxr2)) + 0.5 * B +
//            0.5 * (atan2(cyr2 - cyr1, cxr2 - cxr1)) - pi;
//       cxr2 = x2 + r * sin(t2);
//       cyr2 = y2 - r * cos(t2);
//       frac = r / norm(cxr2, cyr2, x3, y3);
//       if (frac >= 1) {
//         B = pi / 2;
//       } else {
//         B = asin(frac);
//       }
//       f2 = 0.5 * (atan2(y3 - cyr2, x3 - cxr2)) + 0.5 * B +
//            0.5 * (atan2(cyr2 - cyr1, cxr2 - cxr1)) - pi;
//       if (f1 < f2) {
//         b = t2;
//       } else {
//         a = t1;
//       }
//       it++;
//     }
//     theta2 = 0.5 * (a + b);
//   } else {
//     const double dx_2 = x2 - x3;
//     const double dy_2 = y2 - y3;
//
//     double D1 = norm(cxr1, cyr1, cxr2, cyr2);
//     H2 = (x3 - cxr2) * (x3 - cxr2) + (y3 - cyr2) * (y3 - cyr2);
//     double D2 = sqrt(fabs(H2 - r * r));
//     double C2 = sqrt(fabs(1 - r * r / H2));
//     double st2 = sin(theta2);
//     double ct2 = cos(theta2);
//     double g =
//         r * (r * (r + ct2 * -dy_2 + st2 * dx_2) / H2 -
//              (r * r * (ct2 * dx_2 + st2 * dy_2)) / (C2 * H2 * sqrt(H2))) -
//         r * (ct2 * dx_1 + st2 * dy_1) / D1 + r * (ct2 * dx_2 + st2 * dy_2) /
//         D2;
//     int iter = 1;
//     while (fabs(g) > 0.1) {
//       theta2 = theta2 - g * pi / (180 * sqrt(iter));
//       cxr2 = x2 + r * sin(theta2);
//       cyr2 = y2 - r * cos(theta2);
//       D1 = norm(cxr1, cyr1, cxr2, cyr2);
//       H2 = (x3 - cxr2) * (x3 - cxr2) + (y3 - cyr2) * (y3 - cyr2);
//       D2 = sqrt(fabs(H2 - r * r));
//       C2 = sqrt(fabs(1 - r * r / H2));
//       st2 = sin(theta2);
//       ct2 = cos(theta2);
//       g = r * (r * (r + ct2 * -dy_2 + st2 * dx_2) / H2 -
//                (r * r * (ct2 * dx_2 + st2 * dy_2)) / (C2 * H2 * sqrt(H2))) -
//           r * (ct2 * dx_1 + st2 * dy_1) / D1 +
//           r * (ct2 * dx_2 + st2 * dy_2) / D2;
//       iter++;
//       if (iter > exit_it) {
//         break;
//       }
//     }
//   }
//
//   x2 = p2_0.x;
//   y2 = p2_0.y;
//   x3 = p3_0.x;
//   y3 = p3_0.y;
//   theta2 = theta2 + theta1 - pi / 2;
//
//   cxr2 = x2 + r * sin(theta2);
//   cyr2 = y2 - r * cos(theta2);
//   if (sqrt(dx_1 * dx_1 + dy_1 * dy_1) <= r) {
//     const double c1 = norm(x1, y1, x2, y2);
//     const double A = c1 * sqrt(4 * r * r - c1 * c1) + 2 * r * (x1 - x2);
//     const double B = c1 * c1 + 2 * r * (y1 - y2);
//     theta2 = pi - 2 * atan(A / B);
//     cxr2 = x2 + r * sin(theta2);
//     cyr2 = y2 - r * cos(theta2);
//   }
//
//   H2 = norm(cxr2, cyr2, x3, y3);
//   if (H2 <= r) {
//     const double c1 = norm(x2, y2, x3, y3);
//     const double A = c1 * sqrt(4 * r * r - c1 * c1) + 2 * r * (x3 - x2);
//     const double B = c1 * c1 + 2 * r * (y2 - y3);
//     theta2 = 2 * atan(A / B);
//     cxr2 = x2 + r * sin(theta2);
//     cyr2 = y2 - r * cos(theta2);
//   }
//
//   double t2 = 0;
//   urs_.getOmega(x2, y2, x3, y3, r, theta2, t2, true);
//   t2 = t2 + theta2 - pi / 2;
//
//   s2.theta = theta2;
//   s3.theta = t2;
// }
//
// /*****************************************************************************/
// /*****************************************************************************/
// /*****************************************************************************/
// inline void Marching::LLPositive(const State s1, State &s2, State &s3,
//                                  const Point p2_0, const Point p3_0) const {
//   const double x1 = s1.x;
//   const double y1 = s1.y;
//   const double theta1 = s1.theta;
//   const double r = r_;
//   double x2 = s2.x;
//   double y2 = s2.y;
//   double theta2 = s2.theta;
//   double x3 = s3.x;
//   double y3 = s3.y;
//
//   const double st1 = sin(theta1);
//   const double ct1 = cos(theta1);
//   double cxl1 = x1 - r * st1 * st1 - r * ct1 * ct1;
//   double cyl1 = y1 + r * st1 * ct1 - r * ct1 * st1;
//   double cxl2 = x2 - r * sin(theta2);
//   double cyl2 = y2 + r * cos(theta2);
//
//   double frac = r / norm(cxl2, cyl2, x3, y3);
//   double B;
//   if (frac >= 1) {
//     B = pi / 2;
//   } else {
//     B = asin(frac);
//   }
//   theta2 = 0.5 * wrapTo2Pi(atan2(y3 - cyl2, x3 - cxl2)) + 0.5 * B +
//            0.5 * wrapTo2Pi(atan2(cyl2 - cyl1, cxl2 - cxl1));
//
//   cxl2 = x2 - r * sin(theta2);
//   cyl2 = y2 + r * cos(theta2);
//
//   double H2;
//   const double dx_1 = (cxl1 - x2);
//   const double dy_1 = (cyl1 - y2);
//
//   if (GD_bisection_ == 2) {
//     // Use bisection method to find the correct theta2 around the initial
//     guess double a = theta2 - 0.25; double b = theta2 + 0.25; double c = (b -
//     a) / 3; const double eps = 0.001; const int exit_it = 100; int it = 1;
//     while (fabs(b - a) > eps && it < exit_it) {
//       // two interior points
//       double t1 = a + c;
//       double t2 = b - c;
//       double f1, f2;
//       cxl2 = x2 - r * sin(t1);
//       cyl2 = y2 + r * cos(t1);
//       frac = r / norm(cxl2, cyl2, x3, y3);
//       if (frac >= 1) {
//         B = pi / 2;
//       } else {
//         B = asin(frac);
//       }
//       f1 = 0.5 * wrapTo2Pi(atan2(y3 - cyl2, x3 - cxl2)) + 0.5 * B +
//            0.5 * wrapTo2Pi(atan2(cyl2 - cyl1, cxl2 - cxl1));
//       cxl2 = x2 - r * sin(t2);
//       cyl2 = y2 + r * cos(t2);
//       frac = r / norm(cxl2, cyl2, x3, y3);
//       if (frac >= 1) {
//         B = pi / 2;
//       } else {
//         B = asin(frac);
//       }
//       f2 = 0.5 * wrapTo2Pi(atan2(y3 - cyl2, x3 - cxl2)) + 0.5 * B +
//            0.5 * wrapTo2Pi(atan2(cyl2 - cyl1, cxl2 - cxl1));
//       if (f1 < f2) {
//         b = t2;
//       } else {
//         a = t1;
//       }
//       it++;
//     }
//     theta2 = 0.5 * (a + b);
//   } else {
//     const double dx_2 = (x2 - x3);
//     const double dy_2 = (y2 - y3);
//     double D1 = norm(cxl1, cyl1, cxl2, cyl2);
//     H2 = (x3 - cxl2) * (x3 - cxl2) + (y3 - cyl2) * (y3 - cyl2);
//     double D2 = sqrt(fabs(H2 - r * r));
//     double C2 = sqrt(fabs(1 - r * r / H2));
//     double st2 = sin(theta2);
//     double ct2 = cos(theta2);
//     double g =
//         r * ((r * (r + ct2 * dy_2 - st2 * dx_2)) / H2 +
//              (r * r * (ct2 * dx_2 + st2 * dy_2)) / (C2 * H2 * sqrt(H2))) +
//         (r * (ct2 * dx_1 + st2 * dy_1)) / D1 -
//         r * (ct2 * dx_2 + st2 * dy_2) / D2;
//     int iter = 1;
//     while (fabs(g) > 0.1) {
//       theta2 = theta2 - g * pi / (180 * sqrt(iter));
//       cxl2 = x2 - r * sin(theta2);
//       cyl2 = y2 + r * cos(theta2);
//       D1 = sqrt((cxl2 - cxl1) * (cxl2 - cxl1) + (cyl2 - cyl1) * (cyl2 -
//       cyl1)); H2 = (x3 - cxl2) * (x3 - cxl2) + (y3 - cyl2) * (y3 - cyl2); D2
//       = sqrt(fabs(H2 - r * r)); C2 = sqrt(fabs(1 - r * r / H2)); st2 =
//       sin(theta2); ct2 = cos(theta2); g = r * ((r * (r + ct2 * dy_2 - st2 *
//       dx_2)) / H2 +
//                (r * r * (ct2 * dx_2 + st2 * dy_2)) / (C2 * H2 * sqrt(H2))) +
//           (r * (ct2 * dx_1 + st2 * dy_1)) / D1 -
//           r * (ct2 * dx_2 + st2 * dy_2) / D2;
//       iter++;
//       if (iter > exit_it) {
//         break;
//       }
//     }
//   }
//
//   x2 = p2_0.x;
//   y2 = p2_0.y;
//   x3 = p3_0.x;
//   y3 = p3_0.y;
//   theta2 = theta2 + theta1 - pi / 2;
//
//   cxl2 = x2 - r * sin(theta2);
//   cyl2 = y2 + r * cos(theta2);
//   if (sqrt(dx_1 * dx_1 + dy_1 * dy_1) <= r) {
//     const double c1 = norm(x1, y1, x2, y2);
//     const double A = c1 * sqrt(4 * r * r - c1 * c1) + 2 * r * (x1 - x2);
//     const double B = c1 * c1 + 2 * r * (y1 - y2);
//     theta2 = -2 * atan(A / B);
//     cxl2 = x2 - r * sin(theta2);
//     cyl2 = y2 + r * cos(theta2);
//   }
//
//   H2 = norm(cxl2, cyl2, x3, y3);
//   if (H2 <= r) {
//     const double c1 = norm(x2, y2, x3, y3);
//     const double A = c1 * sqrt(4 * r * r - c1 * c1) + 2 * r * (x2 - x3);
//     const double B = c1 * c1 + 2 * r * (y3 - y2);
//     theta2 = 2 * atan(A / B);
//     cxl2 = x2 - r * sin(theta2);
//     cyl2 = y2 + r * cos(theta2);
//   }
//
//   double t2 = 0;
//   urs_.getOmega(x2, y2, x3, y3, r, theta2, t2, true);
//   t2 = t2 + theta2 - pi / 2;
//
//   s2.theta = theta2;
//   s3.theta = t2;
// }
//
// /*****************************************************************************/
// /*****************************************************************************/
// /*****************************************************************************/
// inline void Marching::LLNegative(const State s1, State &s2, State &s3,
//                                  const Point p2_0, const Point p3_0) const {
//   const double x1 = s1.x;
//   const double y1 = s1.y;
//   const double theta1 = s1.theta;
//   const double r = r_;
//   double x2 = s2.x;
//   double y2 = s2.y;
//   double theta2 = s2.theta;
//   double x3 = s3.x;
//   double y3 = s3.y;
//
//   const double st1 = sin(theta1);
//   const double ct1 = cos(theta1);
//   double cxl1 = x1 - r * st1 * st1 - r * ct1 * ct1;
//   double cyl1 = y1 + r * st1 * ct1 - r * ct1 * st1;
//   double cxl2 = x2 - r * sin(theta2);
//   double cyl2 = y2 + r * cos(theta2);
//
//   double frac = r / norm(cxl2, cyl2, x3, y3);
//   double B;
//   if (frac >= 1) {
//     B = pi / 2;
//   } else {
//     B = asin(frac);
//   }
//   theta2 = 0.5 * (atan2(cyl2 - y3, cxl2 - x3)) - 0.5 * B +
//            0.5 * (atan2(cyl1 - cyl2, cxl1 - cxl2));
//
//   cxl2 = x2 - r * sin(theta2);
//   cyl2 = y2 + r * cos(theta2);
//
//   const double dx_1 = (cxl1 - x2);
//   const double dy_1 = (cyl1 - y2);
//   double H2;
//
//   if (GD_bisection_ == 2) {
//     // Use bisection method to find the correct theta2 around the initial
//     guess double a = theta2 - 0.25; double b = theta2 + 0.25; double c = (b -
//     a) / 3; const double eps = 0.001; const int exit_it = 100; int it = 1;
//     while (fabs(b - a) > eps && it < exit_it) {
//       // two interior points
//       double t1 = a + c;
//       double t2 = b - c;
//       double f1, f2;
//       cxl2 = x2 - r * sin(t1);
//       cyl2 = y2 + r * cos(t1);
//       frac = r / norm(cxl2, cyl2, x3, y3);
//       if (frac >= 1) {
//         B = pi / 2;
//       } else {
//         B = asin(frac);
//       }
//       f1 = 0.5 * (atan2(cyl2 - y3, cxl2 - x3)) - 0.5 * B +
//            0.5 * (atan2(cyl1 - cyl2, cxl1 - cxl2));
//       cxl2 = x2 - r * sin(t2);
//       cyl2 = y2 + r * cos(t2);
//       frac = r / norm(cxl2, cyl2, x3, y3);
//       if (frac >= 1) {
//         B = pi / 2;
//       } else {
//         B = asin(frac);
//       }
//       f2 = 0.5 * (atan2(cyl2 - y3, cxl2 - x3)) - 0.5 * B +
//            0.5 * (atan2(cyl1 - cyl2, cxl1 - cxl2));
//       if (f1 < f2) {
//         b = t2;
//       } else {
//         a = t1;
//       }
//       it++;
//     }
//     theta2 = 0.5 * (a + b);
//   } else {
//     const double dx_2 = (x2 - x3);
//     const double dy_2 = (y2 - y3);
//     double D1 = norm(cxl1, cyl1, cxl2, cyl2);
//     H2 = (x3 - cxl2) * (x3 - cxl2) + (y3 - cyl2) * (y3 - cyl2);
//     double D2 = sqrt(fabs(H2 - r * r));
//     double C2 = sqrt(fabs(1 - r * r / H2));
//     double st2 = sin(theta2);
//     double ct2 = cos(theta2);
//     double g = r * (ct2 * dx_1 + st2 * dy_1) / D1 -
//                r * (r * (r + ct2 * dy_2 - st2 * dx_2) / H2 -
//                     r * r * (ct2 * dx_2 + st2 * dy_2) / (C2 * H2 * sqrt(H2)))
//                     -
//                r * (ct2 * dx_2 + st2 * dy_2) / D2;
//
//     int iter = 1;
//     while (fabs(g) > 0.1) {
//       theta2 = theta2 - g * pi / (180 * sqrt(iter));
//       cxl2 = x2 - r * sin(theta2);
//       cyl2 = y2 + r * cos(theta2);
//       D1 = norm(cxl1, cyl1, cxl2, cyl2);
//       H2 = (x3 - cxl2) * (x3 - cxl2) + (y3 - cyl2) * (y3 - cyl2);
//       D2 = sqrt(fabs(H2 - r * r));
//       C2 = sqrt(fabs(1 - r * r / H2));
//       st2 = sin(theta2);
//       ct2 = cos(theta2);
//       g = r * (ct2 * dx_1 + st2 * dy_1) / D1 -
//           r * (r * (r + ct2 * dy_2 - st2 * dx_2) / H2 -
//                r * r * (ct2 * dx_2 + st2 * dy_2) / (C2 * H2 * sqrt(H2))) -
//           r * (ct2 * dx_2 + st2 * dy_2) / D2;
//       iter++;
//       if (iter > exit_it) {
//         break;
//       }
//     }
//   }
//
//   x2 = p2_0.x;
//   y2 = p2_0.y;
//   x3 = p3_0.x;
//   y3 = p3_0.y;
//   theta2 = theta2 + theta1 - pi / 2;
//
//   cxl2 = x2 - r * sin(theta2);
//   cyl2 = y2 + r * cos(theta2);
//   if (sqrt(dx_1 * dx_1 + dy_1 * dy_1) <= r) {
//     const double c1 = norm(x1, y1, x2, y2);
//     const double A = c1 * sqrt(4 * r * r - c1 * c1) + 2 * r * (x2 - x1);
//     const double B = c1 * c1 + 2 * r * (y1 - y2);
//     theta2 = 2 * atan(A / B);
//     cxl2 = x2 - r * sin(theta2);
//     cyl2 = y2 + r * cos(theta2);
//   }
//
//   H2 = norm(cxl2, cyl2, x3, y3);
//   if (H2 <= r) {
//     const double c1 = norm(x2, y2, x3, y3);
//     const double A = c1 * sqrt(4 * r * r - c1 * c1) + 2 * r * (x3 - x2);
//     const double B = c1 * c1 + 2 * r * (y3 - y2);
//     theta2 = -2 * atan2(A, B);
//     cxl2 = x2 - r * sin(theta2);
//     cyl2 = y2 + r * cos(theta2);
//   }
//
//   double t2 = 0;
//   urs_.getOmega(x2, y2, x3, y3, r, theta2, t2, true);
//   t2 = wrapToPi(t2 + theta2 - pi / 2);
//
//   s2.theta = theta2;
//   s3.theta = t2;
// }

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
inline double Marching::evaluateDistance(const int LS_x, const int LS_y,
                                         const double LS_theta, int source,
                                         const int neighbour_x,
                                         const int neighbour_y,
                                         double &last_theta) {
  if (source == cameFrom_(LS_x, LS_y)) {
    const int mp_x = omega_map_.nx() / 2;
    const int mp_y = omega_map_.ny() / 2;
    const double cos_theta = cos(LS_theta);
    const double sin_theta = sin(LS_theta);
    const int dx = neighbour_x - LS_x;
    const int dy = neighbour_y - LS_y;

    const double xn = dx * cos_theta + dy * sin_theta + mp_x;
    const double yn = -dx * sin_theta + dy * cos_theta + mp_y;

    double d_accelerated;
    if (xn < 0 || yn < 0 || xn + 1 >= distance_map_.nx() ||
        yn + 1 >= distance_map_.ny()) {
      double to_theta;
      urs_.getOmega(LS_x, LS_y, neighbour_x, neighbour_y, r_, LS_theta,
                    to_theta);
      to_theta += LS_theta;
      State from = {static_cast<double>(LS_x), static_cast<double>(LS_y),
                    LS_theta + pi / 2};
      State to = {static_cast<double>(neighbour_x),
                  static_cast<double>(neighbour_y), to_theta};
      int condition, Q;
      solver_.acceleratedRSPlanner(from, to, d_accelerated, r_, condition, Q);
    } else {
      d_accelerated = bilinear_interpolation(xn, yn);
    }
    const double distance = gScore_(LS_x, LS_y) + d_accelerated;
    urs_.getOmega(LS_x, LS_y, neighbour_x, neighbour_y, r_, LS_theta,
                  last_theta);
    last_theta += LS_theta;
    return distance;
  } else {
    std::vector<double> x, y, theta;
    x.reserve(20);
    y.reserve(20);
    theta.reserve(20);

    double initial_theta = 0;
    while (true) {
      x.emplace_back(lightSources_[source].x);
      y.emplace_back(lightSources_[source].y);
      source = cameFrom_(lightSources_[source].x, lightSources_[source].y);
      if (source ==
          cameFrom_(lightSources_[source].x, lightSources_[source].y)) {
        x.emplace_back(lightSources_[source].x);
        y.emplace_back(lightSources_[source].y);
        initial_theta = lightSources_[source].theta + pi / 2;
        break;
      }
    }
    std::reverse(x.begin(), x.end());
    std::reverse(y.begin(), y.end());
    x.emplace_back(neighbour_x);
    y.emplace_back(neighbour_y);
    const size_t s = x.size();

    theta.emplace_back(initial_theta);

    for (size_t i = 0; i < s - 2; ++i) {
      const double x1 = x[i], y1 = y[i], theta1 = wrapTo2Pi(theta[i]);
      double x2 = x[i + 1], y2 = y[i + 1], x3 = x[i + 2], y3 = y[i + 2];

      double omega_1;
      urs_.getOmega(x1, y1, x2, y2, r_, theta1, omega_1, true);
      omega_1 += theta1 - pi / 2;

      const double cos_theta1_minus_pi_2 = cos(theta1 - pi / 2);
      const double sin_theta1_minus_pi_2 = sin(theta1 - pi / 2);

      const double dx2 = x1 - x2;
      const double dy2 = y1 - y[i + 1];
      const double dx3 = x1 - x3;
      const double dy3 = y1 - y[i + 2];

      x2 = x1 - cos_theta1_minus_pi_2 * dx2 - sin_theta1_minus_pi_2 * dy2;
      x3 = x1 - cos_theta1_minus_pi_2 * dx3 - sin_theta1_minus_pi_2 * dy3;
      y2 = y1 - cos_theta1_minus_pi_2 * dy2 + sin_theta1_minus_pi_2 * dx2;
      y3 = y1 - cos_theta1_minus_pi_2 * dy3 + sin_theta1_minus_pi_2 * dx3;

      const double atan_3_2 = atan2(y3 - y2, x3 - x2);
      const double atan_2_1 = atan2(y2 - y1, x2 - x1);
      const double theta2 = 0.5 * wrapTo2Pi(atan_3_2 + atan_2_1);

      State s1 = {x1, y1, theta1}, s2 = {x2, y2, theta2}, s3 = {x3, y3, 0};
      Point p2_0 = {x[i + 1], y[i + 1]}, p3_0 = {x[i + 2], y[i + 2]};

      const double x3_t = -cos(omega_1 - pi / 2) * (x[i + 1] - x3) -
                          sin(omega_1 - pi / 2) * (y[i + 1] - y3);

      if (x2 >= x1) {
        if (y2 <= y1) {
          (x3_t >= 0) ? RRNegative(s1, s2, s3, p2_0, p3_0)
                      : RLNegative(s1, s2, s3, p2_0, p3_0);
        } else {
          (x3_t >= 0) ? RRPositive(s1, s2, s3, p2_0, p3_0)
                      : RLPositive(s1, s2, s3, p2_0, p3_0);
        }
      } else {
        if (y2 <= y1) {
          (x3_t >= 0) ? LRNegative(s1, s2, s3, p2_0, p3_0)
                      : LLNegative(s1, s2, s3, p2_0, p3_0);
        } else {
          (x3_t >= 0) ? LRPositive(s1, s2, s3, p2_0, p3_0)
                      : LLPositive(s1, s2, s3, p2_0, p3_0);
        }
      }
      theta.emplace_back(s2.theta);
      if (i == s - 3) {
        theta.emplace_back(s3.theta);
      }
    }
    last_theta = theta[s - 1];
    double d = 0;
    for (size_t i = 0; i < s - 1; ++i) {
      State from = {x[i], y[i], theta[i]};
      State to = {x[i + 1], y[i + 1], theta[i + 1]};
      double d1;
      int condition, Q;
      solver_.acceleratedRSPlanner(from, to, d1, r_, condition, Q);
      d += d1;
    }
    return d;
  }
}
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
// inline double Marching::evaluateDistance(const int LS_x, const int LS_y,
//                                          const double LS_theta, int source,
//                                          const int neighbour_x,
//                                          const int neighbour_y,
//                                          double &last_theta) {
//   if (source == cameFrom_(LS_x, LS_y)) {
//     const int mp_x = omega_map_.nx() / 2;
//     const int mp_y = omega_map_.ny() / 2;
//     const size_t xn = (neighbour_x - LS_x) * cos(LS_theta) +
//                       (neighbour_y - LS_y) * sin(LS_theta) + mp_x;
//     const size_t yn = -(neighbour_x - LS_x) * sin(LS_theta) +
//                       (neighbour_y - LS_y) * cos(LS_theta) + mp_y;
//
//     double d_accelerated;
//     if (xn + 1 >= distance_map_.nx() || yn + 1 >= distance_map_.ny()) {
//       double to_theta;
//       urs_.getOmega(LS_x, LS_y, neighbour_x, neighbour_y, r_, LS_theta,
//                     to_theta);
//       to_theta = to_theta + LS_theta;
//       State from = {static_cast<double>(LS_x), static_cast<double>(LS_y),
//                     LS_theta + pi / 2};
//       State to = {static_cast<double>(neighbour_x),
//                   static_cast<double>(neighbour_y), to_theta};
//       int condition, Q;
//       solver_.acceleratedRSPlanner(from, to, d_accelerated, r_, condition,
//       Q);
//     } else {
//       d_accelerated = bilinear_interpolation(xn, yn);
//     }
//     const double distance = gScore_(LS_x, LS_y) + d_accelerated;
//     urs_.getOmega(LS_x, LS_y, neighbour_x, neighbour_y, r_, LS_theta,
//                   last_theta);
//     last_theta = last_theta + LS_theta;
//     return distance;
//   } else {
//     std::vector<double> x;
//     std::vector<double> y;
//     x.reserve(20);
//     y.reserve(20);
//     double initial_theta = 0;
//     while (true) {
//       x.push_back(lightSources_[source].x);
//       y.push_back(lightSources_[source].y);
//       source = cameFrom_(lightSources_[source].x, lightSources_[source].y);
//       if (source ==
//           cameFrom_(lightSources_[source].x, lightSources_[source].y)) {
//         x.push_back(lightSources_[source].x);
//         y.push_back(lightSources_[source].y);
//         initial_theta = lightSources_[source].theta + pi / 2;
//         break;
//       }
//     }
//     std::reverse(x.begin(), x.end());
//     std::reverse(y.begin(), y.end());
//     x.push_back(neighbour_x);
//     y.push_back(neighbour_y);
//     const size_t s = x.size();
//
//     std::vector<double> theta;
//     theta.reserve(s);
//     theta.push_back(initial_theta);
//
//     double x1, y1, theta1, x2, y2, theta2, x3, y3;
//     // const Point poi = {310 - 1, 265 - 1};
//     // const Point poi = {275 - 1, 330 - 1};
//     // const Point poi = {-1, 0};
//
//     for (size_t i = 0; i < s - 2; ++i) {
//       x1 = x[i];
//       y1 = y[i];
//       theta1 = wrapTo2Pi(theta[i]);
//
//       x2 = x[i + 1];
//       y2 = y[i + 1];
//       x3 = x[i + 2];
//       y3 = y[i + 2];
//
//       double omega_1;
//       urs_.getOmega(x1, y1, x2, y2, r_, theta1, omega_1, true);
//       omega_1 = omega_1 + theta1 - pi / 2;
//       // omega_1 = wrapTo2Pi(omega_1);
//       // double omega_1 = thetas_(x2, y2);
//       const double x3_t = -cos(omega_1 - pi / 2) * (x2 - x3) -
//                           sin(omega_1 - pi / 2) * (y2 - y3);
//
//       // TODO: can SIMD?
//       x2 = x1 - cos(theta1 - pi / 2) * (x1 - x2) -
//            sin(theta1 - pi / 2) * (y1 - y2);
//       x3 = x1 - cos(theta1 - pi / 2) * (x1 - x3) -
//            sin(theta1 - pi / 2) * (y1 - y3);
//       y2 = y1 - cos(theta1 - pi / 2) * (y1 - y2) +
//            sin(theta1 - pi / 2) * (x1 - x[i + 1]);
//       y3 = y1 - cos(theta1 - pi / 2) * (y1 - y3) +
//            sin(theta1 - pi / 2) * (x1 - x[i + 2]);
//
//       double atan_3_2 = atan2(y3 - y2, x3 - x2);
//       double atan_2_1 = atan2(y2 - y1, x2 - x1);
//       theta2 = 0.5 * wrapTo2Pi(atan_3_2 + atan_2_1);
//
//       const State s1 = {x1, y1, theta1};
//       State s2 = {x2, y2, theta2};
//       State s3 = {x3, y3, 0};
//       // const Point neighbour = {static_cast<double>(neighbour_x),
//       // static_cast<double>(neighbour_y)};
//       const Point p2_0 = {x[i + 1], y[i + 1]};
//       const Point p3_0 = {x[i + 2], y[i + 2]};
//
//       if (x2 >= x1) {
//         if (y2 <= y1) {
//           if (x3_t >= 0) {
//             RRNegative(s1, s2, s3, p2_0, p3_0);
//           } else {
//             RLNegative(s1, s2, s3, p2_0, p3_0);
//           }
//         } else {
//           if (x3_t >= 0) {
//             RRPositive(s1, s2, s3, p2_0, p3_0);
//           } else {
//             RLPositive(s1, s2, s3, p2_0, p3_0);
//           }
//         }
//       } else {
//         if (y2 <= y1) {
//           if (x3_t >= 0) {
//             LRNegative(s1, s2, s3, p2_0, p3_0);
//           } else {
//             LLNegative(s1, s2, s3, p2_0, p3_0);
//           }
//         } else {
//           if (x3_t >= 0) {
//             LRPositive(s1, s2, s3, p2_0, p3_0);
//           } else {
//             LLPositive(s1, s2, s3, p2_0, p3_0);
//           }
//         }
//       }
//       theta.push_back(s2.theta);
//       if (i == s - 3) {
//         theta.push_back(s3.theta);
//       }
//     }
//     last_theta = theta[s - 1];
//     double d = 0;
//     State from, to;
//     for (size_t i = 0; i < x.size() - 1; ++i) {
//       from = {x[i], y[i], theta[i]};
//       to = {x[i + 1], y[i + 1], theta[i + 1]};
//       double d1;
//       int condition, Q;
//       solver_.acceleratedRSPlanner(from, to, d1, r_, condition, Q);
//       d += d1;
//     }
//     return d;
//   }
// }

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
// inline double Marching::evaluateDistance(const int LS_x, const int LS_y,
//                                          const double LS_theta, int source,
//                                          const int neighbour_x,
//                                          const int neighbour_y,
//                                          double &last_theta) {
//   // Avoid repeatedly calculating the same value
//   const double cos_LS_theta = cos(LS_theta);
//   const double sin_LS_theta = sin(LS_theta);
//
//   if (source == cameFrom_(LS_x, LS_y)) {
//     // Cache these constants instead of recomputing
//     const int dx = neighbour_x - LS_x;
//     const int dy = neighbour_y - LS_y;
//     const int mp_x = omega_map_.nx() / 2;
//     const int mp_y = omega_map_.ny() / 2;
//
//     // Use precomputed sin/cos values
//     const size_t xn = dx * cos_LS_theta + dy * sin_LS_theta + mp_x;
//     const size_t yn = -dx * sin_LS_theta + dy * cos_LS_theta + mp_y;
//
//     double d_accelerated;
//     if (xn + 1 >= distance_map_.nx() || yn + 1 >= distance_map_.ny()) {
//       // Avoid redundant theta calculation by reusing last_theta directly
//       urs_.getOmega(LS_x, LS_y, neighbour_x, neighbour_y, r_, LS_theta,
//                     last_theta);
//       const double to_theta = last_theta + LS_theta;
//
//       // Use direct initialization instead of assignment
//       const State from{static_cast<double>(LS_x), static_cast<double>(LS_y),
//                        LS_theta + pi / 2};
//       const State to{static_cast<double>(neighbour_x),
//                      static_cast<double>(neighbour_y), to_theta};
//
//       int condition, Q;
//       solver_.acceleratedRSPlanner(from, to, d_accelerated, r_, condition,
//       Q);
//     } else {
//       d_accelerated = bilinear_interpolation(xn, yn);
//     }
//
//     // Compute final distance
//     const double distance = gScore_(LS_x, LS_y) + d_accelerated;
//
//     // Only calculate last_theta if we haven't already
//     if (xn + 1 < distance_map_.nx() && yn + 1 < distance_map_.ny()) {
//       urs_.getOmega(LS_x, LS_y, neighbour_x, neighbour_y, r_, LS_theta,
//                     last_theta);
//       last_theta += LS_theta;
//     }
//
//     return distance;
//   } else {
//     // Pre-allocate with expected size to avoid reallocations
//     std::vector<double> x;
//     std::vector<double> y;
//     x.reserve(20);
//     y.reserve(20);
//
//     // First part: collect path points
//     double initial_theta = 0;
//     int current_source = source;
//
//     // Gather all points in one pass
//     while (true) {
//       x.push_back(lightSources_[current_source].x);
//       y.push_back(lightSources_[current_source].y);
//
//       int next_source = cameFrom_(lightSources_[current_source].x,
//                                   lightSources_[current_source].y);
//       current_source = next_source;
//
//       if (current_source == cameFrom_(lightSources_[current_source].x,
//                                       lightSources_[current_source].y)) {
//         x.push_back(lightSources_[current_source].x);
//         y.push_back(lightSources_[current_source].y);
//         initial_theta = lightSources_[current_source].theta + pi / 2;
//         break;
//       }
//     }
//
//     // Avoid individual reverses by building in the right order from the
//     start const size_t path_size = x.size(); std::vector<double>
//     x_reversed(path_size); std::vector<double> y_reversed(path_size);
//
//     for (size_t i = 0; i < path_size; ++i) {
//       x_reversed[path_size - 1 - i] = x[i];
//       y_reversed[path_size - 1 - i] = y[i];
//     }
//
//     // Add neighbor to the end
//     x_reversed.push_back(neighbour_x);
//     y_reversed.push_back(neighbour_y);
//     const size_t s = x_reversed.size();
//
//     // Preallocate theta vector
//     std::vector<double> theta;
//     theta.reserve(s);
//     theta.push_back(initial_theta);
//
//     // Main loop for calculating path
//     for (size_t i = 0; i < s - 2; ++i) {
//       const double x1 = x_reversed[i];
//       const double y1 = y_reversed[i];
//       const double theta1 = wrapTo2Pi(theta[i]);
//
//       const double x2_orig = x_reversed[i + 1];
//       const double y2_orig = y_reversed[i + 1];
//       const double x3_orig = x_reversed[i + 2];
//       const double y3_orig = y_reversed[i + 2];
//
//       // Cache trigonometric calculations
//       const double cos_theta_offset = cos(theta1 - pi / 2);
//       const double sin_theta_offset = sin(theta1 - pi / 2);
//
//       double omega_1;
//       urs_.getOmega(x1, y1, x2_orig, y2_orig, r_, theta1, omega_1, true);
//       omega_1 = omega_1 + theta1 - pi / 2;
//
//       // Calculate transformed coordinates in one go
//       const double x3_t = -cos(omega_1 - pi / 2) * (x2_orig - x3_orig) -
//                           sin(omega_1 - pi / 2) * (y2_orig - y3_orig);
//
//       // Transform coordinates more efficiently
//       const double dx2 = x1 - x2_orig;
//       const double dy2 = y1 - y2_orig;
//       const double dx3 = x1 - x3_orig;
//       const double dy3 = y1 - y3_orig;
//
//       const double x2 = x1 - cos_theta_offset * dx2 - sin_theta_offset * dy2;
//       const double x3 = x1 - cos_theta_offset * dx3 - sin_theta_offset * dy3;
//       const double y2 = y1 - cos_theta_offset * dy2 + sin_theta_offset * dx2;
//       const double y3 = y1 - cos_theta_offset * dy3 + sin_theta_offset * dx3;
//
//       // Calculate angles once
//       const double atan_3_2 = atan2(y3 - y2, x3 - x2);
//       const double atan_2_1 = atan2(y2 - y1, x2 - x1);
//       const double theta2 = 0.5 * wrapTo2Pi(atan_3_2 + atan_2_1);
//
//       // Define states and points using aggregate initialization
//       const State s1{x1, y1, theta1};
//       State s2{x2, y2, theta2};
//       State s3{x3, y3, 0};
//       const Point p2_0{x2_orig, y2_orig};
//       const Point p3_0{x3_orig, y3_orig};
//
//       // Optimize the branching logic by reducing nested conditions
//       if (x2 >= x1) {
//         if (y2 <= y1) {
//           (x3_t >= 0) ? RRNegative(s1, s2, s3, p2_0, p3_0)
//                       : RLNegative(s1, s2, s3, p2_0, p3_0);
//         } else {
//           (x3_t >= 0) ? RRPositive(s1, s2, s3, p2_0, p3_0)
//                       : RLPositive(s1, s2, s3, p2_0, p3_0);
//         }
//       } else {
//         if (y2 <= y1) {
//           (x3_t >= 0) ? LRNegative(s1, s2, s3, p2_0, p3_0)
//                       : LLNegative(s1, s2, s3, p2_0, p3_0);
//         } else {
//           (x3_t >= 0) ? LRPositive(s1, s2, s3, p2_0, p3_0)
//                       : LLPositive(s1, s2, s3, p2_0, p3_0);
//         }
//       }
//
//       theta.push_back(s2.theta);
//       if (i == s - 3) {
//         theta.push_back(s3.theta);
//       }
//     }
//
//     last_theta = theta[s - 1];
//
//     // Calculate total distance
//     double total_distance = 0;
//     for (size_t i = 0; i < x_reversed.size() - 1; ++i) {
//       const State from{x_reversed[i], y_reversed[i], theta[i]};
//       const State to{x_reversed[i + 1], y_reversed[i + 1], theta[i + 1]};
//
//       double segment_distance;
//       int condition, Q;
//       solver_.acceleratedRSPlanner(from, to, segment_distance, r_, condition,
//                                    Q);
//       total_distance += segment_distance;
//     }
//
//     return total_distance;
//   }
// }

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
double Marching::spherical_interpolation(const double xn, const double yn,
                                         const double LS_theta) const {
  int x0 = static_cast<int>(floor(xn));
  int y0 = static_cast<int>(floor(yn));
  int x1 = x0 + 1;
  int y1 = y0 + 1;

  const double a11 = angle_map_(x0, y0) + LS_theta;
  const double a12 = angle_map_(x0, y1) + LS_theta;
  const double a21 = angle_map_(x1, y0) + LS_theta;
  const double a22 = angle_map_(x1, y1) + LS_theta;

  const double w1 = (x1 - xn) * (y1 - yn);
  const double w2 = (x1 - xn) * (yn - y0);
  const double w3 = (xn - x0) * (y1 - yn);
  const double w4 = (xn - x0) * (yn - y0);

  // Perform slerp on the angles
  const double interpolated_angle =
      atan2(w1 * sin(a11) + w2 * sin(a12) + w3 * sin(a21) + w4 * sin(a22),
            w1 * cos(a11) + w2 * cos(a12) + w3 * cos(a21) + w4 * cos(a22));

  return interpolated_angle;
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
double Marching::omega_interpolation(const double xn, const double yn,
                                     const double LS_theta) const {
  int x0 = static_cast<int>(floor(xn));
  int y0 = static_cast<int>(floor(yn));
  int x1 = x0 + 1;
  int y1 = y0 + 1;

  const double a11 = omega_map_(x0, y0) + LS_theta;
  const double a12 = omega_map_(x0, y1) + LS_theta;
  const double a21 = omega_map_(x1, y0) + LS_theta;
  const double a22 = omega_map_(x1, y1) + LS_theta;

  const double w1 = (x1 - xn) * (y1 - yn);
  const double w2 = (x1 - xn) * (yn - y0);
  const double w3 = (xn - x0) * (y1 - yn);
  const double w4 = (xn - x0) * (yn - y0);

  // Perform slerp on the angles
  const double interpolated_angle =
      atan2(w1 * sin(a11) + w2 * sin(a12) + w3 * sin(a21) + w4 * sin(a22),
            w1 * cos(a11) + w2 * cos(a12) + w3 * cos(a21) + w4 * cos(a22));

  return interpolated_angle;
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
double Marching::bilinear_interpolation(double xn, double yn) const {
  int x0 = static_cast<int>(floor(xn));
  int y0 = static_cast<int>(floor(yn));
  int x1 = x0 + 1;
  int y1 = y0 + 1;

  const double a11 = distance_map_(x0, y0);
  const double a12 = distance_map_(x0, y1);
  const double a21 = distance_map_(x1, y0);
  const double a22 = distance_map_(x1, y1);

  const double w1 = (x1 - xn) * (y1 - yn);
  const double w2 = (x1 - xn) * (yn - y0);
  const double w3 = (xn - x0) * (y1 - yn);
  const double w4 = (xn - x0) * (yn - y0);

  const double interpolated_distance =
      w1 * a11 + w2 * a12 + w3 * a21 + w4 * a22;
  return interpolated_distance;
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
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
  for (size_t i = 0; i < nb_of_sources_; ++i) {
    os << lightSources_[i].x << " " << lightSources_[i].y << " "
       << lightSources_[i].theta << std::endl;
  }
  of.close();
  if (!sharedConfig_->silent) {
    std::cout << "\nSaved light sources" << std::endl;
  }
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
Marching::Marching(Environment &env)
    : sharedVisibilityField_(env.getVisibilityField()),
      sharedConfig_(env.getConfig()) {
  nx_ = sharedVisibilityField_->nx();
  ny_ = sharedVisibilityField_->ny();
  r_ = sharedConfig_->radius;
  exit_it = sharedConfig_->exit_it;
  visibilityThreshold_ = sharedConfig_->visibilityThreshold;
  bumps = sharedConfig_->bumps;
  sharedConfig_->angleThreshold =
      fabs(wrapToPi(sharedConfig_->angleThreshold * pi / 180));
  GD_bisection_ = sharedConfig_->GD_bisection;
  reset();
  // Reserve memoized fields
  auto n = 2 * std::max(nx_, ny_);
  omega_map_.reset(n, n, 0);
  distance_map_.reset(n, n, 0);
  angle_map_.reset(n, n, 0);
  memoize();
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void Marching::memoize() {
  // Algo:
  //  memoize angles:
  //  1- compute omega_map
  //  2- compute distance_map
  //  3- compute the vector field components
  //  4- compute angles representing the vector field components
  //  5- save angles
  //  6- for any from_.theta, do local rotation, do spherical interpolation
  //  extract angle a, and based on it perform marching
  //  7- do the same for distances? maybe later - bilinear interpolation

  auto start_time = std::chrono::high_resolution_clock::now();

  // State from = {1.0 * nx_, 1.0 * ny_, 0 * pi / 180};
  State from = {1.0 * omega_map_.nx() / 2, 1.0 * omega_map_.ny() / 2,
                0 * pi / 180};
  for (size_t i = 0; i < omega_map_.nx(); ++i) {
    for (size_t j = 0; j < omega_map_.ny(); ++j) {
      urs_.getOmega(from.x, from.y, i, j, r_, from.theta, omega_);
      omega_map_(i, j) = (omega_ + from.theta);
    }
  }

  double d_accelerated = 0;
  int Q, condition;
  for (size_t i = 0; i < distance_map_.nx(); ++i) {
    for (size_t j = 0; j < distance_map_.ny(); ++j) {
      to_.x = i;
      to_.y = j;
      to_.theta = omega_map_(i, j);
      solver_.acceleratedRSPlanner(from, to_, d_accelerated, r_, condition, Q,
                                   true);
      distance_map_(i, j) = d_accelerated;
    }
  }

  Field<double> x_component(omega_map_.nx(), omega_map_.ny(), 0);
  Field<double> y_component(omega_map_.nx(), omega_map_.ny(), 0);
  for (size_t i = 0; i < omega_map_.nx(); ++i) {
    for (size_t j = 0; j < omega_map_.ny(); ++j) {
      if (j < from.y) {
        x_component(i, j) = sin(omega_map_(i, j) - pi);
        y_component(i, j) = cos(omega_map_(i, j) - pi);
      } else {
        x_component(i, j) = sin(omega_map_(i, j));
        y_component(i, j) = cos(omega_map_(i, j));
      }
    }
  }

  for (size_t i = 0; i < x_component.nx(); ++i) {
    for (size_t j = 0; j < x_component.ny(); ++j) {
      angle_map_(i, j) =
          wrapToPi(atan2(-y_component(i, j), x_component(i, j)) + pi / 2);
      if (i == nx_ && j == ny_) {
        angle_map_(i, j) = 0;
      }
    }
  }

  auto stop_time = std::chrono::high_resolution_clock::now();
  auto execution_duration = durationInMicroseconds(start_time, stop_time);
  std::cout << "Memoization time in us: " << execution_duration << "us"
            << std::endl;
  std::cout << "Memoization size: "
            << distance_map_.nx() * distance_map_.ny() * 3 * sizeof(double)
            << " bytes" << std::endl;
  std::cout << "Memoized dimensions: " << distance_map_.nx() << "x"
            << distance_map_.ny() << std::endl;

  // const int step = 30;
  // vector_2d u((2 * nx_ - 1) / step + 1,
  //             std::vector<double>((2 * ny_ - 1) / step + 1));
  // vector_2d v((2 * nx_ - 1) / step + 1,
  //             std::vector<double>((2 * ny_ - 1) / step + 1));
  // for (int i = 0; i < 2 * nx_ - 1; i += step) {
  //   for (int j = 0; j < 2 * ny_ - 1; j += step) {
  //     u[i / step][j / step] = x_component(i, j);
  //     v[i / step][j / step] = y_component(i, j);
  //   }
  // }
  //
  // auto [x, y] =
  //     meshgrid(iota(0, step, 2 * nx_ - 1), iota(0, step, 2 * ny_ - 1));
  // std::swap(x, y);
  // std::swap(u, v);
  //
  // figure(1);
  // auto qv = quiver(x, y, u, v, 1);
  // qv->line_width(2);
  // // quiver(x, y, u, v, 2);
  // xlim({0, static_cast<double>(2 * nx_ - 1)});
  // ylim({0, static_cast<double>(2 * ny_ - 1)});
  // gca()->color("white");
  // hold(on);
  //
  // gcf()->size(2 * nx_, 2 * ny_);
  // show();
  // axis("equal");
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void Marching::reset() {
  gScore_.reset(nx_, ny_, std::numeric_limits<double>::max());
  thetas_.reset(nx_, ny_, 0);
  cameFrom_.reset(nx_, ny_, -1);
  origin_.reset(nx_, ny_, -1);
  inOpenSet_.reset(nx_, ny_, false);
  isUpdated_.reset(nx_, ny_, false);

  lightSources_.reset(new pose[nx_ * ny_]);

  visibilityHashMap_.clear();
  openSet_.reset();

  // Reserve openSet_
  std::vector<Node> container;
  container.reserve(nx_ * ny_);
  std::priority_queue<Node, std::vector<Node>, std::less<Node>> heap(
      std::less<Node>(), std::move(container));
  openSet_ = std::make_unique<std::priority_queue<Node>>(heap);

  nb_of_sources_ = 0;
  nb_of_iterations_ = 0;

  // Reserve hash map
  visibilityHashMap_.reserve(nx_ * ny_);

  key_ = true;
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void Marching::runMarching() {
  auto startTime = std::chrono::high_resolution_clock::now();
  // Init
  int x = 0, y = 0, neighbour_x = 0, neighbour_y = 0;
  double theta = 0;

  std::vector<Node> initial_frontline;
  if (sharedConfig_->pose_randomness) {
    std::cout << "Randomizing" << std::endl;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis_x(0, nx_ - 1);
    std::uniform_int_distribution<> dis_y(0, ny_ - 1);
    std::uniform_real_distribution<> dis_theta(-pi, pi);
    int x1, x2, y1, y2;
    double theta1, theta2;
    while (true) {
      bool clear = true;
      x1 = dis_x(gen);
      y1 = dis_y(gen);
      theta1 = dis_theta(gen);
      x2 = dis_x(gen);
      y2 = dis_y(gen);
      theta2 = dis_theta(gen);
      if (sharedVisibilityField_->get(x1, y1) < 1 ||
          sharedVisibilityField_->get(x2, y2) < 1) {
        clear = false;
      }
      if (clear) {
        break;
      }
    }
    initial_frontline.push_back(Node{x1, y1, theta1 * 180 / pi});
    initial_frontline.push_back(Node{x2, y2, theta2 * 180 / pi});
  } else {
    initial_frontline = sharedConfig_->initialFrontline;
  }

  // double f = 0;
  double g = 0;

  for (size_t i = 0; i < initial_frontline.size(); ++i) {
    x = initial_frontline[i].x;
    y = initial_frontline[i].y;
    theta = wrapToPi(initial_frontline[i].f * pi / 180 - pi / 2);
    std::cout << "Initial position: " << x << ", " << y << ", "
              << (theta + pi / 2) * 180 / pi << std::endl;

    if (i == 0) {
      x_start_ = x;
      y_start_ = y;
      theta_start_ = theta;
    } else {
      x_goal_ = x;
      y_goal_ = y;
      theta_goal_ = theta;
    }

    // check if starting positions are inside the map
    if (!isValid(x, y)) {
      std::cout << "###################### Visibility-based solver output "
                   "######################"
                << std::endl;
      std::cout << "At least one of the starting positions is outside the map"
                << std::endl;
      return;
    }

    if (sharedVisibilityField_->get(x, y) < 1) {
      std::cout << "###################### Visibility-based solver output "
                   "######################"
                << std::endl;
      std::cout << "At least one of the starting positions is invalid/occupied"
                << std::endl;
      return;
    }

    g = 0;
    gScore_(x, y) = g;
    isUpdated_(x, y) = true;
    cameFrom_(x, y) = nb_of_sources_;
    origin_(x, y) = nb_of_sources_;
    lightSources_[nb_of_sources_] = {x, y, theta};
    openSet_->push(Node{x, y, g});

    const auto key = hashFunction(x, y, nb_of_sources_);
    visibilityHashMap_[key] = lightStrength_;
    ++nb_of_sources_;
    ++nb_of_iterations_;
  }
  // compute visibility of x_goal, y_goal from x_start, y_start
  auto key = hashFunction(x_goal_, y_goal_, 0);
  updatePointVisibility(0, x_start_, y_start_, x_goal_, y_goal_);
  if (visibilityHashMap_[key] >= visibilityThreshold_) {
    // if (false) {
    auto stopTime = std::chrono::high_resolution_clock::now();
    auto executionDuration = durationInMicroseconds(startTime, stopTime);
    std::cout << "\n------- Target visibile to start -------" << std::endl;

    std::cout << "Marching execution time in us: " << executionDuration << "us"
              << std::endl;

    // combine paths
    std::vector<double> x, y, theta;
    x.push_back(x_start_);
    y.push_back(y_start_);
    theta.push_back(theta_start_);
    x.push_back(x_goal_);
    y.push_back(y_goal_);
    theta.push_back(theta_goal_);
    for (size_t i = 0; i < x.size(); ++i) {
      x[i] += 1;
      y[i] += 1;
    }

    for (size_t i = 0; i < x.size(); ++i) {
      pose p = {static_cast<int>(x[i]), static_cast<int>(y[i]),
                wrapToPi(theta[i])};
      waypoints_.push_back(p);
    }

    double total_d = 0;
    primitives primitives_;
    primitives_.clear();
    primitives_.waypoints.push_back(
        {static_cast<int>(x[0]), static_cast<int>(y[0]), theta[0]});
    total_d = 0;
    State from, to;
    for (size_t i = 0; i < x.size() - 1; ++i) {
      from = {x[i], y[i], theta[i]};
      to = {x[i + 1], y[i + 1], theta[i + 1]};
      std::cout << "From: " << from.x << ", " << from.y << ", "
                << from.theta * 180 / pi << std::endl;
      std::cout << "To: " << to.x << ", " << to.y << ", " << to.theta * 180 / pi
                << std::endl;
      double d1;
      int condition, Q;
      solver_.acceleratedRSPlanner(from, to, d1, r_, condition, Q);

      char motionTypes[5] = {'N', 'N', 'N', 'N', 'N'};
      solver_.getMotionTypes(condition, motionTypes);
      solver_.backProjectMotion(condition, Q, motionTypes);
      std::vector<State> path;
      path.push_back(from);
      solver_.getPath(condition, motionTypes, path, r_, primitives_);
      total_d += d1;
    }

    std::cout << "\n---------------" << std::endl;
    std::cout << "Total distance: " << total_d << std::endl;
    std::cout << "---------------\n" << std::endl;
    savePrimitivesToJson(primitives_, r_, "primitives.json");

    std::cout << "\n------- Saving -------" << std::endl;
    saveField(gScore_, "distance");
    saveField(*sharedVisibilityField_, "occupancy");
    // saveField(distance_map_, "distance");
    saveField(cameFrom_, "cameFrom");
    saveField(origin_, "origin");
    saveField(thetas_, "thetas_");
    saveLightSources();
    std::cout << "Global visibility count: " << global_visibility_count_
              << std::endl;
    return;
  }

  // For queing unique sources from neighbours of neighbour
  std::vector<size_t> potentialSources;
  potentialSources.reserve(10);
  std::vector<triplet> potentialDistances;
  potentialDistances.reserve(10);

  double distance = 0;

  while (openSet_->size() > 0 && key_) {
    auto &current = openSet_->top();
    x = current.x;
    y = current.y;
    openSet_->pop();

    // Expand frontline at current & update neighbours
    for (size_t j = 0; j < 16; j += 2) {
      // NOTE those are always positive
      neighbour_x = x + neighbours_[j];
      neighbour_y = y + neighbours_[j + 1];

      // Box check
      if (!isValid(neighbour_x, neighbour_y)) {
        continue;
      };
      if (isUpdated_(neighbour_x, neighbour_y)) {
        const int origin = origin_(x, y);
        if (origin_(neighbour_x, neighbour_y) != origin &&
            origin_(neighbour_x, neighbour_y) != -1 &&
            cameFrom_(neighbour_x, neighbour_y) != cameFrom_(x, y)) {
          if (origin_(neighbour_x, neighbour_y) > origin) {
            connection_.pivot_1.x = x;
            connection_.pivot_1.y = y;
            connection_.pivot_2.x = neighbour_x;
            connection_.pivot_2.y = neighbour_y;
          } else {
            connection_.pivot_1.x = neighbour_x;
            connection_.pivot_1.y = neighbour_y;
            connection_.pivot_2.x = x;
            connection_.pivot_2.y = y;
          }
          // loop to find the source for each pivot and they must be different
          int source_1 =
              cameFrom_(connection_.pivot_1.x, connection_.pivot_1.y);
          while (true) {
            if (source_1 == cameFrom_(lightSources_[source_1].x,
                                      lightSources_[source_1].y)) {
              break;
            }
            source_1 =
                cameFrom_(lightSources_[source_1].x, lightSources_[source_1].y);
          }
          int source_2 =
              cameFrom_(connection_.pivot_2.x, connection_.pivot_2.y);
          while (true) {
            if (source_2 == cameFrom_(lightSources_[source_2].x,
                                      lightSources_[source_2].y)) {
              break;
            }
            source_2 =
                cameFrom_(lightSources_[source_2].x, lightSources_[source_2].y);
          }
          if (source_1 != source_2) {
            key_ = false;
            break;
          }
        }
        continue;
      };
      if (sharedVisibilityField_->get(neighbour_x, neighbour_y) < 1) {
        if (sharedConfig_->expandInObstacles) {
        }
        cameFrom_(neighbour_x, neighbour_y) = cameFrom_(x, y);
        origin_(neighbour_x, neighbour_y) = origin_(x, y);
        isUpdated_(neighbour_x, neighbour_y) = true;
        continue;
      }

      queuePotentialSources(potentialSources, neighbour_x, neighbour_y);
      getPotentialDistances(potentialSources, potentialDistances, neighbour_x,
                            neighbour_y);
      auto minimum_element =
          std::min_element(potentialDistances.begin(), potentialDistances.end(),
                           [](const auto &lhs, const auto &rhs) {
                             return lhs.distance < rhs.distance;
                           });
      distance = minimum_element->distance;
      const bool cond = (distance == std::numeric_limits<double>::max());
      if (cond) {
        createNewPivot(x, y, neighbour_x, neighbour_y);
        isUpdated_(neighbour_x, neighbour_y) = true;
        ++nb_of_iterations_;
      } else {
        // use source giving least distance
        origin_(neighbour_x, neighbour_y) = origin_(x, y);
        g = minimum_element->distance;
        gScore_(neighbour_x, neighbour_y) = g;
        cameFrom_(neighbour_x, neighbour_y) = minimum_element->source;
        thetas_(neighbour_x, neighbour_y) = wrapTo2Pi(minimum_element->theta);
        openSet_->push(Node{neighbour_x, neighbour_y, g});
        isUpdated_(neighbour_x, neighbour_y) = true;
        ++nb_of_iterations_;
      }
    }
  };

  auto stopTime = std::chrono::high_resolution_clock::now();
  auto executionDuration = durationInMicroseconds(startTime, stopTime);

  if (!sharedConfig_->silent) {
    std::cout << "###################### Visibility-based solver output "
                 "######################"
              << std::endl;
    if (sharedConfig_->timer) {
      std::cout << "Marching execution time in us: " << executionDuration
                << "us" << std::endl;
      std::cout << "Load factor: " << visibilityHashMap_.load_factor()
                << std::endl;
      std::cout << "Iterations: " << nb_of_iterations_ << std::endl;
      std::cout << "Nb of sources: " << nb_of_sources_ << std::endl;
    }
  }

  if (!key_) {
    processConnection();
  } else {
    std::cout << "No connection established" << std::endl;
  }

  std::cout << "\n------- Saving -------" << std::endl;
  saveField(gScore_, "distance");
  saveField(*sharedVisibilityField_, "occupancy");
  // saveField(distance_map_, "distance");
  saveField(cameFrom_, "cameFrom");
  saveField(origin_, "origin");
  saveField(thetas_, "thetas_");
  saveLightSources();
  std::cout << "Global visibility count: " << global_visibility_count_
            << std::endl;
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void Marching::processConnection() {
  bool skip = false;
  std::vector<double> x_1, x_2;
  std::vector<double> y_1, y_2;

  // check if parents of connections are visible to each other
  int source_1 = cameFrom_(connection_.pivot_1.x, connection_.pivot_1.y);
  int LS_x_1 = lightSources_[source_1].x;
  int LS_y_1 = lightSources_[source_1].y;
  int source_2 = cameFrom_(connection_.pivot_2.x, connection_.pivot_2.y);
  int LS_x_2 = lightSources_[source_2].x;
  int LS_y_2 = lightSources_[source_2].y;
  updatePointVisibility(source_1, LS_x_1, LS_y_1, LS_x_2, LS_y_2);
  auto key = hashFunction(LS_x_2, LS_y_2, source_1);
  if (visibilityHashMap_[key] < visibilityThreshold_) {
    x_1.push_back(connection_.pivot_1.x);
    y_1.push_back(connection_.pivot_1.y);
  } else {
    skip = true;
  }

  // Reconstruct first path
  int source = cameFrom_(connection_.pivot_1.x, connection_.pivot_1.y);
  double initial_theta = 0;
  if (source == cameFrom_(lightSources_[source].x, lightSources_[source].y)) {
    initial_theta = lightSources_[source].theta;
    x_1.push_back(lightSources_[source].x);
    y_1.push_back(lightSources_[source].y);
  } else {
    while (true) {
      if (source ==
          cameFrom_(lightSources_[source].x, lightSources_[source].y)) {
        x_1.push_back(lightSources_[source].x);
        y_1.push_back(lightSources_[source].y);
        initial_theta = lightSources_[source].theta;
        break;
      } else {
        x_1.push_back(lightSources_[source].x);
        y_1.push_back(lightSources_[source].y);
      }
      source = cameFrom_(lightSources_[source].x, lightSources_[source].y);
    }
  }
  std::reverse(x_1.begin(), x_1.end());
  std::reverse(y_1.begin(), y_1.end());

  // Reconstruct second path
  source = cameFrom_(connection_.pivot_2.x, connection_.pivot_2.y);
  double final_theta = 0;
  if (source == cameFrom_(lightSources_[source].x, lightSources_[source].y)) {
    final_theta = lightSources_[source].theta;
    x_2.push_back(lightSources_[source].x);
    y_2.push_back(lightSources_[source].y);
  } else {
    while (true) {
      if (source ==
          cameFrom_(lightSources_[source].x, lightSources_[source].y)) {
        x_2.push_back(lightSources_[source].x);
        y_2.push_back(lightSources_[source].y);
        final_theta = lightSources_[source].theta;
        break;
      } else {
        x_2.push_back(lightSources_[source].x);
        y_2.push_back(lightSources_[source].y);
      }
      source = cameFrom_(lightSources_[source].x, lightSources_[source].y);
    }
  }

  // combine paths
  std::vector<double> x, y;
  x.insert(x.end(), x_1.begin(), x_1.end());
  y.insert(y.end(), y_1.begin(), y_1.end());
  x.insert(x.end(), x_2.begin(), x_2.end());
  y.insert(y.end(), y_2.begin(), y_2.end());
  for (size_t i = 0; i < x.size(); ++i) {
    x[i] += 1;
    y[i] += 1;
  }
  size_t s = x.size();

  std::vector<double> theta;
  theta.reserve(s);
  theta.push_back(initial_theta + pi / 2);

  // location of connection pivot
  const size_t s_1 = x_1.size() - 1;
  double x1, y1, theta1, x2, y2, theta2, x3, y3;
  for (size_t i = 0; i < s - 2; ++i) {
    if (i == s_1 - 2 && !skip) {
      int source = cameFrom_(connection_.pivot_1.x, connection_.pivot_1.y);
      theta.push_back(lightSources_[source].theta + pi / 2);
    } else {
      x1 = x[i];
      y1 = y[i];
      theta1 = theta[i];

      x2 = x[i + 1];
      y2 = y[i + 1];
      x3 = x[i + 2];
      y3 = y[i + 2];

      double omega_1;
      urs_.getOmega(x1, y1, x2, y2, r_, theta1, omega_1, true);
      omega_1 = omega_1 + theta1 - pi / 2;
      const double x3_t = -cos(omega_1 - pi / 2) * (x2 - x3) -
                          sin(omega_1 - pi / 2) * (y2 - y3);

      // TODO: can SIMD?
      x2 = x1 - cos(theta1 - pi / 2) * (x1 - x2) -
           sin(theta1 - pi / 2) * (y1 - y2);
      x3 = x1 - cos(theta1 - pi / 2) * (x1 - x3) -
           sin(theta1 - pi / 2) * (y1 - y3);
      y2 = y1 - cos(theta1 - pi / 2) * (y1 - y2) +
           sin(theta1 - pi / 2) * (x1 - x[i + 1]);
      y3 = y1 - cos(theta1 - pi / 2) * (y1 - y3) +
           sin(theta1 - pi / 2) * (x1 - x[i + 2]);

      double atan_3_2 = atan2(y3 - y2, x3 - x2);
      double atan_2_1 = atan2(y2 - y1, x2 - x1);
      theta2 = 0.5 * wrapTo2Pi(atan_3_2 + atan_2_1);

      const State s1 = {x1, y1, theta1};
      State s2 = {x2, y2, theta2};
      State s3 = {x3, y3, 0};
      const Point p2_0 = {x[i + 1], y[i + 1]};
      const Point p3_0 = {x[i + 2], y[i + 2]};

      if (x2 >= x1) {
        if (y2 <= y1) {
          if (x3_t >= 0) {
            RRNegative(s1, s2, s3, p2_0, p3_0);
          } else {
            RLNegative(s1, s2, s3, p2_0, p3_0);
          }
        } else {
          if (x3_t >= 0) {
            RRPositive(s1, s2, s3, p2_0, p3_0);
          } else {
            RLPositive(s1, s2, s3, p2_0, p3_0);
          }
        }
      } else {
        if (y2 <= y1) {
          if (x3_t >= 0) {
            LRNegative(s1, s2, s3, p2_0, p3_0);
          } else {
            LLNegative(s1, s2, s3, p2_0, p3_0);
          }
        } else {
          if (x3_t >= 0) {
            LRPositive(s1, s2, s3, p2_0, p3_0);
          } else {
            LLPositive(s1, s2, s3, p2_0, p3_0);
          }
        }
      }
      theta.push_back(s2.theta);
      if (i == s - 3) {
        theta.push_back(final_theta + pi / 2);
      }
    }

    if (x.size() == 2) {
      double t;
      urs_.getOmega(x[0], y[0], x[1], y[1], r_, theta[0], t, true);
      theta[1] = theta[0] + t - pi / 2;
    }
  }

  // Compute first path
  for (size_t i = 0; i < x.size(); ++i) {
    pose p = {static_cast<int>(x[i]), static_cast<int>(y[i]),
              wrapToPi(theta[i])};
    waypoints_.push_back(p);
  }

  double total_d_1 = 0;
  primitives primitives_1;
  primitives_1.clear();
  primitives_1.turning_radius = r_;
  primitives_1.waypoints.push_back(
      {static_cast<int>(x[0]), static_cast<int>(y[0]), theta[0]});
  State from, to;
  for (size_t i = 0; i < x.size() - 1; ++i) {
    from = {x[i], y[i], theta[i]};
    to = {x[i + 1], y[i + 1], theta[i + 1]};
    double d1;
    int condition, Q;
    solver_.acceleratedRSPlanner(from, to, d1, r_, condition, Q);

    char motionTypes[5] = {'N', 'N', 'N', 'N', 'N'};
    solver_.getMotionTypes(condition, motionTypes);
    solver_.backProjectMotion(condition, Q, motionTypes);
    std::vector<State> path;
    path.push_back(from);
    solver_.getPath(condition, motionTypes, path, r_, primitives_1);
    total_d_1 += d1;
  }

  // compute second path
  // get the difference between thetas_ at the two pivots
  x1 = connection_.pivot_1.x;
  y1 = connection_.pivot_1.y;
  x2 = connection_.pivot_2.x;
  y2 = connection_.pivot_2.y;
  const double diff =
      fabs(wrapToPi(thetas_(x1, y1)) - wrapToPi(thetas_(x2, y2)));
  // Reversing the path ends up avoiding some collisions
  if (diff > pi / 2) {
    theta[0] += pi;
    theta[s - 1] += pi;
  }
  waypoints_.clear();
  for (size_t i = 0; i < x.size(); ++i) {
    pose p = {static_cast<int>(x[i]), static_cast<int>(y[i]),
              wrapToPi(theta[i])};
    waypoints_.push_back(p);
  }
  double total_d_2 = 0;
  primitives primitives_2;
  primitives_2.clear();
  primitives_2.turning_radius = r_;
  primitives_2.waypoints.push_back(
      {static_cast<int>(x[0]), static_cast<int>(y[0]), theta[0]});
  for (size_t i = 0; i < x.size() - 1; ++i) {
    from = {x[i], y[i], theta[i]};
    to = {x[i + 1], y[i + 1], theta[i + 1]};
    double d1;
    int condition, Q;
    solver_.acceleratedRSPlanner(from, to, d1, r_, condition, Q);

    char motionTypes[5] = {'N', 'N', 'N', 'N', 'N'};
    solver_.getMotionTypes(condition, motionTypes);
    solver_.backProjectMotion(condition, Q, motionTypes);
    std::vector<State> path;
    path.push_back(from);
    solver_.getPath(condition, motionTypes, path, r_, primitives_2);
    total_d_2 += d1;
  }

  // Reversing back the primitives
  if (diff > pi / 2) {
    for (size_t i = 0; i < primitives_2.motionDirections.size(); ++i) {
      primitives_2.motionDirections[i] = -primitives_2.motionDirections[i];
      primitives_2.waypoints[i].theta =
          wrapToPi(primitives_2.waypoints[i].theta + pi);
      if (primitives_2.motionTypes[i] == 'R') {
        primitives_2.motionTypes[i] = 'L';
      } else if (primitives_2.motionTypes[i] == 'L') {
        primitives_2.motionTypes[i] = 'R';
      }
    }
    int i = primitives_2.motionDirections.size();
    primitives_2.waypoints[i].theta =
        wrapToPi(primitives_2.waypoints[i].theta + pi);
  }

  auto startTime = std::chrono::high_resolution_clock::now();
  int p1 = generatePathForwardSimOptimized(primitives_1, sharedVisibilityField_,
                                           visibilityThreshold_,
                                           "output/path_1.txt");
  int p2 = generatePathForwardSimOptimized(primitives_2, sharedVisibilityField_,
                                           visibilityThreshold_,
                                           "output/path_2.txt");
  auto stopTime = std::chrono::high_resolution_clock::now();
  auto executionDuration = durationInMicroseconds(startTime, stopTime);
  std::cout << "Forward simulation time in us: " << executionDuration << "us"
            << std::endl;

  std::cout << "p1: " << p1 << std::endl;
  std::cout << "p2: " << p2 << std::endl;
  std::cout << total_d_1 << ", " << total_d_2 << std::endl;

  double total_d = 0;
  primitives primitives_;
  if (p1 < p2) {
    total_d = total_d_1;
    primitives_ = primitives_1;
  } else {
    total_d = total_d_2;
    primitives_ = primitives_2;
  }
  std::cout << "\n---------------" << std::endl;
  std::cout << "Total distance: " << total_d << std::endl;
  std::cout << "---------------\n" << std::endl;
  savePrimitivesToJson(primitives_, r_, "primitives.json");
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void Marching::printProgress(const int iteration, const int size) const {
  std::cout << "\r";
  std::cout << "[";
  for (int j = 0; j < 50; ++j) {
    if (j < 50 * (iteration + 1) / size) {
      std::cout << "=";
    } else {
      std::cout << " ";
    }
  }
  std::cout << "] " << 100 * (iteration + 1) / size << "% | " << (iteration + 1)
            << "/" << size;
  std::cout << std::flush;
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
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

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void Marching::getPotentialDistances(
    const std::vector<size_t> &potentialSources,
    std::vector<triplet> &potentialDistances, const int neighbour_x,
    const int neighbour_y) {

  int LS_x = 0, LS_y = 0, potentialSource = 0;
  double LS_theta = 0;
  double distance = 0;
  double theta = 0;
  potentialDistances.clear();
  for (size_t k = 0; k < potentialSources.size(); ++k) {
    potentialSource = potentialSources[k];
    LS_x = lightSources_[potentialSource].x;
    LS_y = lightSources_[potentialSource].y;
    LS_theta = lightSources_[potentialSource].theta;
    updatePointVisibility(potentialSource, LS_x, LS_y, neighbour_x,
                          neighbour_y);
    distance = std::numeric_limits<double>::max();
    const auto key = hashFunction(neighbour_x, neighbour_y, potentialSource);
    if (visibilityHashMap_.at(key) >= visibilityThreshold_) {
      distance = evaluateDistance(LS_x, LS_y, LS_theta, potentialSource,
                                  neighbour_x, neighbour_y, theta);
    }
    potentialDistances.push_back({distance, theta, potentialSource});
  }
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void Marching::createNewPivot(const int x, const int y, const int neighbour_x,
                              const int neighbour_y) {
  double theta = 0;

  theta = thetas_(x, y) - pi / 2;
  theta = wrapTo2Pi(theta);
  lightSources_[nb_of_sources_] = {x, y, theta};
  const auto key = hashFunction(x, y, nb_of_sources_);
  visibilityHashMap_[key] = lightStrength_;
  // Update neighbours of light source
  int pivot_neighbour_x, pivot_neighbour_y;
  for (size_t p = 0; p < 16; p += 2) {
    // NOTE those are always positive
    pivot_neighbour_x = x + neighbours_[p];
    pivot_neighbour_y = y + neighbours_[p + 1];
    // Box check
    if (!isValid(pivot_neighbour_x, pivot_neighbour_y)) {
      continue;
    }
    // Update neighbour visibility
    updatePointVisibility(nb_of_sources_, x, y, pivot_neighbour_x,
                          pivot_neighbour_y);
  }
  cameFrom_(neighbour_x, neighbour_y) = nb_of_sources_;
  const double d_accelerated = evaluateDistance(
      x, y, theta, nb_of_sources_, neighbour_x, neighbour_y, theta);
  // evaluateDistance returns an absolute root-to-target distance for pivot
  // chains, so adding the pivot cell prefix again creates a distance jump.
  double g = d_accelerated;
  gScore_(neighbour_x, neighbour_y) = g;
  origin_(neighbour_x, neighbour_y) = origin_(x, y);
  ++nb_of_sources_;
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
// void Marching::updatePointVisibility(const size_t lightSourceNumber,
//                                      const int LS_x, const int LS_y,
//                                      const int x, const int y, const int rx,
//                                      const int ry) {
//   global_visibility_count_++;
//   if (!isValid(x, y)) {
//     visibilityHashMap_[hashFunction(x, y, lightSourceNumber)] = 0;
//     return;
//   }
//   // Variable initialization
//   double v = 0;
//   double c = 0;
//
//   // Check if visibility value already exists
//   auto key = hashFunction(x, y, lightSourceNumber);
//   if (visibilityHashMap_.count(key)) {
//     ++return_count_;
//     return;
//   }
//   if (x == LS_x && y == LS_y) {
//     visibilityHashMap_[key] = lightStrength_;
//     return;
//   }
//   if (sharedVisibilityField_->get(x, y) < visibilityThreshold_) {
//     visibilityHashMap_[key] = 0;
//     return;
//   }
//
//   const double LS_theta = lightSources_[lightSourceNumber - 0].theta;
//   const double xn0 = (x - LS_x) * cos(LS_theta) + (y - LS_y) * sin(LS_theta);
//   const double yn0 = -(x - LS_x) * sin(LS_theta) + (y - LS_y) *
//   cos(LS_theta); const int mp_x = omega_map_.nx() / 2; const int mp_y =
//   omega_map_.ny() / 2; const double xn = xn0 + mp_x; const double yn = yn0 +
//   mp_y;
//   // const int xn0_ = xn0;
//   const int yn0_ = yn0;
//
//   double a;
//   if (LS_theta != 0) {
//     if (xn + 1 >= omega_map_.nx() || yn + 1 >= omega_map_.ny() || xn < 0 ||
//         yn < 0) {
//       urs_.getOmega(xn0, yn0, xn, yn, r_, pi / 2, a);
//     } else {
//       a = spherical_interpolation(xn, yn, LS_theta);
//     }
//   } else {
//     a = angle_map_(xn, yn);
//   }
//
//   if ((yn0_ >= 0 && yn0_ <= 1 && a > 0) || (yn0_ <= 0 && yn0_ >= -1 && a <
//   0)) {
//     v = 1;
//     for (int i = 0; i < 16; i += 2) {
//       const int xn3 = x + neighbours_[i];
//       const int yn3 = y + neighbours_[i + 1];
//       const auto key = hashFunction(xn3, yn3, lightSourceNumber);
//       if (visibilityHashMap_.count(key)) {
//         if (visibilityHashMap_.at(key) < visibilityThreshold_) {
//           v = 0;
//           break;
//         }
//       }
//     }
//     key = hashFunction(x, y, lightSourceNumber);
//     visibilityHashMap_[key] = v;
//   }
//
//   if (a == 0) {
//     key = hashFunction(x - 1, y, lightSourceNumber);
//     if (!visibilityHashMap_.count(key)) {
//       if (rx != x - 1) {
//         updatePointVisibility(lightSourceNumber, LS_x, LS_y, x - 1, y, x, y);
//       } else {
//         visibilityHashMap_[key] = sharedVisibilityField_->get(x, y);
//       }
//     }
//     v = visibilityHashMap_.at(key);
//   } else if (a > 0) {
//     if (a < pi / 4) {
//       key = hashFunction(x - 1, y, lightSourceNumber);
//       if (!visibilityHashMap_.count(key)) {
//         if (rx != x - 1) {
//           updatePointVisibility(lightSourceNumber, LS_x, LS_y, x - 1, y, x,
//           y);
//         } else {
//           visibilityHashMap_[key] = sharedVisibilityField_->get(x, y);
//         }
//       }
//       auto key_1 = hashFunction(x - 1, y - 1, lightSourceNumber);
//       if (!visibilityHashMap_.count(key_1)) {
//         if (rx != x - 1 && ry != y - 1) {
//           updatePointVisibility(lightSourceNumber, LS_x, LS_y, x - 1, y - 1,
//           x,
//                                 y);
//         } else {
//           visibilityHashMap_[key_1] = sharedVisibilityField_->get(x, y);
//         }
//       }
//       c = tan(a);
//       double v1 = visibilityHashMap_.at(key);
//       double v2 = visibilityHashMap_.at(key_1);
//       v = v1 - c * (v1 - v2);
//     } else if (a == pi / 4) {
//       key = hashFunction(x - 1, y - 1, lightSourceNumber);
//       if (!visibilityHashMap_.count(key)) {
//         if (rx != x - 1 && ry != y - 1) {
//           updatePointVisibility(lightSourceNumber, LS_x, LS_y, x - 1, y - 1,
//           x,
//                                 y);
//         } else {
//           visibilityHashMap_[key] = sharedVisibilityField_->get(x, y);
//         }
//       }
//       v = visibilityHashMap_.at(key);
//     } else if (a > pi / 4 && a < pi / 2) {
//       key = hashFunction(x, y - 1, lightSourceNumber);
//       if (!visibilityHashMap_.count(key)) {
//         if (ry != y - 1) {
//           updatePointVisibility(lightSourceNumber, LS_x, LS_y, x, y - 1, x,
//           y);
//         } else {
//           visibilityHashMap_[key] = sharedVisibilityField_->get(x, y);
//         }
//       }
//       auto key_1 = hashFunction(x - 1, y - 1, lightSourceNumber);
//       if (!visibilityHashMap_.count(key_1)) {
//         if (rx != x - 1 && ry != y - 1) {
//           updatePointVisibility(lightSourceNumber, LS_x, LS_y, x - 1, y - 1,
//           x,
//                                 y);
//         } else {
//           visibilityHashMap_[key_1] = sharedVisibilityField_->get(x, y);
//         }
//         updatePointVisibility(lightSourceNumber, LS_x, LS_y, x - 1, y - 1);
//       }
//       c = 1 / tan(a);
//       double v1 = visibilityHashMap_.at(key);
//       double v2 = visibilityHashMap_.at(key_1);
//       v = v1 - c * (v1 - v2);
//     } else if (a == pi / 2) {
//       key = hashFunction(x, y - 1, lightSourceNumber);
//       if (!visibilityHashMap_.count(key)) {
//         if (ry != y - 1) {
//           updatePointVisibility(lightSourceNumber, LS_x, LS_y, x, y - 1, x,
//           y);
//         } else {
//           visibilityHashMap_[key] = sharedVisibilityField_->get(x, y);
//         }
//       }
//       v = visibilityHashMap_.at(key);
//     } else if (a > pi / 2 && a < 3 * pi / 4) {
//       key = hashFunction(x, y - 1, lightSourceNumber);
//       if (!visibilityHashMap_.count(key)) {
//         if (ry != y - 1) {
//           updatePointVisibility(lightSourceNumber, LS_x, LS_y, x, y - 1, x,
//           y);
//         } else {
//           visibilityHashMap_[key] = sharedVisibilityField_->get(x, y);
//         }
//       }
//       auto key_1 = hashFunction(x + 1, y - 1, lightSourceNumber);
//       if (!visibilityHashMap_.count(key_1)) {
//         if (rx != x + 1 && ry != y - 1) {
//           updatePointVisibility(lightSourceNumber, LS_x, LS_y, x + 1, y - 1,
//           x,
//                                 y);
//         } else {
//           visibilityHashMap_[key_1] = sharedVisibilityField_->get(x, y);
//         }
//       }
//       c = -1 / tan(a);
//       double v1 = visibilityHashMap_.at(key);
//       double v2 = visibilityHashMap_.at(key_1);
//       v = v1 - c * (v1 - v2);
//     } else if (a == 3 * pi / 4) {
//       key = hashFunction(x + 1, y - 1, lightSourceNumber);
//       if (!visibilityHashMap_.count(key)) {
//         if (rx != x + 1 && ry != y - 1) {
//           updatePointVisibility(lightSourceNumber, LS_x, LS_y, x + 1, y - 1,
//           x,
//                                 y);
//         } else {
//           visibilityHashMap_[key] = sharedVisibilityField_->get(x, y);
//         }
//       }
//       v = visibilityHashMap_.at(key);
//     } else if (a > 3 * pi / 4 && a < pi) {
//       key = hashFunction(x + 1, y, lightSourceNumber);
//       if (!visibilityHashMap_.count(key)) {
//         if (rx != x + 1) {
//           updatePointVisibility(lightSourceNumber, LS_x, LS_y, x + 1, y, x,
//           y);
//         } else {
//           visibilityHashMap_[key] = sharedVisibilityField_->get(x, y);
//         }
//       }
//       auto key_1 = hashFunction(x + 1, y - 1, lightSourceNumber);
//       if (!visibilityHashMap_.count(key_1)) {
//         if (rx != x + 1 && ry != y - 1) {
//           updatePointVisibility(lightSourceNumber, LS_x, LS_y, x + 1, y - 1,
//           x,
//                                 y);
//         } else {
//           visibilityHashMap_[key_1] = sharedVisibilityField_->get(x, y);
//         }
//       }
//       c = -tan(a);
//       double v1 = visibilityHashMap_.at(key);
//       double v2 = visibilityHashMap_.at(key_1);
//       v = v1 - c * (v1 - v2);
//     } else {
//       key = hashFunction(x + 1, y, lightSourceNumber);
//       if (!visibilityHashMap_.count(key)) {
//         if (rx != x + 1) {
//           updatePointVisibility(lightSourceNumber, LS_x, LS_y, x + 1, y, x,
//           y);
//         } else {
//           visibilityHashMap_[key] = sharedVisibilityField_->get(x, y);
//         }
//       }
//       v = visibilityHashMap_.at(key);
//     }
//   } else if (a > -pi / 4) {
//     key = hashFunction(x - 1, y, lightSourceNumber);
//     if (!visibilityHashMap_.count(key)) {
//       if (rx != x - 1) {
//         updatePointVisibility(lightSourceNumber, LS_x, LS_y, x - 1, y, x, y);
//       } else {
//         visibilityHashMap_[key] = sharedVisibilityField_->get(x, y);
//       }
//     }
//     auto key_1 = hashFunction(x - 1, y + 1, lightSourceNumber);
//     if (!visibilityHashMap_.count(key_1)) {
//       if (rx != x - 1 && ry != y + 1) {
//         updatePointVisibility(lightSourceNumber, LS_x, LS_y, x - 1, y + 1, x,
//                               y);
//       } else {
//         visibilityHashMap_[key_1] = sharedVisibilityField_->get(x, y);
//       }
//     }
//     c = -tan(a);
//     double v1 = visibilityHashMap_.at(key);
//     double v2 = visibilityHashMap_.at(key_1);
//     v = v1 - c * (v1 - v2);
//   } else if (a == -pi / 4) {
//     key = hashFunction(x - 1, y + 1, lightSourceNumber);
//     if (!visibilityHashMap_.count(key)) {
//       if (rx != x - 1 && ry != y + 1) {
//         updatePointVisibility(lightSourceNumber, LS_x, LS_y, x - 1, y + 1, x,
//                               y);
//       } else {
//         visibilityHashMap_[key] = sharedVisibilityField_->get(x, y);
//       }
//     }
//     v = visibilityHashMap_.at(key);
//   } else if (a > -pi / 2 && a < -pi / 4) {
//     key = hashFunction(x, y + 1, lightSourceNumber);
//     if (!visibilityHashMap_.count(key)) {
//       if (ry != y + 1) {
//         updatePointVisibility(lightSourceNumber, LS_x, LS_y, x, y + 1, x, y);
//       } else {
//         visibilityHashMap_[key] = sharedVisibilityField_->get(x, y);
//       }
//     }
//     auto key_1 = hashFunction(x - 1, y + 1, lightSourceNumber);
//     if (!visibilityHashMap_.count(key_1)) {
//       if (rx != x - 1 && ry != y + 1) {
//         updatePointVisibility(lightSourceNumber, LS_x, LS_y, x - 1, y + 1, x,
//                               y);
//       } else {
//         visibilityHashMap_[key_1] = sharedVisibilityField_->get(x, y);
//       }
//     }
//     c = -1 / tan(a);
//     double v1 = visibilityHashMap_.at(key);
//     double v2 = visibilityHashMap_.at(key_1);
//     v = v1 - c * (v1 - v2);
//   } else if (a == -pi / 2) {
//     key = hashFunction(x, y + 1, lightSourceNumber);
//     if (!visibilityHashMap_.count(key)) {
//       if (ry != y + 1) {
//         updatePointVisibility(lightSourceNumber, LS_x, LS_y, x, y + 1, x, y);
//       } else {
//         visibilityHashMap_[key] = sharedVisibilityField_->get(x, y);
//       }
//     }
//     v = visibilityHashMap_.at(key);
//   } else if (a < -pi / 2 && a > -3 * pi / 4) {
//     key = hashFunction(x, y + 1, lightSourceNumber);
//     if (!visibilityHashMap_.count(key)) {
//       if (ry != y + 1) {
//         updatePointVisibility(lightSourceNumber, LS_x, LS_y, x, y + 1, x, y);
//       } else {
//         visibilityHashMap_[key] = sharedVisibilityField_->get(x, y);
//       }
//     }
//     auto key_1 = hashFunction(x + 1, y + 1, lightSourceNumber);
//     if (!visibilityHashMap_.count(key_1)) {
//       if (rx != x + 1 && ry != y + 1) {
//         updatePointVisibility(lightSourceNumber, LS_x, LS_y, x + 1, y + 1, x,
//                               y);
//       } else {
//         visibilityHashMap_[key_1] = sharedVisibilityField_->get(x, y);
//       }
//     }
//     c = 1 / tan(a);
//     double v1 = visibilityHashMap_.at(key);
//     double v2 = visibilityHashMap_.at(key_1);
//     v = v1 - c * (v1 - v2);
//   } else if (a < -3 * pi / 4 && a > -pi) {
//     key = hashFunction(x + 1, y, lightSourceNumber);
//     if (!visibilityHashMap_.count(key)) {
//       if (rx != x + 1) {
//         updatePointVisibility(lightSourceNumber, LS_x, LS_y, x + 1, y, x, y);
//       } else {
//         visibilityHashMap_[key] = sharedVisibilityField_->get(x, y);
//       }
//     }
//     auto key_1 = hashFunction(x + 1, y + 1, lightSourceNumber);
//     if (!visibilityHashMap_.count(key_1)) {
//       if (rx != x + 1 && ry != y + 1) {
//         updatePointVisibility(lightSourceNumber, LS_x, LS_y, x + 1, y + 1, x,
//                               y);
//       } else {
//         visibilityHashMap_[key_1] = sharedVisibilityField_->get(x, y);
//       }
//     }
//     c = tan(a);
//     double v1 = visibilityHashMap_.at(key);
//     double v2 = visibilityHashMap_.at(key_1);
//     v = v1 - c * (v1 - v2);
//   } else if (a == -pi) {
//     key = hashFunction(x + 1, y, lightSourceNumber);
//     if (!visibilityHashMap_.count(key)) {
//       if (rx != x + 1) {
//         updatePointVisibility(lightSourceNumber, LS_x, LS_y, x + 1, y, x, y);
//       } else {
//         visibilityHashMap_[key] = sharedVisibilityField_->get(x, y);
//       }
//     }
//     v = visibilityHashMap_.at(key);
//   } else {
//     v = 1;
//   }
//
//   v = v * sharedVisibilityField_->get(x, y);
//   key = hashFunction(x, y, lightSourceNumber);
//   // visibilityHashMap_[key] = v * 0.995;
//   visibilityHashMap_[key] = v;
// }

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void Marching::updatePointVisibility(const size_t lightSourceNumber,
                                     const int LS_x, const int LS_y,
                                     const int x, const int y, const int rx,
                                     const int ry) {
  global_visibility_count_++;

  // Calculate hash key once and reuse it
  const auto key = hashFunction(x, y, lightSourceNumber);

  // Fast path: Check if we already computed this point
  if (visibilityHashMap_.count(key)) {
    ++return_count_;
    return;
  }

  // Early termination cases
  if (!isValid(x, y)) {
    visibilityHashMap_[key] = 0;
    return;
  }

  // Light source case
  if (x == LS_x && y == LS_y) {
    visibilityHashMap_[key] = lightStrength_;
    return;
  }

  // Visibility threshold check
  if (sharedVisibilityField_->get(x, y) < visibilityThreshold_) {
    visibilityHashMap_[key] = 0;
    return;
  }

  // Cache frequently used values
  const double LS_theta = lightSources_[lightSourceNumber - 0].theta;
  const double dx = x - LS_x;
  const double dy = y - LS_y;
  const double cos_theta = cos(LS_theta);
  const double sin_theta = sin(LS_theta);

  // Reduce repeated computations
  const double xn0 = dx * cos_theta + dy * sin_theta;
  const double yn0 = -dx * sin_theta + dy * cos_theta;
  const int mp_x = omega_map_.nx() / 2;
  const int mp_y = omega_map_.ny() / 2;
  const double xn = xn0 + mp_x;
  const double yn = yn0 + mp_y;
  const int yn0_ = yn0;

  // Calculate angle - factored out for clarity
  double a;
  if (LS_theta != 0) {
    if (xn + 1 >= omega_map_.nx() || yn + 1 >= omega_map_.ny() || xn < 0 ||
        yn < 0) {
      urs_.getOmega(xn0, yn0, xn, yn, r_, pi / 2, a);
    } else {
      a = spherical_interpolation(xn, yn, LS_theta);
    }
  } else {
    a = angle_map_(xn, yn);
  }

  // Special case handling
  if ((yn0_ >= 0 && yn0_ <= 1 && a > 0) || (yn0_ <= 0 && yn0_ >= -1 && a < 0)) {
    double v = 1;
    for (int i = 0; i < 16; i += 2) {
      const int xn3 = x + neighbours_[i];
      const int yn3 = y + neighbours_[i + 1];
      const auto neighbor_key = hashFunction(xn3, yn3, lightSourceNumber);
      if (visibilityHashMap_.count(neighbor_key) &&
          visibilityHashMap_.at(neighbor_key) < visibilityThreshold_) {
        v = 0;
        break;
      }
    }
    visibilityHashMap_[key] = v;
    return;
  }

  // Handle angle-based visibility calculation
  double v = calculateAngleBasedVisibility(lightSourceNumber, LS_x, LS_y, x, y,
                                           rx, ry, a);

  // Apply field visibility and store result
  v = v * sharedVisibilityField_->get(x, y);
  visibilityHashMap_[key] = v;
}

// New helper function to reduce code duplication and improve readability
double Marching::calculateAngleBasedVisibility(const size_t lightSourceNumber,
                                               const int LS_x, const int LS_y,
                                               const int x, const int y,
                                               const int rx, const int ry,
                                               const double a) {
  // Process neighbor points and calculate visibility based on angle
  struct NeighborPoint {
    int dx, dy;
    double angle_min, angle_max;
  };

  // Define angle ranges for cleaner code
  const double PI_4 = pi / 4;
  const double PI_2 = pi / 2;
  const double PI_3_4 = 3 * pi / 4;
  const double PI = pi;

  // Determine which neighbors to check based on angle
  int nx1 = 0, ny1 = 0, nx2 = 0, ny2 = 0;
  bool useInterpolation = true;
  double c = 0;

  if (a == 0) {
    nx1 = x - 1;
    ny1 = y;
    useInterpolation = false;
  } else if (a > 0) {
    if (a < PI_4) {
      nx1 = x - 1;
      ny1 = y;
      nx2 = x - 1;
      ny2 = y - 1;
      c = tan(a);
    } else if (a == PI_4) {
      nx1 = x - 1;
      ny1 = y - 1;
      useInterpolation = false;
    } else if (a < PI_2) {
      nx1 = x;
      ny1 = y - 1;
      nx2 = x - 1;
      ny2 = y - 1;
      c = 1 / tan(a);
    } else if (a == PI_2) {
      nx1 = x;
      ny1 = y - 1;
      useInterpolation = false;
    } else if (a < PI_3_4) {
      nx1 = x;
      ny1 = y - 1;
      nx2 = x + 1;
      ny2 = y - 1;
      c = -1 / tan(a);
    } else if (a == PI_3_4) {
      nx1 = x + 1;
      ny1 = y - 1;
      useInterpolation = false;
    } else if (a < PI) {
      nx1 = x + 1;
      ny1 = y;
      nx2 = x + 1;
      ny2 = y - 1;
      c = -tan(a);
    } else {
      nx1 = x + 1;
      ny1 = y;
      useInterpolation = false;
    }
  } else { // a < 0
    if (a > -PI_4) {
      nx1 = x - 1;
      ny1 = y;
      nx2 = x - 1;
      ny2 = y + 1;
      c = -tan(a);
    } else if (a == -PI_4) {
      nx1 = x - 1;
      ny1 = y + 1;
      useInterpolation = false;
    } else if (a > -PI_2) {
      nx1 = x;
      ny1 = y + 1;
      nx2 = x - 1;
      ny2 = y + 1;
      c = -1 / tan(a);
    } else if (a == -PI_2) {
      nx1 = x;
      ny1 = y + 1;
      useInterpolation = false;
    } else if (a > -PI_3_4) {
      nx1 = x;
      ny1 = y + 1;
      nx2 = x + 1;
      ny2 = y + 1;
      c = 1 / tan(a);
    } else if (a == -PI_3_4) {
      nx1 = x + 1;
      ny1 = y + 1;
      useInterpolation = false;
    } else if (a > -PI) {
      nx1 = x + 1;
      ny1 = y;
      nx2 = x + 1;
      ny2 = y + 1;
      c = tan(a);
    } else if (a == -PI) {
      nx1 = x + 1;
      ny1 = y;
      useInterpolation = false;
    } else {
      return 1; // Special case
    }
  }

  // Get or calculate the first neighbor's visibility
  const auto key1 = hashFunction(nx1, ny1, lightSourceNumber);
  if (!visibilityHashMap_.count(key1)) {
    if ((rx != nx1 || ny1 != ry) && (abs(nx1 - x) <= 1 && abs(ny1 - y) <= 1)) {
      updatePointVisibility(lightSourceNumber, LS_x, LS_y, nx1, ny1, x, y);
    } else {
      visibilityHashMap_[key1] = sharedVisibilityField_->get(x, y);
    }
  }

  // Non-interpolation case
  if (!useInterpolation) {
    return visibilityHashMap_.at(key1);
  }

  // Get or calculate the second neighbor's visibility
  const auto key2 = hashFunction(nx2, ny2, lightSourceNumber);
  if (!visibilityHashMap_.count(key2)) {
    if ((rx != nx2 || ny2 != ry) && (abs(nx2 - x) <= 1 && abs(ny2 - y) <= 1)) {
      updatePointVisibility(lightSourceNumber, LS_x, LS_y, nx2, ny2, x, y);
    } else {
      visibilityHashMap_[key2] = sharedVisibilityField_->get(x, y);
    }
  }

  // Interpolate between the two neighbor values
  const double v1 = visibilityHashMap_.at(key1);
  const double v2 = visibilityHashMap_.at(key2);
  return v1 - c * (v1 - v2);
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void Marching::saveImage(const Field<double> &map,
                         const std::string &name) const {
  const int width = nx_;
  const int height = ny_;

  sf::Image image;
  image.create(width, height);

  double minVal = std::numeric_limits<double>::max();
  double maxVal = std::numeric_limits<double>::min();

  // Find min and max values in distances for normalization
  for (int i = 0; i < width; ++i) {
    for (int j = 0; j < height; ++j) {
      double val = map(i, j);
      if (val == std::numeric_limits<double>::max())
        continue;
      if (val < minVal)
        minVal = val;
      if (val > maxVal && val <= 2000)
        maxVal = val;
    }
  }

  // Generate the image
  for (int i = 0; i < width; ++i) {
    for (int j = 0; j < height; ++j) {
      if (sharedVisibilityField_->get(i, j) < 1) {
        image.setPixel(i, j, sf::Color::Black);
      } else {
        double normalized_value = (map(i, j) - minVal) / (maxVal - minVal);
        sf::Color color = getColor(normalized_value);
        image.setPixel(i, j, color);
      }
    }
  }

  // color all initial frontline points as green circles with radius 10
  sf::Color color;
  color.a = 1;
  int x0, y0;
  int radius = 10;
  for (size_t k = 0; k < sharedConfig_->initialFrontline.size(); ++k) {
    x0 = sharedConfig_->initialFrontline[k].x;
    y0 = sharedConfig_->initialFrontline[k].y;
    for (int i = -radius; i <= radius; ++i) {
      for (int j = -radius; j <= radius; ++j) {
        if (i * i + j * j <= radius * radius) {
          if (isValid(x0 + i, y0 + j)) {
            image.setPixel(x0 + i, y0 + j, color.Green);
          }
        }
      }
    }
  }

  // compute the step size based on the max and min values
  const int number_of_contour_lines = sharedConfig_->number_of_contour_lines;
  const double stepSize = (maxVal - minVal) / number_of_contour_lines;

  std::vector<double> contourLevels;
  for (double level = minVal; level <= maxVal; level += stepSize) {
    contourLevels.push_back(level);
  }
  const double lineThickness = (maxVal - minVal) / 350;
  if (name == "distance_") {
    // add contour lines with anti-aliasing
    for (double level : contourLevels) {
      for (int i = 0; i < width; ++i) {
        for (int j = 0; j < height; ++j) {
          if (map(i, j) >= level - lineThickness &&
              map(i, j) <= level + lineThickness) {
            image.setPixel(i, j, sf::Color::Black);
          }
        }
      }
    }

    // add obstacles as red
    for (int i = 0; i < width; ++i) {
      for (int j = 0; j < height; ++j) {
        if (sharedVisibilityField_->get(i, j) < 1) {
          image.setPixel(i, j, sf::Color::Red);
        }
      }
    }
  }

  // add waypoints
  for (size_t k = 0; k < waypoints_.size(); ++k) {
    x0 = waypoints_[k].x;
    y0 = waypoints_[k].y;
    for (int i = -radius; i <= radius; ++i) {
      for (int j = -radius; j <= radius; ++j) {
        if (i * i + j * j <= radius * radius) {
          if (isValid(x0 + i, y0 + j)) {
            image.setPixel(x0 + i, y0 + j, sf::Color::Cyan);
          }
        }
      }
    }
  }

  // draw at each waypoint a small line with heading theta of the waypoint
  for (size_t i = 0; i < waypoints_.size(); ++i) {
    int x = waypoints_[i].x;
    int y = waypoints_[i].y;
    double theta = waypoints_[i].theta;
    int x2 = x + 50 * cos(theta);
    int y2 = y + 50 * sin(theta);
    if (x == x2 && y == y2) {
      continue;
    }
    int dx = x2 - x;
    int dy = y2 - y;
    int steps = abs(dx) > abs(dy) ? abs(dx) : abs(dy);
    double Xinc = dx / (double)steps;
    double Yinc = dy / (double)steps;
    double X = x;
    double Y = y;
    for (int i = 0; i <= steps; i++) {
      for (int k = 0; k < 1; ++k) {
        for (int l = 0; l < 1; ++l) {
          image.setPixel(X * 1 + k, Y * 1 + l, sf::Color::Magenta);
        }
      }
      X += Xinc;
      Y += Yinc;
    }
  }

  // set the first waypoint as green
  if (!waypoints_.empty()) {
    x0 = waypoints_[0].x;
    y0 = waypoints_[0].y;
    for (int i = -radius; i <= radius; ++i) {
      for (int j = -radius; j <= radius; ++j) {
        if (i * i + j * j <= radius * radius) {
          if (isValid(x0 + i, y0 + j)) {
            image.setPixel(x0 + i, (y0 + j), sf::Color::Green);
          }
        }
      }
    }
  }

  std::string outputPath = "./output/" + name + "Image.png";
  image.saveToFile(outputPath);

  // interpolate the image to a larger size including the contour lines
  sf::Image interpolatedImage;
  int interpolationFactor = 3;
  interpolatedImage.create(width * interpolationFactor,
                           height * interpolationFactor);
  for (int i = 0; i < width; ++i) {
    for (int j = 0; j < height; ++j) {
      sf::Color color = image.getPixel(i, j);
      for (int k = 0; k < interpolationFactor; ++k) {
        for (int l = 0; l < interpolationFactor; ++l) {
          interpolatedImage.setPixel(i * interpolationFactor + k,
                                     j * interpolationFactor + l, color);
        }
      }
    }
  }

  std::string interpolatedOutputPath =
      "./output/" + name + "InterpolatedImage.png";
  interpolatedImage.saveToFile(interpolatedOutputPath);
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void Marching::computeEDF() {
  reset();
  edf_.reset(nx_, ny_, std::numeric_limits<double>::max());

  auto startTime = std::chrono::high_resolution_clock::now();
  std::vector<int> initial_frontline;
  for (size_t i = 0; i < nx_; ++i) {
    for (size_t j = 0; j < ny_; ++j) {
      if (sharedVisibilityField_->get(i, j) <= visibilityThreshold_) {
        initial_frontline.push_back(i);
        initial_frontline.push_back(j);
      }
    }
  }
  // Init
  int x = 0, y = 0, neighbour_x = 0, neighbour_y = 0;
  double g = 0;
  // Fill in data from initial frontline
  for (size_t i = 0; i < initial_frontline.size(); i += 2) {
    x = initial_frontline[i];
    y = initial_frontline[i + 1];
    g = 0;
    openSet_->push(Node{x, y, g});

    edf_(x, y) = g;
    isUpdated_(x, y) = true;
    cameFrom_(x, y) = nb_of_sources_;
    lightSources_[nb_of_sources_] = {x, y, 0};
    const auto key = hashFunction(x, y, nb_of_sources_);
    visibilityHashMap_[key] = lightStrength_;
    ++nb_of_sources_;
  }

  // For queing unique sources from neighbours of neighbour
  std::vector<size_t> potentialSources;
  potentialSources.reserve(10);
  std::vector<triplet> potentialDistances;
  potentialDistances.reserve(10);

  double distance = 0;

  while (openSet_->size() > 0) {
    auto &current = openSet_->top();
    x = current.x;
    y = current.y;

    openSet_->pop();

    // Expand frontline at current & update neighbours
    for (size_t j = 0; j < 16; j += 2) {
      // NOTE those are always positive
      neighbour_x = x + neighbours_[j];
      neighbour_y = y + neighbours_[j + 1];

      // Box check
      if (!isValid(neighbour_x, neighbour_y)) {
        continue;
      };
      if (isUpdated_(neighbour_x, neighbour_y)) {
        continue;
      };

      queuePotentialSources(potentialSources, neighbour_x, neighbour_y);
      potentialDistances.clear();
      for (size_t k = 0; k < potentialSources.size(); ++k) {
        int potentialSource = potentialSources[k];
        int LS_x = lightSources_[potentialSource].x;
        int LS_y = lightSources_[potentialSource].y;
        distance = edf_(LS_x, LS_y) +
                   evaluateDistance(LS_x, LS_y, neighbour_x, neighbour_y);
        potentialDistances.push_back({distance, 0.0, potentialSource});
      }

      auto minimum_element =
          std::min_element(potentialDistances.begin(), potentialDistances.end(),
                           [](const auto &lhs, const auto &rhs) {
                             return lhs.distance < rhs.distance;
                           });
      distance = minimum_element->distance;
      // use source giving least distance
      edf_(neighbour_x, neighbour_y) = distance;
      cameFrom_(neighbour_x, neighbour_y) = minimum_element->source;
      openSet_->push(Node{neighbour_x, neighbour_y, distance});
      isUpdated_(neighbour_x, neighbour_y) = true;
    }
  };

  auto stopTime = std::chrono::high_resolution_clock::now();
  auto executionDuration = durationInMicroseconds(startTime, stopTime);

  if (!sharedConfig_->silent) {
    std::cout << "##################### Distance function computation "
                 "output #####################"
              << std::endl;
    if (sharedConfig_->timer) {
      std::cout << "Constructed distance function" << std::endl;
      std::cout << "Marching time in us: " << executionDuration << "us"
                << std::endl;
    }
  }
  saveField(edf_, "EDF");
}

} // namespace RS_marching
