#ifndef MARCHING_HPP
#define MARCHING_HPP

#include <cmath>
#include <queue>
#include <stack> // Make sure this is included in your file if it's not already.
#include <vector>

#include "accelerated/Solver.hpp"
#include "environment/Environment.hpp"
#include "thirdparty/flat_hash_map/flat_hash_map.hpp"
#include "underspecified/UnderSpecifiedRSPlanner.hpp"

const double pi = 3.1415926535897932;

namespace RS_marching {

struct connection {
  pose pivot_1;
  pose pivot_2;
};

struct triplet {
  double distance;
  double theta;
  int source;
};

struct Point {
  double x;
  double y;
};

class Marching {
public:
  using Map = ska::flat_hash_map<size_t, double>;

  explicit Marching(Environment &env);

  // Deconstructor
  ~Marching() = default;

  void runMarching();
  void computeEDF();
  void memoize();

private:
  int x_start_;
  int y_start_;
  double theta_start_;
  int x_goal_;
  int y_goal_;
  double theta_goal_;
  bool key_ = true;
  bool bumps = true;
  // use gradient descent by default instead of bisection method
  int GD_bisection_ = 1;
  void reset();
  double spherical_interpolation(const double xn, const double yn,
                                 const double ls_theta) const;
  double omega_interpolation(const double xn, const double yn,
                             const double ls_theta) const;
  double bilinear_interpolation(const double xn, const double yn) const;
  void saveLightSources() const;
  void processConnection();

  /*!
   * @brief queues sources
   */
  inline void queuePotentialSources(std::vector<size_t> &potentialSources,
                                    const int neighbour_x,
                                    const int neighbour_y);

  /*!
   * @brief gets distances
   */
  inline void getPotentialDistances(const std::vector<size_t> &potentialSources,
                                    std::vector<triplet> &potentialDistances,
                                    const int neighbour_x,
                                    const int neighbour_y);

  inline void createNewPivot(const int x, const int y, const int neighbour_x,
                             const int neighbour_y);

  /*!
   * @brief Updates accessibility/visibility to a point using PDE advection.
   * @param [in] lightSourceNumber number of the lightsource whose visibility of
   * the point we are checking.
   * @param [in] lightSource_x x position of the lightsource.
   * @param [in] lightSource_y y position of the lightsource.
   * @param [in] x position of our queried pixel.
   * @param [in] y position of our queried pixel.
   */
  void updatePointVisibility(const size_t lightSourceNumber,
                             const int lightSource_x, const int lightSource_y,
                             const int x, const int y, const int rx = 0,
                             const int ry = 0);

  double calculateAngleBasedVisibility(const size_t lightSourceNumber,
                                       const int LS_x, const int LS_y,
                                       const int x, const int y, const int rx,
                                       const int ry, const double a);

  // inline int hashFunction(const int x, const int y,
  //                         const int lightSourceNumber) const {
  //   const auto key = y + nx_ * x * ny_ + ny_ * lightSourceNumber;
  //   return key;
  // }

  // inline size_t hashFunction(const int x, const int y,
  //                            const int lightSourceNumber) const {
  //   // Use prime multipliers to improve distribution
  //   static const size_t p1 = 73856093;
  //   static const size_t p2 = 19349663;
  //   static const size_t p3 = 83492791;
  //
  //   // Combine with XOR for better bit mixing
  //   return (static_cast<size_t>(x) * p1) ^ (static_cast<size_t>(y) * p2) ^
  //          (static_cast<size_t>(lightSourceNumber) * p3);
  // }

  // Another possible hash function that is more evenly distributed
  // inline int hashFunction(const int x, const int y,
  //                         const int lightSourceNumber) const {
  //   const int prime1 = 73856093;
  //   const int prime2 = 19349663;
  //   const int prime3 = 83492791;
  //
  //   // Mix the input values using primes and combine them
  //   const auto key = (x * prime1) ^ (y * prime2) ^ (lightSourceNumber *
  //   prime3);
  //   // return key;
  //   const int modulus = 1000000007;
  //   return key % modulus;
  // }

  inline size_t hashFunction(const int x, const int y,
                             const int lightSourceNumber) const {
    return static_cast<size_t>(x) +
           nx_ * (static_cast<size_t>(y) +
                  ny_ * static_cast<size_t>(lightSourceNumber));
  }

  std::shared_ptr<Field<double>> sharedVisibilityField_;
  std::shared_ptr<Config> sharedConfig_;
  void saveImage(const Field<double> &distances, const std::string &name) const;

  Field<double> gScore_;
  Field<double> thetas_;
  Field<double> edf_;
  Field<int> cameFrom_;
  Field<int> origin_;
  Field<bool> inOpenSet_;
  Field<bool> isUpdated_;
  Field<double> omega_map_;
  Field<double> distance_map_;
  Field<double> angle_map_;
  std::unique_ptr<pose[]> lightSources_;
  // Lightstrength, can be decreased. Can add later an alpha that has light
  // decay, enforcing adding a new pivot periodically.
  const double lightStrength_ = 1.0;
  int nb_of_iterations_ = 0;
  size_t nb_of_sources_ = 0;
  connection connection_;
  std::vector<pose> waypoints_;

  // Neighbours
  // [1 0; 0 1; -1 0; 0 -1; 1 1; -1 1; -1 -1; 1 -1] flattened out
  const int neighbours_[16] = {1, 0, 0,  1, -1, 0,  0, -1,
                               1, 1, -1, 1, -1, -1, 1, -1};
  const int neighbours_48_[48] = {
      // First Layer
      1, 0, 0, 1, -1, 0, 0, -1, 1, 1, -1, 1, -1, -1, 1, -1,
      // Second Layer
      2, 0, 0, 2, -2, 0, 0, -2, 2, 2, -2, 2, -2, -2, 2, -2, 2, 1, 1, 2, -1, 2,
      -2, 1, -2, -1, -1, -2, 1, -2, 2, -1};

  inline double evaluateDistance(const int LS_x, const int LS_y,
                                 const double LS_theta, int source,
                                 const int neighbour_x, const int neighbour_y,
                                 double &theta);

  inline double evaluateDistance(const int x1, const int y1, const int x2,
                                 const int y2) const {
    return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
  };

  // Dimensions.
  size_t ny_;
  size_t nx_;

  /*!
   * @brief Checks if requested cell is in grid
   * @param [in] x x position of the cell
   * @param [in] y y position of the cell
   * @return true if cell is in grid, false otherwise
   */
  inline bool isValid(const size_t x, const size_t y) const {
    return ((x < nx_) && (y < ny_));
  }

  // Heap openSet_;
  std::unique_ptr<std::priority_queue<Node>> openSet_;

  double visibilityThreshold_ = 0.5;

  // Flat hash maps
  Map visibilityHashMap_;
  double return_count_ = 0;

  // RS related
  UnderSpecifiedRSPlanner urs_;
  double r_ = 1;
  Solver solver_;
  State to_;
  double omega_;

  void printProgress(const int iteration, const int size) const;

  inline void RLPositive(const State s1, State &s2, State &s3, const Point p2_0,
                         const Point p3_0) const;
  inline void RLNegative(const State s1, State &s2, State &s3, const Point p2_0,
                         const Point p3_0) const;
  inline void LRPositive(const State s1, State &s2, State &s3, const Point p2_0,
                         const Point p3_0) const;
  inline void LRNegative(const State s1, State &s2, State &s3, const Point p2_0,
                         const Point p3_0) const;
  inline void RRPositive(const State s1, State &s2, State &s3, const Point p2_0,
                         const Point p3_0) const;
  inline void RRNegative(const State s1, State &s2, State &s3, const Point p2_0,
                         const Point p3_0) const;
  inline void LLPositive(const State s1, State &s2, State &s3, const Point p2_0,
                         const Point p3_0) const;
  inline void LLNegative(const State s1, State &s2, State &s3, const Point p2_0,
                         const Point p3_0) const;

  inline double norm(const double x1, const double y1, const double x2,
                     const double y2) const {
    return std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
  }
  int exit_it = 10;
  int global_visibility_count_ = 0;
};

} // namespace RS_marching
#endif
