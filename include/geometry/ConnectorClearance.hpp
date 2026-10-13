#ifndef CONNECTOR_CLEARANCE_HPP
#define CONNECTOR_CLEARANCE_HPP

#include "audit/DirectStartClearance.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

namespace RS_marching::geometry {

// The caller binds this certificate to one immutable literal from pose, radius,
// and scene. Entries are turn/gear indexed positive first-arc lengths.
struct FirstArcClear {
  std::array<double, 4> length{};
};

// Shared scene index and fixed scene-local first-arc Clear certificates. The
// existing continuous checker remains authoritative when unresolved.
class ConnectorClearance {
public:
  void reset(const Field<double> &map);
  // Another checker over the same immutable scene index, with its own
  // counters and first-arc certificates; its checks never modify this object.
  // The map passed to reset must remain alive and unchanged while either
  // checker is used. Sharing owns the index, not the borrowed occupancy map.
  [[nodiscard]] ConnectorClearance sharedSceneChecker() const;
  audit::ContinuousClearanceStatus check(
      State from, double radius, const audit::LegacyPathAuditResult &path,
      FirstArcClear *owned = nullptr,
      const audit::ReplayPrimitiveBuffer *prepared = nullptr);

  // Probes may ask only for the conservative spatial filter: an undecided
  // path is then Indeterminate instead of receiving exact interval work.
  bool filter_only = false;
  std::uint64_t queries = 0, box_queries = 0, subdivisions = 0;
  // filtered records calls with no new exact check; retained evidence may contribute.
  std::uint64_t filtered = 0, exact_checks = 0, intervals = 0, events = 0;
  std::uint64_t refinements = 0, filtered_blocked = 0, range_checks = 0, whole_checks = 0;
  std::uint64_t first_arc_cache_lookups = 0, first_arc_cache_hits = 0;
  std::uint64_t owned_first_arc_lookups = 0, owned_first_arc_hits = 0;
  std::uint64_t spatial_queries = 0, spatial_nodes = 0, spatial_clears = 0;
  std::array<std::uint64_t, 5> exact_statuses{};
  std::uint64_t free_box_attempts = 0, free_box_cells = 0, free_box_clears = 0;
  // Valid after a Blocked check; arclength is local to blocked_primitive.
  State blocked_witness{};
  std::size_t blocked_primitive = 0;
  double blocked_arclength = 0.0;
  // Scene index bytes, shared by every checker over this scene.
  [[nodiscard]] std::size_t bytes() const;

private:
  struct ObstacleRectangle {
    double xmin = 0.0, ymin = 0.0, xmax = 0.0, ymax = 0.0;
  };
  struct BvhNode {
    ObstacleRectangle bounds;
    // Preorder subtree end. A leaf has escape == its index + 1.
    std::size_t escape = 0;
  };
  struct FirstArcCertificate {
    std::uint64_t x = 0, y = 0, theta = 0, radius = 0;
    double maximum_clear_length = 0.0;
    std::int8_t turn = 0, gear = 0;
    bool valid = false;
  };
  struct PreparedPrimitive {
    State from;
    double radius = 0.0;
    double turn = 0.0;
    double gear = 0.0;
    double sine = 0.0;
    double cosine = 0.0;
    bool straight = false;

    PreparedPrimitive(State literal_from, double literal_radius,
                      const audit::LegacyPrimitive &leg,
                      const audit::ReplayPrimitiveGeometry *cached = nullptr);
    [[nodiscard]] State point(double arclength, State *literal_end = nullptr) const;
  };

  struct Scene {
    std::vector<std::uint64_t> sums;
    std::vector<BvhNode> bvh;
  };

  bool emptyBox(double xmin, double ymin, double xmax, double ymax);
  static std::size_t buildBvh(Scene &scene, std::size_t begin, std::size_t end,
                              std::vector<ObstacleRectangle> &rectangles,
                              ObstacleRectangle *suffix);
  audit::ContinuousClearanceStatus spatiallyQualifyPrimitive(const PreparedPrimitive &primitive,
                              double center_x, double center_y,
                         double xmin, double ymin, double xmax, double ymax,
                         double margin, double length);
  bool strictBlockedSample(const PreparedPrimitive &primitive, double arclength,
                           double margin);
  audit::ContinuousClearanceStatus enclose(
      const PreparedPrimitive &primitive, double begin, double end,
      State begin_point, State end_point, unsigned depth, State original_from,
      const audit::LegacyPathAuditResult &path, std::size_t index,
      double margin, bool cardinal_free = false);
  void countExact(const audit::ContinuousClearanceResult &result);
  bool firstArcCovered(const State &from, double radius,
                       const audit::LegacyPrimitive &leg);
  void retainFirstArcClear(const State &from, double radius,
                           const audit::LegacyPrimitive &leg);
  const Field<double> *map_ = nullptr;
  std::size_t width_ = 0, height_ = 0;
  std::shared_ptr<const Scene> scene_;
  // Views into *scene_ for the checking paths.
  const std::uint64_t *sums_ = nullptr;
  const BvhNode *bvh_ = nullptr;
  std::size_t bvh_size_ = 0;
  std::array<FirstArcCertificate, 4> first_arc_certificates_{};
};

} // namespace RS_marching::geometry
#endif
