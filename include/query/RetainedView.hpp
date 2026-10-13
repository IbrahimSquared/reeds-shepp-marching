#ifndef QUERY_RETAINED_VIEW_HPP
#define QUERY_RETAINED_VIEW_HPP

#include "accelerated/Solver.hpp"
#include "environment/Field.hpp"

#include <cstddef>
#include <cstdint>

namespace RS_marching {
class Marching;
namespace geometry {
class ConnectorClearance;
}

namespace query {

// Read-only view of one completed geometric march: occupancy, accepted cell
// records, retained pivot prefixes and the marcher's clearance scene. Queries
// read retained state only through this view and never write it.
class RetainedView {
public:
  explicit RetainedView(const Marching &marching) : marching_(marching) {}

  [[nodiscard]] const Field<double> &map() const;
  [[nodiscard]] int width() const;
  [[nodiscard]] int height() const;
  [[nodiscard]] double radius() const;
  [[nodiscard]] bool inside(int x, int y) const {
    return x >= 0 && y >= 0 && x < width() && y < height();
  }
  [[nodiscard]] bool free(int x, int y) const;

  // Accepted cell records. reached() means updated, owned and below the
  // sentinel distance.
  [[nodiscard]] bool updated(int x, int y) const;
  [[nodiscard]] bool reached(int x, int y) const;
  [[nodiscard]] int owner(int x, int y) const;
  [[nodiscard]] double distance(int x, int y) const;
  [[nodiscard]] double heading(int x, int y) const;
  [[nodiscard]] double pivotHeading(int x, int y) const;
  [[nodiscard]] std::uint32_t recordIncomingWord(int x, int y) const;
  [[nodiscard]] std::uint32_t recordOutgoingWord(int x, int y) const;
  // Length of the record's incoming leg, from the owner's preceding pose to its
  // pivot pose (zero for a root owner).
  [[nodiscard]] double recordIncomingDistance(int x, int y) const;
  // The owner pivot pose used by the cell's record: the pivot position with
  // the record's pivot heading, or the physical heading of a root owner.
  [[nodiscard]] State recordPivotPose(int x, int y) const;
  // The root's own cell, whose record has no legs.
  [[nodiscard]] bool rootCell(int x, int y) const;

  // Retained sources; source 0 is the root, whose parent is negative.
  [[nodiscard]] std::size_t sources() const;
  [[nodiscard]] bool validSource(int source) const;
  [[nodiscard]] int parent(int source) const;
  [[nodiscard]] State precedingPose(int source) const;
  [[nodiscard]] double prefixDistance(int source) const;
  [[nodiscard]] double birthHeading(int source) const;
  [[nodiscard]] std::uint32_t incomingWord(int source) const;
  // Geometric pivot position with its birth heading.
  [[nodiscard]] State pivotPose(int source) const;
  // Physical start pose of the root source.
  [[nodiscard]] State rootPose() const;

  [[nodiscard]] const geometry::ConnectorClearance &clearance() const;

private:
  const Marching &marching_;
};

} // namespace query
} // namespace RS_marching

#endif
