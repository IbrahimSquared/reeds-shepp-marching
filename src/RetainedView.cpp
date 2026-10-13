#include "query/RetainedView.hpp"

#include "marching/Marching.hpp"

#include <limits>

namespace RS_marching::query {

const Field<double> &RetainedView::map() const {
  return *marching_.sharedVisibilityField_;
}
int RetainedView::width() const { return static_cast<int>(marching_.nx_); }
int RetainedView::height() const { return static_cast<int>(marching_.ny_); }
double RetainedView::radius() const { return marching_.r_; }
bool RetainedView::free(const int x, const int y) const {
  return marching_.sharedVisibilityField_->get(x, y) >= 1.0;
}

bool RetainedView::updated(const int x, const int y) const {
  return marching_.isUpdated_(x, y);
}
bool RetainedView::reached(const int x, const int y) const {
  // Occupied neighbours are marked updated and owned but keep the reset
  // distance std::numeric_limits<double>::max(); they are not reached.
  const double g = marching_.gScore_(x, y);
  return marching_.isUpdated_(x, y) && marching_.cameFrom_(x, y) >= 0 &&
         std::isfinite(g) && g < std::numeric_limits<double>::max();
}
int RetainedView::owner(const int x, const int y) const {
  return marching_.cameFrom_(x, y);
}
double RetainedView::distance(const int x, const int y) const {
  return marching_.gScore_(x, y);
}
double RetainedView::heading(const int x, const int y) const {
  return marching_.thetas_(x, y);
}
double RetainedView::pivotHeading(const int x, const int y) const {
  return marching_.geometric_pivot_theta_(x, y);
}
std::uint32_t RetainedView::recordIncomingWord(const int x, const int y) const {
  return marching_.geometric_incoming_word_(x, y);
}
std::uint32_t RetainedView::recordOutgoingWord(const int x, const int y) const {
  return marching_.geometric_outgoing_word_(x, y);
}
double RetainedView::recordIncomingDistance(const int x, const int y) const {
  return marching_.geometric_incoming_distance_(x, y);
}
State RetainedView::recordPivotPose(const int x, const int y) const {
  const int source = owner(x, y);
  const Point position = marching_.geometricPivotPosition(source);
  return {position.x, position.y,
          parent(source) < 0 ? marching_.lightSources_[source].theta + pi / 2.0
                             : pivotHeading(x, y)};
}
bool RetainedView::rootCell(const int x, const int y) const {
  const auto &light = marching_.lightSources_[owner(x, y)];
  return parent(owner(x, y)) < 0 && x == light.x && y == light.y &&
         distance(x, y) == 0.0;
}

std::size_t RetainedView::sources() const { return marching_.nb_of_sources_; }
bool RetainedView::validSource(const int source) const {
  return source >= 0 && static_cast<std::size_t>(source) < marching_.nb_of_sources_ &&
         static_cast<std::size_t>(source) < marching_.pivot_prefixes_.size();
}
int RetainedView::parent(const int source) const {
  return marching_.pivot_prefixes_[source].parent_source;
}
State RetainedView::precedingPose(const int source) const {
  return marching_.pivot_prefixes_[source].preceding_pose;
}
double RetainedView::prefixDistance(const int source) const {
  return marching_.pivot_prefixes_[source].distance;
}
double RetainedView::birthHeading(const int source) const {
  return marching_.pivot_prefixes_[source].birth_theta;
}
std::uint32_t RetainedView::incomingWord(const int source) const {
  return marching_.pivot_prefixes_[source].incoming_word;
}
State RetainedView::pivotPose(const int source) const {
  const Point position = marching_.geometricPivotPosition(source);
  return {position.x, position.y, birthHeading(source)};
}
State RetainedView::rootPose() const {
  const auto &root = marching_.lightSources_[0];
  return {static_cast<double>(root.x), static_cast<double>(root.y),
          root.theta + pi / 2.0};
}

const geometry::ConnectorClearance &RetainedView::clearance() const {
  return marching_.geometric_clearance_;
}

} // namespace RS_marching::query
