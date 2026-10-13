#include "geometry/ConnectorClearance.hpp"
#include "misc/AngleRemainder.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <numbers>
#include <limits>
#include <unordered_map>

namespace RS_marching::geometry {

ConnectorClearance::PreparedPrimitive::PreparedPrimitive(
    const State literal_from, const double literal_radius,
    const audit::LegacyPrimitive &leg,
    const audit::ReplayPrimitiveGeometry *const cached)
    : from(literal_from), radius(literal_radius),
      turn(leg.motion_type == 'L' ? 1.0 : -1.0), gear(leg.direction),
      sine(cached != nullptr ? cached->start_sine : std::sin(from.theta)),
      cosine(cached != nullptr ? cached->start_cosine : std::cos(from.theta)),
      straight(leg.motion_type == 'S') {}

State ConnectorClearance::PreparedPrimitive::point(
    const double arclength, State *literal_end) const {
  if (straight) {
    const State point{from.x + gear * arclength * cosine,
                      from.y + gear * arclength * sine, from.theta};
    if (literal_end) *literal_end = arclength == 0.0 ? from : point;
    return point;
  }
  const double angle = from.theta + gear * turn * arclength / radius;
  const double end_sine = std::sin(angle), end_cosine = std::cos(angle);
  if (literal_end) {
    // Match replayLegacyPrimitives' center-then-endpoint expressions exactly;
    // the enclosure below deliberately retains its difference expressions.
    const double center_x = from.x - turn * radius * sine;
    const double center_y = from.y + turn * radius * cosine;
    *literal_end = arclength == 0.0 ? from : State{
        center_x + turn * radius * end_sine,
        center_y - turn * radius * end_cosine, angle};
  }
  return State{from.x + turn * radius * (end_sine - sine),
               from.y - turn * radius * (end_cosine - cosine), angle};
}

void ConnectorClearance::reset(const Field<double> &map) {
  map_ = &map;
  width_ = map.nx();
  height_ = map.ny();
  auto scene = std::make_shared<Scene>();
  auto &sums = scene->sums;
  std::vector<ObstacleRectangle> rectangles;
  sums.assign((width_ + 1) * (height_ + 1), 0);
  for (std::size_t y = 0; y < height_; ++y) {
    std::uint64_t row = 0;
    for (std::size_t x = 0; x < width_; ++x) {
      row += map.get(x, y) < 1.0;
      sums[(y + 1) * (width_ + 1) + x + 1] =
          sums[y * (width_ + 1) + x + 1] + row;
    }
  }
  struct Span {
    std::size_t first = 0, last = 0;
    bool operator==(const Span &) const = default;
  };
  struct SpanHash {
    std::size_t operator()(const Span &span) const {
      return std::hash<std::size_t>{}(span.first) ^
          (std::hash<std::size_t>{}(span.last) << 1);
    }
  };
  std::unordered_map<Span, std::size_t, SpanHash> active;
  for (std::size_t y = 0; y < height_; ++y) {
    std::unordered_map<Span, std::size_t, SpanHash> next;
    for (std::size_t x = 0; x < width_;) {
      if (!(map.get(x, y) < 1.0)) {
        ++x;
        continue;
      }
      const auto first = x++;
      while (x < width_ && map.get(x, y) < 1.0) ++x;
      const Span span{first, x};
      const auto prior = active.find(span);
      if (prior != active.end()) {
        rectangles[prior->second].ymax = static_cast<double>(y) + 0.5;
        next.emplace(span, prior->second);
      } else {
        rectangles.push_back({static_cast<double>(first) - 0.5,
                              static_cast<double>(y) - 0.5,
                              static_cast<double>(x) - 0.5,
                              static_cast<double>(y) + 0.5});
        next.emplace(span, rectangles.size() - 1);
      }
    }
    active.swap(next);
  }
  if (!rectangles.empty()) {
    scene->bvh.reserve(rectangles.size() * 2 - 1);
    std::vector<ObstacleRectangle> suffix(rectangles.size());
    buildBvh(*scene, 0, rectangles.size(), rectangles, suffix.data());
    // Collapse alternating internal levels without changing leaf bounds/order.
    // Snapshot each old escape before writes compact its subtree in place.
    auto &bvh = scene->bvh;
    std::size_t write = 0;
    const auto compact = [&](auto &&self, const std::size_t index,
                             const std::size_t depth) -> void {
      const BvhNode node = bvh[index];
      const bool leaf = node.escape == index + 1;
      const bool keep = leaf || depth % 2 == 0;
      const std::size_t out = write;
      if (keep)
        bvh[write++] = node;
      for (std::size_t child = index + 1; child < node.escape;) {
        const std::size_t child_end = bvh[child].escape;
        self(self, child, depth + 1);
        child = child_end;
      }
      if (keep)
        bvh[out].escape = write;
    };
    compact(compact, 0, 0);
    bvh.resize(write);
  }
  sums_ = scene->sums.data();
  bvh_ = scene->bvh.data();
  bvh_size_ = scene->bvh.size();
  scene_ = std::move(scene);
  queries = box_queries = subdivisions = filtered = exact_checks = 0;
  intervals = events = 0;
  free_box_attempts = free_box_cells = free_box_clears = 0;
  refinements = filtered_blocked = 0;
  range_checks = whole_checks = 0;
  exact_statuses.fill(0);
  first_arc_cache_lookups = first_arc_cache_hits = 0;
  owned_first_arc_lookups = owned_first_arc_hits = 0;
  spatial_queries = spatial_nodes = spatial_clears = 0;
  first_arc_certificates_.fill({});
}

ConnectorClearance ConnectorClearance::sharedSceneChecker() const {
  ConnectorClearance checker;
  checker.map_ = map_;
  checker.width_ = width_;
  checker.height_ = height_;
  checker.scene_ = scene_;
  checker.sums_ = sums_;
  checker.bvh_ = bvh_;
  checker.bvh_size_ = bvh_size_;
  return checker;
}

std::size_t ConnectorClearance::bytes() const {
  return scene_ == nullptr ? 0 :
      scene_->sums.capacity() * sizeof(std::uint64_t) +
      scene_->bvh.capacity() * sizeof(BvhNode);
}

std::size_t ConnectorClearance::buildBvh(Scene &scene, const std::size_t begin,
    const std::size_t end, std::vector<ObstacleRectangle> &rectangles,
    ObstacleRectangle *const suffix) {
  auto &bvh = scene.bvh;
  const auto node_index = bvh.size();
  bvh.push_back({});
  const auto count = end - begin;
  if (count == 1) {
    bvh[node_index].bounds = rectangles[begin];
    bvh[node_index].escape = node_index + 1;
    return node_index;
  }
  const auto unite = [](const ObstacleRectangle &a, const ObstacleRectangle &b) {
    return ObstacleRectangle{std::min(a.xmin, b.xmin), std::min(a.ymin, b.ymin),
                             std::max(a.xmax, b.xmax), std::max(a.ymax, b.ymax)};
  };
  const auto weight = [](const ObstacleRectangle &b) {
    return (b.xmax - b.xmin) + (b.ymax - b.ymin);
  };
  const auto sort_axis = [&](bool x_axis) {
    std::sort(rectangles.begin() + begin, rectangles.begin() + end,
        [x_axis](const ObstacleRectangle &a, const ObstacleRectangle &b) {
          return x_axis ? a.xmin + a.xmax < b.xmin + b.xmax
                        : a.ymin + a.ymax < b.ymin + b.ymax;
        });
  };
  // Search both axes, retaining balanced children. Bounds and leaves still
  // represent precisely the same occupied squares; only the grouping changes.
  const auto minimum_child = std::max<std::size_t>(1, count / 4);
  double best_cost = std::numeric_limits<double>::infinity();
  std::size_t best_split = count / 2;
  bool best_x = false;
  ObstacleRectangle bounds;
  for (const bool x_axis : {true, false}) {
    sort_axis(x_axis);
    suffix[count - 1] = rectangles[end - 1];
    for (std::size_t i = count - 1; i > 0; --i)
      suffix[i - 1] = unite(rectangles[begin + i - 1], suffix[i]);
    bounds = suffix[0];
    auto prefix = rectangles[begin];
    for (std::size_t split = 1; split < count; ++split) {
      if (split >= minimum_child && count - split >= minimum_child) {
        const double cost = weight(prefix) * static_cast<double>(split) +
            weight(suffix[split]) * static_cast<double>(count - split);
        if (cost < best_cost) {
          best_cost = cost;
          best_split = split;
          best_x = x_axis;
        }
      }
      prefix = unite(prefix, rectangles[begin + split]);
    }
  }
  if (best_x) sort_axis(true);
  const auto middle = begin + best_split;
  buildBvh(scene, middle, end, rectangles, suffix);
  buildBvh(scene, begin, middle, rectangles, suffix);
  bvh[node_index] = {bounds, bvh.size()};
  return node_index;
}

bool ConnectorClearance::strictBlockedSample(const PreparedPrimitive &primitive,
                                                   const double arclength,
                                                   const double margin) {
  const auto witness = primitive.point(arclength);
  if (!(witness.x > -0.5 && witness.y > -0.5 &&
        witness.x < width_ - 0.5 && witness.y < height_ - 0.5))
    return false;
  const auto x = static_cast<std::size_t>(std::floor(witness.x + 0.5));
  const auto y = static_cast<std::size_t>(std::floor(witness.y + 0.5));
  if (!(map_->get(x, y) < 1.0 &&
        std::abs(witness.x - static_cast<double>(x)) < 0.5 - margin &&
        std::abs(witness.y - static_cast<double>(y)) < 0.5 - margin))
    return false;
  blocked_witness = witness;
  blocked_arclength = arclength;
  return true;
}

audit::ContinuousClearanceStatus ConnectorClearance::spatiallyQualifyPrimitive(
    const PreparedPrimitive &primitive, const double center_x, const double center_y,
    const double xmin, const double ymin, const double xmax, const double ymax,
    const double margin, const double length) {
  using Status = audit::ContinuousClearanceStatus;
  ++spatial_queries;
  const double radius = primitive.radius;
  const double radial_padding = 4.0 * margin;
  const double padded_xmin = xmin - radial_padding;
  const double padded_ymin = ymin - radial_padding;
  const double padded_xmax = xmax + radial_padding;
  const double padded_ymax = ymax + radial_padding;
  const double outer_radius = radius + radial_padding;
  const double inner_radius = std::max(0.0, radius - radial_padding);
  const double outer_squared = outer_radius * outer_radius;
  const double inner_squared = inner_radius * inner_radius;
  const double map_width = static_cast<double>(width_);
  const double map_height = static_cast<double>(height_);
  if (!std::isfinite(center_x) || !std::isfinite(center_y) ||
      !std::isfinite(radius) || !std::isfinite(radial_padding) ||
      !std::isfinite(padded_xmin) || !std::isfinite(padded_ymin) ||
      !std::isfinite(padded_xmax) || !std::isfinite(padded_ymax) ||
      !std::isfinite(outer_squared) || !std::isfinite(inner_squared) ||
      !std::isfinite(map_width) || !std::isfinite(map_height) ||
      !(padded_xmin > -0.5 && padded_ymin > -0.5 &&
        padded_xmax < map_width - 0.5 && padded_ymax < map_height - 0.5))
    return Status::Indeterminate;
  std::size_t node_index = 0;
  while (node_index < bvh_size_) {
    const auto current = node_index;
    const auto &node = bvh_[current];
    node_index = node.escape;
    ++spatial_nodes;
    const auto &bounds = node.bounds;
    if (bounds.xmax < padded_xmin || bounds.xmin > padded_xmax ||
        bounds.ymax < padded_ymin || bounds.ymin > padded_ymax)
      continue;
    if (primitive.straight) {
      const double nx = -primitive.sine, ny = primitive.cosine;
      const double lx = bounds.xmin - primitive.from.x;
      const double hx = bounds.xmax - primitive.from.x;
      const double ly = bounds.ymin - primitive.from.y;
      const double hy = bounds.ymax - primitive.from.y;
      const double low = nx * (nx < 0.0 ? hx : lx) +
                         ny * (ny < 0.0 ? hy : ly);
      const double high = nx * (nx < 0.0 ? lx : hx) +
                          ny * (ny < 0.0 ? ly : hy);
      if (!std::isfinite(low) || !std::isfinite(high)) return Status::Indeterminate;
      // Segment-box separation needs only the segment box axes and line
      // normal. Whole node bounds suffice; uncertain contact keeps fallback.
      if (low > radial_padding || high < -radial_padding) continue;
      if (node.escape != current + 1) {
        node_index = current + 1;
        continue;
      }
    }
    // Only the obstacle portion inside the arc box can meet this arc.
    const ObstacleRectangle box{
        std::max(bounds.xmin, padded_xmin), std::max(bounds.ymin, padded_ymin),
        std::min(bounds.xmax, padded_xmax), std::min(bounds.ymax, padded_ymax)};
    if (!primitive.straight) {
      const double dx_min = center_x < box.xmin ? box.xmin - center_x
          : (center_x > box.xmax ? center_x - box.xmax : 0.0);
      const double dy_min = center_y < box.ymin ? box.ymin - center_y
          : (center_y > box.ymax ? center_y - box.ymax : 0.0);
      const double dx_max = std::max(std::abs(center_x - box.xmin),
                                     std::abs(center_x - box.xmax));
      const double dy_max = std::max(std::abs(center_y - box.ymin),
                                     std::abs(center_y - box.ymax));
      const double min_squared = dx_min * dx_min + dy_min * dy_min;
      const double max_squared = dx_max * dx_max + dy_max * dy_max;
      if (!std::isfinite(min_squared) || !std::isfinite(max_squared)) return Status::Indeterminate;
      // A node is outside the padded circular band, or outside the padded arc
      // box. Boundary cases remain unresolved for the exact checker.
      if (min_squared > outer_squared || max_squared < inner_squared) continue;
    }
    if (node.escape == current + 1) {
      if (primitive.straight && length > 0.0) {
        // The leaf slab only proposes an interior parameter. The shared
        // literal replay check remains the only collision decision.
        const double dx = primitive.gear * primitive.cosine;
        const double dy = primitive.gear * primitive.sine;
        double low = 0.0, high = length;
        const auto clip = [&](const double origin, const double direction,
                              const double minimum, const double maximum) {
          if (direction == 0.0) return origin > minimum && origin < maximum;
          const double first = (minimum - origin) / direction;
          const double last = (maximum - origin) / direction;
          low = std::max(low, std::min(first, last));
          high = std::min(high, std::max(first, last));
          return std::isfinite(low) && std::isfinite(high);
        };
        if (clip(primitive.from.x, dx, box.xmin + margin, box.xmax - margin) &&
            clip(primitive.from.y, dy, box.ymin + margin, box.ymax - margin) &&
            low < high) {
          const double arclength = low + (high - low) / 2.0;
          if (std::isfinite(arclength) && arclength >= 0.0 &&
              arclength <= length && strictBlockedSample(primitive, arclength, margin))
            return Status::Blocked;
        }
      } else if (!primitive.straight && length > 0.0) {
        // The box chooses a sample, never a clearance or collision decision.
        // Replay that parameter and retain the existing strict interior test.
        const double qx = std::clamp(center_x, box.xmin, box.xmax) - center_x;
        const double qy = std::clamp(center_y, box.ymin, box.ymax) - center_y;
        const double fx = std::abs(box.xmin - center_x) > std::abs(box.xmax - center_x)
            ? box.xmin - center_x : box.xmax - center_x;
        const double fy = std::abs(box.ymin - center_y) > std::abs(box.ymax - center_y)
            ? box.ymin - center_y : box.ymax - center_y;
        const double vx = fx - qx, vy = fy - qy;
        const double a = vx * vx + vy * vy, b = qx * vx + qy * vy;
        const double c = radius * radius - qx * qx - qy * qy;
        if (!(c > 0.0 && a > 0.0)) return Status::Indeterminate;
        const double fraction = c / (b + std::sqrt(b * b + a * c));
        if (!(fraction > 0.0 && fraction < 1.0)) return Status::Indeterminate;
        const double dx = qx + fraction * vx, dy = qy + fraction * vy;
        const double theta = std::atan2(primitive.turn * dx, -primitive.turn * dy);
        double sweep = primitive.gear * primitive.turn *
            detail::boundedNearestAngleRemainder(
                theta - primitive.from.theta, 2.0 * std::numbers::pi);
        if (sweep < 0.0) sweep += 2.0 * std::numbers::pi;
        const double arclength = radius * sweep;
        if (std::isfinite(arclength) && arclength > 0.0 && arclength <= length &&
            strictBlockedSample(primitive, arclength, margin))
          return Status::Blocked;
      }
      return Status::Indeterminate;
    }
    node_index = current + 1;
  }
  ++spatial_clears;
  return Status::Clear;
}

bool ConnectorClearance::firstArcCovered(const State &from, const double radius,
                                         const audit::LegacyPrimitive &leg) {
  ++first_arc_cache_lookups;
  const auto x = std::bit_cast<std::uint64_t>(from.x), y = std::bit_cast<std::uint64_t>(from.y);
  const auto theta = std::bit_cast<std::uint64_t>(from.theta), r = std::bit_cast<std::uint64_t>(radius);
  const auto turn = static_cast<std::int8_t>(leg.motion_type == 'L' ? 1 : -1);
  const auto gear = static_cast<std::int8_t>(leg.direction);
  for (std::size_t index = 0; index < first_arc_certificates_.size(); ++index) {
    auto &entry = first_arc_certificates_[index];
    if (!entry.valid || entry.x != x || entry.y != y || entry.theta != theta ||
        entry.radius != r || entry.turn != turn || entry.gear != gear) continue;
    const auto matched = entry;
    for (std::size_t shift = index; shift > 0; --shift)
      first_arc_certificates_[shift] = first_arc_certificates_[shift - 1];
    first_arc_certificates_[0] = matched;
    const auto maximum = matched.maximum_clear_length;
    if (leg.length <= maximum) { ++first_arc_cache_hits; return true; }
    return false;
  }
  return false;
}

void ConnectorClearance::retainFirstArcClear(const State &from, const double radius,
                                             const audit::LegacyPrimitive &leg) {
  const auto x = std::bit_cast<std::uint64_t>(from.x), y = std::bit_cast<std::uint64_t>(from.y);
  const auto theta = std::bit_cast<std::uint64_t>(from.theta), r = std::bit_cast<std::uint64_t>(radius);
  const auto turn = static_cast<std::int8_t>(leg.motion_type == 'L' ? 1 : -1);
  const auto gear = static_cast<std::int8_t>(leg.direction);
  for (std::size_t index = 0; index < first_arc_certificates_.size(); ++index) {
    auto &entry = first_arc_certificates_[index];
    if (entry.valid && entry.x == x && entry.y == y && entry.theta == theta && entry.radius == r && entry.turn == turn && entry.gear == gear) {
      auto matched = entry;
      if (leg.length > matched.maximum_clear_length) matched.maximum_clear_length = leg.length;
      for (std::size_t shift = index; shift > 0; --shift)
        first_arc_certificates_[shift] = first_arc_certificates_[shift - 1];
      first_arc_certificates_[0] = matched;
      return;
    }
  }
  for (std::size_t shift = first_arc_certificates_.size() - 1; shift > 0; --shift)
    first_arc_certificates_[shift] = first_arc_certificates_[shift - 1];
  first_arc_certificates_[0] = {x, y, theta, r, leg.length, turn, gear, true};
}

bool ConnectorClearance::emptyBox(const double xmin, const double ymin,
                                 const double xmax, const double ymax) {
  ++box_queries;
  if (!(xmin > -0.5 && ymin > -0.5 && xmax < width_ - 0.5 &&
        ymax < height_ - 0.5))
    return false;
  // Include every centered square meeting the closed enclosure. Boundary
  // contacts are deliberately unresolved here, never declared blocked.
  const auto x0 = static_cast<std::size_t>(std::max(0.0, std::ceil(xmin - 0.5)));
  const auto y0 = static_cast<std::size_t>(std::max(0.0, std::ceil(ymin - 0.5)));
  const auto x1 = static_cast<std::size_t>(std::min(
      static_cast<double>(width_), std::floor(xmax + 0.5) + 1.0));
  const auto y1 = static_cast<std::size_t>(std::min(
      static_cast<double>(height_), std::floor(ymax + 0.5) + 1.0));
  const auto sum = [&](std::size_t x, std::size_t y) {
    return sums_[y * (width_ + 1) + x];
  };
  return sum(x1, y1) - sum(x0, y1) == sum(x1, y0) - sum(x0, y0);
}

audit::ContinuousClearanceStatus ConnectorClearance::enclose(
    const PreparedPrimitive &primitive, const double begin, const double end,
    const State begin_point, const State end_point, const unsigned depth,
    const State original_from, const audit::LegacyPathAuditResult &path,
    const std::size_t index, const double margin, bool cardinal_free) {
  const double length = end - begin;
  const bool straight = primitive.straight;
  using Status = audit::ContinuousClearanceStatus;
  // A circular coordinate can only be extremal at an endpoint or cardinal
  // phase. The existing margin covers supported-build endpoint arithmetic.
  double xmin = std::min(begin_point.x, end_point.x);
  double xmax = std::max(begin_point.x, end_point.x);
  double ymin = std::min(begin_point.y, end_point.y);
  double ymax = std::max(begin_point.y, end_point.y);
  double center_x = 0.0, center_y = 0.0;
  if (!straight && !cardinal_free) {
    center_x =
        primitive.from.x - primitive.turn * primitive.radius * primitive.sine;
    center_y =
        primitive.from.y + primitive.turn * primitive.radius * primitive.cosine;
    const double guard = margin / primitive.radius;
    const double low = std::min(begin_point.theta, end_point.theta) - guard;
    const double high = std::max(begin_point.theta, end_point.theta) + guard;
    const double quarter_turn = std::numbers::pi / 2.0;
    const bool full_circle = !std::isfinite(guard) || !std::isfinite(low) ||
        !std::isfinite(high) || high - low >= 2.0 * std::numbers::pi;
    if (full_circle) {
      xmin = center_x - primitive.radius;
      xmax = center_x + primitive.radius;
      ymin = center_y - primitive.radius;
      ymax = center_y + primitive.radius;
    } else {
      const double first_value = std::ceil(low / quarter_turn);
      const double last_value = std::floor(high / quarter_turn);
      const auto first = static_cast<long long>(first_value);
      const auto last = static_cast<long long>(last_value);
      // Parameter subintervals have nested guarded angle ranges. Once this
      // range excludes all cardinals, its descendants cannot introduce one.
      cardinal_free = first > last;
      for (long long cardinal = first; cardinal <= last; ++cardinal) {
        const int phase = static_cast<int>((cardinal % 4 + 4) % 4);
        if (phase == 1 || phase == 3) {
          const double x = center_x + primitive.turn * primitive.radius *
              (phase == 1 ? 1.0 : -1.0);
          xmin = std::min(xmin, x);
          xmax = std::max(xmax, x);
        } else {
          const double y = center_y - primitive.turn * primitive.radius *
              (phase == 0 ? 1.0 : -1.0);
          ymin = std::min(ymin, y);
          ymax = std::max(ymax, y);
        }
      }
    }
  }
  if (emptyBox(xmin - margin, ymin - margin, xmax + margin, ymax + margin))
    return Status::Clear;
  const bool eligible = depth == 0 && index == 0 && begin == 0.0 &&
      end == path.primitives[0].length && end > 0.0 && !straight &&
      (path.primitives[0].motion_type == 'L' || path.primitives[0].motion_type == 'R');
  if (eligible && firstArcCovered(primitive.from, primitive.radius, path.primitives[0]))
    return Status::Clear;
  if (depth == 0) {
    const auto spatial = spatiallyQualifyPrimitive(primitive, center_x, center_y,
        xmin, ymin, xmax, ymax, margin, length);
    if (spatial == Status::Blocked) return spatial;
    if (spatial == Status::Clear) {
      if (eligible) retainFirstArcClear(primitive.from, primitive.radius, path.primitives[0]);
      return spatial;
    }
  }
  const double middle = begin + length / 2.0;
  const auto witness = primitive.point(middle);
  const auto blocked = [&]() {
    blocked_witness = witness;
    blocked_arclength = middle;
    return Status::Blocked;
  };
  // A point strictly inside one occupied square (or strictly outside the
  // domain) proves this connector blocked. The margin requires separation
  // from every relevant boundary, leaving contact semantics to the oracle.
  if (witness.x < -0.5 - margin || witness.y < -0.5 - margin ||
      witness.x > width_ - 0.5 + margin || witness.y > height_ - 0.5 + margin)
    return blocked();
  if (witness.x > -0.5 && witness.y > -0.5 &&
      witness.x < width_ - 0.5 && witness.y < height_ - 0.5) {
    const auto x = static_cast<std::size_t>(std::floor(witness.x + 0.5));
    const auto y = static_cast<std::size_t>(std::floor(witness.y + 0.5));
    if (map_->get(x, y) < 1.0 &&
        std::abs(witness.x - static_cast<double>(x)) < 0.5 - margin &&
        std::abs(witness.y - static_cast<double>(y)) < 0.5 - margin)
      return blocked();
  }
  if (length <= 0.0625 || depth >= 16) {
    if (filter_only)
      return Status::Indeterminate;
    ++range_checks;
    const auto result = audit::continuousPathIntervalClearance(
        *map_, original_from, primitive.radius, path.primitives, path.primitive_count,
        {index, begin, end});
    countExact(result);
    if (result.status == Status::Blocked) {
      blocked_witness = result.witness;
      blocked_arclength = result.witness_arclength;
    }
    if (eligible && result.status == Status::Clear)
      retainFirstArcClear(primitive.from, primitive.radius, path.primitives[0]);
    return result.status;
  }
  ++subdivisions;
  const auto left = enclose(primitive, begin, middle, begin_point, witness, depth + 1,
                            original_from, path, index, margin, cardinal_free);
  if (left == Status::Blocked) return left;
  const auto right = enclose(primitive, middle, end, witness, end_point, depth + 1,
                             original_from, path, index, margin, cardinal_free);
  if (right == Status::Blocked) return right;
  const auto result = left == Status::Clear && right == Status::Clear
                          ? Status::Clear : Status::Indeterminate;
  if (eligible && result == Status::Clear)
    retainFirstArcClear(primitive.from, primitive.radius, path.primitives[0]);
  return result;
}

audit::ContinuousClearanceStatus ConnectorClearance::check(
    const State from, const double radius,
    const audit::LegacyPathAuditResult &path, FirstArcClear *const owned,
    const audit::ReplayPrimitiveBuffer *const prepared) {
  ++queries;
  if (map_ == nullptr || !path.replayed() || !(radius > 0.0) ||
      !std::isfinite(radius) || !std::isfinite(from.x) ||
      !std::isfinite(from.y) || !std::isfinite(from.theta) ||
      path.primitive_count > path.primitives.size())
    return audit::ContinuousClearanceStatus::Invalid;
  for (std::size_t i = 0; i < path.primitive_count; ++i) {
    const auto &leg = path.primitives[i];
    if (!(leg.length >= 0.0) || !std::isfinite(leg.length) ||
        (leg.direction != -1 && leg.direction != 1) ||
        (leg.motion_type != 'L' && leg.motion_type != 'R' && leg.motion_type != 'S'))
      return audit::ContinuousClearanceStatus::Invalid;
  }
  // The double enclosure requires bounded angle arithmetic. In particular,
  // adding a small sweep to an arbitrary finite heading can lose the entire
  // sweep. Defer such words to the original-geometry checker, without wrapping
  // their literal input. For at most five primitives and total angle <= 64*pi,
  // 1e-10 times the global length/coordinate scale conservatively covers the
  // accumulated arithmetic and ordinary libm errors on the supported build.
  double angle_bound = std::abs(from.theta);
  double total_length = 0.0;
  for (std::size_t i = 0; i < path.primitive_count; ++i) {
    const auto &leg = path.primitives[i];
    total_length += leg.length;
    if (leg.motion_type != 'S') angle_bound += leg.length / radius;
  }
  const double scale = std::max({1.0, std::abs(from.x), std::abs(from.y),
                                 radius, total_length});
  const bool bounded = angle_bound <= 64.0 * std::numbers::pi &&
                       std::isfinite(total_length) && scale <= 1e100;
  const double margin = 1e-10 * scale;
  const auto same_double_bits = [](const double a, const double b) {
    return std::bit_cast<std::uint64_t>(a) == std::bit_cast<std::uint64_t>(b);
  };
  const bool use_prepared = prepared != nullptr && prepared->valid &&
      prepared->primitive_count == path.primitive_count &&
      same_double_bits(prepared->source.x, from.x) &&
      same_double_bits(prepared->source.y, from.y) &&
      same_double_bits(prepared->source.theta, from.theta) &&
      same_double_bits(prepared->radius, radius) &&
      std::equal(path.primitives.begin(),
                 path.primitives.begin() + static_cast<std::ptrdiff_t>(path.primitive_count),
                 prepared->literal_primitives.begin(),
                 [&](const audit::LegacyPrimitive &path_leg,
                     const audit::LegacyPrimitive &prepared_leg) {
                   return path_leg.motion_type == prepared_leg.motion_type &&
                       path_leg.direction == prepared_leg.direction &&
                       same_double_bits(path_leg.length, prepared_leg.length);
                 });
  State pose = from;
  const auto exact_before = exact_checks;
  bool enclosed = path.primitive_count > 0 && bounded;
  // Bounds apply to the literal primitive sequence, including accumulated
  // segment endpoints. Do not snap intermediate endpoints to requested poses.
  for (std::size_t i = 0; i < path.primitive_count && enclosed; ++i) {
    const auto *const cached = use_prepared ? &prepared->primitives[i] : nullptr;
    const PreparedPrimitive primitive{pose, radius, path.primitives[i], cached};
    State literal_end;
    const auto end_point = cached != nullptr ? cached->enclosure_endpoint :
        primitive.point(path.primitives[i].length,
            i + 1 < path.primitive_count ? &literal_end : nullptr);
    if (cached != nullptr && i + 1 < path.primitive_count)
      literal_end = cached->literal_endpoint;
    const auto &leg = path.primitives[i];
    const bool owned_first_arc = i == 0 && owned != nullptr && leg.length > 0.0 &&
        (leg.motion_type == 'L' || leg.motion_type == 'R');
    const std::size_t owned_index =
        (leg.motion_type == 'R' ? 2U : 0U) + (leg.direction > 0 ? 1U : 0U);
    bool owned_covered = false;
    if (owned_first_arc) {
      ++owned_first_arc_lookups;
      if (leg.length <= owned->length[owned_index]) {
        ++owned_first_arc_hits;
        owned_covered = true;
      }
    }
    const auto status = owned_covered ? audit::ContinuousClearanceStatus::Clear
                                      : enclose(primitive, 0.0, leg.length, pose, end_point,
                                                0, from, path, i, margin);
    if (status == audit::ContinuousClearanceStatus::Blocked) {
      filtered_blocked += exact_checks == exact_before;
      blocked_primitive = i;
      return status;
    }
    enclosed = status == audit::ContinuousClearanceStatus::Clear;
    if (enclosed && owned_first_arc)
      owned->length[owned_index] = std::max(owned->length[owned_index], leg.length);
    // Only the next enclosure reads this local pose; final and unresolved poses are dead.
    if (enclosed && i + 1 < path.primitive_count) {
      pose = literal_end;
      if (!use_prepared) {
        double angle = std::fmod(pose.theta + std::numbers::pi, 2.0 * std::numbers::pi);
        if (angle < 0.0) angle += 2.0 * std::numbers::pi;
        pose.theta = angle - std::numbers::pi;
      }
    }
  }
  if (enclosed) {
    // filtered means this call required no new exact checker work; a retained
    // first-arc Clear certificate may therefore contribute here.
    filtered += exact_checks == exact_before;
    return audit::ContinuousClearanceStatus::Clear;
  }
  if (filter_only)
    return audit::ContinuousClearanceStatus::Indeterminate;
  ++whole_checks;
  const auto result = audit::continuousPathClearance(
      *map_, from, radius, path.primitives, path.primitive_count);
  countExact(result);
  if (result.status == audit::ContinuousClearanceStatus::Blocked) {
    blocked_witness = result.witness;
    blocked_primitive = result.primitive_index;
    blocked_arclength = result.witness_arclength;
  }
  return result.status;
}

void ConnectorClearance::countExact(const audit::ContinuousClearanceResult &result) {
  ++exact_checks;
  intervals += result.visited_intervals;
  events += result.visited_events;
  free_box_attempts += result.free_box_attempts;
  free_box_cells += result.free_box_cells;
  free_box_clears += result.free_box_clears;
  refinements += result.precision_refinements;
  ++exact_statuses[static_cast<std::size_t>(result.status)];
}

} // namespace RS_marching::geometry
