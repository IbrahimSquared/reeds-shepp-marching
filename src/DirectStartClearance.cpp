#include "audit/DirectStartClearance.hpp"
#include "FreeSpaceClearance.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>
#include <type_traits>
#include <boost/multiprecision/cpp_bin_float.hpp>

namespace RS_marching::audit {
namespace {

template <class Real>
class ClearanceWalker {
 public:
  using Long = Real;

static inline const Long kRoundoffFactor = Long(4096);

[[nodiscard]] static Long pi() {
  if constexpr (std::is_same_v<Long, long double>) {
    return std::acos(-1.0L);
  } else {
    using std::acos;
    return acos(Long(-1));
  }
}

static_assert(std::numeric_limits<Long>::digits >= 64);
static_assert(std::numeric_limits<Long>::radix == 2);
static_assert(!std::is_same_v<Long, long double> ||
              std::numeric_limits<Long>::is_iec559);

[[nodiscard]] static bool realFinite(const Long &value) {
  using std::isfinite;
  return isfinite(value);
}
[[nodiscard]] static Long realAbs(const Long &value) {
  using std::abs;
  return abs(value);
}
[[nodiscard]] static Long realRound(const Long &value) {
  using std::round;
  return round(value);
}
[[nodiscard]] static Long realFloor(const Long &value) {
  using std::floor;
  return floor(value);
}
[[nodiscard]] static Long realCeil(const Long &value) {
  using std::ceil;
  return ceil(value);
}
[[nodiscard]] static Long realSin(const Long &value) {
  using std::sin;
  return sin(value);
}
[[nodiscard]] static Long realCos(const Long &value) {
  using std::cos;
  return cos(value);
}
[[nodiscard]] static Long realAsin(const Long &value) {
  using std::asin;
  return asin(value);
}
[[nodiscard]] static Long realAcos(const Long &value) {
  using std::acos;
  return acos(value);
}

struct LongPose {
  Long x = 0.0L;
  Long y = 0.0L;
  Long theta = 0.0L;
};

struct LocationHint {
  bool x_boundary = false;
  bool y_boundary = false;
  int x_lower_cell = 0;
  int y_lower_cell = 0;
};

struct CoordinateClass {
  bool uncertain = false;
  bool boundary = false;
  int index = 0;
};

struct TraceState {
  explicit TraceState(const Field<double> &occupancy_map) : map(occupancy_map) {}

  const Field<double> &map;
  ContinuousClearanceResult result;
  std::uint8_t primitive_index = ContinuousClearanceResult::kNoPrimitive;
  // Work used to locate a sliced interval cannot disappear from its numerical
  // error budget when large prefix coordinates or lengths later cancel.
  Long roundoff_scale = 1.0L;
};

struct LineEventStream {
  bool active = false;
  int lower_cell = 0;
  int step = 0;
  Long next = std::numeric_limits<Long>::infinity();
  Long delta = std::numeric_limits<Long>::infinity();
};

struct ArcGeometry {
  Long center_x = 0.0L;
  Long center_y = 0.0L;
  Long radius = 0.0L;
  Long theta0 = 0.0L;
  Long kappa = 0.0L;
  Long heading_rate = 0.0L;
  Long roundoff_scale = 1.0L;
};

struct ArcEventStream {
  bool active = false;
  bool x_axis = false;
  int lower_cell = 0;
  int step = 0;
  Long boundary = 0.0L;
  Long next = std::numeric_limits<Long>::infinity();
};

enum class RootStatus : std::uint8_t {
  Missing = 0,
  Valid = 1,
  Uncertain = 2
};

[[nodiscard]] static Long magnitude(const std::initializer_list<Long> values) {
  Long scale = 1.0L;
  for (const Long &value : values) {
    if (realFinite(value)) {
      scale = std::max(scale, realAbs(value));
    }
  }
  return scale;
}

[[nodiscard]] static Long errorEnvelope(
    const std::initializer_list<Long> values, const Long inherited_scale = Long(1)) {
  return kRoundoffFactor * std::numeric_limits<Long>::epsilon() *
         std::max(magnitude(values), inherited_scale);
}

[[nodiscard]] static bool finitePose(const LongPose pose) {
  return realFinite(pose.x) && realFinite(pose.y) &&
         realFinite(pose.theta);
}

[[nodiscard]] static double wrapPi(double angle) {
  constexpr double pi = std::numbers::pi;
  constexpr double two_pi = 2.0 * pi;
  angle = std::fmod(angle + pi, two_pi);
  if (angle < 0.0) {
    angle += two_pi;
  }
  return angle - pi;
}

[[nodiscard]] static State narrowPose(const LongPose pose) {
  return State{static_cast<double>(pose.x), static_cast<double>(pose.y),
               wrapPi(static_cast<double>(pose.theta))};
}

[[nodiscard]] static bool occupied(const Field<double> &map, const int x,
                            const int y) {
  return x >= 0 && y >= 0 && x < static_cast<int>(map.nx()) &&
         y < static_cast<int>(map.ny()) && map.get(x, y) < 1.0;
}

static void fail(TraceState &state, const ContinuousClearanceStatus status,
          const ContinuousClearanceFeature feature, const LongPose witness,
          const Long arclength_begin, const Long arclength_end,
          const std::array<ClearanceCell, 4> &cells = {},
          const std::uint8_t cell_count = 0) {
  state.result.status = status;
  state.result.feature = feature;
  state.result.primitive_index = state.primitive_index;
  state.result.witness = narrowPose(witness);
  state.result.primitive_arclength_begin =
      static_cast<double>(arclength_begin);
  state.result.primitive_arclength_end = static_cast<double>(arclength_end);
  state.result.incident_cells = cells;
  state.result.incident_cell_count = cell_count;
}

[[nodiscard]] static CoordinateClass classifyCoordinate(const Long value,
                                                 const Long error) {
  CoordinateClass result;
  const Long nearest_lower = realRound(value - 0.5L);
  if (nearest_lower < static_cast<Long>(std::numeric_limits<int>::min()) ||
      nearest_lower > static_cast<Long>(std::numeric_limits<int>::max())) {
    result.uncertain = true;
    return result;
  }
  const Long boundary = nearest_lower + 0.5L;
  const Long difference = value - boundary;
  if (difference == 0.0L || realAbs(difference) <= error) {
    result.uncertain = true;
    return result;
  }
  const Long cell = realFloor(value + 0.5L);
  if (cell < static_cast<Long>(std::numeric_limits<int>::min()) ||
      cell > static_cast<Long>(std::numeric_limits<int>::max())) {
    result.uncertain = true;
    return result;
  }
  result.index = static_cast<int>(cell);
  return result;
}

[[nodiscard]] static LocationHint exactSuppliedHint(const LongPose pose) {
  LocationHint hint;
  const auto set = [](const Long value, bool &boundary, int &lower_cell) {
    const Long nearest_lower = realRound(value - 0.5L);
    if (nearest_lower < static_cast<Long>(std::numeric_limits<int>::min()) ||
        nearest_lower > static_cast<Long>(std::numeric_limits<int>::max()) ||
        value != nearest_lower + 0.5L) {
      return;
    }
    boundary = true;
    lower_cell = static_cast<int>(nearest_lower);
  };
  set(pose.x, hint.x_boundary, hint.x_lower_cell);
  set(pose.y, hint.y_boundary, hint.y_lower_cell);
  return hint;
}

[[nodiscard]] static bool checkLocation(TraceState &state, const LongPose pose,
                                 const Long arclength_begin,
                                 const Long arclength_end,
                                 const LocationHint hint,
                                 const bool is_event,
                                 const Long witness_arclength) {
  state.result.witness_arclength = static_cast<double>(witness_arclength);
  if (is_event) {
    ++state.result.visited_events;
  } else {
    ++state.result.visited_intervals;
  }
  if (!finitePose(pose)) {
    fail(state, ContinuousClearanceStatus::Invalid,
         ContinuousClearanceFeature::InvalidInput, pose, arclength_begin,
         arclength_end);
    return false;
  }

  const Long width = static_cast<Long>(state.map.nx());
  const Long height = static_cast<Long>(state.map.ny());
  const Long domain_error = errorEnvelope(
      {pose.x, pose.y, width, height, arclength_begin, arclength_end}, state.roundoff_scale);
  const Long coordinate_error =
      errorEnvelope({pose.x, pose.y, arclength_begin, arclength_end}, state.roundoff_scale);

  const auto domainCoordinate = [&](const Long value, const Long lower,
                                    const Long upper) {
    if (value < lower) {
      return lower - value > domain_error ? -1 : 0;
    }
    if (value > upper) {
      return value - upper > domain_error ? -1 : 0;
    }
    return 1;
  };
  const int domain_x = domainCoordinate(pose.x, -0.5L, width - 0.5L);
  const int domain_y = domainCoordinate(pose.y, -0.5L, height - 0.5L);
  if (domain_x < 0 || domain_y < 0) {
    fail(state, ContinuousClearanceStatus::Blocked,
         ContinuousClearanceFeature::MapExterior, pose, arclength_begin,
         arclength_end);
    return false;
  }
  if (domain_x == 0 || domain_y == 0) {
    fail(state, ContinuousClearanceStatus::Indeterminate,
         ContinuousClearanceFeature::NumericalBoundary, pose,
         arclength_begin, arclength_end);
    return false;
  }

  CoordinateClass x_class;
  if (hint.x_boundary) {
    x_class.boundary = true;
    x_class.index = hint.x_lower_cell;
  } else {
    x_class = classifyCoordinate(pose.x, coordinate_error);
  }
  CoordinateClass y_class;
  if (hint.y_boundary) {
    y_class.boundary = true;
    y_class.index = hint.y_lower_cell;
  } else {
    y_class = classifyCoordinate(pose.y, coordinate_error);
  }
  if (x_class.uncertain || y_class.uncertain) {
    fail(state, ContinuousClearanceStatus::Indeterminate,
         ContinuousClearanceFeature::NumericalBoundary, pose,
         arclength_begin, arclength_end);
    return false;
  }

  if (!x_class.boundary && !y_class.boundary) {
    const std::array<ClearanceCell, 4> cells{
        ClearanceCell{x_class.index, y_class.index}};
    if (occupied(state.map, x_class.index, y_class.index)) {
      fail(state, ContinuousClearanceStatus::Blocked,
           ContinuousClearanceFeature::OccupiedCellInterior, pose,
           arclength_begin, arclength_end, cells, 1);
      return false;
    }
    return true;
  }

  if (x_class.boundary && y_class.boundary) {
    const std::array<ClearanceCell, 4> cells{
        ClearanceCell{x_class.index, y_class.index},
        ClearanceCell{x_class.index + 1, y_class.index},
        ClearanceCell{x_class.index, y_class.index + 1},
        ClearanceCell{x_class.index + 1, y_class.index + 1}};
    if (occupied(state.map, cells[0].x, cells[0].y) &&
        occupied(state.map, cells[1].x, cells[1].y) &&
        occupied(state.map, cells[2].x, cells[2].y) &&
        occupied(state.map, cells[3].x, cells[3].y)) {
      fail(state, ContinuousClearanceStatus::Blocked,
           ContinuousClearanceFeature::OccupiedVertex, pose,
           arclength_begin, arclength_end, cells, 4);
      return false;
    }
    return true;
  }

  const std::array<ClearanceCell, 4> cells =
      x_class.boundary
          ? std::array<ClearanceCell, 4>{
                ClearanceCell{x_class.index, y_class.index},
                ClearanceCell{x_class.index + 1, y_class.index}}
          : std::array<ClearanceCell, 4>{
                ClearanceCell{x_class.index, y_class.index},
                ClearanceCell{x_class.index, y_class.index + 1}};
  if (occupied(state.map, cells[0].x, cells[0].y) &&
      occupied(state.map, cells[1].x, cells[1].y)) {
    fail(state, ContinuousClearanceStatus::Blocked,
         ContinuousClearanceFeature::OccupiedSharedEdge, pose,
         arclength_begin, arclength_end, cells, 2);
    return false;
  }
  return true;
}

[[nodiscard]] static LongPose linePose(const LongPose start, const Long velocity_x,
                                const Long velocity_y,
                                const Long arclength) {
  return LongPose{start.x + arclength * velocity_x,
                  start.y + arclength * velocity_y, start.theta};
}

[[nodiscard]] static LineEventStream makeLineStream(const Long coordinate,
                                             const Long velocity) {
  LineEventStream stream;
  if (velocity == 0.0L) {
    return stream;
  }
  stream.active = true;
  stream.step = velocity > 0.0L ? 1 : -1;
  if (stream.step > 0) {
    stream.lower_cell = static_cast<int>(realFloor(coordinate + 0.5L));
  } else {
    stream.lower_cell = static_cast<int>(realCeil(coordinate - 0.5L)) - 1;
  }
  const Long boundary = static_cast<Long>(stream.lower_cell) + 0.5L;
  stream.next = (boundary - coordinate) / velocity;
  stream.delta = 1.0L / realAbs(velocity);
  return stream;
}

static void advanceLineStream(LineEventStream &stream) {
  stream.lower_cell += stream.step;
  stream.next += stream.delta;
}

[[nodiscard]] static bool traceStraight(TraceState &state, const LongPose start,
                                 const LegacyPrimitive &primitive,
                                 LongPose &end, Long length = Long(-1),
                                 const bool supplied_start = true,
                                 const LocationHint inherited_hint = {}) {
  if (length < 0) length = static_cast<Long>(primitive.length);
  if (length == 0) {
    end = start;
    return true;
  }
  const Long gear = static_cast<Long>(primitive.direction);
  const Long velocity_x = gear * realCos(start.theta);
  const Long velocity_y = gear * realSin(start.theta);
  end = linePose(start, velocity_x, velocity_y, length);

  LineEventStream x_stream = makeLineStream(start.x, velocity_x);
  LineEventStream y_stream = makeLineStream(start.y, velocity_y);
  LocationHint constant_hint = supplied_start ? exactSuppliedHint(start) : inherited_hint;
  if (velocity_x != 0.0L) {
    constant_hint.x_boundary = false;
  }
  if (velocity_y != 0.0L) {
    constant_hint.y_boundary = false;
  }
  Long previous = 0.0L;
  LocationHint end_hint = constant_hint;
  const std::size_t event_limit =
      2U * (state.map.nx() + state.map.ny()) + 16U;
  std::size_t events = 0;

  while (true) {
    const Long next_x = x_stream.active
                            ? x_stream.next
                            : std::numeric_limits<Long>::infinity();
    const Long next_y = y_stream.active
                            ? y_stream.next
                            : std::numeric_limits<Long>::infinity();
    const Long next = std::min(next_x, next_y);
    if (!realFinite(next) || next > length) {
      break;
    }
    const Long parameter_error =
        errorEnvelope({next_x, next_y, previous, length}, state.roundoff_scale);
    if (next <= previous) {
      if (previous - next <= parameter_error && next != previous) {
        fail(state, ContinuousClearanceStatus::Indeterminate,
             ContinuousClearanceFeature::NumericalBoundary,
             linePose(start, velocity_x, velocity_y, previous), previous,
             previous);
        return false;
      }
      if (next_x <= previous) {
        advanceLineStream(x_stream);
      }
      if (next_y <= previous) {
        advanceLineStream(y_stream);
      }
      continue;
    }
    if (++events > event_limit) {
      fail(state, ContinuousClearanceStatus::Indeterminate,
           ContinuousClearanceFeature::NumericalBoundary,
           linePose(start, velocity_x, velocity_y, previous), previous,
           previous);
      return false;
    }

    const bool x_event = next_x == next;
    const bool y_event = next_y == next;
    // A rounded tie is not proof that two events in inherited geometry are
    // simultaneous. Refine before assigning two exact boundary hints.
    if (state.roundoff_scale > 1 && x_event && y_event) {
      fail(state, ContinuousClearanceStatus::Indeterminate,
           ContinuousClearanceFeature::NumericalBoundary,
           linePose(start, velocity_x, velocity_y, next), next, next);
      return false;
    }
    if ((!x_event && realAbs(next_x - next) <= parameter_error) ||
        (!y_event && realAbs(next_y - next) <= parameter_error)) {
      fail(state, ContinuousClearanceStatus::Indeterminate,
           ContinuousClearanceFeature::NumericalBoundary,
           linePose(start, velocity_x, velocity_y, next), next, next);
      return false;
    }

    const Long midpoint = (previous + next) / 2.0L;
    if (!checkLocation(state,
                       linePose(start, velocity_x, velocity_y, midpoint),
                       previous, next, constant_hint, false, midpoint)) {
      return false;
    }
    LocationHint hint = constant_hint;
    if (x_event) {
      hint.x_boundary = true;
      hint.x_lower_cell = x_stream.lower_cell;
    }
    if (y_event) {
      hint.y_boundary = true;
      hint.y_lower_cell = y_stream.lower_cell;
    }
    if (!checkLocation(state, linePose(start, velocity_x, velocity_y, next),
                       next, next, hint, true, next)) {
      return false;
    }
    if (x_event) {
      advanceLineStream(x_stream);
    }
    if (y_event) {
      advanceLineStream(y_stream);
    }
    if (next == length) {
      end_hint = hint;
    }
    previous = next;
  }

  if (previous < length) {
    const Long midpoint = (previous + length) / 2.0L;
    if (!checkLocation(state,
                       linePose(start, velocity_x, velocity_y, midpoint),
                       previous, length, constant_hint, false, midpoint)) {
      return false;
    }
  }
  return checkLocation(state, end, length, length, end_hint, true, length);
}

[[nodiscard]] static Long curvature(const char motion_type) {
  if (motion_type == 'L') {
    return 1.0L;
  }
  if (motion_type == 'R') {
    return -1.0L;
  }
  return 0.0L;
}

[[nodiscard]] static LongPose arcPose(const ArcGeometry &arc,
                               const Long normalized_arclength) {
  const Long theta =
      arc.theta0 + arc.heading_rate * normalized_arclength;
  return LongPose{arc.center_x + arc.kappa * arc.radius * realSin(theta),
                  arc.center_y - arc.kappa * arc.radius * realCos(theta),
                  theta};
}

[[nodiscard]] static RootStatus liftRoot(const Long base, const ArcGeometry &arc,
                                  const Long chunk_begin,
                                  const Long chunk_end, Long &root) {
  const Long pi = ClearanceWalker::pi();
  const Long two_pi = 2.0L * pi;
  const Long theta_begin =
      arc.theta0 + arc.heading_rate * chunk_begin;
  const Long theta_end = arc.theta0 + arc.heading_rate * chunk_end;
  const Long lower = std::min(theta_begin, theta_end);
  const Long upper = std::max(theta_begin, theta_end);
  const Long angle_error =
      errorEnvelope({base, lower, upper, chunk_begin, chunk_end}, arc.roundoff_scale);
  const Long first_shift = realFloor((lower - base) / two_pi);
  bool found = false;
  Long selected = 0.0L;
  for (int offset = -1; offset <= 3; ++offset) {
    const Long candidate =
        base + (first_shift + static_cast<Long>(offset)) * two_pi;
    if (candidate < lower || candidate > upper) {
      if ((candidate < lower && lower - candidate <= angle_error) ||
          (candidate > upper && candidate - upper <= angle_error)) {
        return RootStatus::Uncertain;
      }
      continue;
    }
    const Long parameter =
        (candidate - arc.theta0) / arc.heading_rate;
    if (parameter <= chunk_begin || parameter > chunk_end) {
      if ((parameter < chunk_begin &&
           chunk_begin - parameter <= angle_error) ||
          (parameter > chunk_end && parameter - chunk_end <= angle_error)) {
        return RootStatus::Uncertain;
      }
      continue;
    }
    if (!found) {
      found = true;
      selected = parameter;
      continue;
    }
    if (realAbs(parameter - selected) > angle_error) {
      return RootStatus::Uncertain;
    }
  }
  if (!found) {
    return RootStatus::Missing;
  }
  root = selected;
  return RootStatus::Valid;
}

[[nodiscard]] static RootStatus solveArcRoot(const ArcGeometry &arc,
                                      const bool x_axis,
                                      const Long boundary,
                                      const Long chunk_begin,
                                      const Long chunk_end, Long &root) {
  Long ratio = x_axis
                   ? arc.kappa * (boundary - arc.center_x) / arc.radius
                   : arc.kappa * (arc.center_y - boundary) / arc.radius;
  const Long ratio_error = errorEnvelope(
      {ratio, boundary, arc.center_x, arc.center_y, arc.radius}, arc.roundoff_scale);
  if (ratio < -1.0L || ratio > 1.0L) {
    if (realAbs(realAbs(ratio) - 1.0L) <= ratio_error) {
      return RootStatus::Uncertain;
    }
    return RootStatus::Missing;
  }
  if (realAbs(realAbs(ratio) - 1.0L) <= ratio_error) {
    return RootStatus::Uncertain;
  }
  ratio = std::clamp(ratio, Long(-1), Long(1));

  const Long pi = ClearanceWalker::pi();
  const Long principal = x_axis ? realAsin(ratio) : realAcos(ratio);
  const std::array<Long, 2> bases =
      x_axis ? std::array<Long, 2>{principal, pi - principal}
             : std::array<Long, 2>{principal, -principal};
  bool found = false;
  Long selected = 0.0L;
  for (const Long &base : bases) {
    Long candidate = 0.0L;
    const RootStatus status =
        liftRoot(base, arc, chunk_begin, chunk_end, candidate);
    if (status == RootStatus::Uncertain) {
      return status;
    }
    if (status == RootStatus::Missing) {
      continue;
    }
    if (!found) {
      found = true;
      selected = candidate;
      continue;
    }
    const Long parameter_error =
        errorEnvelope({selected, candidate, chunk_begin, chunk_end}, arc.roundoff_scale);
    if (realAbs(candidate - selected) > parameter_error) {
      return RootStatus::Uncertain;
    }
  }
  if (!found) {
    return RootStatus::Missing;
  }
  root = selected;
  return RootStatus::Valid;
}

[[nodiscard]] static bool initializeArcStream(
    TraceState &state, const ArcGeometry &arc, const bool x_axis,
    const Long coordinate_begin, const Long coordinate_end,
    const Long chunk_begin, const Long chunk_end, ArcEventStream &stream) {
  const Long difference = coordinate_end - coordinate_begin;
  const Long coordinate_error = errorEnvelope(
      {coordinate_begin, coordinate_end, arc.radius, chunk_begin, chunk_end}, state.roundoff_scale);
  if (difference == 0.0L || realAbs(difference) <= coordinate_error) {
    return true;
  }
  stream.x_axis = x_axis;
  stream.step = difference > 0.0L ? 1 : -1;
  if (stream.step > 0) {
    stream.lower_cell =
        static_cast<int>(realFloor(coordinate_begin + 0.5L));
  } else {
    stream.lower_cell =
        static_cast<int>(realCeil(coordinate_begin - 0.5L)) - 1;
  }
  stream.boundary = static_cast<Long>(stream.lower_cell) + 0.5L;
  const Long endpoint_delta =
      stream.step > 0 ? stream.boundary - coordinate_end
                      : coordinate_end - stream.boundary;
  if (endpoint_delta > 0.0L) {
    if (endpoint_delta <= coordinate_error) {
      fail(state, ContinuousClearanceStatus::Indeterminate,
           ContinuousClearanceFeature::NumericalBoundary,
           arcPose(arc, chunk_end), arc.radius * chunk_end,
           arc.radius * chunk_end);
      return false;
    }
    return true;
  }

  const RootStatus root_status = solveArcRoot(
      arc, x_axis, stream.boundary, chunk_begin, chunk_end, stream.next);
  if (root_status != RootStatus::Valid) {
    fail(state, ContinuousClearanceStatus::Indeterminate,
         ContinuousClearanceFeature::NumericalBoundary,
         arcPose(arc, chunk_begin), arc.radius * chunk_begin,
         arc.radius * chunk_end);
    return false;
  }
  stream.active = true;
  return true;
}

[[nodiscard]] static bool advanceArcStream(TraceState &state,
                                    const ArcGeometry &arc,
                                    const Long coordinate_end,
                                    const Long chunk_begin,
                                    const Long chunk_end,
                                    ArcEventStream &stream) {
  stream.lower_cell += stream.step;
  stream.boundary = static_cast<Long>(stream.lower_cell) + 0.5L;
  const Long coordinate_error = errorEnvelope(
      {stream.boundary, coordinate_end, arc.radius, chunk_begin, chunk_end}, state.roundoff_scale);
  const Long endpoint_delta =
      stream.step > 0 ? stream.boundary - coordinate_end
                      : coordinate_end - stream.boundary;
  if (endpoint_delta > 0.0L) {
    if (endpoint_delta <= coordinate_error) {
      fail(state, ContinuousClearanceStatus::Indeterminate,
           ContinuousClearanceFeature::NumericalBoundary,
           arcPose(arc, chunk_end), arc.radius * chunk_end,
           arc.radius * chunk_end);
      return false;
    }
    stream.active = false;
    stream.next = std::numeric_limits<Long>::infinity();
    return true;
  }
  const RootStatus root_status = solveArcRoot(
      arc, stream.x_axis, stream.boundary, chunk_begin, chunk_end,
      stream.next);
  if (root_status != RootStatus::Valid) {
    fail(state, ContinuousClearanceStatus::Indeterminate,
         ContinuousClearanceFeature::NumericalBoundary,
         arcPose(arc, chunk_begin), arc.radius * chunk_begin,
         arc.radius * chunk_end);
    return false;
  }
  stream.active = true;
  return true;
}

[[nodiscard]] static bool traceArcChunk(TraceState &state, const ArcGeometry &arc,
                                 const Long chunk_begin,
                                 const Long chunk_end,
                                 LocationHint &end_hint) {
  const LongPose begin_pose = arcPose(arc, chunk_begin);
  const LongPose end_pose = arcPose(arc, chunk_end);
  ArcEventStream x_stream;
  ArcEventStream y_stream;
  if (!initializeArcStream(state, arc, true, begin_pose.x, end_pose.x,
                           chunk_begin, chunk_end, x_stream) ||
      !initializeArcStream(state, arc, false, begin_pose.y, end_pose.y,
                           chunk_begin, chunk_end, y_stream)) {
    return false;
  }

  Long previous = chunk_begin;
  const std::size_t event_limit =
      2U * (state.map.nx() + state.map.ny()) + 16U;
  std::size_t events = 0;
  while (x_stream.active || y_stream.active) {
    const Long next_x = x_stream.active
                            ? x_stream.next
                            : std::numeric_limits<Long>::infinity();
    const Long next_y = y_stream.active
                            ? y_stream.next
                            : std::numeric_limits<Long>::infinity();
    const Long next = std::min(next_x, next_y);
    const Long parameter_error =
        errorEnvelope({next_x, next_y, previous, chunk_end}, state.roundoff_scale);
    if (!realFinite(next) || next <= previous || next > chunk_end) {
      fail(state, ContinuousClearanceStatus::Indeterminate,
           ContinuousClearanceFeature::NumericalBoundary,
           arcPose(arc, previous), arc.radius * previous,
           arc.radius * previous);
      return false;
    }
    if (++events > event_limit) {
      fail(state, ContinuousClearanceStatus::Indeterminate,
           ContinuousClearanceFeature::NumericalBoundary,
           arcPose(arc, previous), arc.radius * previous,
           arc.radius * previous);
      return false;
    }

    const bool x_event = next_x == next;
    const bool y_event = next_y == next;
    if (state.roundoff_scale > 1 && x_event && y_event) {
      fail(state, ContinuousClearanceStatus::Indeterminate,
           ContinuousClearanceFeature::NumericalBoundary,
           arcPose(arc, next), arc.radius * next, arc.radius * next);
      return false;
    }
    if ((!x_event && realAbs(next_x - next) <= parameter_error) ||
        (!y_event && realAbs(next_y - next) <= parameter_error)) {
      fail(state, ContinuousClearanceStatus::Indeterminate,
           ContinuousClearanceFeature::NumericalBoundary,
           arcPose(arc, next), arc.radius * next, arc.radius * next);
      return false;
    }

    const Long midpoint = (previous + next) / 2.0L;
    if (!checkLocation(state, arcPose(arc, midpoint), arc.radius * previous,
                       arc.radius * next, {}, false, arc.radius * midpoint)) {
      return false;
    }
    const LocationHint hint{x_event, y_event, x_stream.lower_cell,
                            y_stream.lower_cell};
    if (!checkLocation(state, arcPose(arc, next), arc.radius * next,
                       arc.radius * next, hint, true, arc.radius * next)) {
      return false;
    }
    if (x_event &&
        !advanceArcStream(state, arc, end_pose.x, chunk_begin, chunk_end,
                          x_stream)) {
      return false;
    }
    if (y_event &&
        !advanceArcStream(state, arc, end_pose.y, chunk_begin, chunk_end,
                          y_stream)) {
      return false;
    }
    if (next == chunk_end) {
      end_hint = hint;
    }
    previous = next;
  }

  if (previous < chunk_end) {
    const Long midpoint = (previous + chunk_end) / 2.0L;
    if (!checkLocation(state, arcPose(arc, midpoint), arc.radius * previous,
                       arc.radius * chunk_end, {}, false, arc.radius * midpoint)) {
      return false;
    }
  }
  return true;
}

[[nodiscard]] static bool traceArc(TraceState &state, const LongPose start,
                            const LegacyPrimitive &primitive,
                            const Long radius, LongPose &end,
                            Long length = Long(-1)) {
  if (length < 0) length = static_cast<Long>(primitive.length);
  if (length == 0) {
    end = start;
    return true;
  }
  const Long kappa = curvature(primitive.motion_type);
  const Long gear = static_cast<Long>(primitive.direction);
  const Long total = length / radius;
  const ArcGeometry arc{start.x - kappa * radius * realSin(start.theta),
                        start.y + kappa * radius * realCos(start.theta),
                        radius,
                        start.theta,
                        kappa,
                        gear * kappa,
                        state.roundoff_scale};
  end = arcPose(arc, total);

  const Long half_pi = ClearanceWalker::pi() / Long(2);
  const Long raw_limit = realCeil(total / half_pi) + 4.0L;
  if (!realFinite(raw_limit) ||
      raw_limit > static_cast<Long>(std::numeric_limits<std::size_t>::max())) {
    fail(state, ContinuousClearanceStatus::Indeterminate,
         ContinuousClearanceFeature::NumericalBoundary, start, 0.0L,
         length);
    return false;
  }
  const std::size_t chunk_limit = static_cast<std::size_t>(raw_limit);
  std::size_t chunks = 0;
  Long begin = 0.0L;
  LocationHint final_hint;
  while (begin < total) {
    if (++chunks > chunk_limit) {
      fail(state, ContinuousClearanceStatus::Indeterminate,
           ContinuousClearanceFeature::NumericalBoundary,
           arcPose(arc, begin), radius * begin, radius * begin);
      return false;
    }
    const Long theta = arc.theta0 + arc.heading_rate * begin;
    Long next_theta = 0.0L;
    if (arc.heading_rate > 0.0L) {
      next_theta =
          (realFloor(theta / half_pi) + 1.0L) * half_pi;
    } else {
      next_theta = (realCeil(theta / half_pi) - 1.0L) * half_pi;
    }
    Long end_parameter =
        begin + (next_theta - theta) / arc.heading_rate;
    end_parameter = std::min(total, end_parameter);
    if (end_parameter <= begin) {
      fail(state, ContinuousClearanceStatus::Indeterminate,
           ContinuousClearanceFeature::NumericalBoundary,
           arcPose(arc, begin), radius * begin, radius * begin);
      return false;
    }
    LocationHint chunk_end_hint;
    if (!traceArcChunk(state, arc, begin, end_parameter, chunk_end_hint)) {
      return false;
    }
    if (end_parameter == total) {
      final_hint = chunk_end_hint;
    }
    begin = end_parameter;
  }
  return checkLocation(state, end, length, length, final_hint, true, length);
}

[[nodiscard]] static bool validPrimitive(const LegacyPrimitive &primitive) {
  return (primitive.motion_type == 'L' || primitive.motion_type == 'S' ||
          primitive.motion_type == 'R') &&
         (primitive.direction == -1 || primitive.direction == 1) &&
         realFinite(primitive.length) && primitive.length >= 0.0;
}

public:
[[nodiscard]] static ContinuousClearanceResult run(
    const Field<double> &occupancy, const State source, const double radius,
    const std::array<LegacyPrimitive, 5> &primitives,
    const std::size_t primitive_count, const PrimitiveInterval *interval = nullptr) {
  ContinuousClearanceResult invalid;
  if (occupancy.nx() == 0 || occupancy.ny() == 0 ||
      occupancy.nx() >
          static_cast<std::size_t>(std::numeric_limits<int>::max()) ||
      occupancy.ny() >
          static_cast<std::size_t>(std::numeric_limits<int>::max()) ||
      !realFinite(source.x) || !realFinite(source.y) ||
      !realFinite(source.theta) || !realFinite(radius) || radius <= 0.0 ||
      primitive_count > primitives.size()) {
    return invalid;
  }

  TraceState state{occupancy};
  LongPose pose{source.x, source.y, source.theta};
  if (primitive_count == 0) {
    fail(state, ContinuousClearanceStatus::Incomplete,
         ContinuousClearanceFeature::MissingBranch, pose, 0.0L, 0.0L);
    return state.result;
  }

  for (std::size_t index = 0; index < primitive_count; ++index) {
    if (!validPrimitive(primitives[index])) {
      state.primitive_index = static_cast<std::uint8_t>(index);
      fail(state, ContinuousClearanceStatus::Invalid,
           ContinuousClearanceFeature::InvalidInput, pose, 0.0L, 0.0L);
      return state.result;
    }
  }
  if (interval != nullptr) {
    if (interval->primitive >= primitive_count ||
        !realFinite(interval->begin) || !realFinite(interval->end) ||
        interval->begin < 0.0 || interval->end < interval->begin ||
        interval->end > primitives[interval->primitive].length)
      return invalid;
    // Advance the original word without clearance work. Never round the
    // interval's new starting pose back to double before checking it.
    LocationHint interval_hint = exactSuppliedHint(pose);
    const auto advance = [&](LongPose start, const LegacyPrimitive &leg, const Long length) {
      if (length == 0) return start;
      const Long sweep = leg.motion_type == 'S' ? Long(0) : length / Long(radius);
      const Long scale = magnitude({start.x, start.y, Long(radius), length});
      // Carry position, heading-addition and radius normalization work into
      // every later coordinate/event predicate at the current precision.
      state.roundoff_scale += scale * (Long(1) + realAbs(start.theta) + sweep) *
                              std::max(Long(1), Long(1) / Long(radius));
      if (leg.motion_type == 'S') {
        const Long vx = Long(leg.direction) * realCos(start.theta);
        const Long vy = Long(leg.direction) * realSin(start.theta);
        // Only a coordinate that provably never moved retains the supplied
        // contact hint. Never promote a rounded computed equality to a hint.
        if (vx != 0) interval_hint.x_boundary = false;
        if (vy != 0) interval_hint.y_boundary = false;
        return linePose(start, vx, vy, length);
      }
      interval_hint = {};
      const Long kappa = curvature(leg.motion_type);
      const ArcGeometry arc{start.x - kappa * Long(radius) * realSin(start.theta),
                            start.y + kappa * Long(radius) * realCos(start.theta),
                            Long(radius), start.theta, kappa, Long(leg.direction) * kappa};
      return arcPose(arc, length / Long(radius));
    };
    for (std::size_t i = 0; i < interval->primitive; ++i)
      pose = advance(pose, primitives[i], Long(primitives[i].length));
    const auto &leg = primitives[interval->primitive];
    pose = advance(pose, leg, Long(interval->begin));
    state.primitive_index = static_cast<std::uint8_t>(interval->primitive);
    if (!realFinite(state.roundoff_scale)) {
      fail(state, ContinuousClearanceStatus::Indeterminate,
           ContinuousClearanceFeature::NumericalBoundary, pose,
           Long(interval->begin), Long(interval->end));
      return state.result;
    }
    const bool supplied = interval->primitive == 0 && interval->begin == 0.0;
    if (!checkLocation(state, pose, Long(interval->begin), Long(interval->begin),
                       interval_hint, true, Long(interval->begin)))
      return state.result;
    LongPose end;
    const Long length = Long(interval->end) - Long(interval->begin);
    const bool traced = leg.motion_type == 'S'
        ? traceStraight(state, pose, leg, end, length, supplied, interval_hint)
        : traceArc(state, pose, leg, Long(radius), end, length);
    if (!traced) {
      state.result.primitive_arclength_begin += interval->begin;
      state.result.primitive_arclength_end += interval->begin;
      state.result.witness_arclength += interval->begin;
      return state.result;
    }
    state.result.status = ContinuousClearanceStatus::Clear;
    state.result.feature = ContinuousClearanceFeature::None;
    return state.result;
  }
  state.primitive_index = 0;
  if (!checkLocation(state, pose, 0.0L, 0.0L, exactSuppliedHint(pose), true, 0.0L)) {
    return state.result;
  }

  for (std::size_t index = 0; index < primitive_count; ++index) {
    state.primitive_index = static_cast<std::uint8_t>(index);
    LongPose next;
    const bool traced = primitives[index].motion_type == 'S'
                            ? traceStraight(state, pose, primitives[index], next)
                            : traceArc(state, pose, primitives[index],
                                       static_cast<Long>(radius), next);
    if (!traced) {
      return state.result;
    }
    pose = next;
  }

  state.result.status = ContinuousClearanceStatus::Clear;
  state.result.feature = ContinuousClearanceFeature::None;
  return state.result;
}

};

} // namespace

static ContinuousClearanceResult pathClearanceImpl(
    const Field<double> &occupancy, const State source, const double radius,
    const std::array<LegacyPrimitive, 5> &primitives,
    const std::size_t primitive_count, const PrimitiveInterval *interval) {
  const auto before_original = detail::FreeBoxFP::capture();
  const auto ordinary = ClearanceWalker<long double>::run(
      occupancy, source, radius, primitives, primitive_count, interval);
  if (ordinary.status != ContinuousClearanceStatus::Indeterminate) {
    return ordinary;
  }
  using High = boost::multiprecision::number<
      boost::multiprecision::cpp_bin_float<100, boost::multiprecision::backends::digit_base_10,
                                          void, std::int32_t, -16382, 16383>,
      boost::multiprecision::et_off>;
  const auto refined = ClearanceWalker<High>::run(
      occupancy, source, radius, primitives, primitive_count, interval);
  ContinuousClearanceResult result = refined;
  result.precision_refinements = 1;
  result.refined_intervals = refined.visited_intervals;
  result.refined_events = refined.visited_events;
  result.visited_intervals += ordinary.visited_intervals;
  result.visited_events += ordinary.visited_events;
  const auto after_original = detail::FreeBoxFP::capture();
  if (result.status == ContinuousClearanceStatus::Indeterminate) {
    ++result.free_box_attempts;
    const auto box = detail::wholeWordFreeBox(occupancy, source, radius, primitives,
        primitive_count, interval, before_original, after_original);
    const auto after_box = detail::FreeBoxFP::capture();
    result.free_box_cells += box.cells;
    if (box.clear && after_box.supported() && after_original.sameControls(after_box)) {
      ContinuousClearanceResult clear;
      clear.status = ContinuousClearanceStatus::Clear;
      clear.feature = ContinuousClearanceFeature::None;
      clear.visited_intervals = result.visited_intervals;
      clear.visited_events = result.visited_events;
      clear.precision_refinements = result.precision_refinements;
      clear.refined_intervals = result.refined_intervals;
      clear.refined_events = result.refined_events;
      clear.free_box_attempts = result.free_box_attempts;
      clear.free_box_cells = result.free_box_cells;
      clear.free_box_clears = 1;
      return clear;
    }
  }

  return result;
}

ContinuousClearanceResult continuousPathClearance(
    const Field<double> &occupancy, const State source, const double radius,
    const std::array<LegacyPrimitive, 5> &primitives, const std::size_t primitive_count) {
  return pathClearanceImpl(occupancy, source, radius, primitives, primitive_count, nullptr);
}

ContinuousClearanceResult continuousPathIntervalClearance(
    const Field<double> &occupancy, const State source, const double radius,
    const std::array<LegacyPrimitive, 5> &primitives, const std::size_t primitive_count,
    const PrimitiveInterval interval) {
  return pathClearanceImpl(occupancy, source, radius, primitives, primitive_count, &interval);
}

ResolvedChainAuditResult auditResolvedChain(
    const Field<double> &occupancy, const std::vector<State> &poses,
    const double accepted_distance, const double radius,
    const std::vector<std::uint32_t> *const words) {
  ResolvedChainAuditResult result;
  if (!std::isfinite(accepted_distance) || accepted_distance < 0.0 ||
      !std::isfinite(radius) || radius <= 0.0) {
    return result;
  }
  if (poses.size() < 2) {
    result.status = ResolvedChainStatus::Incomplete;
    return result;
  }
  if (words != nullptr && words->size() != poses.size() - 1) {
    result.status = ResolvedChainStatus::Incomplete;
    return result;
  }
  if (std::any_of(poses.begin(), poses.end(), [](const State &pose) {
        return !std::isfinite(pose.x) || !std::isfinite(pose.y) ||
               !std::isfinite(pose.theta);
      })) {
    return result;
  }

  result.segments.reserve(poses.size() - 1);
  result.segment_sum = 0.0;
  result.status = ResolvedChainStatus::Clear;
  const auto retain = [&](const ResolvedChainStatus status) {
    result.status = std::max(result.status, status);
  };
  for (std::size_t index = 0; index + 1 < poses.size(); ++index) {
    LegacyPathAuditResult path;
    if (words != nullptr) {
      path = reconstructRetainedLeg(poses[index], poses[index + 1], radius,
                                    words->at(index));
    } else {
      Solver solver;
      double distance = std::numeric_limits<double>::infinity();
      int condition = 0;
      int quadrant = 0;
      solver.acceleratedRSPlanner(poses[index], poses[index + 1], distance,
                                  radius, condition, quadrant);
      path = replaySelectedLegacyPath(solver, poses[index], poses[index + 1],
                                      distance, radius, condition, quadrant);
    }
    ContinuousClearanceResult clearance;
    if (path.replayed()) {
      clearance = continuousPathClearance(occupancy, poses[index], radius,
                                          path.primitives,
                                          path.primitive_count);
      constexpr std::array clearance_status{
          ResolvedChainStatus::Invalid, ResolvedChainStatus::Incomplete,
          ResolvedChainStatus::Indeterminate, ResolvedChainStatus::Blocked,
          ResolvedChainStatus::Clear};
      retain(clearance_status.at(static_cast<std::size_t>(clearance.status)));
    } else if (path.status ==
               LegacyReplayStatus::PrimitiveMaterializationFailed) {
      retain(ResolvedChainStatus::Unsupported);
    } else if (path.status == LegacyReplayStatus::ReplayMismatch) {
      retain(ResolvedChainStatus::Incomplete);
    } else {
      retain(ResolvedChainStatus::Invalid);
    }
    result.segment_sum += path.distance;
    result.segments.push_back(ResolvedSegmentAudit{path, clearance});
  }

  result.complete_distance_residual =
      std::abs(result.segment_sum - accepted_distance);
  const double distance_scale =
      std::max(radius, std::abs(accepted_distance));
  if (!std::isfinite(result.segment_sum) ||
      result.complete_distance_residual > kReplayTolerance * distance_scale) {
    retain(ResolvedChainStatus::Incomplete);
  }
  return result;
}

} // namespace RS_marching::audit
