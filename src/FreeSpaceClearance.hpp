#pragma once
#include "audit/DirectStartClearance.hpp"
#include <algorithm>
#include <array>
#include <cfenv>
#include <cmath>
#include <cstddef>
#include <limits>
#if defined(__linux__) && defined(__x86_64__) && defined(__SSE2__) && defined(__GNUC__) && !defined(__clang__) && __GNUC__ == 13 && __GNUC_MINOR__ == 3 && !defined(__FAST_MATH__)
#define RS_FREE_BOX_SUPPORTED_BUILD 1
#include <fpu_control.h>
#include <xmmintrin.h>
#else
#define RS_FREE_BOX_SUPPORTED_BUILD 0
#endif
namespace RS_marching::audit::detail {
// Private sufficient whole-word condition. No pose evaluation or contact hints.
inline constexpr std::size_t kFreeBoxCellLimit = 65536;
struct FreeBoxFP {
  int round = -1;
  unsigned csr = 0;
  unsigned short x87 = 0;
  static FreeBoxFP capture() {
#if RS_FREE_BOX_SUPPORTED_BUILD
    fpu_control_t word; _FPU_GETCW(word);
    return {std::fegetround(), _mm_getcsr(), static_cast<unsigned short>(word)};
#else
    return {};
#endif
  }
  bool supported() const {
#if RS_FREE_BOX_SUPPORTED_BUILD
    return sizeof(double) == 8 && std::numeric_limits<double>::is_iec559 &&
      std::numeric_limits<double>::radix == 2 && std::numeric_limits<double>::digits == 53 &&
      std::numeric_limits<double>::has_denorm == std::denorm_present &&
      sizeof(int) == 4 && std::numeric_limits<int>::digits == 31 &&
      round == FE_TONEAREST && (csr & (3u << 13)) == 0 &&
      (csr & ((1u << 15) | (1u << 6))) == 0 &&
      (csr & (0x3fu << 7)) == (0x3fu << 7) &&
      (x87 & 0x3fu) == 0x3fu && (x87 & (3u << 10)) == 0;
#else
    return false;
#endif
  }
  bool sameControls(const FreeBoxFP &other) const {
    // Sticky exception flags are deliberately excluded.
    return round == other.round && (csr & ~0x3fu) == (other.csr & ~0x3fu) && x87 == other.x87;
  }
};
struct FreeBoxResult { bool clear = false; std::size_t cells = 0; };
inline bool boundedFreeBoxArea(std::size_t width, std::size_t height) {
  return width > 0 && height > 0 && width <= kFreeBoxCellLimit &&
    height <= kFreeBoxCellLimit / width;
}
inline FreeBoxResult wholeWordFreeBox(const Field<double> &map, State source, double radius,
    const std::array<LegacyPrimitive, 5> &primitives, std::size_t count,
    const PrimitiveInterval *interval, FreeBoxFP before_original, FreeBoxFP after_original) {
  FreeBoxResult result;
  const auto before = FreeBoxFP::capture();
  if (!before_original.supported() || !after_original.supported() || !before.supported() ||
      !before_original.sameControls(after_original) || !after_original.sameControls(before)) return result;
  if (!std::isfinite(source.x) || !std::isfinite(source.y) || !std::isfinite(source.theta) ||
      !std::isfinite(radius) || radius <= 0 || count == 0 || count > primitives.size()) return result;
  for (std::size_t i = 0; i < count; ++i) {
    const auto &p = primitives[i];
    if ((p.motion_type != 'S' && p.motion_type != 'L' && p.motion_type != 'R') ||
        (p.direction != -1 && p.direction != 1) || !std::isfinite(p.length) || p.length < 0) return result;
  }
  if (interval && (interval->primitive >= count || !std::isfinite(interval->begin) ||
      !std::isfinite(interval->end) || interval->begin < 0 || interval->end < interval->begin ||
      interval->end > primitives[interval->primitive].length)) return result;
  const auto width = map.nx(), height = map.ny();
  if (width == 0 || height == 0 || width > std::size_t(std::numeric_limits<int>::max()) ||
      height > std::size_t(std::numeric_limits<int>::max()) ||
      height > std::numeric_limits<std::size_t>::max() / width) return result;
  const auto storage_cells = width * height;
  if (storage_cells > std::numeric_limits<std::size_t>::max() / sizeof(double)) return result;
  // Initialized immutable backing storage of these dimensions is a Field precondition.
  const double infinity = std::numeric_limits<double>::infinity();
  double reach = 0;
  for (std::size_t i = 0; i < count; ++i) {
    double weight = primitives[i].length;
    if (primitives[i].motion_type != 'S') {
      const double twice_radius = 2.0 * radius;
      if (std::isnan(twice_radius) || twice_radius <= 0 ||
          (!std::isfinite(twice_radius) && twice_radius != infinity)) return result;
      weight = std::min(weight, twice_radius);
    }
    if (!std::isfinite(weight) || weight < 0) return result;
    const double sum = reach + weight;
    if (!std::isfinite(sum) || sum < 0) return result;
    const double outward = sum > 0 ? std::nextafter(sum, infinity) : 0.0;
    if (!std::isfinite(outward) || outward < 0) return result;
    reach = outward;
  }
  std::array<std::size_t, 2> lower{}, upper{};
  const std::array<double, 2> coordinates{source.x, source.y};
  const std::array<std::size_t, 2> dimensions{width, height};
  for (std::size_t axis = 0; axis < 2; ++axis) {
    const double raw_lo = coordinates[axis] - reach, raw_hi = coordinates[axis] + reach;
    if (!std::isfinite(raw_lo) || !std::isfinite(raw_hi)) return result;
    const double lo = std::nextafter(raw_lo, -infinity), hi = std::nextafter(raw_hi, infinity);
    if (!std::isfinite(lo) || !std::isfinite(hi) || lo > hi ||
        !(lo > -.5 && hi < static_cast<double>(dimensions[axis]) - .5)) return result;
    const double shift_lo = lo + .5, shift_hi = hi + .5;
    if (!std::isfinite(shift_lo) || !std::isfinite(shift_hi)) return result;
    const double arg_lo = std::nextafter(shift_lo, -infinity), arg_hi = std::nextafter(shift_hi, infinity);
    if (!std::isfinite(arg_lo) || !std::isfinite(arg_hi)) return result;
    const double floor_lo = std::floor(arg_lo), floor_hi = std::floor(arg_hi);
    if (!std::isfinite(floor_lo) || !std::isfinite(floor_hi) ||
        std::floor(floor_lo) != floor_lo || std::floor(floor_hi) != floor_hi ||
        floor_lo < 0 || floor_hi < floor_lo || floor_hi >= static_cast<double>(dimensions[axis]) ||
        floor_hi > static_cast<double>(std::numeric_limits<int>::max())) return result;
    lower[axis] = static_cast<std::size_t>(static_cast<int>(floor_lo));
    upper[axis] = static_cast<std::size_t>(static_cast<int>(floor_hi));
  }
  const auto span_x = upper[0] - lower[0] + 1, span_y = upper[1] - lower[1] + 1;
  if (!boundedFreeBoxArea(span_x, span_y)) return result;
  for (std::size_t y = lower[1];;) {
    if (y > std::numeric_limits<std::size_t>::max() / width) return result;
    const auto row = y * width;
    for (std::size_t x = lower[0];;) {
      if (x > std::numeric_limits<std::size_t>::max() - row || row + x >= storage_cells ||
          result.cells >= kFreeBoxCellLimit) return result;
      const double value = map.get(x, y); ++result.cells;
      if (!std::isfinite(value) || value < 1.) return result;
      if (x == upper[0]) break;
      if (x == std::numeric_limits<std::size_t>::max()) return result;
      ++x;
    }
    if (y == upper[1]) break;
    if (y == std::numeric_limits<std::size_t>::max()) return result;
    ++y;
  }
  const auto after = FreeBoxFP::capture();
  result.clear = after.supported() && before.sameControls(after);
  return result;
}
} // namespace RS_marching::audit::detail
#undef RS_FREE_BOX_SUPPORTED_BUILD
