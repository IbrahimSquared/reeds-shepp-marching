#pragma once

#include <cmath>

namespace RS_marching::detail {

// Callers pass their already-evaluated original dividend and the fixed positive,
// finite normal angular period (2pi).  For |x| < p fmod is x; for p <= |x| <
// 2p Sterbenz makes abs(x)-p exact, and copysign retains a negative zero.  The
// other cases retain std::fmod, including nonfinite dividends and |x| >= 2p.
[[nodiscard, gnu::always_inline]] inline double
boundedAngleRemainder(const double dividend, const double period) {
  const double magnitude = std::abs(dividend);
  if (magnitude < period) return dividend;
  if (magnitude < 2.0 * period) {
    return std::copysign(magnitude - period, dividend);
  }
  return std::fmod(dividend, period);
}

// Callers pass the original evaluated dividend and fixed pi or 2pi period.
// Exact half and 1.5-period thresholds preserve nearest-even quotient ties:
// half uses quotient zero, while 1.5 falls back to std::remainder.  Sterbenz
// makes abs(x)-p exact in between; the explicit copysign keeps signed zero.
[[nodiscard, gnu::always_inline]] inline double
boundedNearestAngleRemainder(const double dividend, const double period) {
  const double magnitude = std::abs(dividend);
  if (magnitude <= period / 2.0) return dividend;
  if (magnitude < 1.5 * period) {
    const double difference = magnitude - period;
    if (difference == 0.0) return std::copysign(0.0, dividend);
    return std::signbit(dividend) ? -difference : difference;
  }
  return std::remainder(dividend, period);
}

} // namespace RS_marching::detail
