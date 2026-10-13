#ifndef QUERY_RETAINED_REPLAY_MEMO_HPP
#define QUERY_RETAINED_REPLAY_MEMO_HPP

#include "audit/DirectStartAudit.hpp"

#include <array>
#include <bit>
#include <cfenv>
#include <cmath>
#include <cstdint>

namespace RS_marching::query::detail {

// Private search-local scratch: exact reconstruction inputs only, never scene,
// field, clearance or prepared geometry. Copies do not rerun floating-point
// operations; floating-point exception flags/traps are not part of this memo's
// result contract. A rounding-mode change discards all prior entries.
class RetainedReplayMemo {
public:
  [[nodiscard]] audit::LegacyPathAuditResult replay(
      const State from, const State to, const double radius,
      const std::uint32_t word, bool &hit) {
    hit = false;
    const int rounding = std::fegetround();
    if (rounding != rounding_) {
      for (auto &entry : entries_) entry.valid = false;
      rounding_ = rounding;
    }
    const bool finite = std::isfinite(from.x) && std::isfinite(from.y) &&
        std::isfinite(from.theta) && std::isfinite(to.x) &&
        std::isfinite(to.y) && std::isfinite(to.theta) &&
        std::isfinite(radius) && radius > 0.0 && rounding != -1;
    if (!finite) return audit::reconstructRetainedLeg(from, to, radius, word);
    const auto key = makeKey(from, to, radius, word);
    auto &entry = entries_[slot(key)];
    if (entry.valid && entry.key == key) {
      hit = true;
      return entry.result;
    }
    const auto result = audit::reconstructRetainedLeg(from, to, radius, word);
    // Every finite-input miss replaces its slot; failed replays publish no result.
    entry.valid = false;
    if (result.replayed()) {
      entry.key = key;
      entry.result = result;
      entry.valid = true;
    }
    return result;
  }

  [[nodiscard]] static constexpr std::size_t entryBytes() { return sizeof(Entry); }

  // Exposes the production mapping for deterministic collision/eviction tests.
  [[nodiscard]] static std::size_t slotFor(
      State from, State to, double radius, std::uint32_t word) {
    return slot(makeKey(from, to, radius, word));
  }

private:
  using Key = std::array<std::uint64_t, 8>;
  struct Entry {
    Key key{};
    audit::LegacyPathAuditResult result{};
    bool valid = false;
  };
  [[nodiscard]] static Key makeKey(
      State from, State to, double radius, std::uint32_t word) {
    const auto bits = [](double v) { return std::bit_cast<std::uint64_t>(v); };
    return {bits(from.x), bits(from.y), bits(from.theta), bits(to.x),
            bits(to.y), bits(to.theta), bits(radius), word};
  }
  [[nodiscard]] static std::size_t slot(const Key &key) {
    std::uint64_t h = 0x9e3779b97f4a7c15ULL;
    for (const auto v : key) h = std::rotl(h, 9) ^ v;
    h ^= h >> 30; h *= 0xbf58476d1ce4e5b9ULL;
    h ^= h >> 27; h *= 0x94d049bb133111ebULL;
    h ^= h >> 31;
    return h & 15;
  }
  std::array<Entry, 16> entries_{};
  int rounding_ = -1;
};

} // namespace RS_marching::query::detail
#endif
