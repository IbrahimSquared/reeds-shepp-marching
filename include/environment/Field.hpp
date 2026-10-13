#ifndef FIELD_H
#define FIELD_H
#include <algorithm>
#include <cstddef>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>

namespace RS_marching {
// Contiguous row-major storage, including ordinary bool objects (not bit proxies).
template <typename T> class Field {
public:
  using size_t = std::size_t;

  Field() noexcept = default;
  explicit Field(const size_t nx, const size_t ny, const T default_value) {
    reset(nx, ny, default_value);
  }
  Field(const Field &other)
      : nx_(other.nx_), ny_(other.ny_), size_(other.size_), data_(allocate(size_)) {
    if (size_) std::copy_n(other.data_.get(), size_, data_.get());
  }
  Field &operator=(const Field &other) {
    if (this == &other) return *this;
    if (size_ != other.size_) {
      auto replacement = allocate(other.size_);
      if (other.size_) std::copy_n(other.data_.get(), other.size_, replacement.get());
      data_ = std::move(replacement);
    } else if (size_) {
      std::copy_n(other.data_.get(), size_, data_.get());
    }
    nx_ = other.nx_;
    ny_ = other.ny_;
    size_ = other.size_;
    return *this;
  }
  Field(Field &&other) noexcept
      : nx_(std::exchange(other.nx_, 0)), ny_(std::exchange(other.ny_, 0)),
        size_(std::exchange(other.size_, 0)), data_(std::move(other.data_)) {}
  Field &operator=(Field &&other) noexcept {
    if (this != &other) {
      data_ = std::move(other.data_);
      nx_ = std::exchange(other.nx_, 0);
      ny_ = std::exchange(other.ny_, 0);
      size_ = std::exchange(other.size_, 0);
    }
    return *this;
  }

  T &operator()(size_t x, size_t y) noexcept { return data_[y * nx_ + x]; }
  const T &operator()(size_t x, size_t y) const noexcept { return data_[y * nx_ + x]; }
  void set(size_t x, size_t y, T value) noexcept { data_[y * nx_ + x] = value; }
  const T &get(size_t x, size_t y) const noexcept { return data_[y * nx_ + x]; }
  T *row_ptr(size_t y) noexcept { return data_ ? data_.get() + y * nx_ : nullptr; }
  const T *row_ptr(size_t y) const noexcept { return data_ ? data_.get() + y * nx_ : nullptr; }
  size_t nx() const noexcept { return nx_; }
  size_t ny() const noexcept { return ny_; }

  void reset(const size_t nx, const size_t ny, const T default_value) {
    const size_t new_size = checkedSize(nx, ny);
    if (new_size != size_) {
      auto replacement = allocate(new_size);
      if (new_size) std::fill_n(replacement.get(), new_size, default_value);
      // Commit only after allocation and initialization succeed.
      data_ = std::move(replacement);
    } else if (size_) {
      std::fill_n(data_.get(), size_, default_value);
    }
    nx_ = nx;
    ny_ = ny;
    size_ = new_size;
  }
  void fill(size_t firstx, size_t lastx, size_t firsty, size_t lasty, T value) {
    if (firstx == lastx) return;
    for (size_t y = firsty; y < lasty; ++y)
      std::fill_n(row_ptr(y) + firstx, lastx - firstx, value);
  }

private:
  static size_t checkedSize(size_t nx, size_t ny) {
    constexpr size_t limit = std::numeric_limits<size_t>::max() / sizeof(T);
    if (ny && nx > limit / ny) throw std::length_error("field dimensions overflow");
    return nx * ny;
  }
  static std::unique_ptr<T[]> allocate(size_t size) {
    // Default-initialize once; reset supplies the requested values afterwards.
    return size ? std::unique_ptr<T[]>(new T[size]) : nullptr;
  }
  size_t nx_ = 0, ny_ = 0, size_ = 0;
  std::unique_ptr<T[]> data_;
};
} // namespace RS_marching
#endif // FIELD_H
