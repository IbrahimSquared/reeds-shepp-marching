#ifndef FIELD_H
#define FIELD_H
#include <algorithm> // For std::fill_n
#include <cstddef>

namespace RS_marching {
template <typename T> class Field {
public:
  using size_t = std::size_t;

  // Default constructor
  Field() noexcept : nx_(0), ny_(0), size_(0), data_(nullptr) {}

  // Constructor with initialization
  explicit Field(const size_t nx, const size_t ny, const T default_value)
      : nx_(nx), ny_(ny), size_(nx * ny) {
    data_ = new T[size_];
    std::fill_n(data_, size_, default_value);
  }

  // Optimized row-major access
  // This is already row-major: index = x + y*nx
  // (x is the column index, y is the row index)
  // inline T &operator()(size_t x, size_t y) noexcept {
  //   return data_[y * nx_ + x]; // Explicit row-major form for clarity
  // }
  inline T &operator()(size_t x, size_t y) noexcept {
    return *(data_ + y * nx_ + x);
  }

  // inline const T &operator()(size_t x, size_t y) const noexcept {
  //   return data_[y * nx_ + x];
  // }
  inline T &operator()(size_t x, size_t y) const noexcept {
    return *(data_ + y * nx_ + x);
  }

  // Fast setters
  inline void set(const size_t x, const size_t y, const T value) noexcept {
    data_[y * nx_ + x] = value;
  }

  // Fast getters
  inline const T &get(const size_t x, const size_t y) const noexcept {
    return data_[y * nx_ + x];
  }

  // Row access for even more optimized operations
  inline T *row_ptr(size_t y) noexcept { return data_ + y * nx_; }

  inline const T *row_ptr(size_t y) const noexcept { return data_ + y * nx_; }

  // Inline getters for dimensions
  inline size_t nx() const noexcept { return nx_; }
  inline size_t ny() const noexcept { return ny_; }

  // Copy constructor
  Field(const Field &other)
      : nx_(other.nx_), ny_(other.ny_), size_(other.size_) {
    data_ = new T[size_];
    std::copy_n(other.data_, size_, data_);
  }

  // Copy assignment
  Field &operator=(const Field &other) {
    if (this != &other) {
      if (size_ != other.size_) {
        delete[] data_;
        data_ = new T[other.size_];
      }
      nx_ = other.nx_;
      ny_ = other.ny_;
      size_ = other.size_;
      std::copy_n(other.data_, size_, data_);
    }
    return *this;
  }

  // Move constructor
  Field(Field &&other) noexcept
      : nx_(other.nx_), ny_(other.ny_), size_(other.size_), data_(other.data_) {
    // Reset other to prevent double delete
    other.data_ = nullptr;
    other.nx_ = 0;
    other.ny_ = 0;
    other.size_ = 0;
  }

  // Move assignment
  Field &operator=(Field &&other) noexcept {
    if (this != &other) {
      delete[] data_;

      nx_ = other.nx_;
      ny_ = other.ny_;
      size_ = other.size_;
      data_ = other.data_;

      // Reset other to prevent double delete
      other.data_ = nullptr;
      other.nx_ = 0;
      other.ny_ = 0;
      other.size_ = 0;
    }
    return *this;
  }

  // Reset field with new dimensions
  void reset(const size_t nx, const size_t ny, const T default_value) {
    const size_t new_size = nx * ny;

    if (new_size != size_) {
      delete[] data_;
      data_ = new T[new_size];
    }

    nx_ = nx;
    ny_ = ny;
    size_ = new_size;
    std::fill_n(data_, size_, default_value);
  }

  // Highly optimized fill for rectangular regions
  void fill(const size_t firstx, const size_t lastx, const size_t firsty,
            const size_t lasty, const T value) {
    const size_t width = lastx - firstx;

    for (size_t y = firsty; y < lasty; ++y) {
      // Get direct pointer to start of this row segment
      T *row_segment = data_ + (y * nx_ + firstx);

      // Fill entire segment at once - this is cache-friendly in row-major order
      std::fill_n(row_segment, width, value);
    }
  }

  // Destructor
  ~Field() { delete[] data_; }

private:
  size_t nx_;   // Width (number of columns)
  size_t ny_;   // Height (number of rows)
  size_t size_; // Total size (nx * ny)
  T *data_;     // Raw pointer to data array
};
} // namespace RS_marching
#endif // FIELD_H
