#pragma once
// Literal helpers shared by the one-to-all loop and its query modules.
#include "marching/Marching.hpp"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <type_traits>
namespace RS_marching {
template <typename T, typename Value = std::identity>
void saveField(const RS_marching::Field<T> &field, const std::string &name,
               Value value = {}) {
  std::string outputFilePath = "./output/" + name + ".txt";
  // Check if the directory exists, and create it if it doesn't
  namespace fs = std::filesystem;
  fs::path directory = fs::path(outputFilePath).parent_path();
  if (!fs::exists(directory)) {
    if (!fs::create_directories(directory)) {
      std::cerr << "Failed to create directory " << directory.string()
                << std::endl;
      return;
    }
  }
  try {
    std::ofstream file(outputFilePath);
    // Round-trip precision, so identity checks see every bit of the field.
    file << std::setprecision(
        std::numeric_limits<std::decay_t<decltype(value(field(0, 0)))>>::max_digits10);
    for (size_t j = 0; j < field.ny(); ++j) {
      for (size_t i = 0; i < field.nx(); ++i) {
        file << value(field(i, j)) << " ";
      }
      file << "\n";
    }
    std::cout << "Saved " << name << std::endl;
  } catch (const std::exception &e) {
    std::cerr << "Failed to open output file " << outputFilePath << std::endl;
    return;
  }
}

template <typename T> auto durationInMicroseconds(T start, T end) {
  return std::chrono::duration_cast<std::chrono::microseconds>(end - start)
      .count();
}

inline double wrapToPi(double angle) {
  angle = fmod(angle + pi, 2 * pi);
  if (angle < 0)
    angle += 2 * pi;
  return angle - pi;
}

inline double wrapTo2Pi(double angle) {
  while (angle < 0) {
    angle += 2 * pi;
  }
  while (angle >= 2 * pi) {
    angle -= 2 * pi;
  }
  return angle;
}

} // namespace RS_marching
