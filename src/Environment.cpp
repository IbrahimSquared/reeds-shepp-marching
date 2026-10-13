#include "environment/Environment.hpp"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace RS_marching {
namespace {
void validateDimensions(size_t nx, size_t ny) {
  if (nx == 0 || ny == 0 || nx > static_cast<size_t>(std::numeric_limits<int>::max()) ||
      ny > static_cast<size_t>(std::numeric_limits<int>::max()) ||
      nx > std::numeric_limits<size_t>::max() / sizeof(double) / ny)
    throw std::invalid_argument("environment dimensions must be positive and nonoverflowing");
}
} // namespace

Environment::Environment(Config &config)
    : sharedConfig_(std::make_shared<Config>(config)) {
  validateConfig(*sharedConfig_);
  if (sharedConfig_->mapSource == MapSource::Random) {
    nx_ = sharedConfig_->ncols;
    ny_ = sharedConfig_->nrows;

    speedValue_ = sharedConfig_->speedValue;
    if (!sharedConfig_->randomSeed) {
      seedValue_ = sharedConfig_->seedValue;
    }
    generateNewEnvironmentFromSettings();
    if (sharedConfig_->saveResults) {
      saveEnvironment();
    }
  } else if (sharedConfig_->mapSource == MapSource::Image) {
    if (loadImage(sharedConfig_->imagePath)) {
      if (sharedConfig_->saveResults) {
        saveEnvironment();
      }
    } else {
      throw std::runtime_error("Failed to load image: " + sharedConfig_->imagePath);
    }
  }
}

Environment::Environment(Config &config,
                         const Field<std::uint8_t> &occupancy)
    : ny_(occupancy.ny()), nx_(occupancy.nx()),
      sharedConfig_(std::make_shared<Config>(config)) {
  validateConfig(*sharedConfig_);
  validateDimensions(nx_, ny_);
  resetEnvironment();
  for (size_t y = 0; y < ny_; ++y) {
    const auto *occupancyRow = occupancy.row_ptr(y);
    auto *visibilityRow = sharedVisibilityField_->row_ptr(y);
    for (size_t x = 0; x < nx_; ++x) {
      if (occupancyRow[x] == 0) {
        visibilityRow[x] = 0.0;
      }
    }
  }
  std::cout << "Loaded image of dimensions " << nx_ << "x" << ny_
            << " successfully" << std::endl;
  if (sharedConfig_->saveResults) {
    saveEnvironment();
  }
}

void Environment::generateNewEnvironmentFromSettings() {
  validateConfig(*sharedConfig_);
  validateDimensions(nx_, ny_);
  resetEnvironment();
  int seedValue = 0;
  if (!sharedConfig_->randomSeed) {
    seedValue = seedValue_;
  } else {
    // Get the current time point using the high resolution clock
    auto time_point = std::chrono::high_resolution_clock::now();
    // Convert the time point to nanoseconds since the epoch
    auto ns_since_epoch =
        std::chrono::time_point_cast<std::chrono::nanoseconds>(time_point)
            .time_since_epoch()
            .count();
    seedValue = ns_since_epoch;
  }
  std::srand(seedValue);

  for (int i = 0; i < sharedConfig_->nb_of_obstacles; ++i) {
    size_t col_1 = 1 + (std::rand() % (nx_ - 0 + 1));
    size_t col_2 =
        col_1 + sharedConfig_->minWidth +
        (std::rand() % (sharedConfig_->maxWidth - sharedConfig_->minWidth + 1));
    col_1 = std::min(col_1, nx_ - 1);
    col_2 = std::min(col_2, nx_ - 1);

    size_t row_1 = 1 + (std::rand() % (ny_ - 0 + 1));
    size_t row_2 = row_1 + sharedConfig_->minHeight +
                (std::rand() %
                 (sharedConfig_->maxHeight - sharedConfig_->minHeight + 1));
    row_1 = std::min(row_1, ny_ - 1);
    row_2 = std::min(row_2, ny_ - 1);

    for (size_t j = col_1; j < col_2; ++j) {
      for (size_t k = row_1; k < row_2; ++k) {
        sharedVisibilityField_->set(j, k, 0.0);
      }
    }
  }

  if (!sharedConfig_->silent) {
    std::cout << "########################### Environment output "
                 "############################ \n"
              << "Generated new environment based on parsed settings at a seed "
                 "value of: "
              << seedValue << std::endl;
  }
}

bool Environment::loadImage(const std::string &filename) {
  validateConfig(*sharedConfig_);
  sf::Image image;
  // Load the image from a file
  if (!image.loadFromFile(filename)) {
    std::cout << "Error: Failed to load image" << std::endl;
    return false;
  } else {
    auto size = image.getSize();
    validateDimensions(size.x, size.y);
    nx_ = size.x;
    ny_ = size.y;

    resetEnvironment();

    sf::Color pixel;
    // Access the pixel data of the image
    for (size_t x = 0; x < nx_; ++x) {
      for (size_t y = 0; y < ny_; ++y) {
        pixel = image.getPixel(x, y);
        const int gray = 0.3 * pixel.r + 0.59 * pixel.g + 0.11 * pixel.b;
        if (gray > 128) {
          sharedVisibilityField_->set(x, y, 1.0);
        } else {
          sharedVisibilityField_->set(x, y, 0);
        }
      }
    }
    std::cout << "Loaded image of dimensions " << nx_ << "x" << ny_
              << " successfully" << std::endl;
    return true;
  }
}

void Environment::resetEnvironment() {
  sharedVisibilityField_ = std::make_unique<Field<double>>(nx_, ny_, 1.0);
}

void Environment::saveEnvironment() {
  // Define the path to the output file
  std::string outputFilePath = "./output/visibilityField.txt";
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

  // save visibility field
  if (sharedConfig_->saveVisibilityField) {
    std::fstream of(outputFilePath, std::ios::out | std::ios::trunc);
    if (!of.is_open()) {
      std::cerr << "Failed to open output file " << outputFilePath << std::endl;
      return;
    }
    std::ostream &os = of;
    for (int j = ny_ - 1; j >= 0; --j) {
      for (size_t i = 0; i < nx_; ++i) {
        os << sharedVisibilityField_->get(i, j) << " ";
      }
      os << "\n";
    }
    of.close();
    if (!sharedConfig_->silent) {
      std::cout << "Saved visibility field" << std::endl;
    }
  }

  outputFilePath = "./output/speedField.txt";
  if (sharedConfig_->saveSpeedField) {
    // save speed field
    std::fstream of(outputFilePath, std::ios::out | std::ios::trunc);
    if (!of.is_open()) {
      std::cerr << "Failed to open output file " << outputFilePath << std::endl;
      return;
    }
    std::ostream &os = of;
    for (size_t i = 0; i < nx_; ++i) {
      for (size_t j = 0; j < ny_; ++j) {
        // Preserve the legacy x-major speed export, including image source's
        // fixed obstacle speed (2.0) rather than the random-map setting.
        os << (sharedVisibilityField_->get(i, j) == 0.0 ? speedValue_ : 1.0) << " ";
      }
      os << "\n";
    }
    of.close();
    if (!sharedConfig_->silent) {
      std::cout << "Saved speed field" << std::endl;
    }
  }
}

} // namespace RS_marching
