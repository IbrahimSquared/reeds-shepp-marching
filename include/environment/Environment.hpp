#ifndef ENVIRONMENT_HPP
#define ENVIRONMENT_HPP

#include <SFML/Graphics.hpp>
#include <cstdint>
#include <memory>
#include <vector>

#include "environment/Field.hpp"
#include "parser/Parser.hpp"

namespace RS_marching {

// Environment simulator
class Environment {
public:
  /*!
   * Constructor.
   * @brief Initialize an Environment.
   */
  explicit Environment(Config &config);

  // Create independent fields from nonempty binary occupancy (0 occupied, 1 free).
  Environment(Config &config, const Field<std::uint8_t> &occupancy);

  /*!
   * @brief Generate a random new Environment from parsed config settings.
   * Overwrites previously generated Environment.
   */
  void generateNewEnvironmentFromSettings();

  /*!
   * @brief Loads image data using SFML. Overwrites previously generated
   * Environment.
   * @param [in] filename Filename.
   */
  bool loadImage(const std::string &filename);

  // Get visibility field shared pointer.
  inline auto &getVisibilityField() const { return sharedVisibilityField_; };
  // Get parsed configuration.
  inline auto &getConfig() const { return sharedConfig_; };

  // Deconstructor
  ~Environment() = default;

private:
  size_t ny_;
  size_t nx_;
  double speedValue_ = 2.0;
  int seedValue_ = 1;

  // Shared pointer to a Field object of type double that has map occupancy
  // values.
  std::shared_ptr<Field<double>> sharedVisibilityField_;

  // Shared pointer to configuration
  std::shared_ptr<Config> sharedConfig_;

  void saveEnvironment();
  void resetEnvironment();
};

} // namespace RS_marching
#endif // ENVIRONMENT_HPP
