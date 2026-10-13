#include "environment/Environment.hpp"
#include "marching/Marching.hpp"
#include <chrono>
#include <exception>
#include <iostream>
#include <string>
#ifdef RS_COUNT_WORK
extern "C" void rs_heap_checkpoint(const char *) __attribute__((weak));
#endif

int main(int argc, char **argv) try {
  const auto initialization_start = std::chrono::steady_clock::now();
  if (argc > 2) {
    std::cerr << "Usage: " << argv[0] << " [config_path]" << std::endl;
    return 2;
  }
  const std::string config_path =
      argc == 2 ? argv[1] : "config/settings.config";
  RS_marching::ConfigParser &parser = RS_marching::ConfigParser::getInstance();
  if (!parser.parse(config_path)) {
    std::cout << "########################### Parsing results: ####"
                 "########################## \n";
    std::cout << "Error parsing config file" << std::endl;
    return 1;
  } else {
    std::cout << "########################### Parsing results: ####"
                 "########################## \n";
    std::cout << "Config file parsed successfully \n" << std::endl;
  }
  auto config = parser.getConfig();

  RS_marching::Environment env = RS_marching::Environment(config);

  RS_marching::Marching marching = RS_marching::Marching(env);
#ifdef RS_COUNT_WORK
  const auto initial_ars = RS_marching::Solver::query_count;
  const auto initial_urs = RS_marching::UnderSpecifiedRSPlanner::query_count;
  std::cout << "Initialization ARS/URS queries: " << initial_ars << '/'
            << initial_urs << '\n';
  if (rs_heap_checkpoint) rs_heap_checkpoint("initialized");
#endif
  std::cout << "Initialization time in us: "
            << std::chrono::duration_cast<std::chrono::microseconds>(
                   std::chrono::steady_clock::now() - initialization_start).count()
            << "us" << std::endl;
  marching.runMarching();
#ifdef RS_COUNT_WORK
  std::cout << "Live ARS/URS queries: "
            << RS_marching::Solver::query_count - initial_ars << '/'
            << RS_marching::UnderSpecifiedRSPlanner::query_count - initial_urs << '\n';
  if (rs_heap_checkpoint) rs_heap_checkpoint("finished");
#endif

  return 0;
} catch (const std::exception &error) {
  std::cerr << "Solver error: " << error.what() << '\n';
  return 1;
}
