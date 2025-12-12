#include "environment/Environment.hpp"
#include "marching/Marching.hpp"
#include <iostream>

int main() {
  RS_marching::ConfigParser &parser = RS_marching::ConfigParser::getInstance();
  if (!parser.parse("config/settings.config")) {
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
  // marching.computeEDF();
  marching.runMarching();

  // while (true) {
  //   marching.runMarching();
  //
  //   std::ofstream done("done.txt");
  //   done << "Done" << std::endl;
  //   done.close();
  //
  //   while (std::ifstream("done.txt")) {
  //     std::this_thread::sleep_for(std::chrono::milliseconds(200));
  //   }
  // }

  // while (true) {
  //   if (std::cin.get() == '\n') {
  //     marching.runMarching();
  //     // else if escape or q is pressed, break
  //   } else if (std::cin.get() == 'q') {
  //     break;
  //   }
  // }
  return 0;
}
