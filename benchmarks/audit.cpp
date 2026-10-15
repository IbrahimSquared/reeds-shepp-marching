// Benchmark-only adapter to the unchanged continuous-clearance authority.
#include "audit/DirectStartClearance.hpp"
#include "thirdparty/nlohmann/json.hpp"
#include <SFML/Graphics/Image.hpp>
#include <iostream>
#include <string>
using Json = nlohmann::json;
using namespace RS_marching;
int main() {
  std::string line, cached;
  Field<double> map;
  while (std::getline(std::cin, line)) {
    Json out;
    try {
      const auto in = Json::parse(line);
      const auto file = in.at("map").get<std::string>();
      if (file != cached) {
        sf::Image image;
        if (!image.loadFromFile(file)) throw std::runtime_error("cannot read map");
        auto size = image.getSize();
        map.reset(size.x, size.y, 0.);
        for (unsigned y=0; y<size.y; ++y)
          for (unsigned x=0; x<size.x; ++x) map.set(x,y,image.getPixel(x,y).r > 128 ? 1. : 0.);
        cached=file;
      }
      out={{"status","clear"},{"checks",Json::array()}};
      for (const auto & p : in.at("primitives")) {
        const auto a=p.at("start").get<std::vector<double>>();
        if (a.size()!=3) throw std::runtime_error("start must have three coordinates");
        State start{a[0],a[1],a[2]};
        std::array<audit::LegacyPrimitive,5> primitives{};
        const auto type=p.at("type").get<std::string>();
        if (type.size()!=1) throw std::runtime_error("invalid motion type");
        primitives[0]={type[0],p.at("gear").get<int>(),p.at("length").get<double>()};
        const double radius=p.at("radius").get<double>();
        const auto result=audit::continuousPathClearance(map,start,radius,primitives,1);
        const auto end=audit::replayLegacyPrimitives(start,radius,primitives,1);
        const char * status=result.clear()?"clear":
          result.status==audit::ContinuousClearanceStatus::Blocked?"blocked":"uncertain";
        out["checks"].push_back({{"status",status},{"feature",static_cast<int>(result.feature)},
          {"end",{end.x,end.y,end.theta}},
          {"witness",{result.witness.x,result.witness.y,result.witness.theta}},
          {"witness_arclength",result.witness_arclength}});
        if (!result.clear()) out["status"]=status;
      }
    } catch (const std::exception & e) {out={{"status","error"},{"error",e.what()}};}
    std::cout << out.dump() << '\n' << std::flush;
  }
}
