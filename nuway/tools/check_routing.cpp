// Load a lanelet2 map offline, build the routing graph and say whether a route exists.
//
// Written because each full-stack run costs about 8 minutes, while this answers
// "is the map routable" in seconds. It uses exactly what Autoware uses:
// UtmProjector(origin) plus the Germany/Vehicle traffic rules.
//
// When route_handler reported "Failed to find a proper route!" this tool showed the
// map was fine - every lanelet had one successor and getRoute() succeeded. The real
// cause was route_handler's yaw_threshold = pi/2 check against the ego heading, i.e.
// the lane had been laid along the direction of travel while the vehicle was reversing.
//
// Build:
//   g++ -O1 -std=c++17 -o check_routing check_routing.cpp \
//       -I/opt/ros/humble/include -I/usr/include/eigen3 \
//       -L/opt/ros/humble/lib/aarch64-linux-gnu \
//       -llanelet2_core -llanelet2_io -llanelet2_projection -llanelet2_routing \
//       -llanelet2_traffic_rules -Wl,-rpath,/opt/ros/humble/lib/aarch64-linux-gnu
// Run:
//   ./check_routing map.osm <origin_lat> <origin_lon> [start_id] [goal_id]
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <iostream>

int main(int argc, char** argv) {
  if (argc < 4) {
    std::cerr << "usage: check_routing <osm> <origin_lat> <origin_lon> "
                 "[start_id] [goal_id]\n";
    return 2;
  }
  const std::string file = argv[1];
  const double lat = std::stod(argv[2]), lon = std::stod(argv[3]);

  lanelet::ErrorMessages errors;
  lanelet::projection::UtmProjector projector{lanelet::Origin({lat, lon})};
  auto map = lanelet::load(file, projector, &errors);
  std::cout << "loaded with " << errors.size() << " parse error(s)\n";
  for (size_t i = 0; i < errors.size() && i < 10; ++i) std::cout << "  " << errors[i] << "\n";

  std::cout << "lanelets: " << map->laneletLayer.size()
            << "   points: " << map->pointLayer.size()
            << "   linestrings: " << map->lineStringLayer.size() << "\n";

  auto rules = lanelet::traffic_rules::TrafficRulesFactory::create(
      lanelet::Locations::Germany, lanelet::Participants::Vehicle);
  auto graph = lanelet::routing::RoutingGraph::build(*map, *rules);

  std::cout << "\nid      passable  left first/last    right first/last   successors\n";
  std::cout << "---------------------------------------------------------------------\n";
  lanelet::Id lo = std::numeric_limits<lanelet::Id>::max(), hi = 0;
  for (const auto& ll : map->laneletLayer) {
    std::cout << ll.id() << "   " << (rules->canPass(ll) ? "yes" : "no ") << "        "
              << ll.leftBound().front().id() << "/" << ll.leftBound().back().id()
              << "            " << ll.rightBound().front().id() << "/"
              << ll.rightBound().back().id() << "           "
              << graph->following(ll).size() << "\n";
    lo = std::min(lo, ll.id());
    hi = std::max(hi, ll.id());
  }

  const lanelet::Id start_id = argc > 4 ? std::stoll(argv[4]) : lo;
  const lanelet::Id goal_id = argc > 5 ? std::stoll(argv[5]) : hi;
  std::cout << "\nroute " << start_id << " -> " << goal_id << ": ";
  auto route = graph->getRoute(map->laneletLayer.get(start_id),
                               map->laneletLayer.get(goal_id), 0);
  if (route) {
    std::cout << "found, " << route->shortestPath().size() << " lanelets\n";
    return 0;
  }
  std::cout << "NOT found\n";
  return 1;
}
