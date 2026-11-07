// test_travel_time.cpp -- speed -> travel-time conversion, the distance-table parser, graph construction and route
// planning from hand-made forecast snapshots (every ETA below is computed by hand in the comments).
//
// Build (from the repository root):
//   g++ -std=c++17 -O2 -Wall -Wextra -o build/test_travel_time tests/test_travel_time.cpp src/travel_time_graph.cpp src/route_planner.cpp src/speed_forecaster.cpp src/graph.cpp src/traffic_data.cpp src/features.cpp src/metrics.cpp src/linear_regression.cpp src/baseline_pipeline.cpp
// Run from the repository root:   build/test_travel_time
#include "routing_fixtures.h"

using namespace fx;
using tu::expect;
using tu::near;

static void testConversion() {
    expect("convert: 1 mph is 26.8224 m/min (= 1609.344 / 60)", near(routing::METERS_PER_MINUTE_PER_MPH, MILE_M / 60.0, 1e-12));
    expect("convert: 1609.344 m at 60 mph = exactly 1 minute", near(routing::travelTimeMinutes(1609.344, 60.0), 1.0, 1e-12));
    expect("convert: 3218.688 m at 60 mph = 2 minutes", near(routing::travelTimeMinutes(3218.688, 60.0), 2.0, 1e-12));
    expect("convert: 1609.344 m at 30 mph = 2 minutes (half the speed, double the time)", near(routing::travelTimeMinutes(1609.344, 30.0), 2.0, 1e-12));
    expect("convert: 4123.8 m at 45.5 mph matches the formula distance / (speed * 26.8224)", near(routing::travelTimeMinutes(4123.8, 45.5), 4123.8 / (45.5 * 26.8224), 1e-12));
    expect("convert: a very low but valid speed is accepted by default (no floor)", routing::travelTimeMinutes(1000.0, 0.01) > 0);
}

static void testInvalidSpeedsAndDistances() {
    using routing::travelTimeMinutes;
    expect("invalid speed: 0 is rejected (never replaced by zero)", throws<std::invalid_argument>([] { travelTimeMinutes(1000.0, 0.0); }));
    expect("invalid speed: negative is rejected", throws<std::invalid_argument>([] { travelTimeMinutes(1000.0, -5.0); }));
    expect("invalid speed: -0.0 is rejected", throws<std::invalid_argument>([] { travelTimeMinutes(1000.0, -0.0); }));
    expect("invalid speed: NaN is rejected", throws<std::invalid_argument>([] { travelTimeMinutes(1000.0, NaN); }));
    expect("invalid speed: +infinity is rejected", throws<std::invalid_argument>([] { travelTimeMinutes(1000.0, INF); }));
    expect("invalid speed: -infinity is rejected", throws<std::invalid_argument>([] { travelTimeMinutes(1000.0, -INF); }));
    expect("invalid distance: 0 is rejected", throws<std::invalid_argument>([] { travelTimeMinutes(0.0, 50.0); }));
    expect("invalid distance: negative is rejected", throws<std::invalid_argument>([] { travelTimeMinutes(-1.0, 50.0); }));
    expect("invalid distance: NaN is rejected", throws<std::invalid_argument>([] { travelTimeMinutes(NaN, 50.0); }));
    expect("invalid distance: infinity is rejected", throws<std::invalid_argument>([] { travelTimeMinutes(INF, 50.0); }));
    expect("overflow: a result that is not finite is rejected", throws<std::invalid_argument>([] { travelTimeMinutes(1000.0, 1e-320); }));
    expect("floor: a speed below the floor is rejected", throws<std::invalid_argument>([] { travelTimeMinutes(1000.0, 4.9, 5.0); }));
    expect("floor: a speed equal to the floor is accepted", near(travelTimeMinutes(1609.344, 5.0, 5.0), 60.0 / 5.0, 1e-12));
    expect("floor: a negative floor is rejected", throws<std::invalid_argument>([] { travelTimeMinutes(1000.0, 50.0, -1.0); }));
    expect("floor: a NaN floor is rejected", throws<std::invalid_argument>([] { travelTimeMinutes(1000.0, 50.0, NaN); }));
}

static std::vector<std::string> lines(std::initializer_list<const char*> l) { return std::vector<std::string>(l.begin(), l.end()); }

static void testDistanceParser() {
    const char* H = "from_sensor_id,to_sensor_id,distance_m";
    const std::vector<routing::DistanceRecord> ok = routing::parseSensorDistances(lines({H, "716328,716331,4123.8", "716331,716328,3900.5"}), "t");
    expect("parser: the real header and rows are read", ok.size() == 2 && ok[0].from == "716328" && ok[0].to == "716331" && near(ok[0].distanceM, 4123.8) && near(ok[1].distanceM, 3900.5));
    expect("parser: both directions of a pair are separate records (not duplicates)", ok.size() == 2);
    expect("parser: Windows line endings (CRLF) are tolerated", routing::parseSensorDistances(lines({"from_sensor_id,to_sensor_id,distance_m\r", "716328,716331,4123.8\r"}), "t").size() == 1);
    expect("parser: blank lines are skipped", routing::parseSensorDistances(lines({H, "", "716328,716331,10", ""}), "t").size() == 1);
    expect("parser: the incompatible 'from,to,cost' header is refused", throws<std::runtime_error>([&] { routing::parseSensorDistances(lines({"from,to,cost", "716328,716331,10"}), "t"); }));
    expect("parser: a header with an extra column is refused", throws<std::runtime_error>([&] { routing::parseSensorDistances(lines({"from_sensor_id,to_sensor_id,distance_m,extra", "716328,716331,10,1"}), "t"); }));
    expect("parser: an empty file is refused", throws<std::runtime_error>([&] { routing::parseSensorDistances(lines({}), "t"); }));
    expect("parser: a header with no rows is refused", throws<std::runtime_error>([&] { routing::parseSensorDistances(lines({H}), "t"); }));
    expect("parser: a row with the wrong number of fields is refused, naming the line", throwsWith<std::runtime_error>([&] { routing::parseSensorDistances(lines({H, "716328,716331,10", "716328,716337"}), "t"); }, "line 3"));
    expect("parser: an unknown sensor is refused", throwsWith<std::runtime_error>([&] { routing::parseSensorDistances(lines({H, "999999,716331,10"}), "t"); }, "999999"));
    expect("parser: an unknown destination sensor is refused", throws<std::runtime_error>([&] { routing::parseSensorDistances(lines({H, "716328,999999,10"}), "t"); }));
    expect("parser: a sensor with a distance to itself is refused", throws<std::runtime_error>([&] { routing::parseSensorDistances(lines({H, "716328,716328,10"}), "t"); }));
    expect("parser: distance 0 is refused", throws<std::runtime_error>([&] { routing::parseSensorDistances(lines({H, "716328,716331,0"}), "t"); }));
    expect("parser: a negative distance is refused", throws<std::runtime_error>([&] { routing::parseSensorDistances(lines({H, "716328,716331,-5"}), "t"); }));
    expect("parser: a non-numeric distance is refused", throws<std::runtime_error>([&] { routing::parseSensorDistances(lines({H, "716328,716331,abc"}), "t"); }));
    expect("parser: an empty distance is refused", throws<std::runtime_error>([&] { routing::parseSensorDistances(lines({H, "716328,716331,"}), "t"); }));
    expect("parser: NaN and infinity are refused", throws<std::runtime_error>([&] { routing::parseSensorDistances(lines({H, "716328,716331,nan"}), "t"); }) && throws<std::runtime_error>([&] { routing::parseSensorDistances(lines({H, "716328,716331,inf"}), "t"); }));
    expect("parser: the same directed pair twice is refused", throwsWith<std::runtime_error>([&] { routing::parseSensorDistances(lines({H, "716328,716331,10", "716328,716331,11"}), "t"); }, "twice"));
    expect("parser: a missing file is refused", throws<std::runtime_error>([] { routing::loadSensorDistances("data/this_file_does_not_exist.csv"); }));

    const std::vector<routing::DistanceRecord> real = routing::loadSensorDistances("data/sensor_distances.csv");     // the committed, validated file
    bool allPairs = real.size() == 30, positive = true;
    for (size_t a = 0; a < 6; ++a) for (size_t b = 0; b < 6; ++b) {
        if (a == b) continue;
        bool found = false;
        for (const routing::DistanceRecord& r : real) if (r.from == id(a) && r.to == id(b)) found = true;
        allPairs = allPairs && found;
    }
    for (const routing::DistanceRecord& r : real) positive = positive && r.distanceM > 0;
    expect("parser: the committed data/sensor_distances.csv has all 30 directed pairs with positive distances", allPairs && positive);
}

static void testGraphConstruction() {
    // sensors 0,1,2 at 60, 30 and 10 mph; one mile between each ordered pair
    const std::vector<routing::DistanceRecord> d = {rec(0, 1, MILE_M), rec(0, 2, 2 * MILE_M), rec(1, 0, MILE_M), rec(1, 2, MILE_M), rec(2, 0, MILE_M), rec(2, 1, MILE_M)};
    const routing::TravelTimeGraph g = routing::buildTravelTimeGraph(d, snapshot({60.0, 30.0, 10.0}));
    expect("graph: one edge per record, all three sensors are vertices", g.legs.size() == 6 && g.graph.vertexCount() == 3 && g.graph.edgeCount() == 6);
    expect("graph: edge 0->1 uses sensor 0's speed: 1 mile at 60 mph = 1 min", near(g.findLeg(id(0), id(1))->minutes, 1.0, 1e-12) && near(g.findLeg(id(0), id(1))->speedMph, 60.0));
    expect("graph: edge 0->2 also uses sensor 0's speed: 2 miles at 60 mph = 2 min (upstream-speed assumption)", near(g.findLeg(id(0), id(2))->minutes, 2.0, 1e-12) && near(g.findLeg(id(0), id(2))->speedMph, 60.0));
    expect("graph: edge 1->0 uses sensor 1's speed: 1 mile at 30 mph = 2 min", near(g.findLeg(id(1), id(0))->minutes, 2.0, 1e-12));
    expect("graph: edges 2->0 and 2->1 use sensor 2's speed: 1 mile at 10 mph = 6 min", near(g.findLeg(id(2), id(0))->minutes, 6.0, 1e-12) && near(g.findLeg(id(2), id(1))->minutes, 6.0, 1e-12));
    expect("graph: the two directions of a pair get different weights (directed)", g.findLeg(id(0), id(1))->minutes != g.findLeg(id(1), id(0))->minutes);
    expect("graph: findLeg returns nullptr for an edge that does not exist", g.findLeg(id(0), id(0)) == nullptr);
    expect("graph: no sensor is reported unavailable when all have forecasts", g.unavailableSensors.empty());

    const routing::TravelTimeGraph partial = routing::buildTravelTimeGraph(d, snapshot({60.0, NaN, 10.0}));
    expect("graph: a sensor without a forecast loses its outgoing edges (4 of 6 remain)", partial.legs.size() == 4 && partial.findLeg(id(1), id(0)) == nullptr && partial.findLeg(id(1), id(2)) == nullptr);
    expect("graph: edges INTO a sensor without a forecast are kept (they use the upstream sensor's speed)", partial.findLeg(id(0), id(1)) != nullptr && partial.findLeg(id(2), id(1)) != nullptr);
    expect("graph: the sensor without a forecast is listed with its reason and is still a vertex", partial.unavailableSensors.size() == 1 && partial.unavailableSensors[0].sensorId == id(1) && !partial.unavailableSensors[0].reason.empty() && partial.graph.hasVertex(id(1)));

    expect("graph: an available forecast of 0 mph is rejected, not turned into zero", throws<std::invalid_argument>([&] { routing::buildTravelTimeGraph(d, snapshot({0.0, 30.0, 10.0})); }));
    expect("graph: a negative forecast is rejected", throws<std::invalid_argument>([&] { routing::buildTravelTimeGraph(d, snapshot({60.0, -3.0, 10.0})); }));
    expect("graph: an infinite forecast is rejected", throws<std::invalid_argument>([&] { routing::buildTravelTimeGraph(d, snapshot({60.0, INF, 10.0})); }));
    expect("graph: a forecast below the floor is rejected", throws<std::invalid_argument>([&] { routing::buildTravelTimeGraph(d, snapshot({60.0, 30.0, 10.0}), 20.0); }));
    expect("graph: the floor is off by default (10 mph accepted)", routing::buildTravelTimeGraph(d, snapshot({60.0, 30.0, 10.0}), 0.0).legs.size() == 6);
    expect("graph: a record naming a sensor outside the snapshot is rejected", throws<std::invalid_argument>([&] { routing::buildTravelTimeGraph({rec(0, 5, 100.0)}, snapshot({60.0, 30.0, 10.0})); }));
    expect("graph: the same directed pair twice is rejected", throws<std::invalid_argument>([&] { routing::buildTravelTimeGraph({rec(0, 1, 100.0), rec(0, 1, 200.0)}, snapshot({60.0, 30.0})); }));
    expect("graph: an invalid distance is rejected", throws<std::invalid_argument>([&] { routing::buildTravelTimeGraph({rec(0, 1, -100.0)}, snapshot({60.0, 30.0})); }));
}

static double legSum(const routing::RouteResult& r) { double s = 0; for (const routing::RouteLeg& l : r.legs) s += l.minutes; return s; }

static void testPlanning() {
    // Sensors 0 (10 mph), 1 (60 mph), 2 (60 mph). Distances in miles: 0->1 = 2, 0->2 = 1, 2->1 = 1, 1->0 = 1, 1->2 = 1, 2->0 = 1.
    // Direct 0->1 : 2 miles at 10 mph = 12 min.   Via 2 : 0->2 = 1 mile at 10 mph = 6 min, then 2->1 = 1 mile at 60 mph = 1 min -> 7 min.
    const std::vector<routing::DistanceRecord> d = {rec(0, 1, 2 * MILE_M), rec(0, 2, MILE_M), rec(2, 1, MILE_M), rec(1, 0, MILE_M), rec(1, 2, MILE_M), rec(2, 0, MILE_M)};
    const routing::ForecastSnapshot snap = snapshot({10.0, 60.0, 60.0});

    routing::RouteResult detour = routing::planRouteFromSnapshot(snap, d, id(0), id(1));
    expect("plan: the detour 0 -> 2 -> 1 (7 min) beats the direct edge (12 min)", detour.reachable && detour.path == std::vector<std::string>({id(0), id(2), id(1)}) && near(detour.etaMinutes, 7.0, 1e-12));
    expect("plan: the legs list the hops with distance, upstream speed and minutes", detour.legs.size() == 2 && near(detour.legs[0].minutes, 6.0, 1e-12) && near(detour.legs[0].speedMph, 10.0) && near(detour.legs[1].minutes, 1.0, 1e-12) && near(detour.legs[1].speedMph, 60.0) && near(detour.legs[1].distanceM, MILE_M));
    expect("plan: ETA equals the sum of the leg times and the legs chain along the path", near(detour.etaMinutes, legSum(detour), 1e-12) && detour.legs[0].from == detour.path[0] && detour.legs[0].to == detour.path[1] && detour.legs[1].to == detour.path[2]);
    expect("plan: origin, forecast time and horizon are carried into the result", detour.originRow == 100 && detour.horizonSteps == 3 && detour.targetTime - detour.originTime == 900);

    routing::RouteResult back = routing::planRouteFromSnapshot(snap, d, id(1), id(0));
    expect("plan: the opposite direction is priced separately: 1 -> 0 is 1 mile at 60 mph = 1 min", back.reachable && back.path == std::vector<std::string>({id(1), id(0)}) && near(back.etaMinutes, 1.0, 1e-12));

    routing::RouteResult same = routing::planRouteFromSnapshot(snap, d, id(2), id(2));
    expect("plan: source == destination gives path {source} and ETA 0 with no legs", same.reachable && same.path == std::vector<std::string>({id(2)}) && same.etaMinutes == 0.0 && same.legs.empty());

    // the destination's own forecast is never needed: sensor 1 (destination) has no forecast, the answer is unchanged
    routing::RouteResult destNoForecast = routing::planRouteFromSnapshot(snapshot({10.0, NaN, 60.0}), d, id(0), id(1));
    expect("plan: a destination without a forecast still works (only the upstream speeds are used), ETA 7", destNoForecast.reachable && near(destNoForecast.etaMinutes, 7.0, 1e-12) && destNoForecast.unavailableSensors.size() == 1 && destNoForecast.unavailableSensors[0].sensorId == id(1));

    // intermediate sensor 2 has no forecast: its edges are omitted, the direct edge (12 min) is the only way
    routing::RouteResult noMiddle = routing::planRouteFromSnapshot(snapshot({10.0, 60.0, NaN}), d, id(0), id(1));
    expect("plan: an intermediate sensor without a forecast is left out; the direct edge (12 min) is used", noMiddle.reachable && noMiddle.path == std::vector<std::string>({id(0), id(1)}) && near(noMiddle.etaMinutes, 12.0, 1e-12));
    expect("plan: the omitted sensor is reported in the result", noMiddle.unavailableSensors.size() == 1 && noMiddle.unavailableSensors[0].sensorId == id(2) && !noMiddle.unavailableSensors[0].reason.empty());

    // directed: remove 1->0, so 1 -> 0 must go 1 -> 2 -> 0 = (1 mile at 60) + (1 mile at 60) = 2 min
    const std::vector<routing::DistanceRecord> oneWay = {rec(0, 1, 2 * MILE_M), rec(0, 2, MILE_M), rec(2, 1, MILE_M), rec(1, 2, MILE_M), rec(2, 0, MILE_M)};
    routing::RouteResult around = routing::planRouteFromSnapshot(snap, oneWay, id(1), id(0));
    expect("plan: a missing directed edge is not invented; 1 -> 0 goes 1 -> 2 -> 0 = 2 min", around.reachable && around.path == std::vector<std::string>({id(1), id(2), id(0)}) && near(around.etaMinutes, 2.0, 1e-12));

    // unreachable: no edge enters sensor 1
    const std::vector<routing::DistanceRecord> noEntry = {rec(1, 0, MILE_M), rec(1, 2, MILE_M), rec(0, 2, MILE_M), rec(2, 0, MILE_M)};
    routing::RouteResult none = routing::planRouteFromSnapshot(snap, noEntry, id(0), id(1));
    expect("unreachable: reachable = false, empty path and legs, ETA is +infinity (not 0)", !none.reachable && none.path.empty() && none.legs.empty() && std::isinf(none.etaMinutes) && none.etaMinutes > 0);
    expect("unreachable: the origin and forecast time are still reported", none.originRow == 100 && none.horizonSteps == 3);
    routing::RouteResult isolated = routing::planRouteFromSnapshot(snapshot({10.0, 60.0}), std::vector<routing::DistanceRecord>(), id(0), id(1));
    expect("unreachable: with no distance records at all nothing is reachable", !isolated.reachable && std::isinf(isolated.etaMinutes));

    expect("source: a source without a forecast fails clearly (std::runtime_error naming the sensor)", throwsWith<std::runtime_error>([&] { routing::planRouteFromSnapshot(snapshot({NaN, 60.0, 60.0}), d, id(0), id(1)); }, id(0)));
    expect("source: even source == destination fails if the source has no forecast", throws<std::runtime_error>([&] { routing::planRouteFromSnapshot(snapshot({NaN, 60.0, 60.0}), d, id(0), id(0)); }));
    expect("unknown: an unknown source sensor is rejected (std::invalid_argument)", throwsWith<std::invalid_argument>([&] { routing::planRouteFromSnapshot(snap, d, "999999", id(1)); }, "999999"));
    expect("unknown: an unknown destination sensor is rejected", throwsWith<std::invalid_argument>([&] { routing::planRouteFromSnapshot(snap, d, id(0), "888888"); }, "888888"));
    expect("floor: the speed floor reaches the planner (10 mph < 20 mph floor is rejected)", throws<std::invalid_argument>([&] { routing::RouteConfig c; c.minSpeedMph = 20.0; routing::planRouteFromSnapshot(snap, d, id(0), id(1), c); }));
    expect("invalid speed: a snapshot with an available forecast of 0 mph is rejected by the planner", throws<std::invalid_argument>([&] { routing::planRouteFromSnapshot(snapshot({10.0, 0.0, 60.0}), d, id(0), id(1)); }));
    expect("invalid speed: NaN stored as available cannot be created (NaN means unavailable); -infinity is rejected", throws<std::invalid_argument>([&] { routing::planRouteFromSnapshot(snapshot({-INF, 60.0, 60.0}), d, id(0), id(1)); }));

    // equal-cost alternatives give the same ETA whichever route is returned
    const std::vector<routing::DistanceRecord> tie = {rec(0, 1, MILE_M), rec(0, 2, MILE_M), rec(1, 3, MILE_M), rec(2, 3, MILE_M)};
    routing::RouteResult t = routing::planRouteFromSnapshot(snapshot({60.0, 60.0, 60.0, 60.0}), tie, id(0), id(3));
    expect("ties: two equal routes give ETA 2 min and a valid 3-sensor path", t.reachable && near(t.etaMinutes, 2.0, 1e-12) && t.path.size() == 3);
}

int main() {
    testConversion();
    testInvalidSpeedsAndDistances();
    testDistanceParser();
    testGraphConstruction();
    testPlanning();
    return tu::summary();
}
