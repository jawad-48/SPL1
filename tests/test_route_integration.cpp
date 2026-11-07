// test_route_integration.cpp -- the whole chain: Linear Regression forecasts -> travel-time graph -> Dijkstra -> ETA.
// Every ETA is checked against an independent brute-force search over ALL simple paths, so Dijkstra, the graph
// construction and the unit conversion are all verified against a second, much simpler computation.
//
// Build (from the repository root):
//   g++ -std=c++17 -O2 -Wall -Wextra -o build/test_route_integration tests/test_route_integration.cpp src/route_planner.cpp src/travel_time_graph.cpp src/speed_forecaster.cpp src/graph.cpp src/traffic_data.cpp src/features.cpp src/metrics.cpp src/linear_regression.cpp src/baseline_pipeline.cpp
// Run from the repository root (it reads data/traffic_data.csv and data/sensor_distances.csv):   build/test_route_integration
#include <algorithm>
#include <cstdio>
#include <map>
#include <set>

#include "routing_fixtures.h"

using namespace fx;
using tu::expect;
using tu::near;

typedef std::map<std::string, std::map<std::string, double> > Weights;

// cheapest total over every simple path from `at` to `dst` (+infinity if none). Weights are priced independently of the
// production code: distance / (upstream speed * 26.8224), only for upstream sensors that have a forecast.
static double cheapest(const Weights& w, const std::string& at, const std::string& dst, std::set<std::string>& visited) {
    if (at == dst) return 0.0;
    double best = INF;
    visited.insert(at);
    const Weights::const_iterator out = w.find(at);
    if (out != w.end()) {
        for (std::map<std::string, double>::const_iterator e = out->second.begin(); e != out->second.end(); ++e) {
            if (visited.count(e->first)) continue;
            const double rest = cheapest(w, e->first, dst, visited);
            if (rest < INF) best = std::min(best, e->second + rest);
        }
    }
    visited.erase(at);
    return best;
}

static double bruteForceEta(const std::vector<routing::DistanceRecord>& records, const routing::ForecastSnapshot& snap, const std::string& src, const std::string& dst) {
    Weights w;
    for (const routing::DistanceRecord& r : records) {
        const routing::SensorForecast* up = snap.find(r.from);
        if (up == nullptr || !up->available) continue;
        w[r.from][r.to] = r.distanceM / (up->speedMph * 26.8224);
    }
    std::set<std::string> visited;
    return cheapest(w, src, dst, visited);
}

// Plans all 30 ordered pairs at `origin` and verifies each result in detail. Returns how many routes use an intermediate sensor.
static bool checkAllPairs(const baseline::TrafficSeries& series, const routing::SpeedForecaster& f, const std::vector<routing::DistanceRecord>& records,
                          size_t origin, size_t* detours) {
    const routing::ForecastSnapshot snap = f.predictAt(series, origin);
    std::map<std::pair<std::string, std::string>, double> distance;
    for (const routing::DistanceRecord& r : records) distance[std::make_pair(r.from, r.to)] = r.distanceM;
    bool ok = true;
    size_t viaSensors = 0;
    for (size_t a = 0; a < 6; ++a) {
        for (size_t b = 0; b < 6; ++b) {
            if (a == b) continue;
            const routing::RouteResult r = routing::planRoute(series, f, records, origin, id(a), id(b));
            const double brute = bruteForceEta(records, snap, id(a), id(b));
            ok = ok && r.reachable == (brute < INF);
            if (!r.reachable) continue;
            ok = ok && std::isfinite(r.etaMinutes) && r.etaMinutes > 0 && near(r.etaMinutes, brute, 1e-9);
            ok = ok && r.path.size() >= 2 && r.path.front() == id(a) && r.path.back() == id(b) && r.legs.size() == r.path.size() - 1;
            ok = ok && std::set<std::string>(r.path.begin(), r.path.end()).size() == r.path.size();          // no sensor visited twice
            double sum = 0;
            for (size_t i = 0; i < r.legs.size(); ++i) {
                const routing::RouteLeg& leg = r.legs[i];
                ok = ok && leg.from == r.path[i] && leg.to == r.path[i + 1];
                ok = ok && leg.speedMph == snap.find(leg.from)->speedMph;                                       // upstream sensor's forecast
                ok = ok && leg.distanceM == distance[std::make_pair(leg.from, leg.to)];
                ok = ok && near(leg.minutes, leg.distanceM / (leg.speedMph * 26.8224), 1e-12);
                sum += leg.minutes;
            }
            ok = ok && near(r.etaMinutes, sum, 1e-9);
            ok = ok && r.originRow == origin && r.originTime == series.times[origin] && r.targetTime == snap.targetTime && r.horizonSteps == f.features().horizonSteps;
            if (r.path.size() > 2) ++viaSensors;
        }
    }
    if (detours != nullptr) *detours = viaSensors;
    return ok;
}

static bool sameRoute(const routing::RouteResult& x, const routing::RouteResult& y) {
    if (x.reachable != y.reachable || x.path != y.path || x.legs.size() != y.legs.size()) return false;
    if (x.reachable && x.etaMinutes != y.etaMinutes) return false;                    // exact, bit for bit
    return true;
}

static bool sameAllRoutes(const baseline::TrafficSeries& s1, const routing::SpeedForecaster& f1, const baseline::TrafficSeries& s2, const routing::SpeedForecaster& f2,
                          const std::vector<routing::DistanceRecord>& records, size_t origin) {
    bool ok = true;
    for (size_t a = 0; a < 6; ++a) for (size_t b = 0; b < 6; ++b) {
        if (a == b) continue;
        ok = ok && sameRoute(routing::planRoute(s1, f1, records, origin, id(a), id(b)), routing::planRoute(s2, f2, records, origin, id(a), id(b)));
    }
    return ok;
}

static void testSyntheticEndToEnd(const baseline::TrafficSeries& series, const baseline::BaselineConfig& cfg, const routing::SpeedForecaster& f, const std::vector<routing::DistanceRecord>& records) {
    size_t detours = 0;
    expect("e2e synthetic: all 30 routes at origin 300 equal the brute-force optimum, with consistent legs, speeds and units", checkAllPairs(series, f, records, 300, &detours));
    std::printf("      (informational: %zu of the 30 routes use an intermediate sensor)\n", detours);
    expect("e2e synthetic: all 30 routes at the last row (origin 399) are correct too", checkAllPairs(series, f, records, 399, nullptr));
    expect("e2e synthetic: all 30 routes at the first allowed origin (trainEnd - 1) are correct", checkAllPairs(series, f, records, f.earliestOriginRow(), nullptr));

    const routing::RouteResult one = routing::planRoute(series, f, records, 300, id(0), id(3));
    const routing::RouteResult two = routing::planRoute(series, f, records, 300, id(0), id(3));
    expect("determinism: planning the same route twice gives the identical path and ETA", sameRoute(one, two));
    expect("determinism: training from scratch again gives identical routes for all 30 pairs", sameAllRoutes(series, f, series, routing::SpeedForecaster::train(series, cfg), records, 300));

    const routing::RouteResult self = routing::planRoute(series, f, records, 300, id(2), id(2));
    expect("e2e: source == destination gives ETA 0 and path {source}", self.reachable && self.etaMinutes == 0.0 && self.path.size() == 1);

    expect("e2e: an unknown sensor is rejected", throws<std::invalid_argument>([&] { routing::planRoute(series, f, records, 300, "000000", id(1)); }));
    expect("e2e: an origin inside the training period is rejected", throws<std::invalid_argument>([&] { routing::planRoute(series, f, records, 100, id(0), id(1)); }));
    routing::RouteConfig floorCfg;
    floorCfg.minSpeedMph = 1000.0;
    expect("e2e: the speed floor is enforced end to end (every forecast is below 1000 mph)", throws<std::runtime_error>([&] { routing::planRoute(series, f, records, 300, id(0), id(1), floorCfg); }));
    routing::RouteConfig gentle;
    gentle.minSpeedMph = 5.0;
    expect("e2e: a low floor changes nothing", sameRoute(routing::planRoute(series, f, records, 300, id(0), id(4), gentle), routing::planRoute(series, f, records, 300, id(0), id(4))));
}

static void testNoLeakageEndToEnd(const baseline::TrafficSeries& series, const baseline::BaselineConfig& cfg, const routing::SpeedForecaster& f, const std::vector<routing::DistanceRecord>& records) {
    const size_t origin = 300;
    baseline::TrafficSeries garbageAfterOrigin = series;
    for (size_t r = origin + 1; r < series.rows(); ++r) {
        garbageAfterOrigin.times[r] = 1;
        for (size_t s = 0; s < 6; ++s) garbageAfterOrigin.speeds[s][r] = (r % 2) ? NaN : 123456.0;
    }
    expect("leakage e2e: garbage in every row after the origin leaves all 30 routes and ETAs bit-identical", sameAllRoutes(series, f, garbageAfterOrigin, f, records, origin));

    baseline::TrafficSeries garbageAfterCutoff = series;
    for (size_t r = f.trainEndRow(); r < series.rows(); ++r) for (size_t s = 0; s < 6; ++s) garbageAfterCutoff.speeds[s][r] = 777.0;
    const routing::SpeedForecaster trainedOnGarbageFuture = routing::SpeedForecaster::trainBefore(garbageAfterCutoff, cfg, f.trainEndRow());
    expect("leakage e2e: models trained while every row after the cutoff is garbage give bit-identical routes", sameAllRoutes(series, f, series, trainedOnGarbageFuture, records, origin));
}

static void testMissingDataEndToEnd(const baseline::TrafficSeries& series, const routing::SpeedForecaster& f, const std::vector<routing::DistanceRecord>& records) {
    const size_t origin = 300;
    baseline::TrafficSeries hole = series;
    hole.speeds[1][origin - 3] = NaN;                                          // sensor 716331 has an empty cell in its window
    const routing::ForecastSnapshot snap = f.predictAt(hole, origin);
    expect("missing e2e: the sensor with the empty cell is the only one without a forecast", !snap.sensors[1].available && snap.sensors[0].available && snap.sensors[2].available);

    expect("missing e2e: a route FROM that sensor fails clearly (std::runtime_error naming it)", throwsWith<std::runtime_error>([&] { routing::planRoute(hole, f, records, origin, id(1), id(0)); }, id(1)));

    bool ok = true, listed = true, skipsIt = true;
    for (size_t a = 0; a < 6; ++a) {
        if (a == 1) continue;
        for (size_t b = 0; b < 6; ++b) {
            if (b == a) continue;
            const routing::RouteResult r = routing::planRoute(hole, f, records, origin, id(a), id(b));
            const double brute = bruteForceEta(records, snap, id(a), id(b));
            ok = ok && r.reachable && near(r.etaMinutes, brute, 1e-9);
            listed = listed && r.unavailableSensors.size() == 1 && r.unavailableSensors[0].sensorId == id(1) && !r.unavailableSensors[0].reason.empty();
            for (size_t i = 1; i + 1 < r.path.size(); ++i) skipsIt = skipsIt && r.path[i] != id(1);      // never an intermediate stop
        }
    }
    expect("missing e2e: all 20 routes between the other sensors still equal the brute-force optimum", ok);
    expect("missing e2e: the sensor without a forecast is reported in every result", listed);
    expect("missing e2e: the sensor without a forecast is never used as an intermediate stop", skipsIt);
    const routing::RouteResult toIt = routing::planRoute(hole, f, records, origin, id(0), id(1));
    expect("missing e2e: it can still be the DESTINATION (the edge into it uses the upstream speed)", toIt.reachable && near(toIt.etaMinutes, bruteForceEta(records, snap, id(0), id(1)), 1e-9));
}

static void testUnreachableEndToEnd(const baseline::TrafficSeries& series, const routing::SpeedForecaster& f, const std::vector<routing::DistanceRecord>& records) {
    std::vector<routing::DistanceRecord> noEntryTo5;
    for (const routing::DistanceRecord& r : records) if (r.to != id(5)) noEntryTo5.push_back(r);
    bool allUnreachable = true, othersFine = true;
    for (size_t a = 0; a < 5; ++a) {
        const routing::RouteResult r = routing::planRoute(series, f, noEntryTo5, 300, id(a), id(5));
        allUnreachable = allUnreachable && !r.reachable && r.path.empty() && std::isinf(r.etaMinutes) && r.legs.empty();
        othersFine = othersFine && routing::planRoute(series, f, noEntryTo5, 300, id(a), id((a + 1) % 5)).reachable;
    }
    expect("unreachable e2e: with no distance record into sensor 773062, every route to it is unreachable (ETA +infinity)", allUnreachable);
    expect("unreachable e2e: routes between the other sensors are unaffected", othersFine);
    expect("unreachable e2e: sensor 773062 can still be a source", routing::planRoute(series, f, noEntryTo5, 300, id(5), id(0)).reachable);
}

static void testThirtyMinuteHorizon(const baseline::TrafficSeries& series, const routing::SpeedForecaster& f3, const std::vector<routing::DistanceRecord>& records) {
    baseline::BaselineConfig cfg6;
    cfg6.features.horizonSteps = 6;
    const routing::SpeedForecaster f6 = routing::SpeedForecaster::train(series, cfg6);
    const routing::ForecastSnapshot s6 = f6.predictAt(series, 300);
    expect("horizon 6: the snapshot says 6 steps and the target time is 30 minutes after the origin", s6.horizonSteps == 6 && s6.targetTime - s6.originTime == 1800);
    expect("horizon 6: the 30-minute models give different forecasts than the 15-minute models", s6.sensors[0].speedMph != f3.predictAt(series, 300).sensors[0].speedMph);
    expect("horizon 6: all 30 routes equal the brute-force optimum", checkAllPairs(series, f6, records, 300, nullptr));
}

static void testCommittedSample() {
    const baseline::BaselineConfig cfg;
    const baseline::TrafficSeries series = baseline::loadTrafficSeries("data/traffic_data.csv");
    const std::vector<routing::DistanceRecord> records = routing::loadSensorDistances("data/sensor_distances.csv");
    const routing::SpeedForecaster f = routing::SpeedForecaster::train(series, cfg);
    const baseline::SplitBoundaries split = baseline::chronologicalBoundaries(series.rows(), cfg.trainFraction, cfg.validationFraction);
    expect("sample: the committed data loads and the models train (6 sensors, 15-minute horizon)", series.rows() > 100 && records.size() == 30 && f.trainEndRow() == split.trainEnd);
    size_t detours = 0;
    expect("sample: all 30 routes at the first allowed origin equal the brute-force optimum", checkAllPairs(series, f, records, f.earliestOriginRow(), nullptr));
    expect("sample: all 30 routes at the first TEST row equal the brute-force optimum", checkAllPairs(series, f, records, split.validationEnd, &detours));
    std::printf("      (informational: %zu of the 30 routes use an intermediate sensor at the first test row)\n", detours);
    expect("sample: all 30 routes at the last recorded row equal the brute-force optimum", checkAllPairs(series, f, records, series.rows() - 1, nullptr));
    const routing::ForecastSnapshot snap = f.predictAt(series, split.validationEnd);
    bool plausible = true;
    for (const routing::SensorForecast& s : snap.sensors) plausible = plausible && s.available && s.speedMph > 0 && s.speedMph < 120;
    expect("sample: every forecast speed is a plausible positive number of mph", plausible);
    const routing::RouteResult r = routing::planRoute(series, f, records, split.validationEnd, "716328", "773062");
    expect("sample: a real route has a plausible ETA (more than 0 and under 60 minutes)", r.reachable && r.etaMinutes > 0 && r.etaMinutes < 60);
    std::printf("      (informational: 716328 -> 773062 at %s: %.3f min via %zu sensors)\n", dp::formatTimestamp(r.originTime).c_str(), r.etaMinutes, r.path.size());
}

int main() {
    const baseline::TrafficSeries series = syntheticSeries(400);
    const baseline::BaselineConfig cfg;
    const routing::SpeedForecaster f = routing::SpeedForecaster::train(series, cfg);
    const std::vector<routing::DistanceRecord> records = routing::loadSensorDistances("data/sensor_distances.csv");
    testSyntheticEndToEnd(series, cfg, f, records);
    testNoLeakageEndToEnd(series, cfg, f, records);
    testMissingDataEndToEnd(series, f, records);
    testUnreachableEndToEnd(series, f, records);
    testThirtyMinuteHorizon(series, f, records);
    testCommittedSample();
    return tu::summary();
}
