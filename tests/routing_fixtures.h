// routing_fixtures.h -- shared helpers for the routing tests: a deterministic synthetic traffic series, hand-made forecast
// snapshots and distance records, and exception checks. Standard library only.
#pragma once

#include <cmath>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include "test_util.h"
#include "../src/route_planner.h"
#include "../src/speed_forecaster.h"
#include "../src/travel_time_graph.h"

namespace fx {

const double NaN = std::numeric_limits<double>::quiet_NaN();
const double INF = std::numeric_limits<double>::infinity();
const double MILE_M = 1609.344;       // one mile in meters: 1 mile at 60 mph takes exactly 1 minute

inline const std::string& id(size_t i) { return dp::SELECTED_SENSORS.at(i); }

// true if f() throws an exception of type Ex
template <class Ex, class F>
bool throws(F f) {
    try { f(); } catch (const Ex&) { return true; } catch (...) { return false; }
    return false;
}

// true if f() throws an exception of type Ex whose message contains `needle`
template <class Ex, class F>
bool throwsWith(F f, const std::string& needle) {
    try { f(); } catch (const Ex& e) { return std::string(e.what()).find(needle) != std::string::npos; } catch (...) { return false; }
    return false;
}

// Six sensors, `rows` rows exactly 5 minutes apart from 2012-03-01 06:00:00. Smooth waves plus fixed pseudo-random
// noise: always positive (>= 42 mph), different for every sensor, identical on every run and platform.
inline baseline::TrafficSeries syntheticSeries(size_t rows = 400) {
    baseline::TrafficSeries s;
    s.source = "synthetic";
    s.sensors = dp::SELECTED_SENSORS;
    long long start = 0;
    dp::parseTimestamp("2012-03-01 06:00:00", start);
    for (size_t r = 0; r < rows; ++r) s.times.push_back(start + static_cast<long long>(r) * dp::STEP_SECONDS);
    std::mt19937 rng(7u);
    s.speeds.assign(s.sensors.size(), std::vector<double>());
    for (size_t i = 0; i < s.sensors.size(); ++i) {
        const double phase = 0.7 * static_cast<double>(i);
        for (size_t r = 0; r < rows; ++r) {
            const double noise = static_cast<double>(rng() % 2001u) / 1000.0 - 1.0;         // -1 .. +1 mph
            const double x = static_cast<double>(r);
            s.speeds[i].push_back(55.0 + 9.0 * std::sin(x / 18.0 + phase) + 3.0 * std::sin(x / 5.0 + 2.0 * phase) + noise);
        }
    }
    return s;
}

// A forecast snapshot for the first speeds.size() sensors. NaN in `speeds` means "no forecast for that sensor".
// Any other number (even 0 or a negative one) is stored as an AVAILABLE forecast, so invalid values can be tested.
inline routing::ForecastSnapshot snapshot(const std::vector<double>& speeds) {
    routing::ForecastSnapshot snap;
    snap.originRow = 100;
    snap.originTime = 1330581600LL + 100 * dp::STEP_SECONDS;
    snap.horizonSteps = 3;
    snap.targetTime = snap.originTime + 3 * dp::STEP_SECONDS;
    for (size_t i = 0; i < speeds.size(); ++i) {
        routing::SensorForecast f;
        f.sensorId = id(i);
        if (std::isnan(speeds[i])) { f.available = false; f.reason = "test: no data"; }
        else { f.available = true; f.speedMph = speeds[i]; }
        snap.sensors.push_back(f);
    }
    return snap;
}

inline routing::DistanceRecord rec(size_t from, size_t to, double meters) {
    routing::DistanceRecord r;
    r.from = id(from);
    r.to = id(to);
    r.distanceM = meters;
    return r;
}

}  // namespace fx
