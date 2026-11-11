// test_unavailable_sensor.cpp -- a sensor whose last readings are empty (the r4 / 717446 situation): its forecast is reported as
// unavailable with a reason, the other sensors still forecast, and the routing treats the missing forecast safely.
//
// Build (from the repository root):
//   g++ -std=c++17 -O2 -Wall -Wextra -o build/test_unavailable_sensor tests/test_unavailable_sensor.cpp src/routing_cli.cpp src/clock_origin.cpp src/route_planner.cpp src/travel_time_graph.cpp src/speed_forecaster.cpp src/graph.cpp src/traffic_data.cpp src/features.cpp src/metrics.cpp src/linear_regression.cpp src/baseline_pipeline.cpp
// Run from the repository root (it reads data/sensor_distances.csv). Temporary files go to TMPDIR, TEMP or TMP (else the current folder).
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>

#include "routing_fixtures.h"
#include "../src/routing_cli.h"

using namespace fx;
static void expect(const std::string& name, bool ok) { tu::expect(name.c_str(), ok); }
static bool has(const std::string& t, const std::string& n) { return t.find(n) != std::string::npos; }

static std::string tmpPath(const std::string& name) {
    const char* d = std::getenv("TMPDIR"); if (!d) d = std::getenv("TEMP"); if (!d) d = std::getenv("TMP");
    std::string dir = d ? d : ".";
    if (!dir.empty() && dir.back() != '/' && dir.back() != '\\') dir += "/";
    return dir + name;
}
static void writeCsv(const baseline::TrafficSeries& s, const std::string& path) {
    std::ofstream f(path.c_str());
    f << "timestamp";
    for (const std::string& id : s.sensors) f << "," << id;
    f << "\n";
    for (size_t r = 0; r < s.rows(); ++r) {
        f << dp::formatTimestamp(s.times[r]);
        for (size_t k = 0; k < s.sensors.size(); ++k) f << "," << (std::isnan(s.speeds[k][r]) ? std::string() : dp::fmt("%.4f", s.speeds[k][r]));
        f << "\n";
    }
}
struct Run { int code; std::string out, err; };
static Run run(const std::vector<std::string>& args) {
    std::ostringstream o, e; Run r; r.code = routing::runRoutingCli(args, o, e); r.out = o.str(); r.err = e.str(); return r;
}

int main() {
    const size_t ROWS = 400, OUTAGE_FROM = 380, R4 = 3;                  // r4 = 717446 has no readings from row 380 to the end
    baseline::TrafficSeries s = syntheticSeries(ROWS);
    for (size_t r = OUTAGE_FROM; r < ROWS; ++r) s.speeds[R4][r] = NaN;
    const baseline::BaselineConfig config;
    const routing::SpeedForecaster f = routing::SpeedForecaster::train(s, config);

    // ---------------------------------------------------------------- forecasts
    const routing::ForecastSnapshot last = f.predictAt(s, ROWS - 1);
    int availableCount = 0;
    for (size_t k = 0; k < last.sensors.size(); ++k) if (last.sensors[k].available) ++availableCount;
    expect("forecast: at the last row the other five sensors have forecasts and r4 does not", availableCount == 5 && !last.sensors[R4].available);
    expect("forecast: the reason names the missing reading and its timestamp", has(last.sensors[R4].reason, "missing") && has(last.sensors[R4].reason, dp::formatTimestamp(s.times[ROWS - 1])));
    expect("forecast: no speed is invented (an unavailable forecast carries no usable speed)", !last.sensors[R4].available);

    expect("forecast: one row before the outage the window is complete and r4 forecasts", f.predictAt(s, OUTAGE_FROM - 1).sensors[R4].available);
    expect("forecast: a window that only partly overlaps the outage is also unavailable (never filled in)", !f.predictAt(s, OUTAGE_FROM + 2).sensors[R4].available);
    expect("forecast: the reason names the newest empty reading of a partly empty window", has(f.predictAt(s, OUTAGE_FROM + 2).sensors[R4].reason, dp::formatTimestamp(s.times[OUTAGE_FROM + 2])));
    expect("forecast: a window that still contains the outage start (origin OUTAGE_FROM + 5) is unavailable", !f.predictAt(s, OUTAGE_FROM + 5).sensors[R4].available);

    // systematic or isolated? count the unavailable origins of every sensor
    size_t unavailable[6] = {0, 0, 0, 0, 0, 0};
    for (size_t r = f.earliestOriginRow(); r < ROWS; ++r) {
        const routing::ForecastSnapshot snap = f.predictAt(s, r);
        for (size_t k = 0; k < 6; ++k) if (!snap.sensors[k].available) ++unavailable[k];
    }
    expect("scan: exactly the origins whose window touches the outage are unavailable for r4 (rows 380 to 399)", unavailable[R4] == ROWS - OUTAGE_FROM);
    expect("scan: the other five sensors are never unavailable", unavailable[0] == 0 && unavailable[1] == 0 && unavailable[2] == 0 && unavailable[4] == 0 && unavailable[5] == 0);

    // a hole in the middle of the window
    baseline::TrafficSeries mid = syntheticSeries(ROWS);
    mid.speeds[R4][ROWS - 3] = NaN;
    const routing::SpeedForecaster fm = routing::SpeedForecaster::train(mid, config);
    expect("forecast: a single empty reading inside the six makes the sensor unavailable", !fm.predictAt(mid, ROWS - 1).sensors[R4].available && fm.predictAt(mid, ROWS - 1).sensors[0].available);

    // ---------------------------------------------------------------- routing with a missing forecast
    const std::vector<routing::DistanceRecord> dist = routing::loadSensorDistances("data/sensor_distances.csv");
    const routing::RouteResult viaOthers = routing::planRouteFromSnapshot(last, dist, id(0), id(2));          // r1 -> r3
    bool r4InPath = false;
    for (size_t i = 0; i < viaOthers.path.size(); ++i) if (viaOthers.path[i] == id(R4)) r4InPath = true;
    expect("routing: r1 -> r3 is found without r4 and r4 is not an intermediate stop", viaOthers.reachable && !r4InPath);
    expect("routing: the result lists r4 as an unavailable sensor with its reason", viaOthers.unavailableSensors.size() == 1 && viaOthers.unavailableSensors[0].sensorId == id(R4) && !viaOthers.unavailableSensors[0].reason.empty());
    expect("routing: r1 -> r4 (r4 only as the destination) is reachable, priced with r1's forecast", routing::planRouteFromSnapshot(last, dist, id(0), id(R4)).reachable);
    expect("routing: r4 as the SOURCE is refused with an exception that names the sensor",
           throwsWith<std::runtime_error>([&] { routing::planRouteFromSnapshot(last, dist, id(R4), id(0)); }, id(R4)));

    // an unavailable sensor can never be an intermediate vertex, even when it would be the best one: hand-made snapshots
    {
        // r2 is slow (10 mph), r1 is fast (60 mph): r2 -> r1 -> r3 beats the direct r2 -> r3 leg
        const routing::RouteResult withR1 = routing::planRouteFromSnapshot(snapshot({60, 10, 40, 40, 40, 40}), dist, id(1), id(2));
        expect("planner: with r1 available, r2 -> r1 -> r3 is chosen (r1 is the best intermediate stop)", withR1.reachable && withR1.path.size() == 3 && withR1.path[1] == id(0));
        const routing::RouteResult noR1 = routing::planRouteFromSnapshot(snapshot({NaN, 10, 40, 40, 40, 40}), dist, id(1), id(2));
        bool usesR1 = false;
        for (size_t i = 0; i < noR1.path.size(); ++i) if (noR1.path[i] == id(0)) usesR1 = true;
        for (size_t i = 0; i < noR1.legs.size(); ++i) if (noR1.legs[i].from == id(0)) usesR1 = true;
        expect("planner: with r1 unavailable the route still exists and r1 is NOT used as an intermediate vertex", noR1.reachable && !usesR1 && noR1.path.front() == id(1) && noR1.path.back() == id(2));
        expect("planner: the route without r1 is slower than the route with r1", noR1.etaMinutes > withR1.etaMinutes);
        const routing::RouteResult toR1 = routing::planRouteFromSnapshot(snapshot({NaN, 10, 40, 40, 40, 40}), dist, id(1), id(0));
        expect("planner: a route can still END at the unavailable r1 through its incoming edge (priced by r2's forecast)", toR1.reachable && toR1.path.size() == 2 && toR1.path.back() == id(0) && toR1.legs.size() == 1 && toR1.legs[0].speedMph == 10.0);
        expect("planner: an unavailable source is rejected with an exception that explains why",
               throwsWith<std::runtime_error>([&] { routing::planRouteFromSnapshot(snapshot({NaN, 10, 40, 40, 40, 40}), dist, id(0), id(2)); }, "has no forecast"));
        const routing::RouteResult alone = routing::planRouteFromSnapshot(snapshot({60, 10, 40, 40, 40, 40}), dist, id(1), id(1));
        expect("planner: unchanged behaviour when nothing is unavailable (origin equals destination gives a zero-length route)", alone.reachable && alone.path.size() == 1);
    }

    // ---------------------------------------------------------------- the command-line application
    const std::string csv = tmpPath("unavailable_test_traffic.csv");
    writeCsv(s, csv);
    const std::vector<std::string> tail = {"--row", std::to_string(ROWS - 1), "--traffic", csv, "--distances", "data/sensor_distances.csv"};
    auto go = [&](const std::string& from, const std::string& to) { std::vector<std::string> a = {"--from", from, "--to", to}; a.insert(a.end(), tail.begin(), tail.end()); return run(a); };
    const Run ok = go("r1", "r3");
    expect("cli: r1 -> r3 works while r4 is unavailable (exit 0)", ok.code == 0 && has(ok.out, "Total ETA:"));
    expect("cli: the forecast list says r4 is unavailable and why", has(ok.out, "r4  717446  unavailable: a reading in the last 6 is missing"));
    expect("cli: an explanation line names the empty readings, the last real reading and the 6-step rule", has(ok.out, "Why r4 (717446):") && has(ok.out, "last real reading was at " + dp::formatTimestamp(s.times[OUTAGE_FROM - 1])) && has(ok.out, "at most 6 steps") && has(ok.out, "No speed was invented"));
    expect("cli: the other five sensors still print forecasts", has(ok.out, "r1  716328  ") && has(ok.out, "r6  773062  ") && !has(ok.out, "r1  716328  unavailable"));
    expect("cli: a note tells that r4 could not be used as an intermediate stop", has(ok.out, "Note: sensor 717446 (r4) has no forecast"));
    const Run src = go("r4", "r3");
    expect("cli: r4 as the origin exits 3 and says no other speed was substituted", src.code == 3 && has(src.err, "no forecast") && has(src.err, "no other speed was substituted"));
    const Run dst = go("r1", "r4");
    expect("cli: r4 as the destination still gives a route (exit 0)", dst.code == 0 && has(dst.out, "Fastest route: 716328 -> 717446"));
    const Run healthy = run({"--from", "r1", "--to", "r3", "--row", std::to_string(OUTAGE_FROM - 1), "--traffic", csv, "--distances", "data/sensor_distances.csv"});
    expect("cli: at an origin before the outage all six sensors forecast and no explanation is printed", healthy.code == 0 && !has(healthy.out, "unavailable:") && !has(healthy.out, "Why r"));

    // a hole in the time grid inside the window: the explanation must say so and must NOT claim that readings are empty
    {
        baseline::TrafficSeries g = syntheticSeries(ROWS);
        g.times.erase(g.times.begin() + 397);
        for (size_t k = 0; k < g.speeds.size(); ++k) g.speeds[k].erase(g.speeds[k].begin() + 397);
        const std::string gcsv = tmpPath("unavailable_test_grid.csv");
        writeCsv(g, gcsv);
        const Run gr = run({"--from", "r1", "--to", "r3", "--row", std::to_string(g.rows() - 1), "--traffic", gcsv, "--distances", "data/sensor_distances.csv"});
        expect("cli: a missing timestamp in the window makes every sensor unavailable and the origin sensor error exits 3", gr.code == 3 && has(gr.err, "no forecast"));
        expect("cli: the explanation names the broken 5-minute grid", has(gr.out, "Why r1 (716328): the timestamps of its last 6 readings are not exactly 5 minutes apart"));
        expect("cli: it does not claim the readings are empty when they are not", !has(gr.out, "readings are empty"));
        std::remove(gcsv.c_str());
    }

    std::remove(csv.c_str());
    return tu::summary();
}
