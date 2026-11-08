// test_routing_cli.cpp -- the command-line application (src/routing_cli.cpp): arguments, validation, output, exit codes.
// The tests call routing::runRoutingCli with in-memory streams, so no process is started.
//
// Build (from the repository root):
//   g++ -std=c++17 -O2 -Wall -Wextra -o build/test_routing_cli tests/test_routing_cli.cpp src/routing_cli.cpp src/route_planner.cpp src/travel_time_graph.cpp src/speed_forecaster.cpp src/graph.cpp src/traffic_data.cpp src/features.cpp src/metrics.cpp src/linear_regression.cpp src/baseline_pipeline.cpp
// Run from the repository root (it reads data/traffic_data.csv and data/sensor_distances.csv):   build/test_routing_cli
// Temporary files are written to the folder named by TMPDIR, TEMP or TMP (else the current folder) and removed at the end.
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>

#include "routing_fixtures.h"
#include "../src/routing_cli.h"

using namespace fx;
using tu::expect;
using tu::near;

static void expect(const std::string& name, bool ok) { tu::expect(name.c_str(), ok); }      // names built with + are std::string

static const std::string TRAFFIC = "data/traffic_data.csv";
static const std::string DIST = "data/sensor_distances.csv";

struct Run {
    int code;
    std::string out, err;
};

static Run run(const std::vector<std::string>& args) {
    std::ostringstream o, e;
    Run r;
    r.code = routing::runRoutingCli(args, o, e);
    r.out = o.str();
    r.err = e.str();
    return r;
}

// --from/--to plus any extra options, using the committed sample files unless the extras name other files
static Run route(const std::string& from, const std::string& to, std::vector<std::string> extra = std::vector<std::string>(),
                 const std::string& traffic = TRAFFIC, const std::string& dist = DIST) {
    std::vector<std::string> a = {"--from", from, "--to", to};
    a.insert(a.end(), extra.begin(), extra.end());
    a.push_back("--traffic");
    a.push_back(traffic);
    a.push_back("--distances");
    a.push_back(dist);
    return run(a);
}

static bool has(const std::string& text, const std::string& needle) { return text.find(needle) != std::string::npos; }

// the text from `marker` to the end (empty if the marker is absent, so a failing test fails instead of crashing)
static std::string fromMarker(const std::string& text, const std::string& marker) {
    const size_t pos = text.find(marker);
    return pos == std::string::npos ? std::string() : text.substr(pos);
}

// the output line that starts with `prefix` (empty if there is none)
static std::string lineStartingWith(const std::string& text, const std::string& prefix) {
    size_t pos = 0;
    while (pos < text.size()) {
        const size_t end = text.find('\n', pos);
        const std::string line = text.substr(pos, end == std::string::npos ? std::string::npos : end - pos);
        if (line.compare(0, prefix.size(), prefix) == 0) return line;
        if (end == std::string::npos) break;
        pos = end + 1;
    }
    return std::string();
}

static std::vector<std::string> createdFiles;

static std::string tempPath(const std::string& name) {
    const char* dir = std::getenv("TMPDIR");
    if (!dir) dir = std::getenv("TEMP");
    if (!dir) dir = std::getenv("TMP");
    return std::string(dir ? dir : ".") + "/spl1_cli_test_" + name;
}

static std::string writeFile(const std::string& name, const std::string& text) {
    const std::string path = tempPath(name);
    std::ofstream f(path.c_str(), std::ios::binary);
    f << text;
    f.close();
    createdFiles.push_back(path);
    return path;
}

static std::string trafficCsv(const baseline::TrafficSeries& s) {
    std::string text = "timestamp";
    for (const std::string& id : s.sensors) text += "," + id;
    text += "\n";
    for (size_t r = 0; r < s.rows(); ++r) {
        text += dp::formatTimestamp(s.times[r]);
        for (size_t i = 0; i < s.sensors.size(); ++i) text += std::isnan(s.speeds[i][r]) ? "," : dp::fmt(",%.4f", s.speeds[i][r]);
        text += "\n";
    }
    return text;
}

static std::string joinPath(const std::vector<std::string>& p) {
    std::string s;
    for (size_t i = 0; i < p.size(); ++i) s += (i ? " -> " : "") + p[i];
    return s;
}

// ------------------------------------------------------------------------------------------------------------------
static void testHelpAndUsage() {
    Run h = run({"--help"});
    expect("help: --help exits 0 and shows usage, the exit codes and the default relative paths", h.code == 0 && has(h.out, "Usage:") && has(h.out, "Exit codes") && has(h.out, "data/full/processed/traffic_data.csv") && has(h.out, "data/full/processed/sensor_distances.csv") && h.err.empty());
    expect("help: -h works and help wins over other mistakes", run({"-h"}).code == 0 && run({"--bogus", "--help"}).code == 0);
    expect("help: it lists all six sensors", has(h.out, "716328") && has(h.out, "773062"));
    expect("help: no absolute Windows path in the defaults", !has(h.out, "C:\\") && !has(h.out, ":/"));

    Run none = run({});
    expect("usage: no arguments is a command-line mistake (exit 2) that says what is required", none.code == 2 && has(none.err, "--from and --to are required") && none.out.empty());
    expect("usage: --to alone is refused", run({"--to", "773062"}).code == 2);
    expect("usage: an unknown option is refused and named", [] { Run r = run({"--from", "716328", "--to", "773062", "--bogus", "x"}); return r.code == 2 && has(r.err, "--bogus"); }());
    expect("usage: a stray word is refused and named", [] { Run r = run({"716328", "773062"}); return r.code == 2 && has(r.err, "716328"); }());
    expect("usage: an option without a value at the end is refused", [] { Run r = run({"--from", "716328", "--to"}); return r.code == 2 && has(r.err, "--to needs a value"); }());
    expect("usage: an option followed by another option is refused (no value swallowed)", [] { Run r = run({"--from", "--to", "773062"}); return r.code == 2 && has(r.err, "--from needs a value"); }());
    expect("usage: an option given twice is refused", [] { Run r = run({"--from", "716328", "--from", "716331", "--to", "773062"}); return r.code == 2 && has(r.err, "twice"); }());
}

static void testValidRoutes() {
    const baseline::BaselineConfig cfg;
    const baseline::TrafficSeries series = baseline::loadTrafficSeries(TRAFFIC);
    const std::vector<routing::DistanceRecord> records = routing::loadSensorDistances(DIST);
    const routing::SpeedForecaster f = routing::SpeedForecaster::train(series, cfg);
    const baseline::SplitBoundaries split = baseline::chronologicalBoundaries(series.rows(), cfg.trainFraction, cfg.validationFraction);
    const size_t origin = split.validationEnd;                                  // the first test row
    const routing::ForecastSnapshot snap = f.predictAt(series, origin);

    // find a route that uses an intermediate sensor, so the multi-leg output is tested too
    size_t detourA = 0, detourB = 1;
    bool foundDetour = false;
    for (size_t a = 0; a < 6 && !foundDetour; ++a) for (size_t b = 0; b < 6 && !foundDetour; ++b) {
        if (a == b) continue;
        if (routing::planRoute(series, f, records, origin, id(a), id(b)).path.size() > 2) { detourA = a; detourB = b; foundDetour = true; }
    }
    expect("setup: the sample has at least one multi-leg route at the first test row", foundDetour);

    const size_t pairs[2][2] = {{detourA, detourB}, {0, 5}};
    for (int k = 0; k < 2; ++k) {
        const std::string from = id(pairs[k][0]), to = id(pairs[k][1]);
        const routing::RouteResult expected = routing::planRoute(series, f, records, origin, from, to);
        const Run r = route(from, to, {"--row", std::to_string(origin)});
        const std::string label = "route " + from + " -> " + to + ": ";
        expect(label + "exit 0 and nothing on stderr", r.code == 0 && r.err.empty());
        expect(label + "the route is printed as an ordered sensor sequence", has(r.out, "Fastest route: " + joinPath(expected.path) + "\n"));
        expect(label + "the total ETA equals the library result (3 decimals)", has(r.out, "Total ETA: " + dp::fmt("%.3f", expected.etaMinutes) + " minutes\n"));
        bool legsOk = !expected.legs.empty();
        double sum = 0;
        for (size_t i = 0; i < expected.legs.size(); ++i) {
            const routing::RouteLeg& leg = expected.legs[i];
            const std::string row = lineStartingWith(r.out, dp::fmt("  %-3zu  %-6s  %-6s  ", i + 1, leg.from.c_str(), leg.to.c_str()));      // this leg's own line
            legsOk = legsOk && !row.empty() && has(row, dp::fmt("%.1f m", leg.distanceM)) && has(row, dp::fmt("%.2f mph", leg.speedMph)) && has(row, dp::fmt("%.3f min", leg.minutes));
            legsOk = legsOk && near(leg.minutes, leg.distanceM / (leg.speedMph * 26.8224), 1e-9);            // travel time = distance / (speed * 26.8224)
            sum += leg.minutes;
        }
        expect(label + "every leg shows distance, upstream speed and travel time, and the legs add up to the ETA", legsOk && near(sum, expected.etaMinutes, 1e-9));
        bool speedsOk = true;
        for (const routing::SensorForecast& s : snap.sensors) speedsOk = speedsOk && has(r.out, s.sensorId + "  " + dp::fmt("%6.2f", s.speedMph) + " mph");
        expect(label + "the forecast speeds of all six sensors are listed", speedsOk);
        expect(label + "origin row, timestamp and forecast time are shown", has(r.out, "row " + std::to_string(origin) + " (" + dp::formatTimestamp(series.times[origin]) + ") [selected with --row]") && has(r.out, "Forecast speeds for " + dp::formatTimestamp(snap.targetTime) + " (15 min after the origin)"));
        expect(label + "origin and destination sensors are shown", has(r.out, "From / To      : " + from + " -> " + to));
        expect(label + "the output contains no NaN or infinity", !has(r.out, "nan") && !has(r.out, "inf") && !has(r.out, "NaN"));
        if (k == 0) expect(label + "this route has more than one leg", expected.legs.size() >= 2 && has(r.out, "  2    "));
    }

    // --time selects exactly the same row as --row
    const Run byRow = route(id(0), id(5), {"--row", std::to_string(origin)});
    Run byTime = route(id(0), id(5), {"--time", dp::formatTimestamp(series.times[origin])});
    std::string asRow = byTime.out;
    const size_t pos = asRow.find("[selected with --time]");
    if (pos != std::string::npos) asRow.replace(pos, 22, "[selected with --row]");
    expect("--time: the timestamp of a row gives the same result as that row's --row", byTime.code == 0 && asRow == byRow.out);

    // the default origin is the latest row
    const size_t last = series.rows() - 1;
    const Run def = route(id(0), id(5));
    const Run lastRow = route(id(0), id(5), {"--row", std::to_string(last)});
    expect("default: without --row/--time the latest row is used and the output says so", def.code == 0 && has(def.out, "row " + std::to_string(last) + " (" + dp::formatTimestamp(series.times[last]) + ") [default: the latest row in the dataset]"));
    expect("default: the route and ETA equal an explicit --row of the latest row", !fromMarker(def.out, "Fastest route:").empty() && fromMarker(def.out, "Fastest route:") == fromMarker(lastRow.out, "Fastest route:"));

    // eligibility boundary: trainEnd - 1 is the first allowed origin
    const size_t earliest = split.trainEnd - 1;
    expect("origin: the first allowed row (trainEnd - 1) is accepted", route(id(1), id(2), {"--row", std::to_string(earliest)}).code == 0);
    const Run tooEarly = route(id(1), id(2), {"--row", std::to_string(earliest - 1)});
    expect("origin: one row earlier is refused (exit 1), explaining the training period and the earliest allowed row, and nothing is substituted",
           tooEarly.code == 1 && has(tooEarly.err, "training period") && has(tooEarly.err, "earliest allowed origin is row " + std::to_string(earliest) + " (" + dp::formatTimestamp(series.times[earliest]) + ")") && tooEarly.out.empty());

    // all six sources against the library
    bool all = true;
    for (size_t a = 0; a < 6; ++a) {
        const size_t b = (a + 2) % 6;
        const routing::RouteResult e = routing::planRoute(series, f, records, origin, id(a), id(b));
        const Run r = route(id(a), id(b), {"--row", std::to_string(origin)});
        all = all && r.code == 0 && has(r.out, "Total ETA: " + dp::fmt("%.3f", e.etaMinutes) + " minutes") && has(r.out, "Fastest route: " + joinPath(e.path) + "\n");
    }
    expect("six more routes (every sensor as the source) all match the library", all);
}

static void testSameSensor() {
    const Run r = route("716337", "716337", {"--row", "489"});
    expect("same sensor: exit 0, a one-sensor route, ETA 0.000 minutes, no leg table", r.code == 0 && has(r.out, "Fastest route: 716337\n") && has(r.out, "Total ETA: 0.000 minutes\n") && !has(r.out, "Legs (") && has(r.out, "no travel needed"));
}

static void testInvalidSensors() {
    Run bad = route("123456", "773062", {"--row", "489"});
    expect("sensor: an unknown source ID is refused (exit 2), named, with the valid IDs listed, and nothing is run", bad.code == 2 && has(bad.err, "--from '123456'") && has(bad.err, "716328, 716331, 716337, 717446, 717456, 773062") && bad.out.empty());
    Run badTo = route("716328", "abc", {"--row", "489"});
    expect("sensor: an unknown destination ID is refused (exit 2) and named", badTo.code == 2 && has(badTo.err, "--to 'abc'"));
    expect("sensor: an empty ID is refused", route("", "773062").code == 2 && route("716328", "").code == 2);
    expect("sensor: an ID with a trailing space is refused, not trimmed", route("716328 ", "773062").code == 2);
    expect("sensor: a sensor that is only a prefix of a real ID is refused", route("7163", "773062").code == 2);
    Run order = run({"--from", "999999", "--to", "773062", "--traffic", "no_such_file.csv"});
    expect("sensor: the ID is checked before any file is opened (exit 2, not 1)", order.code == 2 && has(order.err, "999999"));
}

static void testMissingSourceForecast() {
    baseline::TrafficSeries s = syntheticSeries(400);
    s.speeds[1][297] = NaN;                                         // 716331 has an empty cell inside the window of origin row 300
    const std::string traffic = writeFile("hole_traffic.csv", trafficCsv(s));

    Run src = route("716331", "716328", {"--row", "300"}, traffic);
    expect("missing source forecast: exit 3 with a clear explanation naming the sensor", src.code == 3 && has(src.err, "No route can be calculated") && has(src.err, "716331") && has(src.err, "no forecast") && has(src.err, "no other speed was substituted"));
    expect("missing source forecast: the forecast table still shows which sensor is unavailable, and no route or ETA is printed", has(src.out, "716331  unavailable:") && !has(src.out, "Fastest route") && !has(src.out, "Total ETA"));
    Run same = route("716331", "716331", {"--row", "300"}, traffic);
    expect("missing source forecast: even source == destination fails (exit 3)", same.code == 3 && !has(same.out, "Total ETA"));

    Run dest = route("716328", "716337", {"--row", "300"}, traffic);
    expect("missing forecast on another sensor: the route is still calculated (exit 0) and the sensor is shown as unavailable", dest.code == 0 && has(dest.out, "716331  unavailable:") && has(dest.out, "Total ETA"));
    Run toIt = route("716328", "716331", {"--row", "300"}, traffic);
    expect("missing forecast on the destination: still works, its own speed is not needed", toIt.code == 0 && has(toIt.out, "Fastest route: "));
    expect("missing source forecast: the same data at an origin where the window is complete works", route("716331", "716328", {"--row", "310"}, traffic).code == 0);
}

static void testInvalidTimestampsAndRows() {
    const baseline::TrafficSeries series = baseline::loadTrafficSeries(TRAFFIC);
    expect("time: unparseable text is refused (exit 2) and quoted", [] { Run r = route("716328", "773062", {"--time", "garbage"}); return r.code == 2 && has(r.err, "'garbage'"); }());
    expect("time: a date without a time is refused", route("716328", "773062", {"--time", "2012-03-02"}).code == 2);
    expect("time: a timestamp with trailing text is refused", route("716328", "773062", {"--time", "2012-03-02 22:45:00 UTC"}).code == 2);
    expect("time: an impossible date (month 13) is refused", route("716328", "773062", {"--time", "2012-13-02 22:45:00"}).code == 2);
    expect("time: a missing value is refused", run({"--from", "716328", "--to", "773062", "--time"}).code == 2);

    Run off = route("716328", "773062", {"--time", "2012-03-02 22:46:00"});
    expect("time: a valid timestamp that is not a row (off the 5-minute grid) is refused (exit 1) and the data range is shown", off.code == 1 && has(off.err, "no row has the timestamp 2012-03-02 22:46:00") && has(off.err, "2012-03-01 06:00:00 to 2012-03-03 05:55:00") && off.out.empty());
    expect("time: a timestamp after the data is refused, not clamped", route("716328", "773062", {"--time", "2013-01-01 00:00:00"}).code == 1);
    expect("time: a timestamp before the data is refused, not clamped", route("716328", "773062", {"--time", "2011-01-01 00:00:00"}).code == 1);
    Run training = route("716328", "773062", {"--time", dp::formatTimestamp(series.times[10])});
    expect("time: a timestamp inside the training period is refused (exit 1) with the reason", training.code == 1 && has(training.err, "training period") && has(training.err, "future information") && training.out.empty());
    expect("time: the timestamp of the last row is accepted", route("716328", "773062", {"--time", dp::formatTimestamp(series.times.back())}).code == 0);

    expect("row: row 10 (inside training) is refused with exit 1", route("716328", "773062", {"--row", "10"}).code == 1);
    expect("row: row 0 is refused with exit 1", route("716328", "773062", {"--row", "0"}).code == 1);
    Run beyond = route("716328", "773062", {"--row", "99999"});
    expect("row: a row beyond the file is refused (exit 1) and the valid range is shown", beyond.code == 1 && has(beyond.err, "does not exist") && has(beyond.err, "rows 0 to 575"));
    expect("row: row == number of rows is refused, row rows-1 is accepted", route("716328", "773062", {"--row", "576"}).code == 1 && route("716328", "773062", {"--row", "575"}).code == 0);
    const char* badRows[] = {"-3", "abc", "1.5", "", "+5", " 5", "5 ", "0x10", "99999999999999999999"};
    bool allRefused = true;
    for (const char* t : badRows) allRefused = allRefused && route("716328", "773062", {"--row", t}).code == 2;
    expect("row: negative, text, decimal, empty, signed, padded, hex and absurdly long values are all refused (exit 2)", allRefused);
    expect("row: the message quotes the bad value", has(route("716328", "773062", {"--row", "abc"}).err, "'abc'"));
    expect("row and time together are refused (exit 2)", route("716328", "773062", {"--row", "489", "--time", "2012-03-02 22:45:00"}).code == 2);
}

static void testUnreachable() {
    std::ifstream in(DIST.c_str());
    std::string line, kept;
    while (std::getline(in, line)) {
        if (!line.empty() && line[line.size() - 1] == '\r') line.erase(line.size() - 1);
        const size_t first = line.find(','), second = line.find(',', first + 1);
        if (first != std::string::npos && second != std::string::npos && line.substr(first + 1, second - first - 1) == "773062") continue;      // drop every record INTO 773062
        kept += line + "\n";
    }
    const std::string dist = writeFile("no_entry_to_773062.csv", kept);
    Run r = route("716328", "773062", {"--row", "489"}, TRAFFIC, dist);
    expect("unreachable: exit 3 and an explanation naming both sensors", r.code == 3 && has(r.err, "No route can be calculated") && has(r.err, "716328") && has(r.err, "773062"));
    expect("unreachable: no route and no ETA is printed (not even 0), but the forecasts are", !has(r.out, "Fastest route") && !has(r.out, "Total ETA") && has(r.out, "Forecast speeds for"));
    expect("unreachable: other routes with the same file still work, including FROM 773062", route("773062", "716328", {"--row", "489"}, TRAFFIC, dist).code == 0 && route("716328", "716331", {"--row", "489"}, TRAFFIC, dist).code == 0);
}

static void testFilesAndDatasets() {
    Run noTraffic = route("716328", "773062", {}, "this_traffic_file_does_not_exist.csv");
    expect("files: a missing traffic file is refused (exit 1) with its path and the option to fix it", noTraffic.code == 1 && has(noTraffic.err, "this_traffic_file_does_not_exist.csv") && has(noTraffic.err, "--traffic PATH") && noTraffic.out.empty());
    Run noDist = route("716328", "773062", {}, TRAFFIC, "this_distance_file_does_not_exist.csv");
    expect("files: a missing distance file is refused (exit 1) with its path and the option to fix it", noDist.code == 1 && has(noDist.err, "this_distance_file_does_not_exist.csv") && has(noDist.err, "--distances PATH") && noDist.out.empty());

    const std::string oldLayout = writeFile("old_layout.csv", "from,to,cost\n716328,716331,10\n");
    Run old = route("716328", "773062", {}, TRAFFIC, oldLayout);
    expect("files: the old from,to,cost distance layout is refused (exit 1)", old.code == 1 && has(old.err, "distance file is not usable") && has(old.err, "from_sensor_id"));
    const std::string unknownSensor = writeFile("unknown_sensor.csv", "from_sensor_id,to_sensor_id,distance_m\n999999,716331,10\n");
    expect("files: a distance file naming an unknown sensor is refused (exit 1)", route("716328", "773062", {}, TRAFFIC, unknownSensor).code == 1);
    const std::string badTraffic = writeFile("bad_traffic.csv", "time,a,b\n2012-03-01 06:00:00,1,2\n");
    expect("files: a traffic file with the wrong columns is refused (exit 1)", [&] { Run r = route("716328", "773062", {}, badTraffic); return r.code == 1 && has(r.err, "traffic file is not usable"); }());
    const std::string emptyTraffic = writeFile("empty_traffic.csv", "");
    expect("files: an empty traffic file is refused (exit 1)", route("716328", "773062", {}, emptyTraffic).code == 1);
    const std::string twoRows = writeFile("two_rows_traffic.csv", trafficCsv(syntheticSeries(2)));
    Run split = route("716328", "773062", {}, twoRows);
    expect("files: a traffic file with too few rows to split into periods is refused (exit 1) with the reason, before any training", split.code == 1 && has(split.err, "cannot be split") && split.out.empty());
    const std::string fiveRows = writeFile("five_rows_traffic.csv", trafficCsv(syntheticSeries(5)));
    Run few = route("716328", "773062", {}, fiveRows);
    expect("files: a traffic file that splits but has too few rows to train is refused (exit 1) with a clear reason", few.code == 1 && has(few.err, "cannot be trained") && has(few.err, "too few usable rows") && !has(few.out, "Fastest route"));
    expect("files: a directory given as the traffic path is refused, not crashed on", route("716328", "773062", {}, "data").code == 1);
}

// ------------------------------------------------------------------------------------------------------------------
// route names r1..r6 and the interactive "where are you / where do you want to go" questions

static Run runAsking(const std::vector<std::string>& args, const std::string& input) {
    std::istringstream in(input);
    std::ostringstream o, e;
    Run r;
    r.code = routing::runRoutingCli(args, in, o, e);
    r.out = o.str();
    r.err = e.str();
    return r;
}

static std::vector<std::string> dataArgs(const std::vector<std::string>& extra = std::vector<std::string>()) {
    std::vector<std::string> a = {"--row", "489", "--traffic", TRAFFIC, "--distances", DIST};
    a.insert(a.end(), extra.begin(), extra.end());
    return a;
}

static size_t countOf(const std::string& text, const std::string& needle) {
    size_t n = 0, pos = 0;
    while ((pos = text.find(needle, pos)) != std::string::npos) { ++n; pos += needle.size(); }
    return n;
}

static void testRouteNames() {
    const baseline::BaselineConfig cfg;
    const baseline::TrafficSeries series = baseline::loadTrafficSeries(TRAFFIC);
    const std::vector<routing::DistanceRecord> records = routing::loadSensorDistances(DIST);
    const routing::SpeedForecaster f = routing::SpeedForecaster::train(series, cfg);

    expect("names: --help explains the route names", has(run({"--help"}).out, "Route names: r1 = 716328, r2 = 716331, r3 = 716337, r4 = 717446, r5 = 717456, r6 = 773062"));
    const Run byName = route("r2", "r3", {"--row", "489"});
    const Run byId = route("716331", "716337", {"--row", "489"});
    expect("names: --from/--to accept r1..r6 and give exactly the same route as the sensor IDs", byName.code == 0 && !fromMarker(byName.out, "Fastest route:").empty() && fromMarker(byName.out, "Fastest route:") == fromMarker(byId.out, "Fastest route:"));
    expect("names: capital R and a mix of names and IDs also work", route("R1", "R6", {"--row", "489"}).code == 0 && route("r1", "773062", {"--row", "489"}).code == 0 && route("716328", "r6", {"--row", "489"}).code == 0);
    bool every = true;
    for (size_t i = 0; i < 6; ++i) every = every && route("r" + std::to_string(i + 1), id(i), {"--row", "489"}).code == 0 && has(route("r" + std::to_string(i + 1), id(i), {"--row", "489"}).out, "Fastest route: " + id(i) + "\n");
    expect("names: r1..r6 are the six sensors in order (r1 = 716328 ... r6 = 773062)", every);

    expect("names: the header shows the legend and the chosen route in both forms", has(byName.out, "Route names    : r1 = 716328, r2 = 716331, r3 = 716337, r4 = 717446, r5 = 717456, r6 = 773062") && has(byName.out, "From / To      : 716331 -> 716337  (r2 -> r3)"));
    bool table = true;
    for (size_t i = 0; i < 6; ++i) table = table && has(byName.out, "  r" + std::to_string(i + 1) + "  " + id(i) + "  ");
    expect("names: every line of the forecast table starts with the sensor's route name", table);
    const routing::RouteResult expected = routing::planRoute(series, f, records, 489, id(1), id(2));
    std::string names;
    for (size_t i = 0; i < expected.path.size(); ++i) {
        size_t k = 0;
        while (id(k) != expected.path[i]) ++k;
        names += (i ? " -> r" : "r") + std::to_string(k + 1);
    }
    expect("names: the route is also printed in route names (r2 -> r1 -> r3 style), matching the library's path", expected.path.size() >= 3 && has(byName.out, "  In route names: " + names + "\n"));

    const char* bad[] = {"r0", "r7", "r", "r10", "rr", "R 1", "r1 ", " r1", "route1", "1"};
    bool allRefused = true;
    for (const char* t : bad) allRefused = allRefused && route(t, "773062", {"--row", "489"}).code == 2 && route("716328", t, {"--row", "489"}).code == 2;
    expect("names: r0, r7, r, r10, rr, 'R 1', names with spaces, route1 and a bare 1 are refused (exit 2)", allRefused);
    Run r7 = route("r7", "r1", {"--row", "489"});
    expect("names: the refusal names the bad value and tells about r1 to r6", r7.code == 2 && has(r7.err, "--from 'r7'") && has(r7.err, "r1 to r6") && r7.out.empty());

    baseline::TrafficSeries hole = syntheticSeries(400);
    hole.speeds[1][297] = NaN;
    const std::string traffic = writeFile("names_hole_traffic.csv", trafficCsv(hole));
    Run un = route("r1", "r3", {"--row", "300"}, traffic);
    expect("names: an unavailable sensor is listed with its route name too", un.code == 0 && has(un.out, "  r2  716331  unavailable:"));
}

static void testInteractive() {
    Run explicit1 = route("716331", "716328", {"--row", "489"});
    const std::string expectedRoute = fromMarker(explicit1.out, "Fastest route:");

    Run basic = runAsking(dataArgs(), "r2\nr1\n");
    expect("ask: with no --from/--to the program asks 'Where are you now?' then 'Where do you want to go?'", basic.code == 0 && has(basic.out, "Where are you now?") && has(basic.out, "Where do you want to go?") && basic.err.empty());
    expect("ask: the question comes first and the destination question second", basic.out.find("Where are you now?") < basic.out.find("Where do you want to go?"));
    bool menu = has(basic.out, "Routes (sensors):");
    for (size_t i = 0; i < 6; ++i) menu = menu && has(basic.out, "  r" + std::to_string(i + 1) + "  " + id(i) + "\n");
    expect("ask: the menu lists r1..r6 with their sensor IDs before the questions", menu && basic.out.find("Routes (sensors):") < basic.out.find("Where are you now?"));
    expect("ask: answering r2 and r1 gives exactly the route of --from 716331 --to 716328", !expectedRoute.empty() && fromMarker(basic.out, "Fastest route:") == expectedRoute && has(basic.out, "From / To      : 716331 -> 716328  (r2 -> r1)"));

    Run styles = runAsking(dataArgs(), "  R2 \r\n716328\r\n");
    expect("ask: capital letters, surrounding spaces, Windows line endings and plain sensor IDs are accepted", styles.code == 0 && fromMarker(styles.out, "Fastest route:") == expectedRoute);

    Run retry = runAsking(dataArgs(), "r9\nabc\nr2\n\nr1\n");      // 2 bad answers for the origin, 1 for the destination (3 tries are allowed per question)
    expect("ask: invalid answers are explained and asked again; the valid answers are used", retry.code == 0 && countOf(retry.out, "is not a route") == 3 && has(retry.out, "'r9'") && has(retry.out, "'abc'") && fromMarker(retry.out, "Fastest route:") == expectedRoute);
    expect("ask: the destination is also asked again after a bad answer", runAsking(dataArgs(), "r2\nzzz\nr1\n").code == 0 && fromMarker(runAsking(dataArgs(), "r2\nzzz\nr1\n").out, "Fastest route:") == expectedRoute);

    Run giveUp = runAsking(dataArgs(), "x\ny\nz\nr1\nr2\n");
    expect("ask: three invalid answers in a row end the program (exit 2) without a route", giveUp.code == 2 && has(giveUp.err, "after 3 tries") && !has(giveUp.out, "Fastest route") && countOf(giveUp.out, "is not a route") == 3);
    Run eof = runAsking(dataArgs(), "");
    expect("ask: input that ends before an answer is an error (exit 2) that points to --from/--to", eof.code == 2 && has(eof.err, "input ended") && has(eof.err, "--from and --to") && !has(eof.out, "Fastest route"));
    expect("ask: input that ends after the first answer is also an error (exit 2)", runAsking(dataArgs(), "r2").code == 2 && runAsking(dataArgs(), "r2\n").code == 2);

    std::vector<std::string> onlyFrom = dataArgs({"--from", "r1"});
    Run askTo = runAsking(onlyFrom, "r6\n");
    Run explicit16 = route("r1", "r6", {"--row", "489"});
    expect("ask: with only --from given, only the destination is asked", askTo.code == 0 && has(askTo.out, "Where do you want to go?") && !has(askTo.out, "Where are you now?") && fromMarker(askTo.out, "Fastest route:") == fromMarker(explicit16.out, "Fastest route:"));
    Run askFrom = runAsking(dataArgs({"--to", "r6"}), "r1\n");
    expect("ask: with only --to given, only the origin is asked", askFrom.code == 0 && has(askFrom.out, "Where are you now?") && !has(askFrom.out, "Where do you want to go?") && fromMarker(askFrom.out, "Fastest route:") == fromMarker(explicit16.out, "Fastest route:"));
    Run both = runAsking(dataArgs({"--from", "r1", "--to", "r6"}), "r3\nr4\n");
    expect("ask: with --from and --to given nothing is asked and the answers waiting on input are ignored", both.code == 0 && !has(both.out, "Where are you now?") && !has(both.out, "Routes (sensors):") && fromMarker(both.out, "Fastest route:") == fromMarker(explicit16.out, "Fastest route:"));

    Run same = runAsking(dataArgs(), "r3\nr3\n");
    expect("ask: the same place twice gives ETA 0.000 minutes", same.code == 0 && has(same.out, "Total ETA: 0.000 minutes") && has(same.out, "Fastest route: 716337\n"));

    Run badOption = runAsking({"--row", "abc", "--traffic", TRAFFIC, "--distances", DIST}, "r1\nr2\n");
    expect("ask: a wrong option is reported before any question is asked (exit 2, nothing asked)", badOption.code == 2 && !has(badOption.out, "Where are you now?") && badOption.out.empty());
    Run badFile = runAsking({"--traffic", "no_such_traffic_file.csv", "--distances", DIST}, "r1\nr2\n");
    expect("ask: a missing file is reported before any question is asked (exit 1, nothing asked)", badFile.code == 1 && !has(badFile.out, "Where are you now?") && has(badFile.err, "no_such_traffic_file.csv"));
    Run badIdThenAsk = runAsking({"--from", "r9", "--traffic", TRAFFIC, "--distances", DIST}, "r1\nr2\n");
    expect("ask: a wrong --from value is refused (exit 2), not replaced by a question", badIdThenAsk.code == 2 && !has(badIdThenAsk.out, "Where"));

    Run tooEarly = runAsking({"--row", "10", "--traffic", TRAFFIC, "--distances", DIST}, "r1\nr2\n");
    expect("ask: after valid answers an origin inside the training period is still refused (exit 1) with the reason", tooEarly.code == 1 && has(tooEarly.err, "training period") && !has(tooEarly.out, "Fastest route"));

    expect("ask: the version without an input stream never asks and still requires --from and --to (exit 2)", run({"--from", "r1"}).code == 2 && has(run({"--from", "r1"}).err, "--from and --to are required") && run({}).code == 2);
}

int main() {
    testHelpAndUsage();
    testValidRoutes();
    testSameSensor();
    testInvalidSensors();
    testMissingSourceForecast();
    testInvalidTimestampsAndRows();
    testUnreachable();
    testFilesAndDatasets();
    testRouteNames();
    testInteractive();
    for (const std::string& path : createdFiles) std::remove(path.c_str());
    return tu::summary();
}
