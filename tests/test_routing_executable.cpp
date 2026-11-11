// test_routing_executable.cpp -- runs the REAL routing program (a separate process) and checks what main() does.
// runRoutingCli() alone cannot show this: main() adds --auto-clock when no origin option is given.
//
// Build both programs first (from the repository root), then run the test with the path of the routing program:
//   g++ -std=c++17 -O2 -Wall -Wextra -o build/routing src/routing_main.cpp src/routing_cli.cpp src/clock_origin.cpp src/route_planner.cpp src/travel_time_graph.cpp src/speed_forecaster.cpp src/graph.cpp src/traffic_data.cpp src/features.cpp src/metrics.cpp src/linear_regression.cpp src/baseline_pipeline.cpp
//   g++ -std=c++17 -O2 -Wall -Wextra -o build/test_routing_executable tests/test_routing_executable.cpp src/traffic_data.cpp src/features.cpp src/metrics.cpp src/linear_regression.cpp src/baseline_pipeline.cpp
//   build/test_routing_executable build/routing            (Windows PowerShell: .\build\test_routing_executable.exe .\build\routing.exe)
// Run it from the repository root (the routing program reads data/sensor_distances.csv).
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#ifndef _WIN32
#include <sys/wait.h>
#endif

#include "routing_fixtures.h"

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
static std::string quote(const std::string& s) { return "\"" + s + "\""; }

struct Run { int code; std::string out; };
static std::string g_exe;

// Starts the program as a separate process with stdout and stderr redirected into one temporary file.
static Run runExe(const std::vector<std::string>& args) {
    const std::string outFile = tmpPath("routing_exe_test_output.txt");
    std::string cmd = quote(g_exe);
    for (size_t i = 0; i < args.size(); ++i) cmd += " " + quote(args[i]);
    cmd += " > " + quote(outFile) + " 2>&1";
#ifdef _WIN32
    cmd = "\"" + cmd + "\"";                                  // cmd.exe /c strips one pair of outer quotes
#endif
    const int rc = std::system(cmd.c_str());
    Run r;
#ifdef _WIN32
    r.code = rc;
#else
    r.code = WIFEXITED(rc) ? WEXITSTATUS(rc) : -1;
#endif
    std::ifstream in(outFile.c_str());
    std::stringstream ss; ss << in.rdbuf(); r.out = ss.str();
    std::remove(outFile.c_str());
    return r;
}
static std::string lineStarting(const std::string& text, const std::string& prefix) {
    std::istringstream in(text); std::string l;
    while (std::getline(in, l)) if (l.compare(0, prefix.size(), prefix) == 0) return l;
    return "";
}
static std::string from(const std::string& text, const std::string& marker) {
    const size_t p = text.find(marker);
    return p == std::string::npos ? std::string() : text.substr(p);
}

int main(int argc, char** argv) {
    if (argc != 2) { std::fprintf(stderr, "usage: test_routing_executable PATH_TO_ROUTING_PROGRAM\n"); return 2; }
    g_exe = argv[1];
#ifdef _WIN32
    for (size_t i = 0; i < g_exe.size(); ++i) if (g_exe[i] == '/') g_exe[i] = '\\';
#endif
    { std::ifstream probe(g_exe.c_str(), std::ios::binary); if (!probe) { std::fprintf(stderr, "ERROR: cannot open the routing program '%s' (build it first)\n", g_exe.c_str()); return 2; } }

    const baseline::TrafficSeries series = syntheticSeries(8064);          // 28 days: the allowed-origin window covers every weekday and time of day
    const std::string csv = tmpPath("routing_exe_test_traffic.csv");
    writeCsv(series, csv);
    const std::vector<std::string> base = {"--from", "r1", "--to", "r3", "--traffic", csv, "--distances", "data/sensor_distances.csv"};
    auto go = [&](std::vector<std::string> extra) { std::vector<std::string> a = base; a.insert(a.end(), extra.begin(), extra.end()); return runExe(a); };

    // 1. no origin option: the program itself must choose --auto-clock
    const Run def = go({});
    expect("exe: with no origin option the program exits 0", def.code == 0);
    expect("exe: with no origin option it prints the system clock line (auto-clock is the default)", has(def.out, "System clock   : ") && has(def.out, "local time, UTC"));
    expect("exe: with no origin option the origin is labelled a clock replay", has(def.out, "[clock replay: same weekday and time of day, latest eligible date]"));
    expect("exe: with no origin option it says historical replay, not live traffic", has(def.out, "NOT live traffic"));
    expect("exe: with no origin option the forecast target line is printed", has(def.out, "Forecast target: ") && has(def.out, "(15 min after the origin)"));
    expect("exe: with no origin option it is NOT the latest row", !has(def.out, "[default: the latest row in the dataset]"));

    // 2. the default run equals an explicit --clock run with the clock value it printed (same mapping, same forecast, same route)
    const std::string clockLine = lineStarting(def.out, "System clock   : ");
    const std::string stamp = clockLine.size() >= 33 ? clockLine.substr(17, 19) : "";
    expect("exe: the printed system clock has the form YYYY-MM-DD HH:MM:SS", stamp.size() == 19 && stamp[4] == '-' && stamp[10] == ' ' && stamp[13] == ':');
    const Run viaClock = go({"--clock", stamp});
    expect("exe: the default run selects the same origin as --clock with that printed time",
           viaClock.code == 0 && lineStarting(viaClock.out, "Forecast origin:") == lineStarting(def.out, "Forecast origin:") &&
           from(viaClock.out, "Forecast speeds") == from(def.out, "Forecast speeds"));
    expect("exe: an explicit --clock is labelled simulated", has(viaClock.out, "simulated with --clock"));

    // 3. every other origin option disables the clock default
    const Run row = go({"--row", "7000"});
    expect("exe: --row 7000 works and no clock lines are printed", row.code == 0 && has(row.out, "[selected with --row]") && !has(row.out, "System clock"));
    const Run tm = go({"--time", dp::formatTimestamp(series.times[7000])});
    expect("exe: --time works and no clock lines are printed", tm.code == 0 && has(tm.out, "[selected with --time]") && !has(tm.out, "System clock"));
    expect("exe: --row and the equivalent --time give the same route", from(row.out, "Fastest route:") == from(tm.out, "Fastest route:"));
    const Run lat = go({"--latest"});
    expect("exe: --latest uses the last row and prints no clock lines", lat.code == 0 && has(lat.out, "row " + std::to_string(series.rows() - 1) + " (") && !has(lat.out, "System clock"));
    const Run ac = go({"--auto-clock"});
    expect("exe: --auto-clock behaves like the default", ac.code == 0 && has(ac.out, "System clock   : ") && has(ac.out, "local time, UTC"));

    // 4. --help must not start a forecast or add an origin option
    const Run help = runExe({"--help"});
    expect("exe: --help exits 0 and documents the clock options", help.code == 0 && has(help.out, "--auto-clock") && has(help.out, "--latest") && !has(help.out, "System clock"));
    // 5. an origin option combined with a conflicting one is still refused (the program must not add --auto-clock on top of --row)
    expect("exe: --row with --latest is refused with exit 2", go({"--row", "7000", "--latest"}).code == 2);

    std::remove(csv.c_str());
    return tu::summary();
}
