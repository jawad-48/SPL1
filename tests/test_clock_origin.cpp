// test_clock_origin.cpp -- clock-based origin selection (src/clock_origin.cpp) and the --clock / --auto-clock / --latest options of the CLI.
//
// Build (from the repository root):
//   g++ -std=c++17 -O2 -Wall -Wextra -o build/test_clock_origin tests/test_clock_origin.cpp src/clock_origin.cpp src/routing_cli.cpp src/route_planner.cpp src/travel_time_graph.cpp src/speed_forecaster.cpp src/graph.cpp src/traffic_data.cpp src/features.cpp src/metrics.cpp src/linear_regression.cpp src/baseline_pipeline.cpp
// Run from the repository root (it reads data/sensor_distances.csv). Temporary files go to TMPDIR, TEMP or TMP (else the current folder).
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>

#include "routing_fixtures.h"
#include "../src/clock_origin.h"
#include "../src/routing_cli.h"

using namespace fx;
static void expect(const std::string& name, bool ok) { tu::expect(name.c_str(), ok); }
static bool has(const std::string& t, const std::string& n) { return t.find(n) != std::string::npos; }

static long long ts(const char* text) { long long t = 0; dp::parseTimestamp(text, t); return t; }

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
static std::string line(const std::string& text, const std::string& prefix) {
    std::istringstream in(text); std::string l;
    while (std::getline(in, l)) if (l.compare(0, prefix.size(), prefix) == 0) return l;
    return "";
}

int main() {
    // ---------------------------------------------------------------- pure selection on 28 days (Thu 2012-03-01 06:00 ...)
    const baseline::TrafficSeries series = syntheticSeries(8064);
    const baseline::SplitBoundaries split = baseline::chronologicalBoundaries(series.rows(), 0.70, 0.15);
    const size_t earliest = split.trainEnd - 1;

    expect("weekday: 2012-03-01 was a Thursday (index 0)", routing::weekdayIndex(ts("2012-03-01 06:00:00")) == 0);
    expect("weekday: 2026-11-11 is a Wednesday (index 6)", routing::weekdayIndex(ts("2026-11-11 11:20:00")) == 6);
    expect("weekday: 2012-06-27 was a Wednesday (the last day of the full dataset)", routing::weekdayIndex(ts("2012-06-27 23:55:00")) == 6);

    routing::ClockOrigin a, b, c, d;
    const bool okA = routing::selectOriginForClock(series.times, earliest, ts("2026-11-11 11:20:00"), a);     // a Wednesday
    expect("select: a Wednesday 11:20 clock finds a row", okA);
    expect("select: the row has time of day 11:20:00", okA && dp::formatTimestamp(series.times[a.row]).substr(11) == "11:20:00");
    expect("select: the row is on a Wednesday", okA && routing::weekdayIndex(series.times[a.row]) == 6 && a.sameWeekday);
    expect("select: the row is not inside the training period", okA && a.row >= earliest);
    expect("select: it is the LATEST such row (no later row has the same weekday and time)", okA && [&] {
        for (size_t r = a.row + 1; r < series.rows(); ++r)
            if (dp::formatTimestamp(series.times[r]).substr(11) == "11:20:00" && routing::weekdayIndex(series.times[r]) == 6) return false;
        return true; }());

    const bool okB = routing::selectOriginForClock(series.times, earliest, ts("2026-11-11 11:25:00"), b);
    expect("select: 11:25 selects the row 5 minutes after the 11:20 row (five-minute grid)", okB && b.row == a.row + 1);
    const bool okC = routing::selectOriginForClock(series.times, earliest, ts("2026-11-11 12:00:00"), c);
    expect("select: 12:00 selects the row 8 steps (40 min) after the 11:20 row", okC && c.row == a.row + 8);
    expect("select: 11:23:41 is rounded DOWN to 11:20 (never to a later reading)", routing::selectOriginForClock(series.times, earliest, ts("2026-11-11 11:23:41"), d) && d.row == a.row);
    routing::ClockOrigin a2;
    expect("select: the same clock value always gives the same row", routing::selectOriginForClock(series.times, earliest, ts("2026-11-11 11:20:00"), a2) && a2.row == a.row);
    routing::ClockOrigin thu;
    expect("select: another weekday (Thursday) selects a different row on a Thursday", routing::selectOriginForClock(series.times, earliest, ts("2026-11-12 11:20:00"), thu) && thu.row != a.row && routing::weekdayIndex(series.times[thu.row]) == 0);
    expect("select: every selected origin has five regular 5-minute steps before it", [&] {
        for (long long m = 0; m < 24 * 60; m += 35) {
            routing::ClockOrigin o;
            if (!routing::selectOriginForClock(series.times, earliest, ts("2026-11-11 00:00:00") + m * 60, o)) return false;
            if (o.row < earliest || o.row < 5 || series.times[o.row] - series.times[o.row - 5] != 1500) return false;
        }
        return true; }());

    // missing timestamps: remove 11:00-11:40 on the newest matching Wednesday; the previous Wednesday must be used instead
    {
        std::vector<long long> t = series.times;
        const long long day = series.times[a.row] - series.times[a.row] % 86400;
        std::vector<long long> kept;
        for (long long x : t) if (!(x >= day + 11 * 3600 && x <= day + 11 * 3600 + 40 * 60)) kept.push_back(x);
        routing::ClockOrigin m;
        const bool ok = routing::selectOriginForClock(kept, earliest, ts("2026-11-11 11:20:00"), m);
        expect("gap: with the newest 11:20 rows missing, the same time one week earlier is used (never a made-up row)",
               ok && m.sameWeekday && series.times[a.row] - kept[m.row] == 7 * 86400);
    }
    // time grid hole just before the newest candidate: that date is skipped
    {
        std::vector<long long> kept = series.times;
        kept.erase(kept.begin() + static_cast<std::ptrdiff_t>(a.row - 2));
        routing::ClockOrigin m;
        expect("gap: a hole in the five steps before a candidate makes that date unusable", routing::selectOriginForClock(kept, earliest, ts("2026-11-11 11:20:00"), m) && series.times[a.row] - kept[m.row] == 7 * 86400);
    }
    // weekday fallback: only three consecutive days available
    {
        std::vector<long long> t(series.times.begin(), series.times.begin() + 3 * 288);       // Thu, Fri, Sat only (06:00 start)
        routing::ClockOrigin m;
        const bool ok = routing::selectOriginForClock(t, 5, ts("2026-11-11 12:00:00"), m);     // clock is a Wednesday
        expect("fallback: no Wednesday in the data -> the latest row with this time of day is used and sameWeekday is false", ok && !m.sameWeekday && dp::formatTimestamp(t[m.row]).substr(11) == "12:00:00");
    }
    // nothing matches: the data covers only 06:00-08:00 of one day
    {
        std::vector<long long> t(series.times.begin(), series.times.begin() + 24);
        routing::ClockOrigin m; m.row = 777;
        expect("no match: a time of day outside the data fails and leaves the result untouched", !routing::selectOriginForClock(t, 5, ts("2026-11-11 23:55:00"), m) && m.row == 777);
        expect("no match: an earliest row beyond the data fails", !routing::selectOriginForClock(t, 99, ts("2026-11-11 06:30:00"), m));
        expect("no match: empty data fails", !routing::selectOriginForClock(std::vector<long long>(), 0, ts("2026-11-11 06:30:00"), m));
    }
    // dataset boundary: the very last row of a Wednesday-ending dataset is selectable
    {
        std::vector<long long> t;
        const long long start = ts("2012-06-14 00:00:00");                                         // Thursday
        for (long long i = 0; i < 14 * 288; ++i) t.push_back(start + i * 300);                    // ends Wednesday 2012-06-27 23:55
        routing::ClockOrigin m;
        expect("boundary: Wednesday 23:55 selects the last row of the dataset", routing::selectOriginForClock(t, 5, ts("2026-11-11 23:55:00"), m) && m.row == t.size() - 1);
    }

    // ---------------------------------------------------------------- additional continuity, weekday, alignment and boundary cases
    {
        // each of the five steps is checked on its own: one missing timestamp AND one duplicate inside the window cancel out
        // in times[r] - times[r-5] (still 25 minutes) but must disqualify the candidate
        std::vector<long long> t;
        const long long start = ts("2012-06-13 06:00:00");                                       // Wednesday
        for (long long i = 0; i < 2 * 288; ++i) t.push_back(start + i * 300);
        const size_t wanted = 288 + 60;                                                          // Thursday 11:00 (row 348)
        routing::ClockOrigin base;
        expect("continuity: a regular grid selects Thursday 11:00", routing::selectOriginForClock(t, 20, ts("2026-11-12 11:00:00"), base) && base.row == wanted);
        std::vector<long long> broken = t;
        broken[wanted - 3] = broken[wanted - 4];                                                 // duplicate (interval 0) ...
        broken[wanted - 2] = broken[wanted - 2] + 300;                                           // ... and an interval of 10 minutes after it
        // times[wanted] - times[wanted-5] is still exactly 1500 s, but the individual steps are 300,0,600(+),300,... not all 300
        expect("continuity: the old first-vs-last check would accept this window", broken[wanted] - broken[wanted - 5] == 1500);
        // The file holds one Thursday, so after the broken Thursday row is rejected the only remaining 11:00 row is on Wednesday
        // (the "no matching weekday" fallback): the broken row must never be returned as a same-weekday match.
        auto rejected = [&](const std::vector<long long>& v) {
            routing::ClockOrigin o;
            const bool found = routing::selectOriginForClock(v, 20, ts("2026-11-12 11:00:00"), o);
            return !found || (o.row != wanted && !o.sameWeekday && routing::weekdayIndex(v[o.row]) != routing::weekdayIndex(ts("2026-11-12 11:00:00")));
        };
        expect("continuity: duplicate + missing timestamp in the window disqualifies the row", rejected(broken));
        std::vector<long long> dup = t;
        dup[wanted - 1] = dup[wanted - 2];                                                       // duplicate only
        expect("continuity: a duplicate timestamp alone disqualifies the row", rejected(dup));
        std::vector<long long> rev = t;
        std::swap(rev[wanted - 2], rev[wanted - 1]);                                             // reversed pair
        expect("continuity: a reversed pair disqualifies the row", rejected(rev));
        expect("continuity: a smaller window (3 readings) tolerates a hole 4 steps back",
               [&] { std::vector<long long> h = t; h.erase(h.begin() + static_cast<std::ptrdiff_t>(wanted - 4)); routing::ClockOrigin o; return routing::selectOriginForClock(h, 20, ts("2026-11-12 11:00:00"), o, 3) && h[o.row] == t[wanted]; }());
    }
    {   // all seven weekdays: each clock weekday selects a row on that weekday, all different
        std::vector<size_t> rows;
        bool allOk = true;
        for (int day = 0; day < 7; ++day) {                                                      // 2026-11-09 is a Monday
            routing::ClockOrigin o;
            const std::string clk = dp::fmt("2026-11-%02d 15:30:00", 9 + day);
            if (!routing::selectOriginForClock(series.times, earliest, ts(clk.c_str()), o) || !o.sameWeekday || routing::weekdayIndex(series.times[o.row]) != routing::weekdayIndex(ts(clk.c_str()))) allOk = false;
            rows.push_back(o.row);
        }
        std::sort(rows.begin(), rows.end());
        expect("weekdays: Monday to Sunday each select a row on their own weekday", allOk);
        expect("weekdays: the seven selected rows are all different", std::adjacent_find(rows.begin(), rows.end()) == rows.end());
    }
    {   // alignment: seconds and minutes between grid points always round DOWN
        routing::ClockOrigin x, y;
        expect("alignment: 11:24:59 rounds down to 11:20 (not up to 11:25)", routing::selectOriginForClock(series.times, earliest, ts("2026-11-11 11:24:59"), x) && x.row == a.row);
        expect("alignment: 11:25:00 is exactly on the grid", routing::selectOriginForClock(series.times, earliest, ts("2026-11-11 11:25:00"), y) && y.row == a.row + 1);
        expect("alignment: 00:00:01 maps to 00:00:00", routing::selectOriginForClock(series.times, earliest, ts("2026-11-11 00:00:01"), x) && dp::formatTimestamp(series.times[x.row]).substr(11) == "00:00:00");
        expect("alignment: 23:59:59 maps to 23:55:00 of the same weekday, never into the next day", routing::selectOriginForClock(series.times, earliest, ts("2026-11-11 23:59:59"), x) && dp::formatTimestamp(series.times[x.row]).substr(11) == "23:55:00" && routing::weekdayIndex(series.times[x.row]) == 6);
    }
    {   // dataset boundaries
        std::vector<long long> t;
        const long long start = ts("2012-06-14 00:00:00");
        for (long long i = 0; i < 7 * 288 + 150; ++i) t.push_back(start + i * 300);               // Thursday 00:00 ... next Thursday 12:25 (last row)
        routing::ClockOrigin m;
        expect("boundary: the very last row is selected when the clock matches it (Thursday 12:25)", routing::selectOriginForClock(t, 5, ts("2026-11-12 12:25:00"), m) && m.row == t.size() - 1);
        expect("boundary: Thursday 12:30 is after the last row, so the previous Thursday's 12:30 row is used",
               routing::selectOriginForClock(t, 5, ts("2026-11-12 12:30:00"), m) && m.sameWeekday && m.row == t.size() - 1 - 7 * 288 + 1);
        const size_t first = 100;                                                                  // earliest allowed row = first
        std::vector<long long> t2(t.begin(), t.begin() + static_cast<std::ptrdiff_t>(first) + 2);  // rows 0..101, so the newest rows are first and first+1
        routing::ClockOrigin e;
        expect("boundary: the earliest allowed row itself can be selected", routing::selectOriginForClock(t2, first, t2[first] + 42 * 86400, e) && e.row == first);
        expect("boundary: a row before the earliest allowed row is never selected", !routing::selectOriginForClock(t2, first + 1, t2[first] + 42 * 86400, e));
        std::vector<long long> tiny(t.begin(), t.begin() + 4);
        expect("boundary: fewer rows than the six-reading window cannot be an origin", !routing::selectOriginForClock(tiny, 0, tiny[3], e));
    }
    // the real clock
    {
        int off = 9999; long long now = 0;
        const bool ok = routing::localClockSeconds(now, off);
        expect("clock: the local clock is read as a plausible date and the UTC offset is within -14h..+14h", ok && now > ts("2025-01-01 00:00:00") && off >= -14 * 60 && off <= 14 * 60);
    }

    // ---------------------------------------------------------------- the CLI on the 28-day file
    const std::string csv = tmpPath("clock_test_traffic.csv");
    writeCsv(series, csv);
    const std::vector<std::string> base = {"--from", "r1", "--to", "r3", "--traffic", csv, "--distances", "data/sensor_distances.csv"};
    auto withExtra = [&](std::vector<std::string> extra) { std::vector<std::string> v = base; v.insert(v.end(), extra.begin(), extra.end()); return run(v); };

    const Run r1120 = withExtra({"--clock", "2026-11-11 11:20:00"});
    const Run r1200 = withExtra({"--clock", "2026-11-11 12:00:00"});
    expect("cli: --clock runs and exits 0", r1120.code == 0 && r1200.code == 0);
    expect("cli: the system clock line shows the simulated clock", has(r1120.out, "System clock   : 2026-11-11 11:20:00 (Wednesday, simulated with --clock)"));
    expect("cli: the selected historical origin is shown with its row and timestamp", has(r1120.out, "Forecast origin: row " + std::to_string(a.row) + " (" + dp::formatTimestamp(series.times[a.row]) + ")"));
    expect("cli: the forecast target is shown 15 minutes after the origin", has(r1120.out, "Forecast target: " + dp::formatTimestamp(series.times[a.row] + 900) + " (15 min after the origin)"));
    expect("cli: the output says it is a historical replay and not live traffic", has(r1120.out, "NOT live traffic"));
    expect("cli: two different clock times select two different historical origins", line(r1120.out, "Forecast origin:") != line(r1200.out, "Forecast origin:") && has(r1200.out, "row " + std::to_string(c.row) + " "));
    expect("cli: the two clock times give different forecasts", r1120.out.substr(r1120.out.find("Forecast speeds")) != r1200.out.substr(r1200.out.find("Forecast speeds")));
    const Run again = withExtra({"--clock", "2026-11-11 11:20:00"});
    expect("cli: the same clock value gives byte-identical output (reproducible)", again.out == r1120.out && again.code == 0);
    const Run viaRow = withExtra({"--row", std::to_string(a.row)});
    expect("cli: --clock gives the same route and ETA as --row of the selected row", viaRow.code == 0 && viaRow.out.substr(viaRow.out.find("Fastest route:")) == r1120.out.substr(r1120.out.find("Fastest route:")));
    expect("cli: --row still works and does not mention the clock", viaRow.code == 0 && !has(viaRow.out, "System clock") && has(viaRow.out, "[selected with --row]"));
    const Run viaTime = withExtra({"--time", dp::formatTimestamp(series.times[a.row])});
    expect("cli: --time still works", viaTime.code == 0 && has(viaTime.out, "[selected with --time]"));
    const Run latest = withExtra({"--latest"});
    expect("cli: --latest uses the last row", latest.code == 0 && has(latest.out, "row " + std::to_string(series.rows() - 1) + " ("));
    const Run noOption = withExtra({});
    expect("cli: the library function without an origin option still uses the latest row (unchanged behaviour)", noOption.code == 0 && has(noOption.out, "[default: the latest row in the dataset]"));
    const Run now = withExtra({"--auto-clock"});
    expect("cli: --auto-clock uses the real local clock and labels it local time", now.code == 0 && has(now.out, "(") && has(now.out, "local time, UTC") && has(now.out, "NOT live traffic"));
    expect("cli: --clock with a bad timestamp is a usage error (exit 2)", withExtra({"--clock", "yesterday"}).code == 2);
    expect("cli: --clock and --row together are refused (exit 2)", withExtra({"--clock", "2026-11-11 11:20:00", "--row", "7000"}).code == 2);
    expect("cli: --latest and --auto-clock together are refused (exit 2)", withExtra({"--latest", "--auto-clock"}).code == 2);
    expect("cli: --latest and --time together are refused (exit 2)", withExtra({"--latest", "--time", "2012-03-28 12:00:00"}).code == 2);
    expect("cli: --help documents the clock options", has(run({"--help"}).out, "--auto-clock") && has(run({"--help"}).out, "--clock") && has(run({"--help"}).out, "--latest"));

    // a file that is too short to cover a time of day: an error, nothing substituted
    const std::string shortCsv = tmpPath("clock_test_short.csv");
    writeCsv(syntheticSeries(400), shortCsv);                       // 400 rows from 06:00: only 11:20-15:25 after trainEnd-1... not 23:55
    const Run noRow = run({"--from", "r1", "--to", "r3", "--traffic", shortCsv, "--distances", "data/sensor_distances.csv", "--clock", "2026-11-11 05:00:00"});
    expect("cli: a clock time the file cannot represent exits 1 with an explanation and no substitution", noRow.code == 1 && has(noRow.err, "no usable row") && has(noRow.err, "No other time was substituted"));

    std::remove(csv.c_str());
    std::remove(shortCsv.c_str());
    return tu::summary();
}
