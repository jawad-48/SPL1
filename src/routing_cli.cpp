#include "routing_cli.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <cctype>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>

#include "clock_origin.h"
#include "route_planner.h"
#include "speed_forecaster.h"
#include "travel_time_graph.h"

namespace routing {
namespace {

// Relative paths (never an absolute Windows path): the processed full dataset, as produced by the data pipeline.
const char* const DEFAULT_TRAFFIC = "data/full/processed/traffic_data.csv";
const char* const DEFAULT_DISTANCES = "data/full/processed/sensor_distances.csv";

const int EXIT_OK = 0, EXIT_DATA = 1, EXIT_USAGE = 2, EXIT_NO_ROUTE = 3;

// A failure that ends the program with a given exit code and a complete message for the user.
struct CliError {
    int code;
    std::string message;
};

std::string routeNameList();     // defined below

std::string sensorList() {
    std::string s;
    for (size_t i = 0; i < dp::SELECTED_SENSORS.size(); ++i) s += (i ? ", " : "") + dp::SELECTED_SENSORS[i];
    return s;
}

void printHelp(std::ostream& out) {
    out << "fastest route: Linear Regression speed forecast + Dijkstra\n"
           "\n"
           "Usage: routing [--from SENSOR] [--to SENSOR] [--row N | --time \"YYYY-MM-DD HH:MM:SS\"]\n"
           "               [--traffic PATH] [--distances PATH]\n"
           "\n"
           "  --from SENSOR      where you are: a sensor ID or a route name r1 to r6\n"
           "  --to SENSOR        where you want to go: a sensor ID or a route name r1 to r6\n"
           "                     (if --from and/or --to are left out, the program asks for them)\n"
           "  --row N            forecast from dataset row N (0 = first data row)\n"
           "  --time TIMESTAMP   forecast from the row with exactly this timestamp, e.g. \"2012-06-27 17:55:00\"\n"
           "  --auto-clock       choose the origin from this computer's clock: same weekday and time of day (5-minute grid)\n"
           "                     in the dataset, latest eligible date. A HISTORICAL REPLAY of 2012 data, not live traffic.\n"
           "                     The routing program does this by default when no origin option is given.\n"
           "  --clock TIMESTAMP  like --auto-clock, but with a simulated local clock, e.g. \"2026-11-11 11:20:00\" (demos, tests)\n"
           "  --latest           use the LATEST row of the dataset\n"
           "                     (--row, --time, --latest and --auto-clock/--clock cannot be combined)\n"
           "  --traffic PATH     cleaned traffic file   (default " << DEFAULT_TRAFFIC << ")\n"
           "  --distances PATH   sensor distance file   (default " << DEFAULT_DISTANCES << ")\n"
           "  --help             show this text\n"
           "\n"
           "Sensors: " << sensorList() << "\n"
           "Route names: " << routeNameList() << "\n"
           "The forecast origin must not lie inside the training period (the first 70% of the rows).\n"
           "The forecast is made for 15 minutes after the origin row; nothing after the origin row is used.\n"
           "\n"
           "Exit codes: 0 route calculated | 1 file, dataset or origin-time problem |\n"
           "            2 command-line mistake | 3 valid request but no route can be calculated\n";
}

bool isValueOption(const std::string& a) {
    return a == "--from" || a == "--to" || a == "--traffic" || a == "--distances" || a == "--time" || a == "--row" || a == "--clock";
}

// Route names: r1 to r6 follow the order of dp::SELECTED_SENSORS (r1 = 716328 ... r6 = 773062).
std::string routeName(const std::string& sensorId) {
    for (size_t i = 0; i < dp::SELECTED_SENSORS.size(); ++i) if (dp::SELECTED_SENSORS[i] == sensorId) return "r" + std::to_string(i + 1);
    return "";
}

std::string routeNameList() {
    std::string s;
    for (size_t i = 0; i < dp::SELECTED_SENSORS.size(); ++i) s += (i ? ", r" : "r") + std::to_string(i + 1) + " = " + dp::SELECTED_SENSORS[i];
    return s;
}

// A sensor ID, or a route name r1 to r6 (r or R). Exact match, no trimming. Returns false if `text` is neither.
bool resolveSensor(const std::string& text, std::string& sensorId) {
    if (std::find(dp::SELECTED_SENSORS.begin(), dp::SELECTED_SENSORS.end(), text) != dp::SELECTED_SENSORS.end()) { sensorId = text; return true; }
    if (text.size() == 2 && (text[0] == 'r' || text[0] == 'R') && text[1] >= '1' && text[1] < '1' + static_cast<char>(dp::SELECTED_SENSORS.size())) {
        sensorId = dp::SELECTED_SENSORS[static_cast<size_t>(text[1] - '1')];
        return true;
    }
    return false;
}

std::string requireSensor(const std::string& option, const std::string& text) {
    std::string id;
    if (!resolveSensor(text, id))
        throw CliError{EXIT_USAGE, "ERROR: " + option + " '" + text + "' is not one of the six sensors. Valid sensors: " + sensorList() + " (or the route names r1 to r6)"};
    return id;
}

// Asks one question on `in`/`out`; accepts r1 to r6 or a sensor ID (surrounding spaces ignored); 3 tries.
std::string askSensor(std::istream& in, std::ostream& out, const std::string& question) {
    for (int attempt = 0; attempt < 3; ++attempt) {
        out << question << " " << std::flush;
        std::string line;
        if (!std::getline(in, line))
            throw CliError{EXIT_USAGE, "ERROR: no answer was entered (the input ended). Give --from and --to on the command line instead."};
        line = dp::trim(line);
        std::string id;
        if (resolveSensor(line, id)) return id;
        out << "  '" << line << "' is not a route. Type r1 to r6, or one of the sensor IDs.\n";
    }
    throw CliError{EXIT_USAGE, "ERROR: no valid route was entered after 3 tries."};
}

void printRouteMenu(std::ostream& out) {
    out << "Routes (sensors):\n";
    for (size_t i = 0; i < dp::SELECTED_SENSORS.size(); ++i) out << "  r" << i + 1 << "  " << dp::SELECTED_SENSORS[i] << "\n";
    out << "\n";
}

void requireReadableFile(const std::string& option, const std::string& what, const std::string& path) {
    std::ifstream in(path.c_str());
    if (!in) throw CliError{EXIT_DATA, "ERROR: the " + what + " file '" + path + "' does not exist or cannot be read. Give the right file with " + option + " PATH."};
}

std::string describeOrigin(const baseline::TrafficSeries& series, size_t row) {
    return dp::fmt("row %zu (%s)", row, dp::formatTimestamp(series.times[row]).c_str());
}

// Why a sensor has no forecast, in plain words, from the cleaned file itself: which of the readings the forecast needs are
// empty and when the sensor last had a real reading. Nothing is invented; this only reads the series.
std::string explainUnavailable(const baseline::TrafficSeries& series, size_t origin, size_t sensor, int lags, const std::string& reason) {
    const std::string name = routeName(series.sensors[sensor]) + " (" + series.sensors[sensor] + ")";
    const size_t n = static_cast<size_t>(lags);
    // same order of checks as SpeedForecaster::predictAt: history length, then the time grid, then the readings themselves
    if (origin + 1 < n) return "  Why " + name + ": only " + std::to_string(origin + 1) + " readings exist up to the origin, " + std::to_string(n) + " are needed.";
    for (size_t i = origin + 1 - n + 1; i <= origin; ++i) {
        const long long d = series.times[i] - series.times[i - 1];
        if (d != dp::STEP_SECONDS)
            return "  Why " + name + ": the timestamps of its last " + std::to_string(n) + " readings are not exactly 5 minutes apart (" + dp::formatTimestamp(series.times[i - 1]) +
                   " to " + dp::formatTimestamp(series.times[i]) + " is " + std::to_string(d) + " s). A forecast needs an unbroken 5-minute series; no value was invented.";
    }
    std::vector<std::string> empty;
    for (size_t k = 0; k < n; ++k) {
        const double v = series.speeds[sensor][origin - k];
        if (!std::isfinite(v) || v <= 0) empty.push_back(dp::formatTimestamp(series.times[origin - k]).substr(11, 8));
    }
    if (empty.empty()) return "  Why " + name + ": no forecast was made (" + reason + ").";
    std::string list;
    for (size_t i = 0; i < empty.size(); ++i) list += (i ? ", " : "") + empty[i];
    std::string s = "  Why " + name + ": " + std::to_string(empty.size()) + " of its last " + std::to_string(n) + " readings are empty in the cleaned file (" + list + ")";
    size_t r = origin;
    while (true) {
        const double v = series.speeds[sensor][r];
        if (std::isfinite(v) && v > 0) break;
        if (r == 0) { r = static_cast<size_t>(-1); break; }
        --r;
    }
    if (r == static_cast<size_t>(-1)) s += "; it has no real reading before the origin";
    else s += dp::fmt("; its last real reading was at %s, %zu steps (%zu min) before the origin", dp::formatTimestamp(series.times[r]).c_str(), origin - r, (origin - r) * 5);
    s += ". The cleaning step carries a reading forward for at most 6 steps (30 min); a longer outage stays empty. No speed was invented.";
    return s;
}

}  // namespace

namespace {

// `in` == nullptr: never ask questions (--from and --to are required).
int runImpl(const std::vector<std::string>& args, std::istream* in, std::ostream& out, std::ostream& err) {
    try {
        // ---------------------------------------------------------------- 1. command line (nothing is read from disk yet)
        if (std::find(args.begin(), args.end(), "--help") != args.end() || std::find(args.begin(), args.end(), "-h") != args.end()) {
            printHelp(out);
            return EXIT_OK;
        }
        std::map<std::string, std::string> v;
        for (size_t i = 0; i < args.size(); ++i) {
            const std::string& a = args[i];
            if (a == "--auto-clock" || a == "--latest") {
                if (v.count(a)) throw CliError{EXIT_USAGE, "ERROR: option " + a + " was given twice."};
                v[a] = "1";
                continue;
            }
            if (!isValueOption(a)) throw CliError{EXIT_USAGE, "ERROR: unknown option '" + a + "'. Try --help."};
            if (i + 1 >= args.size() || args[i + 1].compare(0, 2, "--") == 0) throw CliError{EXIT_USAGE, "ERROR: option " + a + " needs a value. Try --help."};
            if (v.count(a)) throw CliError{EXIT_USAGE, "ERROR: option " + a + " was given twice."};
            v[a] = args[++i];
        }
        const bool mustAsk = !v.count("--from") || !v.count("--to");
        if (mustAsk && in == nullptr) throw CliError{EXIT_USAGE, "ERROR: --from and --to are required. Try --help."};
        std::string source, destination;
        if (v.count("--from")) source = requireSensor("--from", v["--from"]);
        if (v.count("--to")) destination = requireSensor("--to", v["--to"]);
        if (v.count("--row") && v.count("--time")) throw CliError{EXIT_USAGE, "ERROR: use either --row or --time, not both."};
        const bool autoClock = v.count("--auto-clock") || v.count("--clock");
        if ((v.count("--latest") ? 1 : 0) + (autoClock ? 1 : 0) + (v.count("--row") || v.count("--time") ? 1 : 0) > 1)
            throw CliError{EXIT_USAGE, "ERROR: choose only one way to select the origin: --row, --time, --latest, or --auto-clock/--clock."};
        long long simulatedClock = 0;
        if (v.count("--clock") && !dp::parseTimestamp(v["--clock"], simulatedClock))
            throw CliError{EXIT_USAGE, "ERROR: --clock must look like 2026-11-11 11:20:00 (got '" + v["--clock"] + "')."};

        bool haveRow = false, haveTime = false;
        size_t requestedRow = 0;
        long long requestedTime = 0;
        if (v.count("--row")) {
            const std::string& t = v["--row"];
            if (t.empty() || t.size() > 18 || t.find_first_not_of("0123456789") != std::string::npos)
                throw CliError{EXIT_USAGE, "ERROR: --row must be a whole number 0 or greater (got '" + t + "')."};
            requestedRow = static_cast<size_t>(std::stoull(t));
            haveRow = true;
        }
        if (v.count("--time")) {
            if (!dp::parseTimestamp(v["--time"], requestedTime))
                throw CliError{EXIT_USAGE, "ERROR: --time must look like 2012-06-27 17:55:00 (got '" + v["--time"] + "')."};
            haveTime = true;
        }
        const std::string trafficPath = v.count("--traffic") ? v["--traffic"] : DEFAULT_TRAFFIC;
        const std::string distancePath = v.count("--distances") ? v["--distances"] : DEFAULT_DISTANCES;

        // ---------------------------------------------------------------- 2. load the two datasets (existing loaders)
        requireReadableFile("--traffic", "traffic", trafficPath);
        requireReadableFile("--distances", "distance", distancePath);

        // ---- "where am I, where do I want to go": asked only when --from and/or --to were left out, and only after every
        //      other argument and both file names have been checked, so a wrong option never wastes the answers
        if (mustAsk) {
            out << "Where are you now, and where do you want to go?\n\n";
            printRouteMenu(out);
            if (!v.count("--from")) source = askSensor(*in, out, "Where are you now?        (r1-r6 or sensor ID):");
            if (!v.count("--to")) destination = askSensor(*in, out, "Where do you want to go?  (r1-r6 or sensor ID):");
            out << "\n";
        }
        baseline::TrafficSeries series;
        try {
            series = baseline::loadTrafficSeries(trafficPath);
        } catch (const std::exception& e) {
            throw CliError{EXIT_DATA, std::string("ERROR: the traffic file is not usable: ") + e.what()};
        }
        std::vector<DistanceRecord> distances;
        try {
            distances = loadSensorDistances(distancePath);
        } catch (const std::exception& e) {
            throw CliError{EXIT_DATA, std::string("ERROR: the distance file is not usable: ") + e.what()};
        }

        // ---------------------------------------------------------------- 3. the training period and the forecast origin
        const baseline::BaselineConfig config;                       // the established configuration: 6 readings, 3 steps, 70/15/15
        baseline::SplitBoundaries split;
        try {
            split = baseline::chronologicalBoundaries(series.rows(), config.trainFraction, config.validationFraction);
        } catch (const std::exception& e) {
            throw CliError{EXIT_DATA, std::string("ERROR: the traffic file cannot be split into training, validation and test periods: ") + e.what()};
        }
        const size_t earliest = split.trainEnd - 1;                  // first row a model trained on rows 0..trainEnd-1 may forecast from

        size_t origin = series.rows() - 1;
        std::string originHow = "default: the latest row in the dataset";
        if (haveRow) {
            if (requestedRow >= series.rows())
                throw CliError{EXIT_DATA, dp::fmt("ERROR: row %zu does not exist; the traffic file has rows 0 to %zu.", requestedRow, series.rows() - 1)};
            origin = requestedRow;
            originHow = "selected with --row";
        } else if (haveTime) {
            const std::vector<long long>::const_iterator it = std::lower_bound(series.times.begin(), series.times.end(), requestedTime);
            if (it == series.times.end() || *it != requestedTime)
                throw CliError{EXIT_DATA, "ERROR: no row has the timestamp " + dp::formatTimestamp(requestedTime) + ". The traffic file runs from " +
                                          dp::formatTimestamp(series.times.front()) + " to " + dp::formatTimestamp(series.times.back()) +
                                          ", one row every 5 minutes. No other time was substituted."};
            origin = static_cast<size_t>(it - series.times.begin());
            originHow = "selected with --time";
        }
        std::string clockLines;
        if (autoClock) {
            int offset = 0;
            const bool simulated = v.count("--clock") > 0;
            long long clock = simulatedClock;
            if (!simulated && !localClockSeconds(clock, offset))
                throw CliError{EXIT_DATA, "ERROR: the computer's clock could not be read. Choose the origin yourself with --row or --time, or simulate a clock with --clock."};
            ClockOrigin chosen;
            if (!selectOriginForClock(series.times, earliest, clock, chosen, static_cast<size_t>(config.features.lags)))
                throw CliError{EXIT_DATA, "ERROR: the dataset has no usable row for the clock time " + dp::formatTimestamp(clock) +
                                          " (an allowed origin row at this time of day, with five regular 5-minute steps before it, is required). "
                                          "Choose the origin yourself with --row or --time. No other time was substituted."};
            origin = chosen.row;
            originHow = chosen.sameWeekday ? "clock replay: same weekday and time of day, latest eligible date"
                                           : "clock replay: NO eligible row on this weekday, so the latest eligible row with this time of day";
            std::string zone = simulated ? "simulated with --clock" : dp::fmt("local time, UTC%c%02d:%02d", offset < 0 ? '-' : '+', std::abs(offset) / 60, std::abs(offset) % 60);
            clockLines += "System clock   : " + dp::formatTimestamp(clock) + " (" + dp::weekdayName(clock) + ", " + zone + ")\n";
            clockLines += "Replay mapping : clock " + dp::formatTimestamp(clock).substr(11, 5) + " (rounded down to the 5-minute grid) -> dataset " +
                          dp::weekdayName(series.times[origin]) + " " + dp::formatTimestamp(series.times[origin]) + "\n";
        }
        if (origin < earliest)
            throw CliError{EXIT_DATA, "ERROR: " + describeOrigin(series, origin) + " lies inside the training period (rows 0 to " + std::to_string(split.trainEnd - 1) +
                                      "). The models have already seen the data after it, so a forecast from there would use future information. The earliest allowed origin is " +
                                      describeOrigin(series, earliest) + ". No other time was substituted."};

        // ---------------------------------------------------------------- 4. header, then train (the slow step)
        const int horizonMinutes = config.features.horizonSteps * 5, historyMinutes = config.features.lags * 5;
        out << "SPL-1 fastest route: Linear Regression forecast + Dijkstra\n";
        out << "Traffic data   : " << trafficPath << " (" << series.rows() << " rows, " << dp::formatTimestamp(series.times.front()) << " to " << dp::formatTimestamp(series.times.back()) << ")\n";
        out << "Distance data  : " << distancePath << " (" << distances.size() << " directed sensor pairs)\n";
        out << "Training       : rows 0 to " << split.trainEnd - 1 << " (" << dp::formatTimestamp(series.times.front()) << " to " << dp::formatTimestamp(series.times[split.trainEnd - 1])
            << "), one model per sensor, " << config.features.lags << " readings (" << historyMinutes << " min) -> speed " << horizonMinutes << " min ahead\n";
        out << clockLines;
        out << "Forecast origin: " << describeOrigin(series, origin) << " [" << originHow << "]\n";
        if (autoClock)
            out << "Forecast target: " << dp::formatTimestamp(series.times[origin] + static_cast<long long>(config.features.horizonSteps) * dp::STEP_SECONDS)
                << " (" << horizonMinutes << " min after the origin)\n"
                << "Note           : historical replay of the 2012 data driven by the clock, NOT live traffic.\n";
        out << "Route names    : " << routeNameList() << "\n";
        out << "From / To      : " << source << " -> " << destination << "  (" << routeName(source) << " -> " << routeName(destination) << ")\n\n";
        out << "Training the six Linear Regression models ...\n" << std::flush;

        SpeedForecaster forecaster;
        try {
            forecaster = SpeedForecaster::train(series, config);
        } catch (const std::exception& e) {
            throw CliError{EXIT_DATA, std::string("ERROR: the models cannot be trained on this traffic file (it may have too few usable rows): ") + e.what()};
        }
        if (forecaster.trainEndRow() != split.trainEnd) throw std::logic_error("internal error: the forecaster's training cutoff differs from the split");

        // ---------------------------------------------------------------- 5. forecast at the origin
        const ForecastSnapshot snapshot = forecaster.predictAt(series, origin);
        out << "Forecast speeds for " << dp::formatTimestamp(snapshot.targetTime) << " (" << horizonMinutes << " min after the origin), from the readings up to "
            << dp::formatTimestamp(snapshot.originTime) << ":\n";
        for (size_t i = 0; i < snapshot.sensors.size(); ++i) {
            const SensorForecast& s = snapshot.sensors[i];
            if (s.available) out << "  " << routeName(s.sensorId) << "  " << s.sensorId << "  " << dp::fmt("%6.2f", s.speedMph) << " mph\n";
            else out << "  " << routeName(s.sensorId) << "  " << s.sensorId << "  unavailable: " << s.reason << "\n";
        }
        for (size_t i = 0; i < snapshot.sensors.size(); ++i)
            if (!snapshot.sensors[i].available) out << explainUnavailable(series, origin, i, config.features.lags, snapshot.sensors[i].reason) << "\n";
        out << "\n";

        // ---------------------------------------------------------------- 6. travel-time graph + Dijkstra (existing planner)
        RouteResult route;
        try {
            route = planRouteFromSnapshot(snapshot, distances, source, destination);
        } catch (const std::runtime_error& e) {                       // the source sensor has no forecast
            throw CliError{EXIT_NO_ROUTE, std::string("No route can be calculated: ") + e.what() + ". Its outgoing travel times cannot be priced, and no other speed was substituted."};
        }
        if (!route.reachable) {
            std::string why = "No route can be calculated: with the available forecasts and distances there is no way from " + source + " to " + destination + ".";
            for (size_t i = 0; i < route.unavailableSensors.size(); ++i)
                why += "\n  sensor " + route.unavailableSensors[i].sensorId + " has no forecast (" + route.unavailableSensors[i].reason + "), so its outgoing edges were left out.";
            throw CliError{EXIT_NO_ROUTE, why};
        }

        // ---------------------------------------------------------------- 7. result
        out << "Fastest route: ";
        for (size_t i = 0; i < route.path.size(); ++i) out << (i ? " -> " : "") << route.path[i];
        out << "\n";
        out << "  In route names: ";
        for (size_t i = 0; i < route.path.size(); ++i) out << (i ? " -> " : "") << routeName(route.path[i]);
        out << "\n";
        if (route.legs.empty()) {
            out << "  The origin and the destination are the same sensor: no travel needed.\n";
        } else {
            out << "  Legs (speed = forecast of the sensor each leg starts from):\n";
            out << dp::fmt("  %-3s  %-6s  %-6s  %11s    %11s    %12s", "leg", "from", "to", "distance", "speed", "travel time") << "\n";
            for (size_t i = 0; i < route.legs.size(); ++i) {
                const RouteLeg& leg = route.legs[i];
                out << dp::fmt("  %-3zu  %-6s  %-6s  %9.1f m    %7.2f mph    %8.3f min", i + 1, leg.from.c_str(), leg.to.c_str(), leg.distanceM, leg.speedMph, leg.minutes) << "\n";
            }
        }
        out << "Total ETA: " << dp::fmt("%.3f", route.etaMinutes) << " minutes\n";
        for (size_t i = 0; i < route.unavailableSensors.size(); ++i)
            out << "Note: sensor " << route.unavailableSensors[i].sensorId << " (" << routeName(route.unavailableSensors[i].sensorId)
                << ") has no forecast, so no leg can start from it: routes passing through it were not considered (it can still be the destination).\n";
        out << "(Upstream-speed assumption and other limits: docs/predictive_routing.md.)\n";
        return EXIT_OK;
    } catch (const CliError& e) {
        err << e.message << "\n";
        return e.code;
    } catch (const std::exception& e) {                               // anything unexpected, e.g. training failed or a forecast was rejected
        err << "ERROR: " << e.what() << "\n";
        return EXIT_DATA;
    }
}

}  // namespace

int runRoutingCli(const std::vector<std::string>& args, std::ostream& out, std::ostream& err) {
    return runImpl(args, nullptr, out, err);
}

int runRoutingCli(const std::vector<std::string>& args, std::istream& in, std::ostream& out, std::ostream& err) {
    return runImpl(args, &in, out, err);
}

}  // namespace routing
