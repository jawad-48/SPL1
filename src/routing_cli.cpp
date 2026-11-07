#include "routing_cli.h"

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>

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

std::string sensorList() {
    std::string s;
    for (size_t i = 0; i < dp::SELECTED_SENSORS.size(); ++i) s += (i ? ", " : "") + dp::SELECTED_SENSORS[i];
    return s;
}

void printHelp(std::ostream& out) {
    out << "fastest route: Linear Regression speed forecast + Dijkstra\n"
           "\n"
           "Usage: routing --from SENSOR --to SENSOR [--row N | --time \"YYYY-MM-DD HH:MM:SS\"]\n"
           "               [--traffic PATH] [--distances PATH]\n"
           "\n"
           "  --from SENSOR      origin sensor ID (required)\n"
           "  --to SENSOR        destination sensor ID (required)\n"
           "  --row N            forecast from dataset row N (0 = first data row)\n"
           "  --time TIMESTAMP   forecast from the row with exactly this timestamp, e.g. \"2012-06-27 17:55:00\"\n"
           "                     (--row and --time cannot be combined; if neither is given the LATEST row is used)\n"
           "  --traffic PATH     cleaned traffic file   (default " << DEFAULT_TRAFFIC << ")\n"
           "  --distances PATH   sensor distance file   (default " << DEFAULT_DISTANCES << ")\n"
           "  --help             show this text\n"
           "\n"
           "Sensors: " << sensorList() << "\n"
           "The forecast origin must not lie inside the training period (the first 70% of the rows).\n"
           "The forecast is made for 15 minutes after the origin row; nothing after the origin row is used.\n"
           "\n"
           "Exit codes: 0 route calculated | 1 file, dataset or origin-time problem |\n"
           "            2 command-line mistake | 3 valid request but no route can be calculated\n";
}

bool isValueOption(const std::string& a) {
    return a == "--from" || a == "--to" || a == "--traffic" || a == "--distances" || a == "--time" || a == "--row";
}

void requireSensor(const std::string& option, const std::string& id) {
    if (std::find(dp::SELECTED_SENSORS.begin(), dp::SELECTED_SENSORS.end(), id) == dp::SELECTED_SENSORS.end())
        throw CliError{EXIT_USAGE, "ERROR: " + option + " '" + id + "' is not one of the six sensors. Valid sensors: " + sensorList()};
}

void requireReadableFile(const std::string& option, const std::string& what, const std::string& path) {
    std::ifstream in(path.c_str());
    if (!in) throw CliError{EXIT_DATA, "ERROR: the " + what + " file '" + path + "' does not exist or cannot be read. Give the right file with " + option + " PATH."};
}

std::string describeOrigin(const baseline::TrafficSeries& series, size_t row) {
    return dp::fmt("row %zu (%s)", row, dp::formatTimestamp(series.times[row]).c_str());
}

}  // namespace

int runRoutingCli(const std::vector<std::string>& args, std::ostream& out, std::ostream& err) {
    try {
        // ---------------------------------------------------------------- 1. command line (nothing is read from disk yet)
        if (std::find(args.begin(), args.end(), "--help") != args.end() || std::find(args.begin(), args.end(), "-h") != args.end()) {
            printHelp(out);
            return EXIT_OK;
        }
        std::map<std::string, std::string> v;
        for (size_t i = 0; i < args.size(); ++i) {
            const std::string& a = args[i];
            if (!isValueOption(a)) throw CliError{EXIT_USAGE, "ERROR: unknown option '" + a + "'. Try --help."};
            if (i + 1 >= args.size() || args[i + 1].compare(0, 2, "--") == 0) throw CliError{EXIT_USAGE, "ERROR: option " + a + " needs a value. Try --help."};
            if (v.count(a)) throw CliError{EXIT_USAGE, "ERROR: option " + a + " was given twice."};
            v[a] = args[++i];
        }
        if (!v.count("--from") || !v.count("--to")) throw CliError{EXIT_USAGE, "ERROR: --from and --to are required. Try --help."};
        const std::string source = v["--from"], destination = v["--to"];
        requireSensor("--from", source);
        requireSensor("--to", destination);
        if (v.count("--row") && v.count("--time")) throw CliError{EXIT_USAGE, "ERROR: use either --row or --time, not both."};

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
        out << "Forecast origin: " << describeOrigin(series, origin) << " [" << originHow << "]\n";
        out << "From / To      : " << source << " -> " << destination << "\n\n";
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
            if (s.available) out << "  " << s.sensorId << "  " << dp::fmt("%6.2f", s.speedMph) << " mph\n";
            else out << "  " << s.sensorId << "  unavailable: " << s.reason << "\n";
        }
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

}  // namespace routing
