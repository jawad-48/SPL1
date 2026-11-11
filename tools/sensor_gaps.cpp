// sensor_gaps.cpp -- read-only diagnostic: WHY is a sensor's forecast unavailable, and is it a one-off or systematic?
//
// Build (from the repository root; Windows PowerShell and Linux use the same command):
//   g++ -std=c++17 -O2 -Wall -Wextra -o build/sensor_gaps tools/sensor_gaps.cpp src/traffic_data.cpp src/baseline_pipeline.cpp src/features.cpp src/metrics.cpp src/linear_regression.cpp
// Run:
//   build/sensor_gaps --processed data/full/processed/traffic_data.csv [--sensor 717446] [--raw-h5 data/full/metr-la.h5 | --raw-csv PATH] [--last 12]
//
// It never writes anything. It prints, for every selected sensor, how many readings are empty in the cleaned file, the longest empty
// run, and for how many allowed forecast origins the six-reading window has no usable forecast input. The allowed origins and the
// window rule are the ones the routing program uses: the same BaselineConfig split (chronologicalBoundaries) and the same checks as
// SpeedForecaster::predictAt (unbroken 5-minute timestamps, every reading finite and positive). For the chosen sensor it also lists
// the last rows of the cleaned file next to the original raw values (matched by timestamp and by sensor ID).
// It reports facts about the files only; it does not decide the cause for you.
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include "../src/baseline_pipeline.h"
#include "../src/traffic_data.h"
#include "data_pipeline.h"

static bool usable(double v) { return std::isfinite(v) && v > 0; }

int main(int argc, char** argv) {
    std::string processed, rawH5, rawCsv, sensor = "717446";
    size_t last = 12;
    for (int i = 1; i < argc; ++i) {
        const bool more = i + 1 < argc;
        if (!std::strcmp(argv[i], "--processed") && more) processed = argv[++i];
        else if (!std::strcmp(argv[i], "--sensor") && more) sensor = argv[++i];
        else if (!std::strcmp(argv[i], "--raw-h5") && more) rawH5 = argv[++i];
        else if (!std::strcmp(argv[i], "--raw-csv") && more) rawCsv = argv[++i];
        else if (!std::strcmp(argv[i], "--last") && more) {
            const long v = std::strtol(argv[++i], nullptr, 10);
            if (v < 1 || v > 100000) { std::cerr << "ERROR: --last must be a number from 1 to 100000\n"; return 2; }
            last = static_cast<size_t>(v);
        } else { std::cerr << "usage: sensor_gaps --processed FILE [--sensor ID] [--raw-h5 FILE | --raw-csv FILE] [--last N]\n"; return 2; }
    }
    if (processed.empty()) { std::cerr << "ERROR: --processed FILE is required\n"; return 2; }
    if (!rawH5.empty() && !rawCsv.empty()) { std::cerr << "ERROR: give --raw-h5 or --raw-csv, not both\n"; return 2; }
    try {
        const baseline::TrafficSeries s = baseline::loadTrafficSeries(processed);      // throws on an empty file or a file without data rows
        const size_t n = s.rows();
        const baseline::BaselineConfig config;                                         // the routing program's configuration: 6 readings, 70/15/15
        baseline::SplitBoundaries split;
        try {
            split = baseline::chronologicalBoundaries(n, config.trainFraction, config.validationFraction);
        } catch (const std::exception& e) {
            std::cerr << "ERROR: this file (" << n << " rows) cannot be split into training, validation and test periods: " << e.what() << "\n";
            return 1;
        }
        if (split.trainEnd < 1 || split.trainEnd > n) { std::cerr << "ERROR: unexpected split boundary " << split.trainEnd << " for " << n << " rows\n"; return 1; }
        const size_t earliest = split.trainEnd - 1;                                    // same as the routing program
        const size_t lags = static_cast<size_t>(config.features.lags);

        std::cout << "Cleaned file: " << processed << " (" << n << " rows, " << dp::formatTimestamp(s.times.front()) << " to " << dp::formatTimestamp(s.times.back()) << ")\n";
        std::cout << "Allowed forecast origins: rows " << earliest << " to " << n - 1 << " (" << dp::formatTimestamp(s.times[earliest]) << " to " << dp::formatTimestamp(s.times.back()) << ")\n\n";
        std::cout << dp::fmt("%-8s %7s %11s   %s\n", "sensor", "empty", "longest run", "origins without a forecast input");
        std::vector<bool> gridOk(n, false);                                            // the lags timestamps ending at row r are 5 minutes apart
        for (size_t r = lags - 1; r < n; ++r) {
            bool ok = true;
            for (size_t i = r + 1 - lags + 1; i <= r; ++i) if (s.times[i] - s.times[i - 1] != dp::STEP_SECONDS) { ok = false; break; }
            gridOk[r] = ok;
        }
        for (size_t k = 0; k < s.sensors.size(); ++k) {
            size_t empty = 0, run = 0, longest = 0, bad = 0, origins = 0;
            for (size_t r = 0; r < n; ++r) {
                if (!usable(s.speeds[k][r])) { ++empty; ++run; if (run > longest) longest = run; } else run = 0;
            }
            for (size_t r = earliest; r < n; ++r) {
                ++origins;
                bool ok = r + 1 >= lags && gridOk[r];
                for (size_t j = 0; ok && j < lags; ++j) if (!usable(s.speeds[k][r - j])) ok = false;
                if (!ok) ++bad;
            }
            std::cout << dp::fmt("%-8s %7zu %11zu   %zu of %zu (%.2f%%)\n", s.sensors[k].c_str(), empty, longest, bad, origins, origins ? 100.0 * static_cast<double>(bad) / static_cast<double>(origins) : 0.0);
        }
        size_t k = 0;
        while (k < s.sensors.size() && s.sensors[k] != sensor) ++k;
        if (k == s.sensors.size()) { std::cerr << "ERROR: sensor " << sensor << " is not one of the six selected sensors\n"; return 2; }

        std::cout << "\nEmpty runs of " << sensor << " in the cleaned file (start -> end, steps):\n";
        size_t shown = 0, total = 0;
        for (size_t r = 0; r < n; ++r) {
            if (usable(s.speeds[k][r])) continue;
            size_t e = r;
            while (e + 1 < n && !usable(s.speeds[k][e + 1])) ++e;
            ++total;
            if (shown < 15) { std::cout << "  " << dp::formatTimestamp(s.times[r]) << " -> " << dp::formatTimestamp(s.times[e]) << "  (" << e - r + 1 << " steps)" << (e == n - 1 ? "   <- reaches the last row" : "") << "\n"; ++shown; }
            r = e;
        }
        if (total == 0) std::cout << "  none\n"; else if (total > shown) std::cout << "  ... " << total - shown << " more runs\n";

        // raw values: matched by TIMESTAMP (binary search on the raw times) and by SENSOR ID (the raw column whose ID equals `sensor`)
        dp::SpeedTable raw;
        const bool haveRaw = !rawH5.empty() || !rawCsv.empty();
        size_t rawCol = 0;
        if (haveRaw) {
            raw = !rawH5.empty() ? dp::loadSpeedsH5(rawH5) : dp::loadSpeedsCsv(rawCsv);
            while (rawCol < raw.ids.size() && raw.ids[rawCol] != sensor) ++rawCol;
            if (rawCol == raw.ids.size()) { std::cerr << "ERROR: sensor " << sensor << " is not a column of the raw file\n"; return 1; }
        }
        auto rawRow = [&](long long t) -> size_t {
            size_t lo = 0, hi = raw.times.size();
            while (lo < hi) { const size_t mid = (lo + hi) / 2; if (raw.times[mid] < t) lo = mid + 1; else hi = mid; }
            return (lo < raw.times.size() && raw.times[lo] == t) ? lo : raw.times.size();
        };
        std::cout << "\nLast " << last << " rows for " << sensor << (haveRaw ? " (raw value | cleaned value)" : " (cleaned value; give --raw-h5 or --raw-csv to compare with the original)") << ":\n";
        for (size_t r = n > last ? n - last : 0; r < n; ++r) {
            std::string rawText = "-";
            if (haveRaw) {
                const size_t rr = rawRow(s.times[r]);
                rawText = rr == raw.times.size() ? "(no such row)" : (std::isnan(raw.rows[rr][rawCol]) ? "NaN" : dp::fmt("%.4f", raw.rows[rr][rawCol]));
            }
            std::cout << "  " << dp::formatTimestamp(s.times[r]) << "   " << dp::fmt("%-10s", rawText.c_str()) << " | " << (usable(s.speeds[k][r]) ? dp::fmt("%.4f", s.speeds[k][r]) : std::string("empty")) << "\n";
        }
        if (haveRaw) {
            size_t compared = 0, changed = 0, notInRaw = 0, filled = 0;
            for (size_t r = 0; r < n; ++r) {
                const size_t rr = rawRow(s.times[r]);
                if (rr == raw.times.size()) { ++notInRaw; continue; }
                const double rv = raw.rows[rr][rawCol], cv = s.speeds[k][r];
                if (usable(rv) && usable(cv)) { ++compared; if (std::fabs(rv - cv) > 0.0001) ++changed; }
                else if (!usable(rv) && usable(cv)) ++filled;
            }
            std::cout << "\nRaw vs cleaned for " << sensor << ": " << compared << " rows where both have a value (" << changed << " differ by more than 0.0001), "
                      << filled << " rows where the raw value is missing but the cleaned file has a carried-forward value, " << notInRaw << " cleaned rows with no raw row at that timestamp.\n";
        }
        std::cout << "\nA raw value of 0, NaN or below 0 counts as missing. The cleaning policy carries the previous real reading forward for at most 6 steps (30 min);\n"
                     "after that the cleaned value stays empty and the forecaster reports the sensor as unavailable.\n";
    } catch (const std::exception& e) { std::cerr << "ERROR: " << e.what() << "\n"; return 1; }
    return 0;
}
