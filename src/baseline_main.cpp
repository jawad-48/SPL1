// baseline_main.cpp -- command-line entry point of the Linear Regression baseline.
//
// Build (from the repository root):
//   g++ -std=c++17 -O2 -Wall -Wextra -o build/linear_baseline src/baseline_main.cpp src/traffic_data.cpp src/features.cpp src/metrics.cpp src/linear_regression.cpp src/baseline_pipeline.cpp src/baseline_report.cpp
//
// Run:
//   build/linear_baseline                              15-minute forecast, report printed to the screen
//   build/linear_baseline --horizon-steps 6            30-minute forecast
//   build/linear_baseline --report docs/baseline_report_15min.txt   also save the report to a file
// Options:
//   --data PATH           cleaned traffic file (default data/traffic_data.csv)
//   --lags N              past readings used as features (default 6)
//   --horizon-steps N     steps ahead to predict, 5 minutes each (default 3 = 15 min)
//   --train-frac X, --val-frac X   chronological split fractions (default 0.70 and 0.15; the rest is test)
//   --report PATH         also write the report to PATH
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>

#include "baseline_report.h"

static bool parseInt(const char* text, int& out) {
    char* end = nullptr;
    long v = std::strtol(text, &end, 10);
    if (end == text || *end != '\0' || v < 1 || v > 100000) return false;
    out = static_cast<int>(v);
    return true;
}

static bool parseFraction(const char* text, double& out) {
    char* end = nullptr;
    double v = std::strtod(text, &end);
    if (end == text || *end != '\0' || !(v > 0 && v < 1)) return false;
    out = v;
    return true;
}

int main(int argc, char** argv) {
    baseline::BaselineConfig config;
    std::string reportPath;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        const bool hasValue = i + 1 < argc;
        bool ok = true;
        if (arg == "--data" && hasValue) config.dataPath = argv[++i];
        else if (arg == "--report" && hasValue) reportPath = argv[++i];
        else if (arg == "--lags" && hasValue) ok = parseInt(argv[++i], config.features.lags);
        else if (arg == "--horizon-steps" && hasValue) ok = parseInt(argv[++i], config.features.horizonSteps);
        else if (arg == "--train-frac" && hasValue) ok = parseFraction(argv[++i], config.trainFraction);
        else if (arg == "--val-frac" && hasValue) ok = parseFraction(argv[++i], config.validationFraction);
        else {
            std::cerr << "usage: linear_baseline [--data PATH] [--lags N] [--horizon-steps N] [--train-frac X] [--val-frac X] [--report PATH]\n";
            return arg == "--help" ? 0 : 2;
        }
        if (!ok) { std::cerr << "invalid value for " << arg << "\n"; return 2; }
    }
    try {
        const std::string report = baseline::formatReport(baseline::runBaseline(config));
        std::cout << report;
        if (!reportPath.empty()) {
            std::ofstream file(reportPath.c_str(), std::ios::binary);          // binary: keep "\n" exactly on every platform
            if (!file) throw std::runtime_error("cannot write " + reportPath);
            file << report;
        }
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
