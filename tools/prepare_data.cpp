// prepare_data.cpp -- one-time data preparation for the SPL-1 traffic project (plain C++17).
//
// Build (from the repository root):   g++ -std=c++17 -O2 -Wall -Wextra -o build/prepare_data tools/prepare_data.cpp
//
// Run:
//   build/prepare_data --source csv        uses data/sample/speeds_sample.csv   (2 days)
//   build/prepare_data --source h5         uses data/full/metr-la.h5            (full dataset)
//   build/prepare_data --h5-info           only describes what is inside metr-la.h5 and stops
//
// It never modifies the original files. The steps and the missing-value policy are documented in
// data_pipeline.h and data/README.md.
#include "data_pipeline.h"

#include <cstring>

static void describeH5() {
    h5::File f(dp::FULL_H5);
    std::cout << "File: " << dp::FULL_H5 << "\nSuperblock version: " << f.superblockVersion() << "\n";
    size_t root = f.rootHeader();
    std::cout << "Top-level objects: " << f.childNames(root) << "\n";
    size_t df = 0;
    if (!f.findChild(root, "df", df)) return;
    std::cout << "Inside 'df':\n";
    for (const auto& entry : f.listGroup(df)) {
        std::cout << "  " << entry.first << ": ";
        try { std::cout << f.openArray(entry.second).describe() << "\n"; }
        catch (const std::exception& e) { std::cout << "(not readable: " << e.what() << ")\n"; }
    }
}

int main(int argc, char** argv) {
    std::string source = "csv";
    bool info = false;
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--source") && i + 1 < argc) source = argv[++i];
        else if (!std::strcmp(argv[i], "--h5-info")) info = true;
        else { std::cerr << "usage: prepare_data [--source csv|h5] [--h5-info]\n"; return 2; }
    }
    if (source != "csv" && source != "h5") { std::cerr << "--source must be csv or h5\n"; return 2; }
    try {
        if (info) { describeH5(); return 0; }
        dp::buildOutputs(source, "data/", false);
        std::cout << "\nWrote data/traffic_data.csv, data/sensor_distances.csv, data/sensor_locations.csv, data/data_quality_report.txt\n";
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
