// traffic_data.h -- loads the CLEANED six-sensor traffic file for modelling and checks its schema.
//
// The cleaned file (data/traffic_data.csv) is produced by tools/prepare_data.cpp:
//     timestamp,716328,716331,716337,717446,717456,773062
//     2012-03-01 06:00:00,66.8750,67.5000,...        speeds in mph, one row every 5 minutes
// An EMPTY field means "missing" (the cleaning policy left it empty). This module never changes the
// file; it only reads it, restricts it to the six approved sensors and refuses anything that breaks
// the schema, so that a damaged file cannot silently produce a model.
#pragma once

#include <string>
#include <vector>

#include "../tools/data_common.h"   // reuse the approved sensor list (dp::SELECTED_SENSORS) and the CSV/time helpers

namespace baseline {

struct TrafficSeries {
    std::string source;                        // where the data came from (shown in the report)
    std::vector<long long> times;              // seconds since 1970-01-01, strictly increasing
    std::vector<std::string> sensors;          // always exactly dp::SELECTED_SENSORS, in that order
    std::vector<std::vector<double>> speeds;   // speeds[sensor][row]; NaN means the field was empty
    size_t ignoredColumns = 0;                 // columns of the file that are NOT approved sensors
    size_t rows() const { return times.size(); }
};

// Read a file. Throws std::runtime_error with a precise message if the schema is violated.
TrafficSeries loadTrafficSeries(const std::string& path);

// Same checks on text lines already in memory (this is what the tests call).
// lines[0] is the header, every later non-empty line is a data row.
TrafficSeries parseTrafficSeries(const std::vector<std::string>& lines, const std::string& sourceName);

}  // namespace baseline
