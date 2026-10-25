// data_pipeline.h -- the whole data-preparation procedure as one function, buildOutputs().
//
// prepare_data.cpp calls it to write the real files into data/.
// validate_data.cpp calls it again with a different file prefix to prove the result is reproducible.
//
// MISSING-VALUE POLICY (implemented in data_common.h, explained in data/README.md)
//   zero / NaN / negative = missing (zero means "no measurement" in METR-LA)
//   a missing value is replaced by the last real reading while that reading is at most 6 steps old
//   after that the cell stays empty; values above 85 mph are reported but never deleted
#pragma once

#include "data_common.h"
#include "hdf5_min.h"

namespace dp {

// ------------------------------------------------------------------ loading
inline SpeedTable loadSpeedsH5(const std::string& path) {
    h5::File file(path);
    size_t root = file.rootHeader(), df = 0;
    if (!file.findChild(root, "df", df))
        throw std::runtime_error("no group named 'df' in the file. Top-level objects: " + file.childNames(root));

    size_t itemsHeader = 0, stampsHeader = 0, valuesHeader = 0, other = 0;
    if (file.findChild(df, "block1_values", other))
        throw std::runtime_error("the table is split into several data blocks; only a single block is supported");
    if (!file.findChild(df, "block0_values", valuesHeader) || !file.findChild(df, "axis1", stampsHeader))
        throw std::runtime_error("group 'df' does not contain axis1 and block0_values. It contains: " + file.childNames(df));
    if (!file.findChild(df, "block0_items", itemsHeader) && !file.findChild(df, "axis0", itemsHeader))
        throw std::runtime_error("group 'df' has no block0_items or axis0 (sensor IDs)");

    h5::Array items = file.openArray(itemsHeader);
    h5::Array stamps = file.openArray(stampsHeader);
    h5::Array values = file.openArray(valuesHeader);

    SpeedTable t;
    for (size_t i = 0; i < items.count; ++i) {
        if (items.typeClass == 3) t.ids.push_back(items.asString(i));
        else if (items.typeClass == 0) t.ids.push_back(std::to_string(items.asInt(i)));
        else throw std::runtime_error("sensor IDs are stored in an unsupported type: " + items.describe());
    }
    if (stamps.typeClass != 0 || stamps.count == 0) throw std::runtime_error("timestamps are not stored as integers: " + stamps.describe());
    // pandas stores a DatetimeIndex as integers; the time unit is told apart by the size of the first value
    long long first = stamps.asInt(0);
    long long divisor = first > 100000000000000000LL ? 1000000000LL : first > 100000000000000LL ? 1000000LL : first > 100000000000LL ? 1000LL : 1LL;
    for (size_t i = 0; i < stamps.count; ++i) t.times.push_back(stamps.asInt(i) / divisor);

    if (values.dims.size() != 2) throw std::runtime_error("speed table is not two-dimensional: " + values.describe());
    size_t d0 = static_cast<size_t>(values.dims[0]), d1 = static_cast<size_t>(values.dims[1]);
    bool timeFirst;
    if (d0 == t.times.size() && d1 == t.ids.size()) timeFirst = true;
    else if (d1 == t.times.size() && d0 == t.ids.size()) timeFirst = false;
    else throw std::runtime_error(fmt("shape mismatch: values %zu x %zu, timestamps %zu, sensors %zu", d0, d1, t.times.size(), t.ids.size()));
    t.rows.assign(t.times.size(), std::vector<double>(t.ids.size()));
    for (size_t r = 0; r < t.times.size(); ++r)
        for (size_t c = 0; c < t.ids.size(); ++c)
            t.rows[r][c] = values.asDouble(timeFirst ? r * d1 + c : c * d1 + r);
    return t;
}

// source is "csv" (the 2-day sample) or "h5" (the full dataset)
inline SpeedTable loadSpeeds(const std::string& source, std::string& sourceName) {
    if (source == "h5") { sourceName = FULL_H5; return loadSpeedsH5(FULL_H5); }
    sourceName = SAMPLE_SPEEDS;
    return loadSpeedsCsv(SAMPLE_SPEEDS);
}

// ------------------------------------------------------------------ small formatting helpers
inline std::string histogramText(const std::map<int, int>& h) {
    std::string s = "{";
    bool firstItem = true;
    for (const auto& kv : h) { s += (firstItem ? "" : ", ") + std::to_string(kv.first) + ": " + std::to_string(kv.second); firstItem = false; }
    return s + "}";
}

inline void writeFile(const std::string& path, const std::string& content) {
    std::ofstream out(path.c_str(), std::ios::binary);          // binary: keep "\n" exactly, on every platform
    if (!out) throw std::runtime_error("cannot write " + path);
    out << content;
}

// ------------------------------------------------------------------ the pipeline
// Writes <prefix>traffic_data.csv, <prefix>sensor_distances.csv, <prefix>sensor_locations.csv and
// <prefix>data_quality_report.txt. With prefix "data/" these are the real project files.
inline std::string buildOutputs(const std::string& source, const std::string& prefix, bool quiet) {
    std::string report;
    auto say = [&](const std::string& line = "") { report += line + "\n"; };

    // 1. load
    std::string sourceName;
    SpeedTable table = loadSpeeds(source, sourceName);
    if (table.times.empty()) throw std::runtime_error("the speed file has no rows");
    say("DATA QUALITY REPORT");
    say("===================");
    say("Source speed file : " + sourceName);
    say(fmt("Rows x sensors    : %zu x %zu", table.times.size(), table.ids.size()));
    say("Time range        : " + formatTimestamp(table.times.front()) + "  ->  " + formatTimestamp(table.times.back()));
    say(std::string("First / last day  : ") + weekdayName(table.times.front()) + " / " + weekdayName(table.times.back()));

    // 2. IDs must agree across the four files
    std::vector<std::string> idList = readIdList(ID_LIST);
    std::map<std::string, Location> locations = readLocations(LOCATIONS);
    std::set<std::string> wanted(SELECTED_SENSORS.begin(), SELECTED_SENSORS.end());
    DistanceData distances = readDistances(DISTANCES, wanted);
    std::set<std::string> speedIds(table.ids.begin(), table.ids.end());
    std::set<std::string> locationIds;
    for (const auto& kv : locations) locationIds.insert(kv.first);
    size_t inDistance = 0;
    for (const std::string& id : table.ids) inDistance += distances.everyId.count(id);
    say("");
    say("ID CONSISTENCY");
    say(std::string("  speed IDs == ID list (same order)      : ") + (table.ids == idList ? "True" : "False"));
    say(std::string("  speed IDs == location file IDs (as set): ") + (speedIds == locationIds ? "True" : "False"));
    say(fmt("  speed IDs found in distance file       : %zu of %zu", inDistance, table.ids.size()));
    say(fmt("  distance file covers %zu sensors in total (the whole LA network, not only these %zu)", distances.everyId.size(), table.ids.size()));
    std::string missingIds;
    for (const std::string& s : SELECTED_SENSORS)
        if (!speedIds.count(s) || !locations.count(s) || !distances.everyId.count(s)) missingIds += " " + s;
    if (!missingIds.empty()) throw std::runtime_error("selected sensors not found in all files:" + missingIds);

    // 3. timestamps
    bool inOrder = true;
    size_t duplicates = 0;
    std::map<int, int> gaps;
    for (size_t i = 1; i < table.times.size(); ++i) {
        long long g = table.times[i] - table.times[i - 1];
        if (g < 0) inOrder = false;
        if (g == 0) ++duplicates;
        gaps[static_cast<int>(g)]++;
    }
    bool strictlyIncreasing = inOrder && duplicates == 0;
    say("");
    say("TIMESTAMPS");
    say(std::string("  strictly increasing : ") + (strictlyIncreasing ? "True" : "False"));
    say(fmt("  duplicate timestamps: %zu", duplicates));
    say("  spacing (seconds -> count): " + histogramText(gaps));
    if (!strictlyIncreasing) throw std::runtime_error("timestamps are unordered or duplicated; decide a policy before continuing");

    // whole-network context (information only)
    size_t totalCells = table.times.size() * table.ids.size(), zeroCells = 0;
    std::vector<long long> allMissingRows;
    for (size_t r = 0; r < table.rows.size(); ++r) {
        size_t m = 0;
        for (double v : table.rows[r]) m += isMissing(v);
        zeroCells += m;
        if (m == table.ids.size()) allMissingRows.push_back(table.times[r]);
    }
    say("");
    say(fmt("WHOLE NETWORK (information only, all %zu sensors)", table.ids.size()));
    say(fmt("  missing cells (zero/NaN): %zu of %zu (%.2f%%)", zeroCells, totalCells, 100.0 * zeroCells / totalCells));
    say(fmt("  timestamps where EVERY sensor is missing: %zu", allMissingRows.size()));
    for (size_t i = 0; i < allMissingRows.size() && i < 10; ++i) say("     " + formatTimestamp(allMissingRows[i]));
    if (allMissingRows.size() > 10) say(fmt("     ... (%zu more)", allMissingRows.size() - 10));

    // 4 + 5. select the six sensors and clean them
    say("");
    say("SELECTED SENSORS: " + [&] { std::string s; for (size_t i = 0; i < SELECTED_SENSORS.size(); ++i) s += (i ? ", " : "") + SELECTED_SENSORS[i]; return s; }());
    say(fmt("  policy: zero/NaN/negative = missing; carry the last real reading forward for up to %d steps (%d min); after that the value stays empty",
            MAX_FILL_STEPS, MAX_FILL_STEPS * 5));
    say(fmt("  sensor   missing  missing%%  imputed  left_empty  implausible(>%g)  longest_identical_run  min   mean   max", IMPLAUSIBLE_MPH));
    std::vector<CleanResult> cleaned;
    size_t totalMissing = 0, totalImputed = 0, totalLeft = 0;
    std::map<int, int> allRuns;
    for (const std::string& s : SELECTED_SENSORS) {
        size_t j = static_cast<size_t>(std::find(table.ids.begin(), table.ids.end(), s) - table.ids.begin());
        std::vector<double> raw;
        for (const auto& row : table.rows) raw.push_back(row[j]);
        CleanResult c = cleanColumn(table.times, raw);
        size_t missing = 0, left = 0, implausible = 0, goodCount = 0;
        double lo = 1e300, hi = -1e300, sum = 0;
        for (double v : raw) {
            if (isMissing(v)) ++missing;
            else { ++goodCount; sum += v; lo = std::min(lo, v); hi = std::max(hi, v); }
            if (!std::isnan(v) && v > IMPLAUSIBLE_MPH) ++implausible;
        }
        for (double v : c.values) left += std::isnan(v);
        if (goodCount == 0) throw std::runtime_error("sensor " + s + " has no valid readings at all");
        say(fmt("  %s  %7zu  %7.2f%%  %7d  %10zu  %16zu  %21d  %.1f  %.1f  %.1f", s.c_str(), missing, 100.0 * missing / raw.size(),
                c.imputed, left, implausible, longestIdenticalRun(raw), lo, sum / goodCount, hi));
        totalMissing += missing; totalImputed += static_cast<size_t>(c.imputed); totalLeft += left;
        for (const auto& kv : c.runs) allRuns[kv.first] += kv.second;
        cleaned.push_back(c);
    }
    say(fmt("  TOTAL    %7zu  %8s  %7zu  %10zu", totalMissing, "", totalImputed, totalLeft));
    say("  missing-run length histogram over the six sensors (steps -> runs): " + histogramText(allRuns));
    say(fmt("  rows before cleaning: %zu | rows after cleaning: %zu (no rows are dropped, so the 5-minute grid stays regular)",
            table.times.size(), table.times.size()));

    // 6. write the three data files
    std::string traffic = "timestamp";
    for (const std::string& s : SELECTED_SENSORS) traffic += "," + s;
    traffic += "\n";
    for (size_t r = 0; r < table.times.size(); ++r) {
        traffic += formatTimestamp(table.times[r]);
        for (size_t k = 0; k < SELECTED_SENSORS.size(); ++k) traffic += "," + formatSpeed(cleaned[k].values[r]);
        traffic += "\n";
    }
    writeFile(prefix + "traffic_data.csv", traffic);

    std::string locs = "sensor_id,latitude,longitude\n";
    for (const std::string& s : SELECTED_SENSORS) locs += s + "," + locations.at(s).latitude + "," + locations.at(s).longitude + "\n";
    writeFile(prefix + "sensor_locations.csv", locs);

    std::string dist = "from_sensor_id,to_sensor_id,distance_m\n";
    std::vector<Edge> edges;
    for (const std::string& a : SELECTED_SENSORS)
        for (const std::string& b : SELECTED_SENSORS) {
            if (a == b) continue;
            auto it = distances.kept.find(Edge(a, b));
            if (it == distances.kept.end()) continue;
            dist += a + "," + b + "," + it->second + "\n";
            edges.push_back(Edge(a, b));
        }
    writeFile(prefix + "sensor_distances.csv", dist);

    say("");
    say("GRAPH (distance file, meters)");
    if (edges.empty()) throw std::runtime_error("none of the selected sensors are connected in the distance file");
    double lo = 1e300, hi = -1e300;
    size_t reversible = 0;
    for (const Edge& e : edges) {
        double c = asNumber(distances.kept.at(e));
        lo = std::min(lo, c); hi = std::max(hi, c);
        reversible += distances.kept.count(Edge(e.second, e.first));
    }
    size_t n = SELECTED_SENSORS.size();
    say(fmt("  nodes: %zu | directed edges found: %zu of %zu possible", n, edges.size(), n * (n - 1)));
    say(fmt("  distance min/max: %.1f / %.1f m", lo, hi));
    say(fmt("  edges whose reverse edge also exists: %zu", reversible));
    say(std::string("  strongly connected: ") + (stronglyConnected(SELECTED_SENSORS, edges) ? "True" : "False"));
    say("  note: self-distance rows (0.0) are omitted; no edge was invented or removed.");

    writeFile(prefix + "data_quality_report.txt", report);
    if (!quiet) std::cout << report;
    return report;
}

}  // namespace dp
