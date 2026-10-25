// validate_data.cpp -- independent checks on the prepared files.
//
// Build:  g++ -std=c++17 -O2 -Wall -Wextra -o build/validate_data tools/validate_data.cpp
// Run (from the repository root, with the same --source you used for prepare_data):
//   build/validate_data --source csv
//   build/validate_data --source h5
//
// It re-reads the ORIGINAL data and the files written by prepare_data and checks that they agree.
// Every check prints PASS or FAIL; the exit code is non-zero if anything failed.
// The expected value of every cell is re-derived here with its own small loop (not by calling the
// cleaning function the preparation step used), so a bug in one place cannot hide in both.
#include "data_pipeline.h"

#include <cstring>

static std::vector<std::string> failures;
static int passCount = 0;

static void check(const std::string& name, bool ok, const std::string& detail = "") {
    std::printf("%s  %s%s\n", ok ? "PASS" : "FAIL", name.c_str(), detail.empty() ? "" : ("   [" + detail + "]").c_str());
    if (ok) ++passCount; else failures.push_back(name);
}

typedef std::vector<std::vector<std::string>> Rows;

static Rows readCsv(const std::string& path) {
    Rows rows;
    for (const std::string& line : dp::readLines(path)) if (!line.empty()) rows.push_back(dp::splitLine(line));
    return rows;
}

// Run a shell command, capture its text output in `out`; true if it exited with status 0.
static bool capture(const std::string& command, std::string& out) {
    const char* tmp = ".validate_tmp.txt";
    int rc = std::system((command + " > " + tmp + " 2>&1").c_str());
    try { out = dp::trim(dp::readWholeFile(tmp)); } catch (...) { out.clear(); }
    std::remove(tmp);
    return rc == 0;
}

int main(int argc, char** argv) {
    std::string source = "csv", dir = "data";
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--source") && i + 1 < argc) source = argv[++i];
        else if (!std::strcmp(argv[i], "--data") && i + 1 < argc) dir = argv[++i];
        else { std::cerr << "usage: validate_data [--source csv|h5] [--data folder]\n"; return 2; }
    }
    const std::vector<std::string>& sensors = dp::SELECTED_SENSORS;
    const std::string prefix = dir + "/";
    try {
        // ---- the original data, loaded again from scratch
        std::string sourceName;
        dp::SpeedTable raw = dp::loadSpeeds(source, sourceName);
        auto rawColumn = [&](const std::string& id) { return std::find(raw.ids.begin(), raw.ids.end(), id) - raw.ids.begin(); };

        // ---- traffic_data.csv
        Rows traffic = readCsv(prefix + "traffic_data.csv");
        if (traffic.empty()) throw std::runtime_error(prefix + "traffic_data.csv is empty");
        std::vector<std::string> expectedHeader = {"timestamp"};
        expectedHeader.insert(expectedHeader.end(), sensors.begin(), sensors.end());
        check("traffic_data: header is timestamp + the six pinned sensor IDs", traffic[0] == expectedHeader);
        check("traffic_data: exactly six sensor columns plus timestamp", traffic[0].size() == 7);
        bool sevenFields = true;
        for (const auto& r : traffic) sevenFields = sevenFields && r.size() == 7;
        check("traffic_data: every row has 7 fields", sevenFields);
        bool allExist = true;
        for (const std::string& s : sensors) allExist = allExist && std::count(raw.ids.begin(), raw.ids.end(), s) == 1;
        check("traffic_data: every selected sensor exists in the original speed data", allExist);
        size_t rows = traffic.size() - 1;
        check(dp::fmt("traffic_data: row count equals original (%zu)", raw.times.size()), rows == raw.times.size());
        bool sameTimes = rows == raw.times.size();
        for (size_t i = 0; sameTimes && i < rows; ++i) sameTimes = traffic[i + 1][0] == dp::formatTimestamp(raw.times[i]);   // rows are non-empty lines
        check("traffic_data: timestamps identical to original", sameTimes);
        std::set<long long> steps;
        for (size_t i = 1; i < raw.times.size(); ++i) steps.insert(raw.times[i] - raw.times[i - 1]);
        std::string stepText;
        for (long long s : steps) stepText += (stepText.empty() ? "" : ",") + std::to_string(s);
        check("traffic_data: timestamps strictly increasing with a constant 300 s step", steps.size() == 1 && *steps.begin() == dp::STEP_SECONDS, "steps seen: " + stepText);

        // ---- the missing-value policy, re-derived cell by cell
        size_t badCells = 0, filledCells = 0, emptyCells = 0, rawMissing = 0, rawBig = 0, outBig = 0;
        bool allPositive = true;
        for (size_t k = 0; k < sensors.size(); ++k) {
            size_t j = static_cast<size_t>(rawColumn(sensors[k]));
            bool haveLast = false;
            double last = 0;
            int age = 0;
            for (size_t i = 0; i < rows && i < raw.rows.size(); ++i) {
                if (i > 0 && raw.times[i] - raw.times[i - 1] != dp::STEP_SECONDS) haveLast = false;
                double v = raw.rows[i][j];
                if (!std::isnan(v) && v > dp::IMPLAUSIBLE_MPH) ++rawBig;
                std::string expected;
                if (!dp::isMissing(v)) { expected = dp::formatSpeed(v); last = v; haveLast = true; age = 0; }
                else {
                    ++rawMissing;
                    ++age;
                    if (haveLast && age <= dp::MAX_FILL_STEPS) { expected = dp::formatSpeed(last); ++filledCells; }
                    else { expected = ""; ++emptyCells; }
                }
                const std::string got = traffic[i + 1].size() > k + 1 ? traffic[i + 1][k + 1] : "<missing field>";
                if (got != expected) ++badCells;
                if (!got.empty()) {
                    double g = dp::parseNumber(got);
                    allPositive = allPositive && !std::isnan(g) && g > 0;
                    if (g > dp::IMPLAUSIBLE_MPH) ++outBig;
                }
            }
        }
        check(dp::fmt("policy: all %zu cells match the independently re-derived values", rows * sensors.size()), badCells == 0,
              dp::fmt("%zu mismatching cells", badCells));
        check("policy: no zero, negative or non-numeric value remains in traffic_data", allPositive);
        check(dp::fmt("policy: values above %g mph are kept, not deleted (original %zu, output %zu)", dp::IMPLAUSIBLE_MPH, rawBig, outBig), rawBig == outBig);

        // ---- the report agrees with the files
        size_t repMissing = 0, repImputed = 0, repLeft = 0;
        for (const std::string& line : dp::readLines(prefix + "data_quality_report.txt")) {
            if (dp::trim(line).rfind("TOTAL", 0) == 0) {
                std::istringstream ss(line);
                std::string word;
                ss >> word >> repMissing >> repImputed >> repLeft;
            }
        }
        check(dp::fmt("report: missing/imputed/left-empty counts match the files (%zu / %zu / %zu)", rawMissing, filledCells, emptyCells),
              repMissing == rawMissing && repImputed == filledCells && repLeft == emptyCells,
              dp::fmt("report says %zu / %zu / %zu", repMissing, repImputed, repLeft));

        // ---- sensor_locations.csv
        std::map<std::string, dp::Location> originalLocations = dp::readLocations(dp::LOCATIONS);
        Rows loc = readCsv(prefix + "sensor_locations.csv");
        if (loc.empty()) throw std::runtime_error(prefix + "sensor_locations.csv is empty");
        check("locations: header is sensor_id,latitude,longitude", loc[0] == std::vector<std::string>({"sensor_id", "latitude", "longitude"}));
        bool sameOrder = loc.size() == sensors.size() + 1, sameCoords = sameOrder;
        for (size_t i = 0; sameOrder && i < sensors.size(); ++i) {
            if (loc[i + 1].size() != 3) { sameOrder = sameCoords = false; break; }
            sameOrder = sameOrder && loc[i + 1][0] == sensors[i];
            auto it = originalLocations.find(loc[i + 1][0]);
            sameCoords = sameCoords && it != originalLocations.end() && it->second.latitude == loc[i + 1][1] && it->second.longitude == loc[i + 1][2];
        }
        check("locations: exactly the six selected sensors, in order", sameOrder);
        check("locations: coordinates equal the original location file", sameCoords);

        // ---- sensor_distances.csv
        std::set<std::string> wanted(sensors.begin(), sensors.end());
        dp::DistanceData original = dp::readDistances(dp::DISTANCES, wanted);
        Rows dist = readCsv(prefix + "sensor_distances.csv");
        if (dist.empty()) throw std::runtime_error(prefix + "sensor_distances.csv is empty");
        check("distances: header is from_sensor_id,to_sensor_id,distance_m", dist[0] == std::vector<std::string>({"from_sensor_id", "to_sensor_id", "distance_m"}));
        std::vector<dp::Edge> pairs;
        std::set<dp::Edge> unique;
        bool onlySelected = true, noSelfLoops = true, positive = true, equalsOriginal = true;
        for (size_t i = 1; i < dist.size(); ++i) {
            if (dist[i].size() != 3) { equalsOriginal = false; continue; }
            dp::Edge e(dist[i][0], dist[i][1]);
            pairs.push_back(e);
            unique.insert(e);
            onlySelected = onlySelected && wanted.count(e.first) && wanted.count(e.second);
            noSelfLoops = noSelfLoops && e.first != e.second;
            positive = positive && dp::asNumber(dist[i][2]) > 0;
            auto it = original.kept.find(e);
            equalsOriginal = equalsOriginal && it != original.kept.end() && it->second == dist[i][2];
        }
        check("distances: only selected sensor IDs appear", onlySelected);
        check("distances: no self-loops and no duplicate edges", noSelfLoops && unique.size() == pairs.size());
        check("distances: every edge has a positive numeric distance", positive);
        check("distances: every edge equals its record in distances_la_2012.csv", equalsOriginal);
        std::set<dp::Edge> expectedEdges;
        for (const auto& kv : original.kept) expectedEdges.insert(kv.first);
        size_t possible = sensors.size() * (sensors.size() - 1);
        check(dp::fmt("distances: no edge dropped (%zu of %zu possible pairs exist in the source)", expectedEdges.size(), possible), unique == expectedEdges);
        check("graph: the six sensors are strongly connected", dp::stronglyConnected(sensors, pairs));

        // ---- git safety
        std::string out;
        if (!capture("git --version", out)) {
            std::printf("SKIP  git checks (git not found)\n");
        } else {
            bool ok = capture("git status --porcelain -- data/sample data/full", out);
            check("originals: nothing under data/sample or data/full is modified", ok && out.empty(), out.substr(0, 80));
            ok = capture("git ls-files data/full", out);
            check("git: no file under data/full is tracked", ok && out.empty(), out.substr(0, 80));
            ok = capture("git check-ignore data/full/metr-la.h5", out);
            check("git: .gitignore protects data/full/metr-la.h5", ok);
        }

        // ---- reproducibility: run the whole pipeline again into temporary files and compare
        const std::vector<std::string> names = {"traffic_data.csv", "sensor_distances.csv", "sensor_locations.csv", "data_quality_report.txt"};
        dp::buildOutputs(source, ".repro_", true);
        bool same = true;
        for (const std::string& n : names) {
            same = same && dp::readWholeFile(".repro_" + n) == dp::readWholeFile(prefix + n);
            std::remove((".repro_" + n).c_str());
        }
        check("reproducibility: re-running prepare_data gives byte-identical files", same);
    } catch (const std::exception& e) {
        std::printf("FAIL  validation stopped by an error: %s\n", e.what());
        return 1;
    }

    std::printf("\n");
    if (!failures.empty()) {
        std::printf("RESULT: %zu CHECK(S) FAILED, %d passed\n", failures.size(), passCount);
        return 1;
    }
    std::printf("RESULT: all %d checks passed\n", passCount);
    return 0;
}
