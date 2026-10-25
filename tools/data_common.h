// data_common.h -- shared helpers for the data-preparation tools (standard library only).
//
// Contents:   settings, timestamp conversion, CSV reading, the missing-value policy,
//             and a small graph check. Every tool in tools/ includes this one file.
#pragma once

// On Windows (MinGW) make printf understand %zu / %lld exactly like Linux does.
// This must come before the first standard header, so every program includes this file first.
#if defined(__MINGW32__) && !defined(__USE_MINGW_ANSI_STDIO)
#define __USE_MINGW_ANSI_STDIO 1
#endif

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace dp {

// ------------------------------------------------------------------ settings
// The six sensors were chosen by select_sensors.cpp (rules are documented in data/README.md).
const std::vector<std::string> SELECTED_SENSORS = {"716328", "716331", "716337", "717446", "717456", "773062"};
const int MAX_FILL_STEPS = 6;                 // carry a reading forward for at most 6 steps = 30 minutes
const double IMPLAUSIBLE_MPH = 85.0;          // reported only, never deleted
const long long STEP_SECONDS = 300;           // METR-LA is sampled every 5 minutes

const char* const SAMPLE_SPEEDS = "data/sample/speeds_sample.csv";
const char* const FULL_H5 = "data/full/metr-la.h5";
const char* const ID_LIST = "data/sample/graph_sensor_ids.txt";
const char* const LOCATIONS = "data/sample/graph_sensor_locations.csv";
const char* const DISTANCES = "data/sample/distances_la_2012.csv";

const double NAN_VALUE = std::numeric_limits<double>::quiet_NaN();

// ------------------------------------------------------------------ small string helpers
inline std::string fmt(const char* format, ...) {
    va_list args;
    va_start(args, format);
    va_list copy;
    va_copy(copy, args);
    int n = std::vsnprintf(nullptr, 0, format, args);
    va_end(args);
    std::string out(n > 0 ? static_cast<size_t>(n) : 0, '\0');
    std::vsnprintf(&out[0], out.size() + 1, format, copy);
    va_end(copy);
    return out;
}

inline std::string trim(const std::string& s) {
    size_t a = 0, b = s.size();
    while (a < b && (s[a] == ' ' || s[a] == '\t' || s[a] == '\r' || s[a] == '\n')) ++a;
    while (b > a && (s[b - 1] == ' ' || s[b - 1] == '\t' || s[b - 1] == '\r' || s[b - 1] == '\n')) --b;
    return s.substr(a, b - a);
}

// Split one CSV line at commas. These files never contain quoted commas, so this is enough.
inline std::vector<std::string> splitLine(const std::string& line) {
    std::vector<std::string> fields;
    std::string current;
    for (char c : line) {
        if (c == ',') { fields.push_back(current); current.clear(); }
        else if (c != '\r') current.push_back(c);          // tolerate Windows line endings
    }
    fields.push_back(current);
    return fields;
}

// Read a whole file as lines (without line terminators). Throws if it cannot be opened.
inline std::vector<std::string> readLines(const std::string& path) {
    std::ifstream in(path.c_str());
    if (!in) throw std::runtime_error("cannot open " + path);
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(in, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        lines.push_back(line);
    }
    return lines;
}

inline std::string readWholeFile(const std::string& path) {
    std::ifstream in(path.c_str(), std::ios::binary);
    if (!in) throw std::runtime_error("cannot open " + path);
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

// A number, or NaN if the text is empty or not a number.
inline double parseNumber(const std::string& text) {
    std::string t = trim(text);
    if (t.empty()) return NAN_VALUE;
    char* end = nullptr;
    double v = std::strtod(t.c_str(), &end);
    if (end == t.c_str() || *end != '\0') return NAN_VALUE;
    return v;
}

// ------------------------------------------------------------------ timestamps
// We keep every timestamp as "seconds since 1970-01-01 00:00:00" (an integer), so that the spacing
// between two rows is a simple subtraction. These two functions convert to and from calendar dates.
inline long long daysFromCivil(long long y, int m, int d) {
    y -= (m <= 2);
    const long long era = (y >= 0 ? y : y - 399) / 400;
    const long long yoe = y - era * 400;
    const long long doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    const long long doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + doe - 719468;
}

inline void civilFromDays(long long z, int& y, int& m, int& d) {
    z += 719468;
    const long long era = (z >= 0 ? z : z - 146096) / 146097;
    const long long doe = z - era * 146097;
    const long long yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    const long long doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    const long long mp = (5 * doy + 2) / 153;
    d = static_cast<int>(doy - (153 * mp + 2) / 5 + 1);
    m = static_cast<int>(mp < 10 ? mp + 3 : mp - 9);
    y = static_cast<int>(yoe + era * 400 + (m <= 2));
}

inline bool parseTimestamp(const std::string& text, long long& seconds) {
    int y, mo, d, h, mi, s;
    char tail;
    if (std::sscanf(text.c_str(), "%d-%d-%d %d:%d:%d%c", &y, &mo, &d, &h, &mi, &s, &tail) != 6) return false;
    if (mo < 1 || mo > 12 || d < 1 || d > 31 || h < 0 || h > 23 || mi < 0 || mi > 59 || s < 0 || s > 60) return false;
    seconds = daysFromCivil(y, mo, d) * 86400 + h * 3600 + mi * 60 + s;
    return true;
}

inline std::string formatTimestamp(long long seconds) {
    long long days = seconds >= 0 ? seconds / 86400 : -((-seconds + 86399) / 86400);
    long long rest = seconds - days * 86400;
    int y, m, d;
    civilFromDays(days, y, m, d);
    return fmt("%04d-%02d-%02d %02d:%02d:%02d", y, m, d, static_cast<int>(rest / 3600),
               static_cast<int>(rest % 3600 / 60), static_cast<int>(rest % 60));
}

inline const char* weekdayName(long long seconds) {
    static const char* names[] = {"Thursday", "Friday", "Saturday", "Sunday", "Monday", "Tuesday", "Wednesday"};
    long long days = seconds >= 0 ? seconds / 86400 : -((-seconds + 86399) / 86400);
    return names[((days % 7) + 7) % 7];     // 1970-01-01 was a Thursday
}

// ------------------------------------------------------------------ the speed table
struct SpeedTable {
    std::vector<long long> times;                // one entry per row
    std::vector<std::string> ids;                // one entry per sensor column
    std::vector<std::vector<double>> rows;       // rows[time][sensor], NaN if not a number
};

// Wide CSV: first column is the timestamp, every other column is one sensor (header = sensor ID).
inline SpeedTable loadSpeedsCsv(const std::string& path) {
    std::vector<std::string> lines = readLines(path);
    if (lines.empty()) throw std::runtime_error(path + " is empty");
    std::vector<std::string> header = splitLine(lines[0]);
    SpeedTable t;
    t.ids.assign(header.begin() + 1, header.end());
    for (size_t i = 1; i < lines.size(); ++i) {
        if (lines[i].empty()) continue;
        std::vector<std::string> f = splitLine(lines[i]);
        if (f.size() != header.size())
            throw std::runtime_error(fmt("%s: row %zu has %zu fields, expected %zu", path.c_str(), i + 1, f.size(), header.size()));
        long long ts;
        if (!parseTimestamp(f[0], ts)) throw std::runtime_error(fmt("%s: bad timestamp on row %zu: '%s'", path.c_str(), i + 1, f[0].c_str()));
        t.times.push_back(ts);
        std::vector<double> row;
        row.reserve(f.size() - 1);
        for (size_t j = 1; j < f.size(); ++j) row.push_back(parseNumber(f[j]));
        t.rows.push_back(row);
    }
    return t;
}

// ------------------------------------------------------------------ the three small files
inline std::vector<std::string> readIdList(const std::string& path) {
    std::vector<std::string> ids;
    for (const std::string& part : splitLine(trim(readWholeFile(path)))) ids.push_back(trim(part));
    return ids;
}

struct Location { std::string latitude, longitude; };

inline int columnIndex(const std::vector<std::string>& header, const std::string& name, const std::string& file) {
    for (size_t i = 0; i < header.size(); ++i) if (header[i] == name) return static_cast<int>(i);
    throw std::runtime_error(file + ": missing column '" + name + "'");
}

inline std::map<std::string, Location> readLocations(const std::string& path) {
    std::vector<std::string> lines = readLines(path);
    std::vector<std::string> header = splitLine(lines.at(0));
    int ci = columnIndex(header, "sensor_id", path), cl = columnIndex(header, "latitude", path), co = columnIndex(header, "longitude", path);
    std::map<std::string, Location> out;
    for (size_t i = 1; i < lines.size(); ++i) {
        if (lines[i].empty()) continue;
        std::vector<std::string> f = splitLine(lines[i]);
        out[f.at(ci)] = Location{f.at(cl), f.at(co)};
    }
    return out;
}

typedef std::pair<std::string, std::string> Edge;

struct DistanceData {
    std::map<Edge, std::string> kept;            // (from,to) -> distance text, only pairs inside `wanted`
    std::set<std::string> everyId;               // every sensor ID that appears anywhere in the file
    size_t rows = 0;
};

inline DistanceData readDistances(const std::string& path, const std::set<std::string>& wanted) {
    std::ifstream in(path.c_str());
    if (!in) throw std::runtime_error("cannot open " + path);
    std::string line;
    std::getline(in, line);
    std::vector<std::string> header = splitLine(line);
    int cf = columnIndex(header, "from", path), ct = columnIndex(header, "to", path), cc = columnIndex(header, "cost", path);
    DistanceData d;
    while (std::getline(in, line)) {
        if (line.empty() || line == "\r") continue;
        std::vector<std::string> f = splitLine(line);
        ++d.rows;
        d.everyId.insert(f.at(cf));
        d.everyId.insert(f.at(ct));
        if (f[cf] != f[ct] && wanted.count(f[cf]) && wanted.count(f[ct])) d.kept[Edge(f[cf], f[ct])] = f.at(cc);
    }
    return d;
}

// ------------------------------------------------------------------ missing-value policy
// In METR-LA a speed of 0 means "no measurement". NaN (from empty/non-numeric text) and negative
// values are treated the same way.
inline bool isMissing(double v) { return std::isnan(v) || v <= 0; }

struct CleanResult {
    std::vector<double> values;                  // NaN = left empty
    int imputed = 0;
    std::map<int, int> runs;                     // length of a run of missing values -> how many such runs
};

// Apply the policy to ONE sensor's readings. Every decision for row i uses only rows 0..i:
// a missing value is replaced by the last real reading as long as that reading is at most
// MAX_FILL_STEPS old. We never look ahead to see how long the gap will turn out to be.
inline CleanResult cleanColumn(const std::vector<long long>& times, const std::vector<double>& column) {
    CleanResult r;
    double lastValid = NAN_VALUE;
    bool haveLast = false;
    int run = 0;
    for (size_t i = 0; i < column.size(); ++i) {
        if (i > 0 && times[i] - times[i - 1] != STEP_SECONDS) haveLast = false;   // hole in the time grid
        double v = column[i];
        if (!isMissing(v)) {
            if (run > 0) r.runs[run]++;
            run = 0;
            lastValid = v;
            haveLast = true;
            r.values.push_back(v);
        } else {
            ++run;
            if (haveLast && run <= MAX_FILL_STEPS) { r.values.push_back(lastValid); ++r.imputed; }
            else r.values.push_back(NAN_VALUE);
        }
    }
    if (run > 0) r.runs[run]++;
    return r;
}

// Longest stretch of identical consecutive real readings (a very long one suggests a stuck sensor).
inline int longestIdenticalRun(const std::vector<double>& column) {
    int best = 0, current = 0;
    bool havePrev = false;
    double prev = 0;
    for (double v : column) {
        if (isMissing(v)) { current = 0; havePrev = false; continue; }
        current = (havePrev && v == prev) ? current + 1 : 1;
        prev = v;
        havePrev = true;
        best = std::max(best, current);
    }
    return best;
}

// Speeds are written with exactly 4 decimals, rounded half-up on the value itself, so the text
// does not depend on the C library's printf rounding. Empty text means "missing".
inline std::string formatSpeed(double v) {
    if (std::isnan(v)) return "";
    long long q = std::llround(v * 10000.0);
    return fmt("%lld.%04lld", q / 10000, q % 10000);
}

// ------------------------------------------------------------------ graph check
inline bool stronglyConnected(const std::vector<std::string>& nodes, const std::vector<Edge>& edges) {
    if (nodes.empty()) return true;
    std::map<std::string, int> index;
    for (size_t i = 0; i < nodes.size(); ++i) index[nodes[i]] = static_cast<int>(i);
    std::vector<std::vector<int>> forward(nodes.size()), backward(nodes.size());
    for (const Edge& e : edges) {
        forward[index.at(e.first)].push_back(index.at(e.second));
        backward[index.at(e.second)].push_back(index.at(e.first));
    }
    auto reachesAll = [&](const std::vector<std::vector<int>>& adj) {
        std::vector<bool> seen(nodes.size(), false);
        std::vector<int> stack = {0};
        seen[0] = true;
        size_t count = 1;
        while (!stack.empty()) {
            int u = stack.back();
            stack.pop_back();
            for (int w : adj[u]) if (!seen[w]) { seen[w] = true; ++count; stack.push_back(w); }
        }
        return count == nodes.size();
    };
    return reachesAll(forward) && reachesAll(backward);
}

// distance text -> number, for the report
inline double asNumber(const std::string& text) { return std::strtod(text.c_str(), nullptr); }

}  // namespace dp
