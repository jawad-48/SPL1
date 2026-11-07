#include "traffic_data.h"

namespace baseline {

TrafficSeries parseTrafficSeries(const std::vector<std::string>& lines, const std::string& sourceName) {
    if (lines.empty()) throw std::runtime_error(sourceName + ": the file is empty");

    // ---- header: "timestamp" followed by sensor IDs, each ID at most once
    const std::vector<std::string> header = dp::splitLine(lines[0]);
    if (header.size() < 2 || dp::trim(header[0]) != "timestamp")
        throw std::runtime_error(sourceName + ": the first header column must be 'timestamp'");
    std::map<std::string, size_t> column;                      // sensor ID -> column index
    for (size_t c = 1; c < header.size(); ++c) {
        const std::string id = dp::trim(header[c]);
        if (!column.insert(std::make_pair(id, c)).second)
            throw std::runtime_error(sourceName + ": sensor ID '" + id + "' appears twice in the header");
    }

    // ---- exactly the six approved sensors must be present (anything else is ignored, but counted)
    TrafficSeries series;
    series.source = sourceName;
    series.sensors = dp::SELECTED_SENSORS;
    std::vector<size_t> selectedColumn;
    std::string missing;
    for (const std::string& id : series.sensors) {
        std::map<std::string, size_t>::const_iterator it = column.find(id);
        if (it == column.end()) missing += (missing.empty() ? "" : ", ") + id;
        else selectedColumn.push_back(it->second);
    }
    if (!missing.empty()) throw std::runtime_error(sourceName + ": required sensor(s) missing from the header: " + missing);
    series.ignoredColumns = (header.size() - 1) - series.sensors.size();
    series.speeds.assign(series.sensors.size(), std::vector<double>());

    // ---- data rows
    for (size_t i = 1; i < lines.size(); ++i) {
        if (dp::trim(lines[i]).empty()) continue;
        const std::vector<std::string> f = dp::splitLine(lines[i]);
        if (f.size() != header.size())
            throw std::runtime_error(dp::fmt("%s: line %zu has %zu fields, expected %zu", sourceName.c_str(), i + 1, f.size(), header.size()));
        long long t;
        if (!dp::parseTimestamp(dp::trim(f[0]), t))
            throw std::runtime_error(dp::fmt("%s: line %zu has an invalid timestamp '%s'", sourceName.c_str(), i + 1, f[0].c_str()));
        if (!series.times.empty() && t <= series.times.back())
            throw std::runtime_error(dp::fmt("%s: line %zu: timestamps must be strictly increasing (no duplicates, no going back)", sourceName.c_str(), i + 1));
        series.times.push_back(t);
        for (size_t s = 0; s < selectedColumn.size(); ++s) {
            const std::string text = dp::trim(f[selectedColumn[s]]);
            if (text.empty()) { series.speeds[s].push_back(dp::NAN_VALUE); continue; }      // empty = missing, by the cleaning policy
            const double v = dp::parseNumber(text);
            if (!std::isfinite(v))
                throw std::runtime_error(dp::fmt("%s: line %zu, sensor %s: '%s' is not a number", sourceName.c_str(), i + 1, series.sensors[s].c_str(), text.c_str()));
            if (v <= 0)
                throw std::runtime_error(dp::fmt("%s: line %zu, sensor %s: speed %s is not positive; cleaned data marks missing values with an empty field", sourceName.c_str(), i + 1, series.sensors[s].c_str(), text.c_str()));
            series.speeds[s].push_back(v);
        }
    }
    if (series.times.empty()) throw std::runtime_error(sourceName + ": the file has a header but no data rows");
    return series;
}

TrafficSeries loadTrafficSeries(const std::string& path) {
    return parseTrafficSeries(dp::readLines(path), path);
}

}  // namespace baseline
