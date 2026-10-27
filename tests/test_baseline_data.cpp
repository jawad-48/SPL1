// test_baseline_data.cpp -- tests for src/traffic_data.* and src/features.*
//
// Build (from the repository root):
//   g++ -std=c++17 -O2 -Wall -Wextra -o build/test_baseline_data tests/test_baseline_data.cpp src/traffic_data.cpp src/features.cpp
// Run:
//   build/test_baseline_data          (needs data/traffic_data.csv, so run it from the repository root)
#include <functional>

#include "../src/features.h"
#include "test_util.h"

using namespace baseline;

static const std::vector<std::string>& SIX = dp::SELECTED_SENSORS;

// One CSV text line: a timestamp 5 minutes * row after 2012-03-01 00:00:00 plus the given fields.
static std::string line(long long row, const std::vector<std::string>& fields) {
    std::string s = dp::formatTimestamp(1330560000LL + 300 * row);
    for (const std::string& f : fields) s += "," + f;
    return s;
}

static std::string headerLine(const std::vector<std::string>& ids) {
    std::string s = "timestamp";
    for (const std::string& id : ids) s += "," + id;
    return s;
}

// n rows of the six sensors; sensor s at row r holds 100 + 10*s + r (a ramp, easy to check by eye)
static std::vector<std::string> rampLines(int n) {
    std::vector<std::string> lines = {headerLine(SIX)};
    for (int r = 0; r < n; ++r) {
        std::vector<std::string> f;
        for (int s = 0; s < 6; ++s) f.push_back(std::to_string(100 + 10 * s + r));
        lines.push_back(line(r, f));
    }
    return lines;
}

static bool throwsWith(const std::function<void()>& action, const std::string& fragment) {
    try { action(); } catch (const std::exception& e) { return std::string(e.what()).find(fragment) != std::string::npos; }
    return false;
}

static TrafficSeries ramp(int n) { return parseTrafficSeries(rampLines(n), "ramp"); }

int main() {
    // ------------------------------------------------------------------ sensor selection
    {
        std::vector<std::string> ids = {"999999", "773062", "716328", "111111", "717456", "717446", "716337", "716331"};   // shuffled + 2 unapproved
        std::vector<std::string> lines = {headerLine(ids), line(0, {"1", "2", "3", "4", "5", "6", "7", "8"})};
        TrafficSeries s = parseTrafficSeries(lines, "shuffled");
        tu::expect("sensors: exactly the six approved IDs, in the approved order", s.sensors == SIX && s.speeds.size() == 6);
        // column 773062 holds "2", 716328 holds "3", 717456 holds "5", 717446 "6", 716337 "7", 716331 "8"
        tu::expect("sensors: every value is stored under the right sensor ID",
                   s.speeds[0][0] == 3 && s.speeds[1][0] == 8 && s.speeds[2][0] == 7 && s.speeds[3][0] == 6 && s.speeds[4][0] == 5 && s.speeds[5][0] == 2);
        tu::expect("sensors: unapproved columns are ignored and counted", s.ignoredColumns == 2);
    }
    {
        std::vector<std::string> ids(SIX.begin(), SIX.end() - 1);                                   // sensor 773062 missing
        tu::expect("sensors: a missing approved sensor is rejected and named",
                   throwsWith([&] { parseTrafficSeries({headerLine(ids), line(0, {"1", "2", "3", "4", "5"})}, "x"); }, "773062"));
    }
    {
        std::vector<std::string> ids = SIX;
        ids.push_back("716328");
        tu::expect("sensors: a duplicated header ID is rejected",
                   throwsWith([&] { parseTrafficSeries({headerLine(ids), line(0, {"1", "2", "3", "4", "5", "6", "7"})}, "x"); }, "twice"));
        tu::expect("schema: the first header column must be 'timestamp'",
                   throwsWith([&] { parseTrafficSeries({"time,716328,716331,716337,717446,717456,773062", line(0, {"1", "2", "3", "4", "5", "6"})}, "x"); }, "timestamp"));
    }

    // ------------------------------------------------------------------ time ordering and row format
    {
        std::vector<std::string> lines = rampLines(3);
        std::swap(lines[2], lines[3]);                                                              // rows 1 and 2 swapped
        tu::expect("time order: out-of-order timestamps are rejected", throwsWith([&] { parseTrafficSeries(lines, "x"); }, "strictly increasing"));
        std::vector<std::string> dup = rampLines(3);
        dup[3] = dup[2];
        tu::expect("time order: a duplicated timestamp is rejected", throwsWith([&] { parseTrafficSeries(dup, "x"); }, "strictly increasing"));
        std::vector<std::string> bad = rampLines(2);
        bad[2] = "not-a-date,1,2,3,4,5,6";
        tu::expect("time order: an invalid timestamp is rejected", throwsWith([&] { parseTrafficSeries(bad, "x"); }, "invalid timestamp"));
        std::vector<std::string> shortRow = rampLines(2);
        shortRow[2] += ",9";
        tu::expect("schema: a row with the wrong number of fields is rejected", throwsWith([&] { parseTrafficSeries(shortRow, "x"); }, "fields"));
        tu::expect("schema: an empty file is rejected", throwsWith([&] { parseTrafficSeries({}, "x"); }, "empty"));
        tu::expect("schema: a header without data rows is rejected", throwsWith([&] { parseTrafficSeries({headerLine(SIX)}, "x"); }, "no data rows"));
    }

    // ------------------------------------------------------------------ missing values and invalid numbers
    {
        std::vector<std::string> lines = rampLines(2);
        lines[2] = line(1, {"", "5", "6", "7", "8", "9"});                                          // sensor 716328 empty at row 1
        TrafficSeries s = parseTrafficSeries(lines, "x");
        tu::expect("missing: an empty field becomes NaN and nothing else changes",
                   std::isnan(s.speeds[0][1]) && s.speeds[1][1] == 5 && !std::isnan(s.speeds[0][0]));
        std::vector<std::string> text = rampLines(2);
        text[2] = line(1, {"abc", "5", "6", "7", "8", "9"});
        tu::expect("missing: non-numeric text is rejected, not silently treated as missing", throwsWith([&] { parseTrafficSeries(text, "x"); }, "not a number"));
        std::vector<std::string> zero = rampLines(2);
        zero[2] = line(1, {"0", "5", "6", "7", "8", "9"});
        tu::expect("missing: a zero speed is rejected (cleaned data uses empty fields)", throwsWith([&] { parseTrafficSeries(zero, "x"); }, "not positive"));
        std::vector<std::string> neg = rampLines(2);
        neg[2] = line(1, {"-4.5", "5", "6", "7", "8", "9"});
        tu::expect("missing: a negative speed is rejected", throwsWith([&] { parseTrafficSeries(neg, "x"); }, "not positive"));
    }
    {
        TrafficSeries s = ramp(12);                                                                 // lags 3, horizon 2 -> origins 2..9
        s.speeds[0][4] = dp::NAN_VALUE;                                                             // row 4 is used by origins 4,5,6 as input and by origin 2 as target
        SampleSet set = buildSamples(s, 0, FeatureConfig{3, 2});
        // origin 2: target row 4 -> dropped; origins 4,5,6: row 4 is an input (lag 0,1,2) -> dropped; origin 3 and 7..9 survive
        tu::expect("missing: samples touching an empty value are dropped (as input or as target)", set.droppedMissing == 4 && set.samples.size() == 4);
        tu::expect("missing: samples far from the empty value are kept", set.samples[0].originRow == 3 && set.samples[1].originRow == 7);
        tu::expect("missing: every candidate is accounted for", set.candidates == set.samples.size() + set.droppedMissing + set.droppedIrregular);
    }
    {
        std::vector<std::string> lines = {headerLine(SIX)};
        for (int r = 0; r < 10; ++r) {
            std::vector<std::string> f(6, std::to_string(50 + r));
            lines.push_back(line(r < 5 ? r : r + 1, f));                                            // a 10-minute gap between rows 4 and 5
        }
        TrafficSeries s = parseTrafficSeries(lines, "gap");
        SampleSet set = buildSamples(s, 0, FeatureConfig{2, 1});
        // windows are rows t-1..t+1 for t = 1..8; the window contains the gap if t <= 5 and t >= 4 -> origins 4 and 5
        tu::expect("time grid: windows that span a hole in the timestamps are dropped", set.droppedIrregular == 2 && set.samples.size() == 6 && set.droppedMissing == 0);
    }

    // ------------------------------------------------------------------ feature / target alignment
    {
        TrafficSeries s = ramp(10);                                                                 // sensor 0 at row r holds 100 + r
        SampleSet set = buildSamples(s, 0, FeatureConfig{3, 2});
        const Sample& first = set.samples.front();
        tu::expect("alignment: the first sample's origin, target row and counts are right",
                   first.originRow == 2 && first.targetRow == 4 && set.samples.size() == 10 - 2 - 2 && set.candidates == 6);
        tu::expect("alignment: x holds speed(t), speed(t-1), speed(t-2), newest first",
                   first.x.size() == 3 && first.x[0] == 102 && first.x[1] == 101 && first.x[2] == 100);
        tu::expect("alignment: the target is speed(t + horizon) and lastObserved is speed(t)", first.y == 104 && first.lastObserved == 102);
        const Sample& last = set.samples.back();
        tu::expect("alignment: the last sample ends exactly at the final row", last.originRow == 7 && last.targetRow == 9 && last.y == 109);
        SampleSet other = buildSamples(s, 3, FeatureConfig{3, 2});                                  // sensor 3 holds 130 + r
        tu::expect("alignment: each sensor is built from its own column", other.samples.front().x[0] == 132 && other.samples.front().y == 134);
        bool consistent = true;
        for (const Sample& smp : set.samples) consistent = consistent && smp.targetRow == smp.originRow + 2 && smp.x.size() == 3 && smp.y == 100 + smp.targetRow && smp.lastObserved == smp.x[0];
        tu::expect("alignment: target row = origin + horizon for every sample", consistent);
    }

    // ------------------------------------------------------------------ no future-data leakage
    {
        TrafficSeries original = ramp(40);
        for (size_t s = 0; s < 6; ++s)                                                              // make the data irregular, so a wrong index would show
            for (size_t r = 0; r < 40; ++r) original.speeds[s][r] = 50 + static_cast<double>((r * 37 + s * 11) % 23);
        const FeatureConfig cfg{5, 3};
        SampleSet reference = buildSamples(original, 2, cfg);

        bool inputsIgnoreFuture = true, targetSeesFuture = true;
        for (const Sample& smp : reference.samples) {
            TrafficSeries corrupted = original;
            for (size_t r = smp.originRow + 1; r < corrupted.rows(); ++r) corrupted.speeds[2][r] = 999.0;   // destroy EVERYTHING after the origin
            SampleSet again = buildSamples(corrupted, 2, cfg);
            const Sample* match = nullptr;
            for (const Sample& c : again.samples) if (c.originRow == smp.originRow) match = &c;
            inputsIgnoreFuture = inputsIgnoreFuture && match != nullptr && match->x == smp.x && match->lastObserved == smp.lastObserved;
            targetSeesFuture = targetSeesFuture && match != nullptr && match->y == 999.0;           // only the target may look ahead
        }
        tu::expect("leakage: changing every value after the origin never changes the inputs", inputsIgnoreFuture);
        tu::expect("leakage: the target is the only quantity that depends on the future", targetSeesFuture);

        // cutting the series after row 25 must reproduce exactly the samples whose target is at row <= 25
        TrafficSeries truncated = original;
        truncated.times.resize(26);
        for (auto& col : truncated.speeds) col.resize(26);
        SampleSet cut = buildSamples(truncated, 2, cfg);
        bool prefixSame = true;
        size_t expected = 0;
        for (const Sample& smp : reference.samples) {
            if (smp.targetRow > 25) continue;
            prefixSame = prefixSame && expected < cut.samples.size() && cut.samples[expected].x == smp.x && cut.samples[expected].y == smp.y;
            ++expected;
        }
        tu::expect("leakage: truncating the future leaves earlier samples identical", prefixSame && cut.samples.size() == expected && expected > 0);

        bool ordered = true;
        for (size_t i = 1; i < reference.samples.size(); ++i) ordered = ordered && reference.samples[i].originRow > reference.samples[i - 1].originRow;
        tu::expect("time order: samples come out in strictly increasing chronological order", ordered);
    }

    // ------------------------------------------------------------------ configuration errors
    {
        TrafficSeries s = ramp(10);
        auto throwsInvalid = [&](FeatureConfig c, size_t sensor) {
            try { buildSamples(s, sensor, c); } catch (const std::invalid_argument&) { return true; }
            return false;
        };
        tu::expect("config: lags < 1, horizon < 1 and an unknown sensor index are rejected",
                   throwsInvalid(FeatureConfig{0, 1}, 0) && throwsInvalid(FeatureConfig{3, 0}, 0) && throwsInvalid(FeatureConfig{3, 1}, 6));
        tu::expect("config: a series shorter than lags + horizon gives zero samples, not an error", buildSamples(ramp(4), 0, FeatureConfig{3, 3}).samples.empty());
    }

    // ------------------------------------------------------------------ the real committed file
    {
        TrafficSeries s = loadTrafficSeries("data/traffic_data.csv");
        tu::expect("real data: data/traffic_data.csv has the six approved sensors and no extra columns", s.sensors == SIX && s.ignoredColumns == 0);
        tu::expect("real data: 576 rows, 2012-03-01 06:00:00 to 2012-03-03 05:55:00",
                   s.rows() == 576 && dp::formatTimestamp(s.times.front()) == "2012-03-01 06:00:00" && dp::formatTimestamp(s.times.back()) == "2012-03-03 05:55:00");
        bool noNaN = true;
        for (const auto& col : s.speeds) for (double v : col) noNaN = noNaN && !std::isnan(v);
        tu::expect("real data: the committed file has no empty fields (cleaning left none)", noNaN);
        SampleSet set = buildSamples(s, 0, FeatureConfig{});
        tu::expect("real data: default config (6 lags, 3 steps) uses 568 of 568 candidate origins", set.candidates == 568 && set.samples.size() == 568);
    }

    return tu::summary();
}
