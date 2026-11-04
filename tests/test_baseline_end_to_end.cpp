// test_baseline_end_to_end.cpp -- runs the whole baseline on the real committed data and checks it from the outside.
//
// Build (from the repository root):
//   g++ -std=c++17 -O2 -Wall -Wextra -o build/test_baseline_end_to_end tests/test_baseline_end_to_end.cpp src/traffic_data.cpp src/features.cpp src/metrics.cpp src/linear_regression.cpp src/baseline_pipeline.cpp src/baseline_report.cpp
// Run (from the repository root, because it reads data/ and docs/):
//   build/test_baseline_end_to_end
//
// This test only READS files. It never writes into data/ or docs/.
#include <fstream>

#include "../src/baseline_report.h"
#include "test_util.h"

using namespace baseline;

// A deliberately naive reader, independent of src/traffic_data.cpp: speeds[sensor][row] from the cleaned CSV.
static std::vector<std::vector<double>> naiveRead(const std::string& path, std::vector<std::string>& header) {
    std::ifstream in(path.c_str());
    std::string text;
    std::getline(in, text);
    header = dp::splitLine(text);
    std::vector<std::vector<double>> columns(header.size() - 1);
    while (std::getline(in, text)) {
        if (text.empty()) continue;
        std::vector<std::string> f = dp::splitLine(text);
        for (size_t c = 1; c < f.size(); ++c) columns[c - 1].push_back(std::atof(f[c].c_str()));
    }
    return columns;
}

static std::string readText(const std::string& path) {
    std::ifstream in(path.c_str(), std::ios::binary);
    std::string s((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    std::string lf;                                                  // CRLF -> LF, so a checkout with Windows line endings still compares equal
    lf.reserve(s.size());
    for (size_t i = 0; i < s.size(); ++i)
        if (!(s[i] == '\r' && i + 1 < s.size() && s[i + 1] == '\n')) lf += s[i];
    return lf;
}

int main() {
    const BaselineConfig defaults;                                   // 6 lags, 3 steps, 70/15/15
    const BaselineResult result = runBaseline(defaults);

    // ---- what was evaluated
    {
        bool sensorsOk = result.sensors.size() == 6;
        for (size_t k = 0; sensorsOk && k < 6; ++k) sensorsOk = result.sensors[k].sensorId == dp::SELECTED_SENSORS[k];
        tu::expect("data: exactly the six approved sensors were evaluated, in the approved order", sensorsOk && result.series.sensors == dp::SELECTED_SENSORS);
        tu::expect("data: 576 rows, split at rows 403 and 489", result.series.rows() == 576 && result.split.trainEnd == 403 && result.split.validationEnd == 489);
        bool counts = true;
        for (const SensorResult& s : result.sensors)
            counts = counts && s.candidates == 568 && s.usable == 568 && s.trainSamples == 395 && s.validationSamples == 86 && s.testSamples == 87 && s.droppedMissing == 0 && s.droppedIrregular == 0;
        tu::expect("data: every sensor has 568 usable samples = 395 train + 86 validation + 87 test", counts);
        bool converged = true;
        for (const SensorResult& s : result.sensors) converged = converged && s.gd.converged && s.olsAvailable && (s.trainRmseGd - s.trainRmseOls) / s.trainRmseOls < 1e-6;
        tu::expect("training: gradient descent converged for every sensor and matches the exact least-squares fit (relative gap < 1e-6)", converged);
    }

    // ---- recompute the held-out numbers from the raw file with code that shares nothing with the pipeline
    {
        std::vector<std::string> header;
        const std::vector<std::vector<double>> raw = naiveRead("data/traffic_data.csv", header);
        bool persistenceOk = true, linearOk = true, actualOk = true;
        for (size_t k = 0; k < 6; ++k) {
            size_t col = 0;                                           // find this sensor's column by its ID
            for (size_t c = 1; c < header.size(); ++c) if (header[c] == dp::SELECTED_SENSORS[k]) col = c - 1;
            const SensorResult& r = result.sensors[k];
            double sumPersist = 0, sumLinear = 0;
            size_t n = 0;
            for (size_t target = result.split.validationEnd; target < raw[col].size(); ++target) {     // every test target row
                const size_t origin = target - 3;
                double prediction = r.intercept;
                for (size_t lag = 0; lag < 6; ++lag) prediction += r.coefficients[lag] * raw[col][origin - lag];
                sumPersist += std::fabs(raw[col][origin] - raw[col][target]);
                sumLinear += std::fabs(prediction - raw[col][target]);
                ++n;
            }
            persistenceOk = persistenceOk && n == r.testSamples && tu::near(sumPersist / static_cast<double>(n), r.persistenceTest.mae, 1e-9);
            linearOk = linearOk && tu::near(sumLinear / static_cast<double>(n), r.linearTest.mae, 1e-9);
            for (size_t i = 0; i < r.test.targetRows.size(); ++i) actualOk = actualOk && r.test.actual[i] == raw[col][r.test.targetRows[i]];
        }
        tu::expect("independent check: persistence test MAE recomputed from the raw CSV matches for all six sensors", persistenceOk);
        tu::expect("independent check: Linear Regression test MAE recomputed from the raw CSV and the printed coefficients matches", linearOk);
        tu::expect("independent check: every 'actual' value is the raw CSV value at its target row", actualOk);
        tu::expect("metrics: pooled test MAE is the sample-weighted mean of the sensors' MAEs (equal n, so the plain mean)",
                   [&] { double m = 0; for (const SensorResult& s : result.sensors) m += s.linearTest.mae / 6.0; return tu::near(m, result.overallLinearTest.mae, 1e-9); }());
    }

    // ---- reproducibility
    {
        const std::string first = formatReport(result), second = formatReport(runBaseline(defaults));
        tu::expect("reproducibility: two independent runs produce byte-identical reports", first == second && !first.empty());
        bool mentionsAll = true;
        for (const std::string& id : dp::SELECTED_SENSORS) mentionsAll = mentionsAll && first.find(id) != std::string::npos;
        tu::expect("report: names all six sensors, the data span and the limitation warning",
                   mentionsAll && first.find("2.00 days") != std::string::npos && first.find("NOT a reliable estimate") != std::string::npos);
        tu::expect("report: the printed pooled test RMSE is the computed one",
                   first.find(dp::fmt("%8.4f", result.overallLinearTest.rmse)) != std::string::npos && first.find(dp::fmt("%8.4f", result.overallPersistenceTest.rmse)) != std::string::npos);
    }

    // ---- the committed reports must be what the code produces today
    {
        BaselineConfig thirty;
        thirty.features.horizonSteps = 6;
        tu::expect("committed report docs/baseline_report_15min.txt matches a fresh run (regenerate it if data or code changed)",
                   readText("docs/baseline_report_15min.txt") == formatReport(result));
        tu::expect("committed report docs/baseline_report_30min.txt matches a fresh run (regenerate it if data or code changed)",
                   readText("docs/baseline_report_30min.txt") == formatReport(runBaseline(thirty)));
    }

    // ---- other settings and failure handling
    {
        BaselineConfig thirty;
        thirty.features.horizonSteps = 6;
        BaselineResult r30 = runBaseline(thirty);
        bool horizonOk = !r30.sensors[0].test.targetRows.empty();
        for (const SensorResult& s : r30.sensors) horizonOk = horizonOk && s.trainSamples == 392 && s.testSamples == 87;
        tu::expect("settings: a 30-minute horizon (6 steps) gives 392 training and 87 test samples per sensor (3 fewer training samples than at 15 minutes)", horizonOk);
        BaselineConfig missing;
        missing.dataPath = "data/does_not_exist.csv";
        bool threw = false;
        try { runBaseline(missing); } catch (const std::runtime_error& e) { threw = std::string(e.what()).find("cannot open") != std::string::npos; }
        tu::expect("failure: a missing data file stops with a clear error", threw);
    }

    return tu::summary();
}
