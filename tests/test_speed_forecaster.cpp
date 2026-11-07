// test_speed_forecaster.cpp -- the forecaster reuses the baseline Linear Regression exactly, trains only on the past,
// reads only rows <= the origin, handles missing data without imputing, and rejects invalid predicted speeds.
//
// Build (from the repository root):
//   g++ -std=c++17 -O2 -Wall -Wextra -o build/test_speed_forecaster tests/test_speed_forecaster.cpp src/speed_forecaster.cpp src/traffic_data.cpp src/features.cpp src/metrics.cpp src/linear_regression.cpp src/baseline_pipeline.cpp
// Run from the repository root:   build/test_speed_forecaster
#include <algorithm>

#include "routing_fixtures.h"

using namespace fx;
using tu::expect;

static const size_t TRAIN_END = 280;          // 400 rows, 70% training
static const size_t ORIGIN = 300;             // a validation-period origin: its window is rows 295..300

static bool sameForecast(const routing::ForecastSnapshot& a, const routing::ForecastSnapshot& b) {
    if (a.sensors.size() != b.sensors.size() || a.originRow != b.originRow || a.originTime != b.originTime || a.targetTime != b.targetTime) return false;
    for (size_t i = 0; i < a.sensors.size(); ++i) {
        const routing::SensorForecast &x = a.sensors[i], &y = b.sensors[i];
        if (x.sensorId != y.sensorId || x.available != y.available) return false;
        if (x.available && x.speedMph != y.speedMph) return false;          // exact, bit-for-bit
    }
    return true;
}

static bool sameModels(const routing::SpeedForecaster& a, const routing::SpeedForecaster& b) {
    for (size_t s = 0; s < 6; ++s) {
        if (a.model(s).coefficients() != b.model(s).coefficients() || a.model(s).intercept() != b.model(s).intercept()) return false;
        if (a.trainSamples(s) != b.trainSamples(s)) return false;
    }
    return true;
}

static void testReuseOfTheBaseline(const baseline::TrafficSeries& series, const baseline::BaselineConfig& cfg, const routing::SpeedForecaster& f) {
    const baseline::BaselineResult base = baseline::runBaselineOnSeries(series, cfg);
    expect("reuse: the training cutoff is the baseline's train/validation boundary", f.trainEndRow() == base.split.trainEnd && base.split.trainEnd == TRAIN_END);
    expect("reuse: the earliest allowed origin is trainEnd - 1", f.earliestOriginRow() == TRAIN_END - 1);
    expect("reuse: settings come from the baseline config (6 lags, 3 steps)", f.features().lags == 6 && f.features().horizonSteps == 3 && f.sensors() == dp::SELECTED_SENSORS);

    bool coefficients = true, samples = true;
    for (size_t s = 0; s < 6; ++s) {
        coefficients = coefficients && f.model(s).coefficients() == base.sensors[s].coefficients && f.model(s).intercept() == base.sensors[s].intercept;
        samples = samples && f.trainSamples(s) == base.sensors[s].trainSamples && base.sensors[s].trainSamples > 100;
    }
    expect("reuse: every sensor's coefficients and intercept equal the baseline's, bit for bit", coefficients);
    expect("reuse: the same number of training samples is used per sensor", samples);

    // The baseline's first two held-out samples per sensor have their origin just BEFORE the training cutoff (target row trainEnd and
    // trainEnd+1, origin trainEnd-3 and trainEnd-2). Their inputs are legal for the baseline, but the forecaster is stricter: it refuses
    // any origin before trainEnd-1 (see "origin rules"). Those 2 samples per sensor are skipped here and counted.
    size_t compared = 0, refused = 0;
    bool identical = true, refusedAreRefused = true;
    for (size_t s = 0; s < 6; ++s) {
        const baseline::Predictions* parts[2] = {&base.sensors[s].validation, &base.sensors[s].test};
        for (const baseline::Predictions* p : parts) {
            for (size_t i = 0; i < p->targetRows.size(); ++i) {
                const size_t origin = p->targetRows[i] - 3;
                if (origin < f.earliestOriginRow()) {
                    ++refused;
                    refusedAreRefused = refusedAreRefused && throws<std::invalid_argument>([&] { f.predictAt(series, origin); });
                    continue;
                }
                const routing::ForecastSnapshot snap = f.predictAt(series, origin);
                identical = identical && snap.sensors[s].available && snap.sensors[s].speedMph == p->linear[i];
                ++compared;
            }
        }
    }
    expect("reuse: forecasts equal the baseline's held-out predictions bit for bit (validation and test, 600+ forecasts)", identical && compared > 600);
    expect("reuse: exactly the 2 earliest held-out samples per sensor (12 in all) are refused as origins, and they are refused", refused == 12 && refusedAreRefused);

    // the window order: x[0] is the NEWEST reading
    const routing::ForecastSnapshot snap = f.predictAt(series, ORIGIN);
    std::vector<double> x, reversed;
    for (size_t k = 0; k < 6; ++k) x.push_back(series.speeds[0][ORIGIN - k]);
    reversed.assign(x.rbegin(), x.rend());
    expect("window: x[k] = speed(t - k) with k = 0 the newest reading (rows t-5..t)", snap.sensors[0].speedMph == f.model(0).predict(x));
    expect("window: the reversed order would give a different number (so the order is really tested)", f.model(0).predict(reversed) != snap.sensors[0].speedMph);
    expect("snapshot: origin time is the time of row t, target time is 15 minutes later, horizon 3", snap.originRow == ORIGIN && snap.originTime == series.times[ORIGIN] && snap.targetTime == series.times[ORIGIN] + 900 && snap.horizonSteps == 3);
    expect("snapshot: all six sensors are available on complete data, in the approved order", snap.sensors.size() == 6 && std::all_of(snap.sensors.begin(), snap.sensors.end(), [](const routing::SensorForecast& s) { return s.available && s.speedMph > 0; }) && snap.sensors[3].sensorId == "717446");
    expect("snapshot: find() locates a sensor and returns nullptr for an unknown one", snap.find("773062") != nullptr && snap.find("000000") == nullptr);
}

static void testOriginRules(const baseline::TrafficSeries& series, const routing::SpeedForecaster& f) {
    expect("origin: trainEnd - 2 is refused (the model already knew the readings after it)", throwsWith<std::invalid_argument>([&] { f.predictAt(series, TRAIN_END - 2); }, "future"));
    expect("origin: row 0 is refused", throws<std::invalid_argument>([&] { f.predictAt(series, 0); }));
    expect("origin: trainEnd - 1 is the first allowed origin", f.predictAt(series, TRAIN_END - 1).sensors[0].available);
    expect("origin: trainEnd is allowed", f.predictAt(series, TRAIN_END).sensors[0].available);
    expect("origin: the last row of the series is allowed (the forecast looks past the data, only reads rows <= t)", f.predictAt(series, series.rows() - 1).sensors[5].available);
    expect("origin: a row beyond the series is refused", throws<std::invalid_argument>([&] { f.predictAt(series, series.rows()); }));
    baseline::BaselineConfig cfg;
    expect("train: trainEndRow 0 is refused", throws<std::invalid_argument>([&] { routing::SpeedForecaster::trainBefore(series, cfg, 0); }));
    expect("train: trainEndRow beyond the series is refused", throws<std::invalid_argument>([&] { routing::SpeedForecaster::trainBefore(series, cfg, series.rows() + 1); }));
    expect("train: too little data to train fails with a clear message naming the sensor", throwsWith<std::runtime_error>([&] { routing::SpeedForecaster::trainBefore(series, cfg, 10); }, "716328"));
    baseline::TrafficSeries odd = series;
    odd.sensors.pop_back();
    odd.speeds.pop_back();
    expect("train: a series without exactly the six selected sensors is refused", throws<std::invalid_argument>([&] { routing::SpeedForecaster::train(odd, cfg); }));
    expect("predict: a series with other sensors than the trained ones is refused", throws<std::invalid_argument>([&] { f.predictAt(odd, ORIGIN); }));
}

static void testNoLeakage(const baseline::TrafficSeries& series, const baseline::BaselineConfig& cfg, const routing::SpeedForecaster& f) {
    // ---- training: rows at or after the cutoff must not influence the models in any way
    baseline::TrafficSeries garbageAfterCutoff = series;
    for (size_t r = TRAIN_END; r < series.rows(); ++r) {
        garbageAfterCutoff.times[r] = 7;                                                   // not even the time stamps are real
        for (size_t s = 0; s < 6; ++s) garbageAfterCutoff.speeds[s][r] = (r % 3 == 0) ? NaN : 1.0e6 + static_cast<double>(r);
    }
    const routing::SpeedForecaster fromGarbage = routing::SpeedForecaster::trainBefore(garbageAfterCutoff, cfg, TRAIN_END);
    expect("leakage/training: replacing every row at or after the cutoff with garbage leaves all models bit-identical", sameModels(f, fromGarbage));

    baseline::TrafficSeries scaledAfter = series;
    for (size_t r = TRAIN_END; r < series.rows(); ++r) for (size_t s = 0; s < 6; ++s) scaledAfter.speeds[s][r] *= 0.5;
    expect("leakage/training: halving every later speed also changes nothing", sameModels(f, routing::SpeedForecaster::trainBefore(scaledAfter, cfg, TRAIN_END)));

    baseline::TrafficSeries lastTrainingRow = series;                                       // sensitivity: the LAST training row does matter
    lastTrainingRow.speeds[0][TRAIN_END - 1] += 25.0;
    const routing::SpeedForecaster changed = routing::SpeedForecaster::trainBefore(lastTrainingRow, cfg, TRAIN_END);
    expect("leakage/training (sensitivity): changing row trainEnd-1 DOES change sensor 0's model, so the check above can fail", changed.model(0).coefficients() != f.model(0).coefficients());
    expect("leakage/training: it changes only that sensor's model", changed.model(1).coefficients() == f.model(1).coefficients());

    const routing::SpeedForecaster later = routing::SpeedForecaster::trainBefore(series, cfg, TRAIN_END + 40);
    expect("leakage/training: a later cutoff trains on more rows and does give a different model", later.model(0).coefficients() != f.model(0).coefficients() && later.earliestOriginRow() == TRAIN_END + 39);

    // ---- prediction: only rows <= origin are read
    const routing::ForecastSnapshot expected = f.predictAt(series, ORIGIN);
    baseline::TrafficSeries garbageAfterOrigin = series;
    for (size_t r = ORIGIN + 1; r < series.rows(); ++r) {
        garbageAfterOrigin.times[r] = -5;
        for (size_t s = 0; s < 6; ++s) garbageAfterOrigin.speeds[s][r] = (r % 2 == 0) ? NaN : 9.9e9;
    }
    expect("leakage/prediction: garbage in every row after the origin gives the identical forecast", sameForecast(expected, f.predictAt(garbageAfterOrigin, ORIGIN)));

    baseline::TrafficSeries cut = series;                                                   // the series simply ends at the origin
    cut.times.resize(ORIGIN + 1);
    for (size_t s = 0; s < 6; ++s) cut.speeds[s].resize(ORIGIN + 1);
    expect("leakage/prediction: a series that ends at the origin row gives the identical forecast", sameForecast(expected, f.predictAt(cut, ORIGIN)));

    baseline::TrafficSeries doubled = series;
    for (size_t r = ORIGIN + 1; r < series.rows(); ++r) for (size_t s = 0; s < 6; ++s) doubled.speeds[s][r] *= 2.0;
    expect("leakage/prediction: changing only future speeds changes nothing", sameForecast(expected, f.predictAt(doubled, ORIGIN)));

    // sensitivity: rows inside the window matter, rows just outside do not
    baseline::TrafficSeries newest = series;  newest.speeds[0][ORIGIN] += 10.0;
    baseline::TrafficSeries oldestInWindow = series;  oldestInWindow.speeds[0][ORIGIN - 5] += 10.0;
    baseline::TrafficSeries justOutside = series;  justOutside.speeds[0][ORIGIN - 6] += 10.0;
    expect("leakage/prediction (sensitivity): changing row t changes the forecast", f.predictAt(newest, ORIGIN).sensors[0].speedMph != expected.sensors[0].speedMph);
    expect("leakage/prediction (sensitivity): changing row t-5 (the oldest reading used) changes the forecast", f.predictAt(oldestInWindow, ORIGIN).sensors[0].speedMph != expected.sensors[0].speedMph);
    expect("leakage/prediction: changing row t-6 (just outside the window) changes nothing", sameForecast(expected, f.predictAt(justOutside, ORIGIN)));
    expect("leakage/prediction: changing sensor 0 does not change any other sensor's forecast (one model per sensor)", f.predictAt(newest, ORIGIN).sensors[1].speedMph == expected.sensors[1].speedMph);

    expect("determinism: the same forecast twice gives bit-identical numbers", sameForecast(expected, f.predictAt(series, ORIGIN)));
    const routing::SpeedForecaster again = routing::SpeedForecaster::train(series, cfg);
    expect("determinism: training twice gives bit-identical models and forecasts", sameModels(f, again) && sameForecast(expected, again.predictAt(series, ORIGIN)));
}

static void testMissingData(const baseline::TrafficSeries& series, const routing::SpeedForecaster& f) {
    const routing::ForecastSnapshot expected = f.predictAt(series, ORIGIN);

    baseline::TrafficSeries hole = series;  hole.speeds[2][ORIGIN - 3] = NaN;
    routing::ForecastSnapshot h = f.predictAt(hole, ORIGIN);
    expect("missing: an empty cell inside the window makes that sensor unavailable, with a reason", !h.sensors[2].available && h.sensors[2].reason.find("missing") != std::string::npos && std::isnan(h.sensors[2].speedMph));
    bool othersSame = true;
    for (size_t s = 0; s < 6; ++s) if (s != 2) othersSame = othersSame && h.sensors[s].available && h.sensors[s].speedMph == expected.sensors[s].speedMph;
    expect("missing: the other five sensors are forecast exactly as before", othersSame);

    baseline::TrafficSeries outside = series;  outside.speeds[2][ORIGIN - 6] = NaN;
    baseline::TrafficSeries future = series;  future.speeds[2][ORIGIN + 1] = NaN;
    expect("missing: an empty cell just before the window (row t-6) does not matter", sameForecast(expected, f.predictAt(outside, ORIGIN)));
    expect("missing: an empty cell after the origin does not matter", sameForecast(expected, f.predictAt(future, ORIGIN)));

    baseline::TrafficSeries newestMissing = series;  newestMissing.speeds[4][ORIGIN] = NaN;
    expect("missing: an empty newest reading makes the sensor unavailable (no carrying forward at forecast time)", !f.predictAt(newestMissing, ORIGIN).sensors[4].available);

    baseline::TrafficSeries zero = series;  zero.speeds[1][ORIGIN - 1] = 0.0;
    baseline::TrafficSeries negative = series;  negative.speeds[1][ORIGIN - 1] = -4.0;
    expect("missing: a reading of 0 or a negative reading is not a valid observation, so the sensor is unavailable", !f.predictAt(zero, ORIGIN).sensors[1].available && !f.predictAt(negative, ORIGIN).sensors[1].available);

    baseline::TrafficSeries gap = series;  gap.times[ORIGIN - 2] += 60;                      // a hole in the 5-minute grid inside the window
    routing::ForecastSnapshot g = f.predictAt(gap, ORIGIN);
    expect("missing: a hole in the time grid inside the window makes every sensor unavailable", std::all_of(g.sensors.begin(), g.sensors.end(), [](const routing::SensorForecast& s) { return !s.available && s.reason.find("5 minutes") != std::string::npos; }));
    baseline::TrafficSeries gapOutside = series;  gapOutside.times[ORIGIN - 7] += 60;
    expect("missing: a hole in the grid before the window does not matter", sameForecast(expected, f.predictAt(gapOutside, ORIGIN)));

    baseline::TrafficSeries infinite = series;  infinite.speeds[0][ORIGIN - 2] = INF;
    expect("missing: a non-finite reading is unavailable, never fed to the model", !f.predictAt(infinite, ORIGIN).sensors[0].available);
}

static void testInvalidPredictions(const baseline::TrafficSeries& series, const routing::SpeedForecaster& f) {
    expect("floor: a floor above every forecast rejects the prediction, naming the sensor", throwsWith<std::runtime_error>([&] { f.predictAt(series, ORIGIN, 1000.0); }, "716328"));
    expect("floor: the rejection message says it is below the floor", throwsWith<std::runtime_error>([&] { f.predictAt(series, ORIGIN, 1000.0); }, "floor"));
    expect("floor: a floor below the forecasts changes nothing", sameForecast(f.predictAt(series, ORIGIN, 0.0), f.predictAt(series, ORIGIN, 5.0)));
    expect("floor: a negative floor is refused", throws<std::invalid_argument>([&] { f.predictAt(series, ORIGIN, -1.0); }));
    expect("floor: a NaN floor is refused", throws<std::invalid_argument>([&] { f.predictAt(series, ORIGIN, NaN); }));
    expect("floor: an infinite floor is refused", throws<std::invalid_argument>([&] { f.predictAt(series, ORIGIN, INF); }));

    // Build a window whose model output is NEGATIVE: take sensor 0's most negative coefficient c_k and choose that one input so
    // that intercept + sum(c_j * x_j) = -5, with every other input at 60 mph. All inputs stay finite and positive.
    const baseline::LinearModel& m = f.model(0);
    const std::vector<double> c = m.coefficients();
    size_t k = 0;
    for (size_t j = 1; j < c.size(); ++j) if (c[j] < c[k]) k = j;
    double others = 0;
    for (size_t j = 0; j < c.size(); ++j) if (j != k) others += c[j] * 60.0;
    const bool haveNegativeCoefficient = c[k] < -1e-6;
    const double xk = haveNegativeCoefficient ? (-5.0 - m.intercept() - others) / c[k] : 60.0;
    expect("invalid prediction: the trained model has a negative coefficient and the crafted input is a valid positive speed", haveNegativeCoefficient && std::isfinite(xk) && xk > 0);
    baseline::TrafficSeries crafted = series;
    for (size_t j = 0; j < 6; ++j) crafted.speeds[0][ORIGIN - j] = (j == k) ? xk : 60.0;
    std::vector<double> window;
    for (size_t j = 0; j < 6; ++j) window.push_back(crafted.speeds[0][ORIGIN - j]);
    expect("invalid prediction: the model really does output a negative speed for that window", m.predict(window) < 0);
    expect("invalid prediction: a negative forecast is rejected (std::runtime_error naming the sensor), never replaced by 0", throwsWith<std::runtime_error>([&] { f.predictAt(crafted, ORIGIN); }, "716328") && throwsWith<std::runtime_error>([&] { f.predictAt(crafted, ORIGIN); }, "positive"));
}

int main() {
    const baseline::TrafficSeries series = syntheticSeries(400);
    baseline::BaselineConfig cfg;                                           // defaults: 6 lags, 3 steps, 70/15/15
    const routing::SpeedForecaster f = routing::SpeedForecaster::train(series, cfg);
    testReuseOfTheBaseline(series, cfg, f);
    testOriginRules(series, f);
    testNoLeakage(series, cfg, f);
    testMissingData(series, f);
    testInvalidPredictions(series, f);
    return tu::summary();
}
