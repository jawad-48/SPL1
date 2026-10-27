// test_baseline_model.cpp -- tests for src/metrics.*, src/linear_regression.* and src/baseline_pipeline.*
//
// Build (from the repository root):
//   g++ -std=c++17 -O2 -Wall -Wextra -o build/test_baseline_model tests/test_baseline_model.cpp src/traffic_data.cpp src/features.cpp src/metrics.cpp src/linear_regression.cpp src/baseline_pipeline.cpp
// Run:
//   build/test_baseline_model
//
// Randomness appears only in synthetic test data, always from std::mt19937 with a fixed seed.
#include <functional>
#include <random>

#include "../src/baseline_pipeline.h"
#include "test_util.h"

using namespace baseline;

// ------------------------------------------------------------------ helpers
static TrafficSeries makeSeries(size_t rows, const std::function<double(size_t sensor, size_t row)>& speed) {
    TrafficSeries s;
    s.source = "synthetic";
    s.sensors = dp::SELECTED_SENSORS;
    s.speeds.assign(s.sensors.size(), std::vector<double>(rows));
    for (size_t r = 0; r < rows; ++r) s.times.push_back(1330560000LL + 300 * static_cast<long long>(r));
    for (size_t k = 0; k < s.sensors.size(); ++k) for (size_t r = 0; r < rows; ++r) s.speeds[k][r] = speed(k, r);
    return s;
}

// A mean-reverting random series: it drifts back towards 60 mph, with a little noise. Fixed seed.
static TrafficSeries meanReverting(size_t rows, unsigned seed) {
    std::mt19937 gen(seed);
    auto noise = [&] { return (static_cast<double>(gen()) / 4294967296.0 - 0.5) * 8.0; };   // uniform in [-4, 4)
    TrafficSeries s = makeSeries(rows, [](size_t, size_t) { return 60.0; });
    for (size_t k = 0; k < 6; ++k)
        for (size_t r = 1; r < rows; ++r) s.speeds[k][r] = 60.0 + 0.9 * (s.speeds[k][r - 1] - 60.0) + noise();
    return s;
}

static bool sameVector(const std::vector<double>& a, const std::vector<double>& b) { return a == b; }

int main() {
    // ------------------------------------------------------------------ metrics
    {
        Metrics m = computeMetrics({1, 2, 3}, {1, 4, 0});                     // errors 0, -2, +3
        tu::expect("metrics: MAE and RMSE match a hand calculation (1.6667 and 2.0817)",
                   m.n == 3 && tu::near(m.mae, 5.0 / 3.0) && tu::near(m.rmse, std::sqrt(13.0 / 3.0)));
        Metrics perfect = computeMetrics({5, 6}, {5, 6});
        tu::expect("metrics: identical predictions give zero error", perfect.mae == 0 && perfect.rmse == 0);
        Metrics sign = computeMetrics({10, 10}, {12, 8});                     // +2 and -2 must not cancel
        tu::expect("metrics: errors in opposite directions do not cancel", tu::near(sign.mae, 2.0) && tu::near(sign.rmse, 2.0));
        Metrics empty = computeMetrics({}, {});
        tu::expect("metrics: no predictions gives n = 0 and NaN, not a fake zero", empty.n == 0 && std::isnan(empty.mae) && std::isnan(empty.rmse));
        bool threw = false;
        try { computeMetrics({1, 2}, {1}); } catch (const std::invalid_argument&) { threw = true; }
        tu::expect("metrics: mismatched lengths are rejected", threw);
        Metrics outlier = computeMetrics({0, 0, 0, 0}, {0, 0, 0, 8});
        tu::expect("metrics: RMSE punishes one large miss more than MAE does", tu::near(outlier.mae, 2.0) && tu::near(outlier.rmse, 4.0));
    }

    // ------------------------------------------------------------------ standardiser: learns from the data it is given, nothing else
    {
        Standardizer sc;
        sc.fit({{1, 10}, {3, 10}, {5, 10}}, {2, 4, 6});
        const double sd = std::sqrt(8.0 / 3.0);
        tu::expect("scaling: mean and standard deviation are those of the data given to fit()",
                   tu::near(sc.featureMean[0], 3) && tu::near(sc.featureMean[1], 10) && tu::near(sc.featureStd[0], sd) &&
                   tu::near(sc.targetMean, 4) && tu::near(sc.targetStd, sd));
        tu::expect("scaling: a constant column gets std 1 and scales to 0 instead of NaN", sc.featureStd[1] == 1.0 && sc.scaleFeatures({5, 10})[1] == 0.0);
        const std::vector<double> before = sc.featureMean;
        sc.scaleFeatures({1000, 1000});                                        // transforming later data...
        tu::expect("scaling: transforming new data never changes the stored numbers", sameVector(before, sc.featureMean) && tu::near(sc.scaleFeatures({3, 10})[0], 0));
    }

    // ------------------------------------------------------------------ gradient descent
    {
        Matrix X;
        std::vector<double> y;
        for (int i = 0; i < 60; ++i) {
            const double a = i % 7, b = (i * 3) % 5 + 0.5 * (i % 3);
            X.push_back({a, b});
            y.push_back(3.0 + 2.0 * a - 1.5 * b);
        }
        LinearModel model;
        GdResult r = model.fit(X, y, GdOptions());
        bool exact = true;
        for (size_t i = 0; i < X.size(); ++i) exact = exact && tu::near(model.predict(X[i]), y[i], 1e-5);
        const std::vector<double> c = model.coefficients();
        tu::expect("gradient descent: recovers a noiseless law y = 3 + 2a - 1.5b",
                   exact && tu::near(c[0], 2.0, 1e-4) && tu::near(c[1], -1.5, 1e-4) && tu::near(model.intercept(), 3.0, 1e-4));
        tu::expect("gradient descent: reports that it converged", r.converged && r.epochs > 0 && r.finalLoss < 1e-10);
    }
    {
        // 400 noisy, strongly correlated samples: gradient descent should land very near the exact least-squares fit
        TrafficSeries s = meanReverting(420, 42);
        SampleSet set = buildSamples(s, 0, FeatureConfig());
        Matrix X;
        std::vector<double> y;
        for (const Sample& smp : set.samples) { X.push_back(smp.x); y.push_back(smp.y); }
        LinearModel model;
        model.fit(X, y, GdOptions());
        const double gd = model.trainRmseGradientDescent(), ols = model.trainRmseOls();
        tu::expect("gradient descent: ends within 0.1% of the exact least-squares training error (correlated lags)",
                   model.olsAvailable() && gd >= ols - 1e-9 && (gd - ols) / ols < 1e-3);
    }
    {
        std::vector<double> w;
        double b = 0;
        tu::expect("normal equation: solves y = 2x + 1 exactly", solveNormalEquation({{1}, {2}, {3}}, {3, 5, 7}, w, b) && tu::near(w[0], 2) && tu::near(b, 1));
        tu::expect("normal equation: reports a singular system (duplicate columns) instead of dividing by zero",
                   !solveNormalEquation({{1, 1}, {2, 2}, {3, 3}, {4, 4}}, {1, 2, 3, 4}, w, b));
    }
    {
        LinearModel unfitted;
        bool a = false, b = false, c = false;
        try { unfitted.predict({1}); } catch (const std::logic_error&) { a = true; }
        try { unfitted.fit({{1}, {2}}, {1, 2}, GdOptions()); } catch (const std::invalid_argument&) { b = true; }       // needs d + 2 samples
        LinearModel fitted;
        fitted.fit({{1}, {2}, {3}, {4}}, {2, 4, 6, 8}, GdOptions());
        try { fitted.predict({1, 2}); } catch (const std::invalid_argument&) { c = true; }
        tu::expect("model: predict before fit, too few samples and a wrong-length input are all rejected", a && b && c);
    }

    // ------------------------------------------------------------------ chronological split
    {
        SplitBoundaries b = chronologicalBoundaries(576, 0.70, 0.15);
        tu::expect("split: 576 rows -> train rows [0,403), validation [403,489), test [489,576)", b.trainEnd == 403 && b.validationEnd == 489 && b.rows == 576);
        tu::expect("split: a target row falls in exactly one part, in time order",
                   partOfTargetRow(0, b) == Part::Train && partOfTargetRow(402, b) == Part::Train && partOfTargetRow(403, b) == Part::Validation &&
                   partOfTargetRow(488, b) == Part::Validation && partOfTargetRow(489, b) == Part::Test && partOfTargetRow(575, b) == Part::Test);
        auto bad = [](double t, double v, size_t rows) {
            try { chronologicalBoundaries(rows, t, v); } catch (const std::invalid_argument&) { return true; }
            return false;
        };
        tu::expect("split: impossible fractions or too few rows are rejected", bad(0.9, 0.2, 100) && bad(0, 0.5, 100) && bad(0.5, 0, 100) && bad(0.5, 0.2, 2));
        SplitBoundaries round = chronologicalBoundaries(1000, 0.70, 0.15);
        tu::expect("split: 0.70 + 0.15 does not lose a row to floating-point error", round.trainEnd == 700 && round.validationEnd == 850);
    }

    // ------------------------------------------------------------------ pipeline: a ramp, where the right answer is known exactly
    {
        // speed(sensor, row) = 50 + 10*sensor + 0.1*row, so the speed 3 steps later is exactly 0.3 mph higher
        TrafficSeries s = makeSeries(300, [](size_t k, size_t r) { return 50.0 + 10.0 * k + 0.1 * static_cast<double>(r); });
        BaselineResult res = runBaselineOnSeries(s, BaselineConfig());
        bool linearExact = true, persistenceRight = true;
        for (const SensorResult& r : res.sensors) {
            // gradient descent stops once its standardised loss is below 1e-14, which leaves an error of about 1e-6 mph;
            // 1e-5 mph is still 30,000 times smaller than the 0.3 mph that persistence is off by
            linearExact = linearExact && r.linearTest.mae < 1e-5 && r.linearValidation.mae < 1e-5;
            persistenceRight = persistenceRight && tu::near(r.persistenceTest.mae, 0.3, 1e-9) && tu::near(r.persistenceTest.rmse, 0.3, 1e-9);
        }
        tu::expect("pipeline/ramp: Linear Regression predicts a straight-line series exactly on held-out data", linearExact);
        tu::expect("pipeline/ramp: persistence is off by exactly horizon*0.1 = 0.3 mph", persistenceRight);
        tu::expect("pipeline/ramp: perfectly collinear lags (singular normal equations) are handled, not fatal", !res.sensors[0].olsAvailable);
        tu::expect("pipeline/ramp: all six sensors are evaluated, in the approved order", res.sensors.size() == 6 && res.sensors[0].sensorId == dp::SELECTED_SENSORS[0] && res.sensors[5].sensorId == dp::SELECTED_SENSORS[5]);
    }

    // ------------------------------------------------------------------ pipeline: chronology, counts, leakage, reproducibility
    {
        const TrafficSeries s = meanReverting(300, 7);
        BaselineResult a = runBaselineOnSeries(s, BaselineConfig());
        bool ordered = true, inRange = true, counts = true;
        for (const SensorResult& r : a.sensors) {
            for (size_t i = 0; i < r.validation.targetRows.size(); ++i) {
                inRange = inRange && r.validation.targetRows[i] >= a.split.trainEnd && r.validation.targetRows[i] < a.split.validationEnd;
                ordered = ordered && (i == 0 || r.validation.targetRows[i] > r.validation.targetRows[i - 1]);
            }
            for (size_t i = 0; i < r.test.targetRows.size(); ++i) {
                inRange = inRange && r.test.targetRows[i] >= a.split.validationEnd && r.test.targetRows[i] < a.split.rows;
                ordered = ordered && (i == 0 || r.test.targetRows[i] > r.test.targetRows[i - 1]);
            }
            counts = counts && r.trainSamples + r.validationSamples + r.testSamples == r.usable &&
                     r.usable + r.droppedMissing + r.droppedIrregular == r.candidates && r.trainSamples > 0 && r.validationSamples > 0 && r.testSamples > 0;
        }
        tu::expect("pipeline/split: validation targets lie in the validation period and test targets in the test period, oldest first", inRange && ordered);
        tu::expect("pipeline/split: train + validation + test = usable, and usable + dropped = candidates", counts);

        // Leakage check: overwrite EVERY speed from the first validation row onward with unrelated numbers.
        // Training uses only rows before that point, so the fitted model must not change AT ALL.
        TrafficSeries damaged = s;
        std::mt19937 gen(99);
        for (auto& column : damaged.speeds)
            for (size_t r = a.split.trainEnd; r < column.size(); ++r) column[r] = 20.0 + static_cast<double>(gen() % 4000) / 100.0;
        BaselineResult b = runBaselineOnSeries(damaged, BaselineConfig());
        bool sameModel = true, differentScores = false;
        for (size_t k = 0; k < 6; ++k) {
            sameModel = sameModel && sameVector(a.sensors[k].coefficients, b.sensors[k].coefficients) && a.sensors[k].intercept == b.sensors[k].intercept &&
                        a.sensors[k].gd.epochs == b.sensors[k].gd.epochs && a.sensors[k].trainSamples == b.sensors[k].trainSamples;
            differentScores = differentScores || a.sensors[k].linearTest.rmse != b.sensors[k].linearTest.rmse;
        }
        tu::expect("pipeline/leakage: rewriting all validation and test data leaves the trained model bit-for-bit identical", sameModel);
        tu::expect("pipeline/leakage: ...while the held-out scores do change (so the experiment really did use that data)", differentScores);

        BaselineResult again = runBaselineOnSeries(s, BaselineConfig());
        bool identical = true;
        for (size_t k = 0; k < 6; ++k)
            identical = identical && sameVector(a.sensors[k].coefficients, again.sensors[k].coefficients) && sameVector(a.sensors[k].test.linear, again.sensors[k].test.linear);
        tu::expect("pipeline/reproducibility: running twice gives identical coefficients and predictions", identical && a.overallLinearTest.rmse == again.overallLinearTest.rmse);
    }

    // ------------------------------------------------------------------ pipeline: reported numbers come from the held-out predictions
    {
        const TrafficSeries s = meanReverting(300, 5);
        const BaselineConfig cfg;
        BaselineResult res = runBaselineOnSeries(s, cfg);
        const SensorResult& r = res.sensors[3];
        bool recomputed = true;
        double sumAbs = 0;
        for (size_t i = 0; i < r.test.targetRows.size(); ++i) {
            const size_t target = r.test.targetRows[i], origin = target - static_cast<size_t>(cfg.features.horizonSteps);
            double p = r.intercept;                                                // rebuild the prediction by hand from the printed coefficients
            for (size_t k = 0; k < r.coefficients.size(); ++k) p += r.coefficients[k] * s.speeds[3][origin - k];
            recomputed = recomputed && tu::near(p, r.test.linear[i], 1e-9) && r.test.actual[i] == s.speeds[3][target] && r.test.persistence[i] == s.speeds[3][origin];
            sumAbs += std::fabs(p - s.speeds[3][target]);
        }
        tu::expect("pipeline/metrics: predictions rebuilt by hand from the coefficients and the raw data match the stored ones", recomputed && !r.test.targetRows.empty());
        tu::expect("pipeline/metrics: the reported test MAE equals the MAE of those held-out predictions", tu::near(r.linearTest.mae, sumAbs / static_cast<double>(r.test.targetRows.size()), 1e-9));
        size_t total = 0;
        for (const SensorResult& x : res.sensors) total += x.testSamples;
        tu::expect("pipeline/metrics: overall test n equals the number of test samples across all sensors", res.overallLinearTest.n == total && res.overallPersistenceTest.n == total);
    }

    // ------------------------------------------------------------------ pipeline: it can find structure, and refuses impossible jobs
    {
        const TrafficSeries s = meanReverting(2000, 11);
        BaselineResult res = runBaselineOnSeries(s, BaselineConfig());
        tu::expect("sanity: on a mean-reverting series Linear Regression beats persistence on held-out data (RMSE)",
                   res.overallLinearTest.rmse < res.overallPersistenceTest.rmse && res.overallLinearValidation.rmse < res.overallPersistenceValidation.rmse);
    }
    {
        TrafficSeries s = meanReverting(300, 3);
        for (size_t r = 40; r < 60; ++r) s.speeds[2][r] = dp::NAN_VALUE;                // a gap inside the training period
        for (size_t r = 270; r < 275; ++r) s.speeds[2][r] = dp::NAN_VALUE;              // and one in the test period
        BaselineResult res = runBaselineOnSeries(s, BaselineConfig());
        const SensorResult& r = res.sensors[2];
        tu::expect("pipeline/missing: samples that touch empty values are dropped and counted; the others still train and score",
                   r.droppedMissing > 0 && r.usable + r.droppedMissing + r.droppedIrregular == r.candidates && res.sensors[0].droppedMissing == 0 && r.testSamples > 0);
        bool finite = true;
        for (double v : r.test.linear) finite = finite && std::isfinite(v);
        tu::expect("pipeline/missing: no NaN reaches the predictions", finite && std::isfinite(res.overallLinearTest.rmse));
    }
    {
        bool threw = false;
        try { runBaselineOnSeries(meanReverting(12, 1), BaselineConfig()); } catch (const std::runtime_error& e) { threw = std::string(e.what()).find("cannot train") != std::string::npos; }
        catch (const std::invalid_argument&) { threw = true; }
        tu::expect("pipeline: a series too short to train on is rejected with an error, not a meaningless model", threw);
    }

    return tu::summary();
}
