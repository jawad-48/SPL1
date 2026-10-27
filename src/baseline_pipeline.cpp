#include "baseline_pipeline.h"

#include <cmath>

namespace baseline {

SplitBoundaries chronologicalBoundaries(size_t rows, double trainFraction, double validationFraction) {
    if (!(trainFraction > 0) || !(validationFraction > 0) || !(trainFraction + validationFraction < 1))
        throw std::invalid_argument("fractions must satisfy 0 < train, 0 < validation and train + validation < 1");
    SplitBoundaries b;
    b.rows = rows;
    // the tiny epsilon only protects against 0.7 + 0.15 being stored as 0.8499999...
    b.trainEnd = static_cast<size_t>(std::floor(static_cast<double>(rows) * trainFraction + 1e-9));
    b.validationEnd = static_cast<size_t>(std::floor(static_cast<double>(rows) * (trainFraction + validationFraction) + 1e-9));
    if (b.trainEnd == 0 || b.validationEnd <= b.trainEnd || b.validationEnd >= rows)
        throw std::invalid_argument("too few rows to make non-empty training, validation and test parts");
    return b;
}

Part partOfTargetRow(size_t targetRow, const SplitBoundaries& b) {
    if (targetRow < b.trainEnd) return Part::Train;
    if (targetRow < b.validationEnd) return Part::Validation;
    return Part::Test;
}

static void addPrediction(Predictions& p, const Sample& s, const LinearModel& model) {
    p.targetRows.push_back(s.targetRow);
    p.actual.push_back(s.y);
    p.linear.push_back(model.predict(s.x));
    p.persistence.push_back(s.lastObserved);
}

static void append(std::vector<double>& to, const std::vector<double>& from) { to.insert(to.end(), from.begin(), from.end()); }

BaselineResult runBaselineOnSeries(const TrafficSeries& series, const BaselineConfig& config) {
    BaselineResult result;
    result.config = config;
    result.series = series;
    result.split = chronologicalBoundaries(series.rows(), config.trainFraction, config.validationFraction);

    std::vector<double> valActual, valLinear, valPersist, testActual, testLinear, testPersist;   // pooled over sensors
    for (size_t s = 0; s < series.sensors.size(); ++s) {
        SensorResult r;
        r.sensorId = series.sensors[s];
        const SampleSet set = buildSamples(series, s, config.features);
        r.candidates = set.candidates;
        r.usable = set.samples.size();
        r.droppedMissing = set.droppedMissing;
        r.droppedIrregular = set.droppedIrregular;

        // 1. training samples: only those whose TARGET lies in the training period
        Matrix trainX;
        std::vector<double> trainY;
        for (const Sample& sample : set.samples) {
            if (partOfTargetRow(sample.targetRow, result.split) != Part::Train) continue;
            trainX.push_back(sample.x);
            trainY.push_back(sample.y);
        }
        r.trainSamples = trainX.size();

        // 2. fit (scaling and weights come from these training samples alone)
        LinearModel model;
        try {
            r.gd = model.fit(trainX, trainY, config.gd);
        } catch (const std::invalid_argument& e) {
            throw std::runtime_error("sensor " + r.sensorId + ": cannot train, " + e.what() + dp::fmt(" (%zu training samples)", trainX.size()));
        }
        r.olsAvailable = model.olsAvailable();
        r.trainRmseGd = model.trainRmseGradientDescent();
        r.trainRmseOls = model.trainRmseOls();
        r.coefficients = model.coefficients();
        r.intercept = model.intercept();

        // 3. predict the held-out parts
        for (const Sample& sample : set.samples) {
            const Part part = partOfTargetRow(sample.targetRow, result.split);
            if (part == Part::Validation) addPrediction(r.validation, sample, model);
            else if (part == Part::Test) addPrediction(r.test, sample, model);
        }
        r.validationSamples = r.validation.actual.size();
        r.testSamples = r.test.actual.size();

        // 4. score
        r.linearValidation = computeMetrics(r.validation.linear, r.validation.actual);
        r.persistenceValidation = computeMetrics(r.validation.persistence, r.validation.actual);
        r.linearTest = computeMetrics(r.test.linear, r.test.actual);
        r.persistenceTest = computeMetrics(r.test.persistence, r.test.actual);
        append(valActual, r.validation.actual); append(valLinear, r.validation.linear); append(valPersist, r.validation.persistence);
        append(testActual, r.test.actual); append(testLinear, r.test.linear); append(testPersist, r.test.persistence);
        result.sensors.push_back(r);
    }
    result.overallLinearValidation = computeMetrics(valLinear, valActual);
    result.overallPersistenceValidation = computeMetrics(valPersist, valActual);
    result.overallLinearTest = computeMetrics(testLinear, testActual);
    result.overallPersistenceTest = computeMetrics(testPersist, testActual);
    return result;
}

BaselineResult runBaseline(const BaselineConfig& config) {
    return runBaselineOnSeries(loadTrafficSeries(config.dataPath), config);
}

}  // namespace baseline
