// baseline_pipeline.h -- the whole baseline experiment: split the timeline, train, predict, score.
//
// Experiment (one independent Linear Regression per sensor, plus a persistence baseline):
//   1. build samples from the sensor's own past readings (features.h)
//   2. split the TIMELINE chronologically into train | validation | test (default 70% | 15% | 15% of rows).
//      A sample belongs to the part that contains its TARGET row; its inputs are all earlier readings,
//      so training never sees a reading from the validation or test period, not even as an input.
//   3. fit the model on the training samples only (scaling and weights are learned from them alone)
//   4. predict the validation and test samples; score with MAE and RMSE
//   5. "persistence" predicts that the speed stays at the most recent reading; it is scored on exactly
//      the same samples, so the comparison is fair.
// The validation part is reported but not used to tune anything in this baseline (no tuning was done),
// so the test part has not influenced any decision either.
// Nothing here is random: weights start at zero, so repeated runs give identical numbers.
#pragma once

#include <string>
#include <vector>

#include "features.h"
#include "linear_regression.h"
#include "metrics.h"

namespace baseline {

struct BaselineConfig {
    std::string dataPath = "data/traffic_data.csv";
    FeatureConfig features;                 // 6 lags, 3 steps (15 minutes) ahead
    double trainFraction = 0.70;
    double validationFraction = 0.15;       // whatever is left over (15%) is the test part
    GdOptions gd;
};

// Rows [0, trainEnd) are the training period, [trainEnd, validationEnd) validation, [validationEnd, rows) test.
struct SplitBoundaries {
    size_t rows = 0, trainEnd = 0, validationEnd = 0;
};

enum class Part { Train, Validation, Test };

// Throws std::invalid_argument for fractions that are not 0 < train, 0 < validation, train + validation < 1,
// or when there are too few rows for each part to be non-empty.
SplitBoundaries chronologicalBoundaries(size_t rows, double trainFraction, double validationFraction);
Part partOfTargetRow(size_t targetRow, const SplitBoundaries& boundaries);

struct Predictions {
    std::vector<size_t> targetRows;         // which row each prediction is for
    std::vector<double> actual;             // the real speed at that row
    std::vector<double> linear;             // Linear Regression's prediction
    std::vector<double> persistence;        // the most recent reading, carried forward
};

struct SensorResult {
    std::string sensorId;
    size_t candidates = 0, usable = 0, droppedMissing = 0, droppedIrregular = 0;
    size_t trainSamples = 0, validationSamples = 0, testSamples = 0;
    GdResult gd;
    bool olsAvailable = false;
    double trainRmseGd = 0, trainRmseOls = 0;
    std::vector<double> coefficients;       // raw units: coefficient k multiplies speed(t - k)
    double intercept = 0;
    Predictions validation, test;
    Metrics linearValidation, linearTest, persistenceValidation, persistenceTest;
};

struct BaselineResult {
    BaselineConfig config;
    TrafficSeries series;
    SplitBoundaries split;
    std::vector<SensorResult> sensors;                       // in dp::SELECTED_SENSORS order
    // all sensors pooled together (every prediction counts once)
    Metrics overallLinearValidation, overallLinearTest, overallPersistenceValidation, overallPersistenceTest;
};

BaselineResult runBaselineOnSeries(const TrafficSeries& series, const BaselineConfig& config);
BaselineResult runBaseline(const BaselineConfig& config);    // loads config.dataPath first

}  // namespace baseline
