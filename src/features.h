// features.h -- turns one sensor's speed series into (features, target) pairs for forecasting.
//
// For an "origin" row t (the moment the forecast is made):
//     features   x[k] = speed(t - k)         k = 0 .. lags-1     (lag_0 is the most recent reading)
//     target     y    = speed(t + horizon)   the speed `horizon` steps (5 minutes each) later
// Every feature uses row t or EARLIER, never later, so no future observation can leak into the inputs.
//
// A candidate origin is dropped (and counted) when
//   * any input or the target is missing (empty in the cleaned file), or
//   * the rows from t-lags+1 to t+horizon are not exactly 5 minutes apart (a hole in the time grid).
//
// A rolling average of the same lags is NOT added: for a linear model it is a weighted sum of the
// lags it already has, so it carries no new information (see docs/linear_regression_baseline.md).
#pragma once

#include <vector>

#include "traffic_data.h"

namespace baseline {

struct FeatureConfig {
    int lags = 6;              // how many past readings are used (6 = the last 30 minutes)
    int horizonSteps = 3;      // how far ahead the target lies (3 steps = 15 minutes)
};

struct Sample {
    size_t originRow;          // row t at which the forecast is made
    size_t targetRow;          // row t + horizon, whose speed is the target
    std::vector<double> x;     // x[k] = speed(t - k)
    double y;                  // speed(t + horizon)
    double lastObserved;       // speed(t); this is what the persistence baseline predicts
};

struct SampleSet {
    std::vector<Sample> samples;       // usable samples, in chronological order of originRow
    size_t candidates = 0;             // origins that fit inside the series
    size_t droppedMissing = 0;         // dropped because an input or the target was empty
    size_t droppedIrregular = 0;       // dropped because the time grid has a hole in the window
};

// Throws std::invalid_argument for lags < 1, horizon < 1 or a sensor index that does not exist.
SampleSet buildSamples(const TrafficSeries& series, size_t sensorIndex, const FeatureConfig& config);

}  // namespace baseline
