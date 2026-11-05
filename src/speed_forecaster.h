// speed_forecaster.h -- trains the per-sensor Linear Regression models ONCE and forecasts every sensor's speed
// from its latest six readings. It reuses baseline::LinearModel, baseline::buildSamples and
// baseline::fitOnTargetsBefore; no training or prediction algorithm is written again here.
//
// Forecast made at origin row t (a moment in the recorded history):
//     x[k] = speed(t - k),  k = 0 .. lags-1        (exactly rows t-5 .. t when lags = 6)
//     prediction = the sensor's speed `horizonSteps` steps (5 minutes each) after t, in mph
// A sensor can only be forecast when those rows exist, are exactly 5 minutes apart and have no empty cell. Nothing
// is imputed here: otherwise the sensor is reported "unavailable" with a reason.
//
// NO FUTURE DATA (this is checked by tests/test_speed_forecaster.cpp):
//   1. Training sees ONLY rows 0 .. trainEndRow-1: the series is cut to those rows before any sample is built, and
//      only samples whose target row is before trainEndRow are used. The scaling statistics and the weights are
//      learned from those samples alone.
//   2. A forecast at origin t reads ONLY rows <= t. Nothing after t is ever looked at.
//   3. A model trained on rows < trainEndRow is only allowed to forecast from origins t >= trainEndRow - 1, i.e.
//      from the moment all of its training data had really been observed. Earlier origins are refused, because the
//      model already "knew" readings after them.
#pragma once

#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include "baseline_pipeline.h"

namespace routing {

struct SensorForecast {
    std::string sensorId;
    bool available = false;
    double speedMph = std::numeric_limits<double>::quiet_NaN();   // predicted speed at originTime + horizon; valid only if available
    std::string reason;                                           // why it is unavailable (empty when available)
};

struct ForecastSnapshot {
    size_t originRow = 0;
    long long originTime = 0;               // seconds since 1970-01-01 of the last reading used
    long long targetTime = 0;               // originTime + horizonSteps * 300 s: the moment the speeds are predicted for
    int horizonSteps = 0;
    std::vector<SensorForecast> sensors;    // in dp::SELECTED_SENSORS order
    const SensorForecast* find(const std::string& sensorId) const;   // nullptr if the sensor is not in the snapshot
};

class SpeedForecaster {
public:
    // Trains on the chronological TRAINING period of the series (config.trainFraction / validationFraction, exactly as
    // the baseline experiment splits it). Throws std::invalid_argument for bad settings, std::runtime_error naming
    // the sensor if a model cannot be trained.
    static SpeedForecaster train(const baseline::TrafficSeries& series, const baseline::BaselineConfig& config);

    // Same, but with an explicit cutoff: only rows 0 .. trainEndRow-1 are used. 1 <= trainEndRow <= series.rows().
    static SpeedForecaster trainBefore(const baseline::TrafficSeries& series, const baseline::BaselineConfig& config, size_t trainEndRow);

    size_t trainEndRow() const { return trainEndRow_; }
    size_t earliestOriginRow() const { return trainEndRow_ - 1; }     // first origin a forecast may be made from
    const baseline::FeatureConfig& features() const { return features_; }
    const std::vector<std::string>& sensors() const { return sensors_; }
    const baseline::LinearModel& model(size_t sensorIndex) const { return models_.at(sensorIndex); }
    size_t trainSamples(size_t sensorIndex) const { return trainSamples_.at(sensorIndex); }

    // Forecast all sensors at origin row t of `series`. Reads only rows <= t.
    //   * originRow beyond the series, or before earliestOriginRow(): std::invalid_argument.
    //   * missing data around t: that sensor is returned unavailable (with a reason); the others are still forecast.
    //   * a model output that is NaN, infinite, <= 0, or below minSpeedMph (0 = no extra floor): std::runtime_error
    //     naming the sensor and the value. A bad prediction is never replaced by zero or any other number.
    ForecastSnapshot predictAt(const baseline::TrafficSeries& series, size_t originRow, double minSpeedMph = 0.0) const;

private:
    baseline::FeatureConfig features_;
    size_t trainEndRow_ = 0;
    std::vector<std::string> sensors_;
    std::vector<baseline::LinearModel> models_;       // one per sensor, same order as sensors_
    std::vector<size_t> trainSamples_;
};

}  // namespace routing
