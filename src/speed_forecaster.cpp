#include "speed_forecaster.h"

#include <cmath>
#include <stdexcept>

namespace routing {

const SensorForecast* ForecastSnapshot::find(const std::string& sensorId) const {
    for (const SensorForecast& s : sensors) if (s.sensorId == sensorId) return &s;
    return nullptr;
}

// A copy of the first `rows` rows of the series. Training is done on this copy, so a training step cannot reach a
// later row even by mistake.
static baseline::TrafficSeries firstRows(const baseline::TrafficSeries& series, size_t rows) {
    baseline::TrafficSeries cut;
    cut.source = series.source;
    cut.sensors = series.sensors;
    cut.ignoredColumns = series.ignoredColumns;
    cut.times.assign(series.times.begin(), series.times.begin() + static_cast<std::ptrdiff_t>(rows));
    cut.speeds.resize(series.speeds.size());
    for (size_t s = 0; s < series.speeds.size(); ++s)
        cut.speeds[s].assign(series.speeds[s].begin(), series.speeds[s].begin() + static_cast<std::ptrdiff_t>(rows));
    return cut;
}

SpeedForecaster SpeedForecaster::train(const baseline::TrafficSeries& series, const baseline::BaselineConfig& config) {
    const baseline::SplitBoundaries split = baseline::chronologicalBoundaries(series.rows(), config.trainFraction, config.validationFraction);
    return trainBefore(series, config, split.trainEnd);
}

SpeedForecaster SpeedForecaster::trainBefore(const baseline::TrafficSeries& series, const baseline::BaselineConfig& config, size_t trainEndRow) {
    if (trainEndRow < 1 || trainEndRow > series.rows())
        throw std::invalid_argument(dp::fmt("trainEndRow must be between 1 and the number of rows (%zu), got %zu", series.rows(), trainEndRow));
    if (series.sensors != dp::SELECTED_SENSORS)
        throw std::invalid_argument("the series must contain exactly the six selected sensors, in the approved order");

    const baseline::TrafficSeries visible = firstRows(series, trainEndRow);     // nothing at or after trainEndRow exists below this line
    SpeedForecaster f;
    f.features_ = config.features;
    f.trainEndRow_ = trainEndRow;
    f.sensors_ = series.sensors;
    for (size_t s = 0; s < visible.sensors.size(); ++s) {
        const baseline::SampleSet set = baseline::buildSamples(visible, s, config.features);
        baseline::LinearModel model;
        size_t used = 0;
        try {
            baseline::fitOnTargetsBefore(model, set, trainEndRow, config.gd, used);
        } catch (const std::invalid_argument& e) {
            throw std::runtime_error("sensor " + visible.sensors[s] + ": cannot train, " + e.what() + dp::fmt(" (%zu training samples)", used));
        }
        f.models_.push_back(model);
        f.trainSamples_.push_back(used);
    }
    return f;
}

ForecastSnapshot SpeedForecaster::predictAt(const baseline::TrafficSeries& series, size_t originRow, double minSpeedMph) const {
    if (series.sensors != sensors_)
        throw std::invalid_argument("the series must contain exactly the six selected sensors, in the approved order");
    if (originRow >= series.rows())
        throw std::invalid_argument(dp::fmt("origin row %zu is outside the series (%zu rows)", originRow, series.rows()));
    if (originRow < earliestOriginRow())
        throw std::invalid_argument(dp::fmt("origin row %zu is before row %zu: the models were trained on rows 0..%zu, so a forecast from an earlier "
                                            "origin would use data from its future", originRow, earliestOriginRow(), trainEndRow_ - 1));
    if (!(minSpeedMph >= 0.0) || !std::isfinite(minSpeedMph))
        throw std::invalid_argument("minSpeedMph must be finite and >= 0");

    const size_t lags = static_cast<size_t>(features_.lags);
    ForecastSnapshot snapshot;
    snapshot.originRow = originRow;
    snapshot.originTime = series.times[originRow];
    snapshot.horizonSteps = features_.horizonSteps;
    snapshot.targetTime = snapshot.originTime + static_cast<long long>(features_.horizonSteps) * dp::STEP_SECONDS;

    for (size_t s = 0; s < sensors_.size(); ++s) {
        SensorForecast f;
        f.sensorId = sensors_[s];

        // ---- the window: rows originRow-lags+1 .. originRow, and nothing later
        if (originRow + 1 < lags) {
            f.reason = dp::fmt("fewer than %zu readings of history", lags);
        } else {
            const size_t first = originRow + 1 - lags;
            bool regular = true;
            for (size_t i = first + 1; i <= originRow; ++i)
                if (series.times[i] - series.times[i - 1] != dp::STEP_SECONDS) { regular = false; break; }
            if (!regular) {
                f.reason = dp::fmt("the last %zu readings are not exactly 5 minutes apart", lags);
            } else {
                std::vector<double> x;                                   // x[k] = speed(t - k)
                for (size_t k = 0; k < lags; ++k) {
                    const double v = series.speeds[s][originRow - k];
                    if (!std::isfinite(v) || v <= 0) { f.reason = "a reading in the last " + std::to_string(lags) + " is missing (" + dp::formatTimestamp(series.times[originRow - k]) + ")"; break; }
                    x.push_back(v);
                }
                if (f.reason.empty()) {
                    const double p = models_[s].predict(x);
                    if (!std::isfinite(p) || p <= 0 || p < minSpeedMph)
                        throw std::runtime_error("sensor " + f.sensorId + ": rejected predicted speed " + dp::fmt("%.6g", p) +
                                                 " mph at origin " + dp::formatTimestamp(snapshot.originTime) +
                                                 (minSpeedMph > 0 && std::isfinite(p) && p > 0 ? dp::fmt(" (below the %.6g mph floor)", minSpeedMph) : std::string(" (must be finite and positive)")));
                    f.available = true;
                    f.speedMph = p;
                }
            }
        }
        snapshot.sensors.push_back(f);
    }
    return snapshot;
}

}  // namespace routing
