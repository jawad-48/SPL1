#include "features.h"

namespace baseline {

SampleSet buildSamples(const TrafficSeries& series, size_t sensorIndex, const FeatureConfig& config) {
    if (config.lags < 1) throw std::invalid_argument("lags must be at least 1");
    if (config.horizonSteps < 1) throw std::invalid_argument("horizon must be at least 1 step");
    if (sensorIndex >= series.sensors.size()) throw std::invalid_argument("sensor index out of range");

    const size_t lags = static_cast<size_t>(config.lags);
    const size_t horizon = static_cast<size_t>(config.horizonSteps);
    const size_t n = series.rows();
    const std::vector<double>& speed = series.speeds[sensorIndex];

    SampleSet out;
    // The first origin needs `lags-1` earlier rows; the last origin needs `horizon` later rows (for the target only).
    for (size_t t = lags - 1; t + horizon < n; ++t) {
        ++out.candidates;
        const size_t first = t + 1 - lags;

        bool regular = true;                                   // rows first..t+horizon exactly 5 minutes apart?
        for (size_t i = first + 1; i <= t + horizon; ++i)
            if (series.times[i] - series.times[i - 1] != dp::STEP_SECONDS) { regular = false; break; }
        if (!regular) { ++out.droppedIrregular; continue; }

        Sample sample;
        sample.originRow = t;
        sample.targetRow = t + horizon;
        sample.y = speed[t + horizon];
        bool complete = !std::isnan(sample.y);
        for (size_t k = 0; k < lags && complete; ++k) {
            const double v = speed[t - k];                     // k = 0 is row t itself, never a later row
            if (std::isnan(v)) complete = false;
            else sample.x.push_back(v);
        }
        if (!complete) { ++out.droppedMissing; continue; }
        sample.lastObserved = sample.x[0];
        out.samples.push_back(sample);
    }
    return out;
}

}  // namespace baseline
