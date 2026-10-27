#include "metrics.h"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace baseline {

Metrics computeMetrics(const std::vector<double>& predicted, const std::vector<double>& actual) {
    if (predicted.size() != actual.size()) throw std::invalid_argument("predicted and actual must have the same length");
    Metrics m;
    m.n = predicted.size();
    if (m.n == 0) {
        m.mae = m.rmse = std::numeric_limits<double>::quiet_NaN();
        return m;
    }
    double sumAbs = 0, sumSq = 0;
    for (size_t i = 0; i < m.n; ++i) {
        const double error = predicted[i] - actual[i];
        sumAbs += std::fabs(error);
        sumSq += error * error;
    }
    m.mae = sumAbs / static_cast<double>(m.n);
    m.rmse = std::sqrt(sumSq / static_cast<double>(m.n));
    return m;
}

}  // namespace baseline
