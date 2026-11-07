// metrics.h -- regression error measures.
//
//   MAE  = (1/n) * sum |predicted - actual|             average size of the miss, in mph
//   RMSE = sqrt( (1/n) * sum (predicted - actual)^2 )   like MAE but large misses count more, in mph
//
// MAPE (percentage error) is deliberately NOT provided: it divides by the actual speed, and a speed
// near zero would make it explode. MAE and RMSE are what the approved proposal asks for.
#pragma once

#include <vector>

namespace baseline {

struct Metrics {
    size_t n = 0;          // how many predictions were scored
    double mae = 0;        // NaN when n == 0
    double rmse = 0;       // NaN when n == 0
};

// Throws std::invalid_argument if the two vectors have different lengths.
// With no predictions at all (n == 0) both errors are NaN: "nothing was measured" is not "zero error".
Metrics computeMetrics(const std::vector<double>& predicted, const std::vector<double>& actual);

}  // namespace baseline
