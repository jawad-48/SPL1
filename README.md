# SPL1


SPL-1 project development in progress.

## Linear Regression baseline

Forecasts the speed of the six selected sensors 15 or 30 minutes ahead with Linear Regression (batch gradient
descent) and compares it with a persistence baseline. See `docs/linear_regression_baseline.md` for the method,
the exact commands, the results and the limitations, and `docs/baseline_report_15min.txt` /
`docs/baseline_report_30min.txt` for the evaluation reports.

```
g++ -std=c++17 -O2 -Wall -Wextra -o build/linear_baseline src/baseline_main.cpp src/traffic_data.cpp src/features.cpp src/metrics.cpp src/linear_regression.cpp src/baseline_pipeline.cpp src/baseline_report.cpp
build/linear_baseline
```
