# Linear Regression baseline

Short-term traffic-speed forecasting for the six selected METR-LA sensors, using Linear Regression
trained by Batch Gradient Descent, compared against a simple "persistence" baseline.
Plain C++17, standard library only (as in the approved proposal).

## 1. What it predicts

| Item | Choice |
|---|---|
| Sensors | `716328, 716331, 716337, 717446, 717456, 773062` (the list `dp::SELECTED_SENSORS` in `tools/data_common.h`) |
| Data | `data/traffic_data.csv`, cleaned by `tools/prepare_data.cpp` (speeds in mph, one row per 5 minutes) |
| Target | the speed of the **same sensor** `h` steps after the forecast time `t` |
| Horizon | default `h = 3` steps = **15 minutes**; `h = 6` (30 minutes) is also reported. The proposal asks for 15-30 minutes |
| Features | the last 6 readings of that sensor: `speed(t), speed(t-1), ..., speed(t-5)` (the last 30 minutes) |
| Model | one Linear Regression per sensor, trained by batch gradient descent |
| Comparison | persistence: predict that the speed stays at `speed(t)` |
| Metrics | MAE and RMSE in mph (MAPE is left out because it divides by the speed) |

Not included in this baseline, on purpose:

* **Rolling averages.** The average of the same 6 lags is a weighted sum of them, so for a *linear* model it adds no information.
* **Time-of-day / day-of-week.** They are part of the planned feature work, but two days of data cannot teach a model a daily or weekly pattern.
* **Readings of other sensors.** The project plan does not call for them at this stage, and the graph/Dijkstra work is a separate milestone.

## 2. How leakage is prevented

* Every feature is `speed(t - k)` with `k >= 0`. The target is the only value from the future (`src/features.cpp`).
* The timeline is split chronologically (default 70% / 15% / 15% of the rows). A sample belongs to the part that contains its **target** row, so the training samples use no reading from the validation or test period, not even as an input.
* The scaling (mean and standard deviation of inputs and target) and the weights are learned from the training samples only. Validation and test samples are transformed with those fixed numbers.
* The validation part is reported but **nothing was tuned on it**, so the test part has not influenced any choice either.
* Tests rewrite every validation and test value and require the trained model to stay bit-for-bit identical (`tests/test_baseline_model.cpp`).

## 3. Missing values

The cleaned file leaves a cell **empty** when the cleaning policy could not fill it. A forecast sample is dropped (and counted in the report) if any of its inputs or its target is empty, or if the 5-minute time grid has a hole inside its window.

Known weakness: cells the cleaning policy *filled* (last reading carried forward for up to 30 minutes) cannot be told apart from real readings in the cleaned file, so they are used as inputs and as targets. A carried-forward target is easy for persistence to predict. In the committed 2-day file the only filled cells are 12 values at 2012-03-01 06:15 and 06:20, which lie in the training period.

## 4. Commands (from the repository root)

PowerShell on Windows (needs `g++` 9 or newer):

```powershell
mkdir build -Force

# build and run the tests
g++ -std=c++17 -O2 -Wall -Wextra -o build/test_baseline_data tests/test_baseline_data.cpp src/traffic_data.cpp src/features.cpp
g++ -std=c++17 -O2 -Wall -Wextra -o build/test_baseline_model tests/test_baseline_model.cpp src/traffic_data.cpp src/features.cpp src/metrics.cpp src/linear_regression.cpp src/baseline_pipeline.cpp
g++ -std=c++17 -O2 -Wall -Wextra -o build/test_baseline_end_to_end tests/test_baseline_end_to_end.cpp src/traffic_data.cpp src/features.cpp src/metrics.cpp src/linear_regression.cpp src/baseline_pipeline.cpp src/baseline_report.cpp
.\build\test_baseline_data.exe
.\build\test_baseline_model.exe
.\build\test_baseline_end_to_end.exe

# build and run the baseline
g++ -std=c++17 -O2 -Wall -Wextra -o build/linear_baseline src/baseline_main.cpp src/traffic_data.cpp src/features.cpp src/metrics.cpp src/linear_regression.cpp src/baseline_pipeline.cpp src/baseline_report.cpp
.\build\linear_baseline.exe                                          # 15 minutes, report printed
.\build\linear_baseline.exe --horizon-steps 6                        # 30 minutes
.\build\linear_baseline.exe --report docs/baseline_report_15min.txt  # also save the report
.\build\linear_baseline.exe --horizon-steps 6 --report docs/baseline_report_30min.txt
```

On Linux/macOS use `./build/linear_baseline` and so on. Other options: `--data PATH`, `--lags N`, `--train-frac X`, `--val-frac X`.

The existing data tests still apply: `tools/test_cleaning.cpp` and `tools/validate_data.cpp` (see `data/README.md`). The new tests only read `data/`; none of them writes into `data/` or `docs/`.

Everything is deterministic: weights start at zero and no random numbers are used in the baseline, so the same data always gives the same report. The test `test_baseline_end_to_end` fails if `docs/baseline_report_*.txt` no longer matches what the code produces, so regenerate those two files whenever `data/traffic_data.csv` or the model changes.

## 5. Results on the committed data (2 days)

`data/traffic_data.csv` currently holds the **2-day sample** (576 rows, 2012-03-01 06:00 to 2012-03-03 05:55), because the full dataset is kept only on the author's machine. The split is:

| Part | Rows | Period |
|---|---|---|
| train | [0, 403) | 2012-03-01 06:00 to 2012-03-02 15:30 |
| validation | [403, 489) | 2012-03-02 15:35 to 22:40 |
| test | [489, 576) | 2012-03-02 22:45 to 2012-03-03 05:55 (7.25 hours) |

Per sensor: 568 usable samples at 15 minutes (395 train, 86 validation, 87 test); 565 at 30 minutes (392, 86, 87). Nothing was dropped.

Pooled over the six sensors (mph, held-out; full per-sensor tables are in the two report files):

| Horizon | Part | n | Linear Regression MAE / RMSE | Persistence MAE / RMSE | RMSE change |
|---|---|---|---|---|---|
| 15 min | validation | 516 | 3.6337 / 5.7806 | 3.9201 / 6.1348 | -5.8% |
| 15 min | test | 522 | 4.8605 / 6.9282 | 5.1416 / 7.5402 | -8.1% |
| 30 min | validation | 516 | 3.9945 / 6.3710 | 4.0980 / 6.5112 | -2.2% |
| 30 min | test | 522 | 5.6625 / 8.0408 | 5.9545 / 8.8893 | -9.5% |

("RMSE change" is Linear Regression's RMSE relative to persistence; negative means the smaller error.)

What these numbers do and do not show:

* Pooled over the six sensors, Linear Regression had the lower MAE and RMSE than persistence in all four rows.
* It was **not better everywhere**. At 15 minutes on validation, sensor `716328` had higher error than persistence (RMSE +6.0%); on test, sensor `717456` had a marginally higher MAE (5.4156 vs 5.3823) although a lower RMSE. At 30 minutes on validation, `716328` (+12.7%) and `773062` (+3.1%) had higher RMSE.
* Each sensor's test result rests on 87 consecutive 5-minute samples from one 7.25-hour stretch (22:45 to 05:55 on the file's clock). Consecutive samples are strongly correlated, so they carry far less independent information than 87 separate observations, and no significance test was done. **These numbers show that the pipeline runs and is internally consistent; they do not establish how accurate the model is.**
* At 15 minutes the per-sensor test errors range from about 3.1 to 6.4 mph (MAE) and 4.3 to 8.6 mph (RMSE), which is large for a 15-minute forecast. This is a plain baseline that has seen only two days of speeds.

## 6. How the results were checked

* Unit tests: 36 (data and features), 35 (model, split, leakage) and 15 (end to end), all passing at the time of writing.
* An independent re-implementation (NumPy exact least squares on the raw CSV, with its own indexing) gave the same test MAE and RMSE as the C++ program to four decimals, for every sensor and pooled, for both Linear Regression and persistence.
* Gradient descent ended within about 1e-10 (relative) of the exact least-squares training error for all six sensors.

## 7. Limitations

* **Only 2 days of data are committed.** Re-run the preparation with the full file (`data/README.md`), then regenerate the two reports; with a full file the tool warns if fewer than 7 days are present.
* One model per sensor, using only that sensor's own recent speeds.
* Neighbouring lags are strongly correlated, so single coefficients are unreliable even though predictions are fine.
* METR-LA is Los Angeles traffic from 2012, not Dhaka.
* No cross-validation or hyperparameter search was done (the number of lags and the horizon are fixed settings).

## 8. What the later Dijkstra milestone will need (not implemented here)

* `data/sensor_distances.csv`: 30 verified directed road distances in meters between the six sensors. It is a table of shortest road distances between sensor pairs, **not** a list of adjacent road segments (see `data/README.md`, section 4).
* A predicted speed per sensor for a chosen forecast time. The baseline currently only reports predictions for its held-out rows (`BaselineResult` in `src/baseline_pipeline.h`); a function that predicts for any requested row will have to be added at that milestone.
* A decision on which speed to use for an edge between two sensors (for example the destination sensor's, or the mean of the two), to turn distance into a travel time. That choice is not made here.
