# Short-Term Traffic Congestion Prediction and Alternate Route Recommendation System

**SPL-1 Project | Institute of Information Technology, University of Dhaka**

## Project Overview

This project aims to predict short-term traffic congestion and recommend faster alternative routes using historical traffic-speed data.

The system forecasts traffic speed 15 minutes ahead using Linear Regression, converts predicted speeds into estimated travel-time costs, and applies Dijkstra's shortest-path algorithm to identify the route with the minimum estimated travel time.

The current implementation provides a C++17 command-line application with interactive origin and destination selection.

## Objectives

- Predict short-term traffic speed for selected road sensors.
- Estimate travel time using predicted traffic speeds.
- Find the minimum estimated travel-time route using Dijkstra's algorithm.
- Compare Linear Regression predictions against a persistence baseline.
- Build an integrated and testable C++ application.

## System Architecture

The system follows this workflow:

1. **Historical Traffic Data:** Load traffic-speed observations for six selected sensors.
2. **Data Preprocessing:** Clean invalid readings and handle missing values.
3. **Speed Prediction:** Train six Linear Regression models, one for each sensor.
4. **Travel-Time Estimation:** Convert predicted speeds and directed sensor distances into travel-time costs.
5. **Route Optimization:** Apply Dijkstra's algorithm to find the minimum-cost route.
6. **Route Recommendation:** Display the recommended sensor route, individual leg costs, and estimated total travel time.

## Dataset

The project uses six selected traffic sensors from the METR-LA dataset.

| Alias | Sensor ID |
|---|---:|
| r1 | 716328 |
| r2 | 716331 |
| r3 | 716337 |
| r4 | 717446 |
| r5 | 717456 |
| r6 | 773062 |

- Observation interval: 5 minutes
- Speed unit: miles per hour (mph)
- Full local dataset: 34,272 observations
- Committed sample dataset: 576 rows covering two days

**Note:** The full dataset is maintained locally and is not included in the repository.

## Data Preprocessing

The preprocessing pipeline includes:

- Treating zero, negative, and NaN speed readings as missing values.
- Forward-filling missing readings using the previous valid observation, up to six consecutive observations.
- Preparing chronological traffic-speed data for model training and evaluation.

## Linear Regression Baseline

The current forecasting implementation uses Linear Regression trained with batch gradient descent.

### Model configuration

- Six independent models, one per sensor.
- Input features: the latest six speed readings (30 minutes of historical observations).
- Forecast horizon: 15 minutes; a 30-minute horizon is also supported by the baseline evaluation.
- Chronological dataset split: 70% training, 15% validation, and 15% testing.
- Feature standardization is performed using training data.

### Baseline comparison

The Linear Regression model is compared against a persistence baseline, which predicts that the future speed remains equal to the latest observed speed.

Performance is evaluated using:

- Mean Absolute Error (MAE)
- Root Mean Squared Error (RMSE)

See the following reports for the actual evaluation results:

- `docs/baseline_report_15min.txt`
- `docs/baseline_report_30min.txt`
- `docs/linear_regression_baseline.md`

## Predictive Route Recommendation

The routing component combines predicted traffic speeds with directed sensor-to-sensor distances.

For each directed edge, estimated travel time is calculated as:

**Travel time (minutes) = Distance (metres) / (Predicted speed (mph) × 26.8224)**

The resulting travel-time values are used as edge weights in a directed graph.

Dijkstra's algorithm then identifies the route with the minimum total estimated travel time between the selected origin and destination sensors.

The routing application displays:

- Predicted speeds for all six sensors.
- Recommended route.
- Individual route-leg distances and estimated travel times.
- Total estimated travel time.

## Build and Run

The project uses C++17 and the standard library.

The standalone Linear Regression baseline can be built using:

```bash
mkdir -p build

g++ -std=c++17 -O2 -Wall -Wextra \
-o build/linear_baseline \
src/baseline_main.cpp \
src/traffic_data.cpp \
src/features.cpp \
src/metrics.cpp \
src/linear_regression.cpp \
src/baseline_pipeline.cpp \
src/baseline_report.cpp

./build/linear_baseline
```

For the predictive-routing CLI, refer to the corresponding build instructions and executable configuration in the project source and documentation.

## Testing

The project includes automated tests covering:

- Data preparation and validation
- Linear Regression and forecasting
- Graph algorithms
- Travel-time calculation
- Route planning
- Command-line interaction
- End-to-end routing integration

The midterm test record reports 426 passing tests across eight test suites. These tests assess software correctness, not predictive accuracy.

## Current Implementation Status

### Completed

- Traffic-data preparation and validation
- Data cleaning and preprocessing
- Linear Regression baseline
- Model evaluation against persistence
- Dijkstra's shortest-path implementation
- Predictive travel-time graph
- Route planning
- Interactive command-line interface
- Origin and destination selection
- Automated software testing

### In Progress

- Evaluation using the full 34,272-observation dataset

### Planned

- Feedforward Neural Network implementation
- Comparison between Linear Regression and Neural Network models
- Graphical user interface
- Full system integration and refinement

## Current Limitations

- Routing operates on selected sensor waypoints rather than a complete street-level road network.
- Travel-time estimates use a forecast snapshot rather than continuously changing traffic conditions throughout a journey.
- The full-dataset evaluation is not yet complete.
- The current application uses a command-line interface.

## Future Work

The remaining development work focuses on implementing and evaluating a Neural Network model, developing a graphical interface, improving system integration, and conducting further testing and refinement.

---

**Project:** SPL-1  
**Student:** Muhtasim Jawad  
**Roll:** 1706  
**Institute:** Institute of Information Technology, University of Dhaka
