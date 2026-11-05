# Predictive routing: Linear Regression + Dijkstra

Forecast each sensor's speed with the existing Linear Regression baseline, turn the forecasts into travel
times, and let the existing Dijkstra find the fastest route. Plain C++17, standard library only. No neural
networks and no user interface at this stage.

## 1. Pipeline

1. **Forecast** (`src/speed_forecaster.*`). Six Linear Regression models, one per sensor, are trained once on the
   training period. A forecast at origin row `t` uses the sensor's readings at rows `t-5 .. t` (the last 30 minutes) and
   predicts its speed 3 steps = 15 minutes later. The model is `baseline::LinearModel`, the samples come from
   `baseline::buildSamples`, and the choice of training samples is `baseline::fitOnTargetsBefore`, which the baseline
   experiment itself now calls. No training or prediction algorithm exists twice.
2. **Travel times** (`src/travel_time_graph.*`). For every directed pair `U -> V` in the distance table:
   `travel_time_minutes = distance_m / (predicted_speed_mph * 26.8224)` (1 mph = 1609.344 m / 60 min = 26.8224 m/min).
3. **Route** (`src/route_planner.*`). The weighted graph goes to `spl1::Graph::shortestPath`; the total cost is the ETA in
   minutes. The result carries the path, the ETA, one leg per hop (distance, speed, minutes), the forecast times and the
   sensors that had no forecast.

Linear Regression (`baseline::`) and Dijkstra (`spl1::`) do not know about each other. The only things passed between
them are a `ForecastSnapshot` and a `TravelTimeGraph`.

## 2. Modelling assumptions (limits of the ETA)

1. **Upstream speed.** Every edge leaving sensor `U` is driven at `U`'s predicted speed. The destination's own speed is
   never used.
2. **The distance table is not a road map.** `from_sensor_id,to_sensor_id,distance_m` holds the directed *shortest road
   distance between pairs of sensors*, not adjacent road segments. For the six sensors all 30 directed pairs exist, so
   the graph is complete. A route `A -> C -> B` means "drive to sensor C, then on to sensor B", with each leg priced on its
   own. It can be longer on the road than `A -> B` directly, because pair distances need not add up.
3. **One speed stands for a whole leg.** A point measurement represents a leg of 0.3 km to 7.8 km.
4. **Snapshot ETA.** All edges use speeds forecast for the same moment (origin + 15 minutes). The ETA does not model
   conditions changing during the trip.
5. **Consequence of 1 and 2 together.** A route can look faster only because it changes which sensor's speed prices a
   stretch. On the committed 2-day sample, about a quarter of the test-period routes use an intermediate sensor, and many
   of them save more than 10% against the direct edge. Treat multi-sensor routes as model output, not as proven road routes.

## 3. No future data (verified by tests)

1. Training sees only rows `0 .. trainEndRow-1`. The series is cut to those rows before any sample is built, and only
   samples whose target row is before `trainEndRow` are used. Scaling statistics and weights come from them alone.
2. A forecast at origin `t` reads only rows `<= t`.
3. A model trained on rows `< trainEndRow` may only forecast from origins `t >= trainEndRow - 1`, the first moment at
   which all of its training data had really been observed. Earlier origins are refused with `std::invalid_argument`.
   This is stricter than the baseline's validation split, whose first two samples per sensor have origins just before the
   cutoff; those two are refused here.
4. The models are trained once (`SpeedForecaster::train`). They are not retrained at each origin.

## 4. Errors and missing data

| Situation | Behaviour |
|---|---|
| Window has an empty cell, a reading <= 0, a non-finite reading, or a gap in the 5-minute grid | The sensor is *unavailable* (with a reason). Nothing is imputed or carried forward at forecast time. |
| Unavailable sensor is an intermediate stop | Its outgoing edges are omitted and it is listed in `RouteResult::unavailableSensors`. |
| Source sensor unavailable | `std::runtime_error` naming the sensor (also when source == destination). |
| Destination unavailable | Fine: its own speed is not used. |
| Model output NaN, infinite or <= 0 | `std::runtime_error` naming the sensor and value. Never replaced by zero or another number. |
| `RouteConfig::minSpeedMph` > 0 and a forecast is below it | Same rejection, mentioning the floor. The floor is off (0) by default. |
| Distance file with a wrong header, unknown sensor, self-pair, distance not finite and > 0, or a repeated directed pair | `std::runtime_error` with the line number. The old `from,to,cost` layout is refused. |
| Unknown source or destination sensor | `std::invalid_argument` naming it. |
| No route with the available edges | `reachable = false`, empty path, `etaMinutes = +infinity` (never 0). |
| Origin outside the series or before `trainEndRow - 1` | `std::invalid_argument`. |
| Too few training samples | `std::runtime_error` naming the sensor (from the baseline). |

## 5. Build and run the tests (Windows PowerShell, MinGW g++ 9 or newer; from the repository root)

The tests read `data/traffic_data.csv` and `data/sensor_distances.csv`, so run them from the repository root.

```powershell
mkdir build -Force

$base  = "src/traffic_data.cpp","src/features.cpp","src/metrics.cpp","src/linear_regression.cpp","src/baseline_pipeline.cpp"
$route = "src/graph.cpp","src/speed_forecaster.cpp","src/travel_time_graph.cpp","src/route_planner.cpp"

g++ -std=c++17 -O2 -Wall -Wextra -o build/test_travel_time.exe        tests/test_travel_time.cpp        $route $base
g++ -std=c++17 -O2 -Wall -Wextra -o build/test_speed_forecaster.exe   tests/test_speed_forecaster.cpp   $route $base
g++ -std=c++17 -O2 -Wall -Wextra -o build/test_route_integration.exe  tests/test_route_integration.cpp  $route $base
.\build\test_travel_time.exe
.\build\test_speed_forecaster.exe
.\build\test_route_integration.exe
```

The existing suites are unchanged (commands in `docs/linear_regression_baseline.md`; Dijkstra:
`g++ -std=c++17 -O2 -Wall -Wextra -o build/test_graph tests/test_graph.cpp src/graph.cpp`). The two committed baseline
reports are still byte-identical to a fresh run.

## 6. Not done yet

No command-line tool or UI, no time-dependent ETA, no retraining at every origin, and no accuracy study on the full
dataset (the committed data is the 2-day sample).
