# Fastest-route command-line application

`routing` forecasts the speeds of the six sensors, prices every sensor-to-sensor leg in minutes and prints the
fastest route with its ETA. It only connects existing code: `baseline::loadTrafficSeries`,
`baseline::chronologicalBoundaries`, `routing::SpeedForecaster`, `routing::loadSensorDistances` and
`routing::planRouteFromSnapshot`. Nothing about forecasting, Dijkstra or travel times is written again in the CLI.

Files: `src/routing_main.cpp` (only `main`), `src/routing_cli.h/.cpp` (the logic, callable from tests),
`tests/test_routing_cli.cpp`.

## 1. Use

Run from the repository root.

```
routing [--from SENSOR] [--to SENSOR] [--row N | --time "YYYY-MM-DD HH:MM:SS"] [--traffic PATH] [--distances PATH]
```

If `--from` and/or `--to` are left out, the program asks for them (section 2).

| Option | Meaning |
|---|---|
| `--from`, `--to` | where you are and where you want to go: a sensor ID (716328, 716331, 716337, 717446, 717456, 773062) or a route name r1 to r6. If one or both are missing the program asks. |
| `--row N` | forecast from dataset row N (0 = first data row) |
| `--time "..."` | forecast from the row with exactly this timestamp (not rounded to a nearby row) |
| neither | the latest row of the file is used, and the output says so |
| `--traffic PATH` | cleaned traffic file, default `data/full/processed/traffic_data.csv` |
| `--distances PATH` | distance file, default `data/full/processed/sensor_distances.csv` |
| `--help` | usage text |

Paths are relative to where you run the program; no absolute path is built in. `--row` and `--time` cannot be combined.

```powershell
.\build\routing.exe
.\build\routing.exe --from r1 --to r6
.\build\routing.exe --from 716328 --to 773062
.\build\routing.exe --from 716328 --to 773062 --row 30000
.\build\routing.exe --from 716331 --to 716337 --traffic data/traffic_data.csv --distances data/sensor_distances.csv --row 489
```

**The origin must be after the training period.** Training uses the first 70% of the rows. A forecast from an earlier
row is refused, because the models have already seen the data after it. On the committed 2-day sample the first
allowed row is 402; on a file with 34,272 rows it is 23,989. The error message names the earliest allowed row and
timestamp. The program never picks another time for you.

Training happens on every run and is the slow part: about 0.05 s for the 576-row sample and about 12 s for a
synthetic 34,272-row file (Linux, `-O2`). Build with `-O2`. A "Training ..." line is printed first.

## 2. Route names r1 to r6 and the questions

The six sensors have short names, in the order of the sensor list. They can be used anywhere a sensor ID is accepted
(`--from r2`), and they are shown next to the IDs in the output.

| Name | Sensor | Name | Sensor |
|---|---|---|---|
| r1 | 716328 | r4 | 717446 |
| r2 | 716331 | r5 | 717456 |
| r3 | 716337 | r6 | 773062 |

The order is the ID order used everywhere else in the project, not a position along the road.

Run `routing` without `--from` and/or `--to` and it asks:

```
Where are you now, and where do you want to go?

Routes (sensors):
  r1  716328
  ...
  r6  773062

Where are you now?        (r1-r6 or sensor ID): r2
Where do you want to go?  (r1-r6 or sensor ID): r1
```

Answers may be r1 to r6 (r or R), or a sensor ID; surrounding spaces are ignored. A wrong answer is explained and asked
again, and after 3 wrong answers to the same question the program stops (exit 2). If the input ends before the answers
(for example when the input is piped from a file), it stops with a message that points to `--from` and `--to`.
The questions are asked only after every other argument and both file names have been checked, so a mistake in
`--row`, `--time` or a path never wastes your answers. If both `--from` and `--to` are given, nothing is asked.

## 3. Example (committed sample, row 489)

```
SPL-1 fastest route: Linear Regression forecast + Dijkstra
Traffic data   : data/traffic_data.csv (576 rows, 2012-03-01 06:00:00 to 2012-03-03 05:55:00)
Distance data  : data/sensor_distances.csv (30 directed sensor pairs)
Training       : rows 0 to 402 (2012-03-01 06:00:00 to 2012-03-02 15:30:00), one model per sensor, 6 readings (30 min) -> speed 15 min ahead
Forecast origin: row 489 (2012-03-02 22:45:00) [selected with --row]
Route names    : r1 = 716328, r2 = 716331, r3 = 716337, r4 = 717446, r5 = 717456, r6 = 773062
From / To      : 716331 -> 716337  (r2 -> r3)

Training the six Linear Regression models ...
Forecast speeds for 2012-03-02 23:00:00 (15 min after the origin), from the readings up to 2012-03-02 22:45:00:
  r1  716328   54.30 mph
  r2  716331   33.01 mph
  r3  716337   49.88 mph
  r4  717446   31.33 mph
  r5  717456   43.77 mph
  r6  773062   33.61 mph

Fastest route: 716331 -> 716328 -> 716337
  In route names: r2 -> r1 -> r3
  Legs (speed = forecast of the sensor each leg starts from):
  leg  from    to         distance          speed     travel time
  1    716331  716328      874.5 m      33.01 mph       0.988 min
  2    716328  716337     5179.6 m      54.30 mph       3.557 min
Total ETA: 4.544 minutes
(Upstream-speed assumption and other limits: docs/predictive_routing.md.)
```

## 4. Exit codes and messages

| Code | Meaning | Examples |
|---|---|---|
| 0 | route calculated | |
| 1 | file, dataset or origin-time problem | file missing or invalid, wrong header, `--row` beyond the file, timestamp not in the file, origin inside the training period, too few rows to train, a forecast rejected as not positive |
| 2 | command-line mistake | unknown or repeated option, missing value, sensor ID not one of the six, bad `--row` or `--time` text, both `--row` and `--time` |
| 3 | valid request, no route can be calculated | the source sensor has no forecast (an empty or missing reading in its last 6 readings), or the destination cannot be reached with the available distances and forecasts |

Errors and the explanation for exit 3 go to standard error; the report goes to standard output. Nothing is ever
replaced silently: no other sensor, row, timestamp or speed is substituted, and an unreachable destination never
shows an ETA of 0.

The sensor IDs are checked before any file is read, and the origin row is checked before training starts, so a wrong
argument fails immediately.

## 5. Build and test (Windows PowerShell, MinGW g++; from the repository root)

```powershell
mkdir build -Force
$base  = "src/traffic_data.cpp","src/features.cpp","src/metrics.cpp","src/linear_regression.cpp","src/baseline_pipeline.cpp"
$route = "src/graph.cpp","src/speed_forecaster.cpp","src/travel_time_graph.cpp","src/route_planner.cpp"

g++ -std=c++17 -O2 -Wall -Wextra -o build/routing.exe src/routing_main.cpp src/routing_cli.cpp $route $base
g++ -std=c++17 -O2 -Wall -Wextra -o build/test_routing_cli.exe tests/test_routing_cli.cpp src/routing_cli.cpp $route $base
.\build\test_routing_cli.exe
```

`test_routing_cli` reads the committed sample (`data/traffic_data.csv`, `data/sensor_distances.csv`) and writes small
temporary files to the folder named by `TMPDIR`, `TEMP` or `TMP` (else the current folder), removing them at the end.
It does not need the full dataset.

## 6. Limits

The ETA assumptions in `docs/predictive_routing.md` apply unchanged: each leg is driven at the forecast speed of the
sensor it starts from, the distance table holds pairwise shortest road distances (not road segments), and the ETA is a
snapshot for 15 minutes after the origin. There is no GUI, no live data feed and no time-dependent simulation.
