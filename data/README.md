# Data for the SPL-1 project

Short-term traffic speed prediction and route recommendation on the METR-LA benchmark.

## 1. Files in this folder

| Path | Tracked in Git | What it is |
|---|---|---|
| `full/metr-la.h5` | **No** (ignored) | Original full METR-LA speed table (HDF5). Never edited. |
| `sample/speeds_sample.csv` | Yes | Original 2-day excerpt: 576 timestamps x 207 sensors. Never edited. |
| `sample/graph_sensor_ids.txt` | Yes | The 207 sensor IDs (comma separated). Never edited. |
| `sample/graph_sensor_locations.csv` | Yes | `index,sensor_id,latitude,longitude` for the 207 sensors. Never edited. |
| `sample/distances_la_2012.csv` | Yes | `from,to,cost` road distances. Never edited. |
| `traffic_data.csv` | Yes | **Generated.** Timestamp + the six selected sensors, cleaned. |
| `sensor_distances.csv` | Yes | **Generated.** The 30 verified directed road distances between the six sensors. |
| `sensor_locations.csv` | Yes | **Generated.** Latitude/longitude of the six sensors. |
| `data_quality_report.txt` | Yes | **Generated.** Counts before/after cleaning and all verification results. |

The generated files are produced by `tools/prepare_data.cpp` and checked by `tools/validate_data.cpp` (plain C++17, standard library only).

**Important:** the generated files in the repository were first produced from the 2-day sample.
Prediction needs the full dataset, so re-run the preparation with `--source h5` (section 8) and
commit the result. The "Source speed file" line of `data_quality_report.txt` names the source used.

## 2. Source

* METR-LA: loop-detector speeds on Los Angeles County highways, 207 sensors, one reading every
  5 minutes, starting 2012-03-01 (Li et al., "Diffusion Convolutional Recurrent Neural Network",
  ICLR 2018). The file names of the ID, location and distance files match the DCRNN project's
  `data/sensor_graph` folder.
* Where and when this copy was downloaded: *(fill in: URL and date)*

## 3. Dimensions, units and sampling

* Sample file: 576 timestamps, 2012-03-01 06:00:00 to 2012-03-03 05:55:00 (Thursday 06:00 to Saturday 05:55),
  207 sensors. That is only two days, so it contains no complete week and is too short for
  time-of-week features or a chronological train/test split.
* Sampling interval: 300 seconds. Verified: all 575 intervals in the sample are exactly 300 s.
* Speed unit: **miles per hour**. This is the documented METR-LA convention. The numbers are
  consistent with it (maximum 70.0, median about 62) but the values alone cannot prove the unit.
* Distance unit: **meters**. Evidence from the data: for pairs of the 207 sensors more than 200 m apart, road distance divided by the
  straight-line distance computed from the coordinates has median 1.58, and no pair has a road distance shorter than the straight line.

## 4. What the distance file really is (verified)

* `distances_la_2012.csv` covers 4,106 sensors (the whole LA network), not only the 207 in METR-LA, and has 295,374 rows.
  Rows starting with `1201054` come first in the file, but that sensor is one of the roughly 3,900 sensors outside the 207,
  so it is not evidence of a different network. All 207 METR-LA IDs appear in the file, and the ID list, location file and speed header agree exactly (same IDs, same order).
* Among the 207 sensors it holds 11,546 directed pairs. It is **directed** (4,167 pairs differ by more than 1 m between
  the two directions) and **sparse** (each sensor has between 7 and 111 of the 206 possible partners).
* It is a table of shortest road distances between sensor pairs, **not a list of adjacent road segments**: the triangle
  inequality holds up to 45.8 m of noise. Consequence for the later Dijkstra stage: with distance as the edge weight the
  direct edge almost always wins, so alternative routes will only appear once the weight is predicted travel time.

## 5. The six selected sensors

`716328, 716331, 716337, 717446, 717456, 773062` (downtown Los Angeles, about 34.05-34.08 N, 118.23-118.29 W).

Selection rules, applied by `tools/select_sensors.cpp` (run it to reproduce the shortlist):

1. Every ID exists in the speed file, the location file and the ID list.
2. At most 1% of the sensor's readings are missing (zero).
3. All 30 ordered pairs among the six have a real record in the distance file, so every edge is genuine.
4. Each of those 30 distances is at most 8,000 m.
5. The two farthest sensors are at least 3,000 m apart, so the six are not just one interchange.

On the sample, 167 of 207 sensors pass rules 1-2 and 80,592 six-sensor groups pass rules 3-5. Ranking: lowest worst-sensor
missing rate, then largest spread, then smallest IDs. The result has 30 edges between 336.5 m and 7,819.8 m, and is strongly connected.

Speed variability was deliberately **not** a selection rule, because choosing sensors for how they behave over the whole period
would leak information about the later test period.

Limit of this choice: rule 2 was evaluated on two days. After the full-data run, check the `missing%` column in
`data_quality_report.txt`; if any of the six is much higher than expected, re-run `build\select_sensors --source h5`
and decide before changing `SELECTED_SENSORS` in `tools/data_common.h`.

## 6. Cleaning decisions

| Finding | Decision | Why |
|---|---|---|
| Zero speed means "no measurement" in METR-LA (4.24% of the sample's cells) | Treat zero, NaN, non-numeric and negative values as missing | A real speed of 0 mph on a highway sensor is not a measurement of traffic |
| 2 consecutive timestamps (2012-03-01 06:15, 06:20) where all 207 sensors are 0 | Keep both rows; treat as a network-wide outage | Dropping them would break the regular 5-minute grid that lag features need |
| Short gaps | Carry the last real reading forward for at most 6 steps (30 min) | Past information only; each row is decided using rows up to itself, never ahead |
| Gaps longer than 6 steps, or at the very start | Leave the cell **empty** | Inventing them would fabricate data; the C++ loader must treat an empty field as missing |
| Values above 85 mph | Report only, never delete | None exist in the sample, and valid readings must not be deleted by assumption |
| Timestamps unordered or duplicated | Stop with an error | None exist in the sample; a policy would be needed first |

Other formatting choices: speeds are written with exactly 4 decimals (rounded on the value itself, so the text does not
depend on the platform's `printf`); self-distance rows (0.0) are left out; no rows are dropped. Imputed values are real past readings, not new measurements, so a later evaluation should
not score predictions against an imputed target.

## 7. How the HDF5 file is read without a library

`metr-la.h5` is a binary HDF5 container. `tools/hdf5_min.h` is a small reader written for this project. It does **not**
implement HDF5 in general, only the plain shape pandas writes: superblock version 0 or 1, groups stored with a symbol
table, version-1 object headers, and contiguous **uncompressed** arrays of integers, floats or fixed-length strings.
It expects the group `df` with `axis0`/`block0_items` (sensor IDs), `axis1` (timestamps) and `block0_values` (speeds).

If your file differs (for example compressed, chunked, or written with a newer HDF5 layout), the program stops with a
message naming exactly what it found; it never guesses. Before processing, you can ask it what it sees:

```powershell
.\build\prepare_data.exe --h5-info
```

## 8. Build and reproduce (PowerShell, from the repository root)

Needs only `g++` (MinGW-w64 / MSYS2, version 9 or newer, or any Linux/macOS g++).

```powershell
mkdir build -Force
foreach ($t in "prepare_data","select_sensors","validate_data","test_cleaning") {
    g++ -std=c++17 -O2 -Wall -Wextra -o "build/$t" "tools/$t.cpp"
}

.\build\test_cleaning.exe                       # unit tests for the cleaning policy

# 2-day sample (no HDF5 needed)
.\build\prepare_data.exe --source csv
.\build\validate_data.exe --source csv

# full dataset
.\build\prepare_data.exe --h5-info              # check the file layout first
.\build\prepare_data.exe --source h5
.\build\validate_data.exe --source h5
```

On Linux/macOS use `./build/prepare_data` and so on, without `.exe`.

`tools/` holds one-time preprocessing programs. Your main program in `src/` only has to read the generated CSV files;
remember that an empty field in `traffic_data.csv` means "missing".
