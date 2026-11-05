// travel_time_graph.h -- turns forecast sensor speeds and the sensor distance table into a travel-time graph.
//
// MODELLING ASSUMPTIONS (also in docs/predictive_routing.md):
//   1. Upstream-speed assumption: the edge from sensor U to sensor V is driven at U's predicted speed, so every edge
//      leaving U uses the forecast made for U, whatever V is. Only the sensor an edge starts at needs a forecast;
//      the destination's own speed is never used.
//   2. The distance table (data/.../sensor_distances.csv) holds the directed SHORTEST ROAD DISTANCE between pairs of
//      sensors, not adjacent road segments. The graph is therefore (almost) complete, and a "route" A -> C -> B means
//      "go to sensor C, then on to sensor B", with each leg priced separately. Such a route can be longer on the
//      road than A -> B directly (the pair distances need not add up) and a leg's speed is one point measurement
//      standing for the whole leg, which is 0.3 km to 7.8 km long in the current data.
//   3. Every edge uses the speeds forecast for the same moment (origin + horizon). The ETA is a snapshot, not a
//      time-dependent simulation of conditions changing during the trip.
//
// Unit conversion: travel_time_minutes = distance_m / (speed_mph * 26.8224), because 1 mph = 1609.344 m / 60 min.
#pragma once

#include <string>
#include <vector>

#include "graph.h"
#include "speed_forecaster.h"

namespace routing {

const double METERS_PER_MINUTE_PER_MPH = 26.8224;

struct DistanceRecord {
    std::string from, to;
    double distanceM = 0;
};

// Reads the distance table. The header must be exactly  from_sensor_id,to_sensor_id,distance_m  (the incompatible
// from,to,cost layout is refused). Every row must name two different sensors from dp::SELECTED_SENSORS and a finite
// distance > 0; a directed pair may appear only once. Throws std::runtime_error naming the line otherwise.
std::vector<DistanceRecord> parseSensorDistances(const std::vector<std::string>& lines, const std::string& sourceName);
std::vector<DistanceRecord> loadSensorDistances(const std::string& path);

// distance_m / (speed_mph * 26.8224). Throws std::invalid_argument unless the distance is finite and > 0, the speed is
// finite, > 0 and >= minSpeedMph, and the result is finite and > 0. Nothing is ever replaced by zero.
double travelTimeMinutes(double distanceM, double speedMph, double minSpeedMph = 0.0);

struct RouteLeg {
    std::string from, to;
    double distanceM = 0;
    double speedMph = 0;       // the upstream (from) sensor's forecast
    double minutes = 0;
};

struct UnavailableSensor {
    std::string sensorId, reason;
};

struct TravelTimeGraph {
    spl1::Graph graph;                                   // all six sensors are vertices; only priced edges exist
    std::vector<RouteLeg> legs;                          // every edge that was added, in file order
    std::vector<UnavailableSensor> unavailableSensors;   // sensors without a forecast: their outgoing edges were omitted
    const RouteLeg* findLeg(const std::string& from, const std::string& to) const;   // nullptr if there is no such edge
};

// Builds the graph for one forecast snapshot. A record whose from-sensor has no forecast is skipped (and that sensor
// is listed in unavailableSensors). Throws std::invalid_argument for a record naming a sensor that is not in the
// snapshot, for a duplicated directed pair, and for an invalid speed or distance.
TravelTimeGraph buildTravelTimeGraph(const std::vector<DistanceRecord>& distances, const ForecastSnapshot& snapshot, double minSpeedMph = 0.0);

}  // namespace routing
