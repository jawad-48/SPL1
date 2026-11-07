// route_planner.h -- forecast speeds -> travel-time graph -> Dijkstra -> fastest route and its ETA.
// Linear Regression (speed_forecaster) and Dijkstra (graph) stay separate; this file only connects them through the
// ForecastSnapshot and the TravelTimeGraph. See travel_time_graph.h for the modelling assumptions.
#pragma once

#include <limits>
#include <string>
#include <vector>

#include "speed_forecaster.h"
#include "travel_time_graph.h"

namespace routing {

struct RouteConfig {
    double minSpeedMph = 0.0;      // optional floor for predicted speeds; 0 = disabled (only "finite and > 0" is required)
};

struct RouteResult {
    bool reachable = false;                                              // false: no route exists with the available forecasts
    std::vector<std::string> path;                                       // source ... destination; empty when not reachable
    double etaMinutes = std::numeric_limits<double>::infinity();         // total travel time; +infinity when not reachable
    std::vector<RouteLeg> legs;                                          // one entry per hop of the path
    size_t originRow = 0;
    long long originTime = 0;                                            // time of the last reading used (seconds since 1970-01-01)
    long long targetTime = 0;                                            // the moment the speeds are forecast for
    int horizonSteps = 0;
    std::vector<UnavailableSensor> unavailableSensors;                   // sensors that could not be forecast (their edges were omitted)
};

// Plan from an existing snapshot. Throws std::invalid_argument for an unknown source or destination sensor, and
// std::runtime_error if the SOURCE sensor has no forecast (its outgoing edges would all be unpriced).
// source == destination gives path {source}, ETA 0 (the source still has to be available).
RouteResult planRouteFromSnapshot(const ForecastSnapshot& snapshot, const std::vector<DistanceRecord>& distances,
                                  const std::string& source, const std::string& destination, const RouteConfig& config = RouteConfig());

// Forecast at originRow (reads only rows <= originRow), then plan.
RouteResult planRoute(const baseline::TrafficSeries& series, const SpeedForecaster& forecaster, const std::vector<DistanceRecord>& distances,
                      size_t originRow, const std::string& source, const std::string& destination, const RouteConfig& config = RouteConfig());

}  // namespace routing
