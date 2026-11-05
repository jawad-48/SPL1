#include "route_planner.h"

#include <stdexcept>

namespace routing {

RouteResult planRouteFromSnapshot(const ForecastSnapshot& snapshot, const std::vector<DistanceRecord>& distances,
                                  const std::string& source, const std::string& destination, const RouteConfig& config) {
    const SensorForecast* from = snapshot.find(source);
    if (from == nullptr) throw std::invalid_argument("unknown source sensor '" + source + "'");
    if (snapshot.find(destination) == nullptr) throw std::invalid_argument("unknown destination sensor '" + destination + "'");
    if (!from->available) throw std::runtime_error("source sensor " + source + " has no forecast: " + from->reason);

    const TravelTimeGraph tt = buildTravelTimeGraph(distances, snapshot, config.minSpeedMph);
    const spl1::PathResult found = tt.graph.shortestPath(source, destination);

    RouteResult result;
    result.originRow = snapshot.originRow;
    result.originTime = snapshot.originTime;
    result.targetTime = snapshot.targetTime;
    result.horizonSteps = snapshot.horizonSteps;
    result.unavailableSensors = tt.unavailableSensors;
    result.reachable = found.reachable;
    if (!found.reachable) return result;                         // etaMinutes stays +infinity, path and legs stay empty

    result.path = found.path;
    result.etaMinutes = found.totalCost;
    for (size_t i = 0; i + 1 < found.path.size(); ++i) {
        const RouteLeg* leg = tt.findLeg(found.path[i], found.path[i + 1]);
        if (leg == nullptr) throw std::logic_error("internal error: path edge " + found.path[i] + " -> " + found.path[i + 1] + " has no recorded leg");
        result.legs.push_back(*leg);
    }
    return result;
}

RouteResult planRoute(const baseline::TrafficSeries& series, const SpeedForecaster& forecaster, const std::vector<DistanceRecord>& distances,
                      size_t originRow, const std::string& source, const std::string& destination, const RouteConfig& config) {
    const ForecastSnapshot snapshot = forecaster.predictAt(series, originRow, config.minSpeedMph);
    return planRouteFromSnapshot(snapshot, distances, source, destination, config);
}

}  // namespace routing
