#include "travel_time_graph.h"

#include <cmath>
#include <set>
#include <stdexcept>
#include <utility>

namespace routing {

std::vector<DistanceRecord> parseSensorDistances(const std::vector<std::string>& lines, const std::string& sourceName) {
    if (lines.empty()) throw std::runtime_error(sourceName + ": the file is empty");
    const std::vector<std::string> header = dp::splitLine(lines[0]);
    if (header.size() != 3 || dp::trim(header[0]) != "from_sensor_id" || dp::trim(header[1]) != "to_sensor_id" || dp::trim(header[2]) != "distance_m")
        throw std::runtime_error(sourceName + ": the header must be exactly 'from_sensor_id,to_sensor_id,distance_m'");

    const std::set<std::string> known(dp::SELECTED_SENSORS.begin(), dp::SELECTED_SENSORS.end());
    std::set<std::pair<std::string, std::string> > seen;
    std::vector<DistanceRecord> records;
    for (size_t i = 1; i < lines.size(); ++i) {
        if (dp::trim(lines[i]).empty()) continue;
        const std::vector<std::string> f = dp::splitLine(lines[i]);
        if (f.size() != 3) throw std::runtime_error(dp::fmt("%s: line %zu has %zu fields, expected 3", sourceName.c_str(), i + 1, f.size()));
        DistanceRecord r;
        r.from = dp::trim(f[0]);
        r.to = dp::trim(f[1]);
        if (!known.count(r.from)) throw std::runtime_error(dp::fmt("%s: line %zu: '%s' is not one of the six selected sensors", sourceName.c_str(), i + 1, r.from.c_str()));
        if (!known.count(r.to)) throw std::runtime_error(dp::fmt("%s: line %zu: '%s' is not one of the six selected sensors", sourceName.c_str(), i + 1, r.to.c_str()));
        if (r.from == r.to) throw std::runtime_error(dp::fmt("%s: line %zu: a sensor cannot have a distance to itself (%s)", sourceName.c_str(), i + 1, r.from.c_str()));
        r.distanceM = dp::parseNumber(dp::trim(f[2]));
        if (!std::isfinite(r.distanceM) || r.distanceM <= 0)
            throw std::runtime_error(dp::fmt("%s: line %zu: distance '%s' must be a finite number greater than 0", sourceName.c_str(), i + 1, dp::trim(f[2]).c_str()));
        if (!seen.insert(std::make_pair(r.from, r.to)).second)
            throw std::runtime_error(dp::fmt("%s: line %zu: the directed pair %s -> %s appears twice", sourceName.c_str(), i + 1, r.from.c_str(), r.to.c_str()));
        records.push_back(r);
    }
    if (records.empty()) throw std::runtime_error(sourceName + ": the file has a header but no distance rows");
    return records;
}

std::vector<DistanceRecord> loadSensorDistances(const std::string& path) {
    return parseSensorDistances(dp::readLines(path), path);
}

double travelTimeMinutes(double distanceM, double speedMph, double minSpeedMph) {
    if (!std::isfinite(distanceM) || distanceM <= 0) throw std::invalid_argument("distance must be finite and greater than 0 (meters)");
    if (!std::isfinite(speedMph) || speedMph <= 0) throw std::invalid_argument("speed must be finite and greater than 0 (mph)");
    if (!(minSpeedMph >= 0.0) || !std::isfinite(minSpeedMph)) throw std::invalid_argument("minSpeedMph must be finite and >= 0");
    if (speedMph < minSpeedMph) throw std::invalid_argument(dp::fmt("speed %.6g mph is below the %.6g mph floor", speedMph, minSpeedMph));
    const double minutes = distanceM / (speedMph * METERS_PER_MINUTE_PER_MPH);
    if (!std::isfinite(minutes) || minutes <= 0) throw std::invalid_argument("travel time is not a finite positive number for this distance and speed");
    return minutes;
}

const RouteLeg* TravelTimeGraph::findLeg(const std::string& from, const std::string& to) const {
    for (const RouteLeg& leg : legs) if (leg.from == from && leg.to == to) return &leg;
    return nullptr;
}

TravelTimeGraph buildTravelTimeGraph(const std::vector<DistanceRecord>& distances, const ForecastSnapshot& snapshot, double minSpeedMph) {
    TravelTimeGraph out;
    for (const SensorForecast& s : snapshot.sensors) {
        out.graph.addVertex(s.sensorId);
        if (!s.available) {
            UnavailableSensor u;
            u.sensorId = s.sensorId;
            u.reason = s.reason;
            out.unavailableSensors.push_back(u);
        }
    }
    std::set<std::pair<std::string, std::string> > seen;
    for (const DistanceRecord& r : distances) {
        const SensorForecast* upstream = snapshot.find(r.from);
        if (upstream == nullptr || snapshot.find(r.to) == nullptr)
            throw std::invalid_argument("distance record " + r.from + " -> " + r.to + " names a sensor that is not in the forecast snapshot");
        if (!seen.insert(std::make_pair(r.from, r.to)).second)
            throw std::invalid_argument("the directed pair " + r.from + " -> " + r.to + " appears twice in the distance records");
        if (!upstream->available) continue;                     // no forecast for this sensor: its edges are left out, not guessed
        RouteLeg leg;
        leg.from = r.from;
        leg.to = r.to;
        leg.distanceM = r.distanceM;
        leg.speedMph = upstream->speedMph;                       // upstream-speed assumption
        leg.minutes = travelTimeMinutes(r.distanceM, upstream->speedMph, minSpeedMph);
        out.graph.addDirectedEdge(leg.from, leg.to, leg.minutes);
        out.legs.push_back(leg);
    }
    return out;
}

}  // namespace routing
