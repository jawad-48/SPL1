// routing_cli.h -- the command-line application "fastest route": load the traffic and distance files, train the Linear
// Regression forecaster on the chronological training period, forecast the speeds at the requested origin time,
// build the travel-time graph, run Dijkstra, and print the route, its legs, the forecast speeds and the total ETA.
//
// This file only connects existing pieces (baseline::loadTrafficSeries, baseline::chronologicalBoundaries,
// routing::SpeedForecaster, routing::loadSensorDistances, routing::planRouteFromSnapshot). It contains no forecasting,
// graph or travel-time logic of its own. main() lives in routing_main.cpp; keeping the logic here lets the tests call it
// with in-memory streams instead of starting a process.
//
// Exit codes (also printed by --help):
//   0  a route was calculated
//   1  a file, dataset or origin-time problem (missing or invalid file, row or timestamp not usable, forecast rejected)
//   2  command-line mistake (unknown or repeated option, missing value, sensor ID not one of the six, bad number or timestamp)
//   3  the request is valid but no route can be calculated (source sensor has no forecast, or destination unreachable)
#pragma once

#include <iosfwd>
#include <string>
#include <vector>

namespace routing {

// args = the command-line arguments WITHOUT the program name. Results go to `out`; errors and the explanation of why a
// route could not be calculated go to `err`. Never throws.
int runRoutingCli(const std::vector<std::string>& args, std::ostream& out, std::ostream& err);

}  // namespace routing
