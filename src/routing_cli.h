// routing_cli.h -- the command-line application "fastest route": load the traffic and distance files, train the Linear
// Regression forecaster on the chronological training period, forecast the speeds at the requested origin time,
// build the travel-time graph, run Dijkstra, and print the route, its legs, the forecast speeds and the total ETA.
//
// This file only connects existing pieces (baseline::loadTrafficSeries, baseline::chronologicalBoundaries,
// routing::SpeedForecaster, routing::loadSensorDistances, routing::planRouteFromSnapshot). It contains no forecasting,
// graph or travel-time logic of its own. main() lives in routing_main.cpp; keeping the logic here lets the tests call it
// with in-memory streams instead of starting a process.
//
// Route names: the six sensors can be given as r1 to r6 (in the order of dp::SELECTED_SENSORS: r1 = 716328, r2 = 716331,
// r3 = 716337, r4 = 717446, r5 = 717456, r6 = 773062) wherever a sensor ID is accepted, and are shown next to the IDs.
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
// This version never asks questions: --from and --to are required.
int runRoutingCli(const std::vector<std::string>& args, std::ostream& out, std::ostream& err);

// Same, but if --from and/or --to are missing the program asks for them on `in` ("Where are you now?" / "Where do you want
// to go?"), showing the route names r1 to r6. main() uses this version with std::cin.
int runRoutingCli(const std::vector<std::string>& args, std::istream& in, std::ostream& out, std::ostream& err);

}  // namespace routing
