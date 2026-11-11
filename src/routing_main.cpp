// routing_main.cpp -- entry point of the fastest-route command-line application. All logic is in routing_cli.cpp.
//
// Build (from the repository root, Windows PowerShell or Linux; see docs/routing_cli.md):
//   g++ -std=c++17 -O2 -Wall -Wextra -o build/routing src/routing_main.cpp src/routing_cli.cpp src/clock_origin.cpp src/route_planner.cpp src/travel_time_graph.cpp src/speed_forecaster.cpp src/graph.cpp src/traffic_data.cpp src/features.cpp src/metrics.cpp src/linear_regression.cpp src/baseline_pipeline.cpp
#include <iostream>
#include <string>
#include <vector>

#include "routing_cli.h"

int main(int argc, char** argv) {
    std::vector<std::string> args;
    for (int i = 1; i < argc; ++i) args.push_back(argv[i]);
    // The application's default origin is the computer's clock (historical replay). The library function keeps "latest row"
    // when no origin option is given, so --latest, --row and --time behave exactly as before.
    bool originChosen = false;
    for (const std::string& a : args)
        if (a == "--row" || a == "--time" || a == "--latest" || a == "--auto-clock" || a == "--clock" || a == "--help" || a == "-h") originChosen = true;
    if (!originChosen) args.push_back("--auto-clock");
    return routing::runRoutingCli(args, std::cin, std::cout, std::cerr);
}
