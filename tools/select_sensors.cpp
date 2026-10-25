// select_sensors.cpp -- ONE-TIME helper that documents HOW the six sensors were chosen.
//
// Build:  g++ -std=c++17 -O2 -Wall -Wextra -o build/select_sensors tools/select_sensors.cpp
// Run:    build/select_sensors [--source csv|h5]
//
// It writes no data files; it prints the shortlist so the choice can be justified. The chosen IDs
// are pinned in data_common.h (SELECTED_SENSORS), so later runs do not depend on this search.
//
// Selection rules (all computed from the speed source you choose with --source):
//   R1  every sensor ID exists in the speed file, the location file and the ID list
//   R2  data quality: at most 1% of a sensor's readings are missing (zero = missing in METR-LA)
//   R3  graph: every ordered pair among the six has a real record in the distance file
//       (so all 30 directed edges are genuine, none are invented)
//   R4  every one of those 30 distances is <= 8000 m (short local trips only)
//   R5  spread: the two farthest sensors are >= 3000 m apart in a straight line
//       (otherwise the six sensors are one interchange and routing is meaningless)
// Ranking among the groups that pass: lowest worst-sensor missing rate, then larger spread,
// then smallest IDs, so the answer is deterministic.
//
// Speed variability / congestion is deliberately NOT a rule: choosing sensors because they look
// "interesting" over the whole period would leak information about the later test period.
#include "data_pipeline.h"

#include <cstring>
#include <tuple>

static const double MAX_MISSING = 0.01, MAX_EDGE_M = 8000.0, MIN_SPREAD_M = 3000.0;

static double haversineMeters(const dp::Location& a, const dp::Location& b) {
    const double r = 6371000.0, rad = 3.14159265358979323846 / 180.0;
    double la1 = std::atof(a.latitude.c_str()) * rad, lo1 = std::atof(a.longitude.c_str()) * rad;
    double la2 = std::atof(b.latitude.c_str()) * rad, lo2 = std::atof(b.longitude.c_str()) * rad;
    double h = std::sin((la2 - la1) / 2) * std::sin((la2 - la1) / 2) +
               std::cos(la1) * std::cos(la2) * std::sin((lo2 - lo1) / 2) * std::sin((lo2 - lo1) / 2);
    return 2 * r * std::asin(std::sqrt(h));
}

typedef std::tuple<double, double, std::vector<std::string>> Found;     // (worst missing rate, -spread, group)

static std::map<std::string, dp::Location> locations;
static std::map<std::string, double> missingRate;
static std::map<std::string, std::set<std::string>> compatible;          // pairs usable together (R3 + R4)
static std::vector<Found> found;

// Grow `group` one sensor at a time, always choosing from sensors compatible with every member.
static void extend(std::vector<std::string>& group, const std::vector<std::string>& pool) {
    if (group.size() == 6) {
        double spread = 0, worst = 0;
        for (size_t i = 0; i < group.size(); ++i) {
            worst = std::max(worst, missingRate[group[i]]);
            for (size_t j = i + 1; j < group.size(); ++j) spread = std::max(spread, haversineMeters(locations[group[i]], locations[group[j]]));
        }
        if (spread >= MIN_SPREAD_M) found.push_back(Found(worst, -spread, group));
        return;
    }
    for (size_t i = 0; i < pool.size(); ++i) {
        std::vector<std::string> next;
        for (size_t k = i + 1; k < pool.size(); ++k) if (compatible[pool[i]].count(pool[k])) next.push_back(pool[k]);
        group.push_back(pool[i]);
        extend(group, next);
        group.pop_back();
    }
}

int main(int argc, char** argv) {
    std::string source = "csv";
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--source") && i + 1 < argc) source = argv[++i];
        else { std::cerr << "usage: select_sensors [--source csv|h5]\n"; return 2; }
    }
    try {
        std::string name;
        dp::SpeedTable table = dp::loadSpeeds(source, name);
        locations = dp::readLocations(dp::LOCATIONS);
        std::set<std::string> known(table.ids.begin(), table.ids.end());
        dp::DistanceData d = dp::readDistances(dp::DISTANCES, known);

        std::vector<std::string> candidates;                              // R1 + R2
        for (size_t j = 0; j < table.ids.size(); ++j) {
            size_t missing = 0;
            for (const auto& row : table.rows) missing += dp::isMissing(row[j]);
            missingRate[table.ids[j]] = static_cast<double>(missing) / table.rows.size();
            if (locations.count(table.ids[j]) && missingRate[table.ids[j]] <= MAX_MISSING) candidates.push_back(table.ids[j]);
        }
        std::sort(candidates.begin(), candidates.end());

        for (const std::string& a : candidates)                           // R3 + R4, for pairs
            for (const std::string& b : candidates) {
                if (a == b) continue;
                auto ab = d.kept.find(dp::Edge(a, b)), ba = d.kept.find(dp::Edge(b, a));
                if (ab != d.kept.end() && ba != d.kept.end() && dp::asNumber(ab->second) <= MAX_EDGE_M && dp::asNumber(ba->second) <= MAX_EDGE_M)
                    compatible[a].insert(b);
            }
        for (const std::string& s : candidates) {                         // groups: s is the smallest ID of its group
            std::vector<std::string> group = {s}, pool;
            for (const std::string& t : candidates) if (t > s && compatible[s].count(t)) pool.push_back(t);
            extend(group, pool);
        }
        std::sort(found.begin(), found.end());                            // R5 applied in extend(); ranking = tuple order

        std::cout << "source: " << name << "\n";
        std::cout << dp::fmt("sensors passing R1+R2: %zu of %zu\n", candidates.size(), table.ids.size());
        std::cout << dp::fmt("six-sensor groups passing R3-R5: %zu\n", found.size());
        for (size_t i = 0; i < found.size() && i < 5; ++i) {
            std::string ids;
            for (const std::string& s : std::get<2>(found[i])) ids += (ids.empty() ? "" : ",") + s;
            std::cout << dp::fmt("  worst missing rate %.4f | spread %5.0f m | %s\n", std::get<0>(found[i]), -std::get<1>(found[i]), ids.c_str());
        }
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
