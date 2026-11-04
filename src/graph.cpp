// graph.cpp -- implementation of spl1::Graph and Dijkstra's shortest path. See graph.h for the rules.
#include "graph.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <queue>
#include <stdexcept>
#include <utility>

namespace spl1 {

namespace {
const std::size_t NONE = static_cast<std::size_t>(-1);       // "no previous vertex"
}

bool Graph::addVertex(const std::string& sensorId) {
    if (sensorId.empty()) throw std::invalid_argument("Graph::addVertex: sensor ID must not be empty");
    if (index_.find(sensorId) != index_.end()) return false;
    index_.emplace(sensorId, ids_.size());
    ids_.push_back(sensorId);
    adjacency_.emplace_back();
    return true;
}

bool Graph::hasVertex(const std::string& sensorId) const {
    return index_.find(sensorId) != index_.end();
}

std::size_t Graph::vertexCount() const { return ids_.size(); }

std::size_t Graph::edgeCount() const { return edgeCount_; }

std::size_t Graph::indexOf(const std::string& sensorId, const char* role) const {
    const auto it = index_.find(sensorId);
    if (it == index_.end())
        throw std::invalid_argument(std::string("Graph: unknown ") + role + " vertex '" + sensorId + "'");
    return it->second;
}

void Graph::addDirectedEdge(const std::string& from, const std::string& to, double weight) {
    if (!std::isfinite(weight)) throw std::invalid_argument("Graph::addDirectedEdge: weight must be finite (NaN and infinity are rejected)");
    if (weight < 0.0) throw std::invalid_argument("Graph::addDirectedEdge: weight must not be negative");
    const std::size_t u = indexOf(from, "source");
    const std::size_t v = indexOf(to, "target");
    if (weight == 0.0) weight = 0.0;                          // turns -0.0 into +0.0 so a cost never prints as "-0"

    for (Edge& e : adjacency_[u]) {                           // duplicate directed edge: keep the lower weight
        if (e.to == v) {
            if (weight < e.weight) e.weight = weight;
            return;
        }
    }
    adjacency_[u].push_back(Edge{v, weight});
    ++edgeCount_;
}

PathResult Graph::shortestPath(const std::string& source, const std::string& destination) const {
    const std::size_t s = indexOf(source, "source");
    const std::size_t t = indexOf(destination, "destination");
    const std::size_t n = ids_.size();
    const double INF = std::numeric_limits<double>::infinity();

    std::vector<double> dist(n, INF);                         // best known cost from s
    std::vector<std::size_t> prev(n, NONE);                   // previous vertex on the best known route

    // Min-heap ordered by (distance, vertex index): std::greater turns the max-heap into a min-heap, and the
    // index breaks distance ties the same way on every run.
    typedef std::pair<double, std::size_t> Entry;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> queue;

    dist[s] = 0.0;
    queue.push(Entry(0.0, s));
    while (!queue.empty()) {
        const Entry top = queue.top();
        queue.pop();
        const double d = top.first;
        const std::size_t u = top.second;
        if (d > dist[u]) continue;                            // stale entry: u was improved after this was pushed
        if (u == t) break;                                    // t is final once it is popped (weights are >= 0)
        for (const Edge& e : adjacency_[u]) {
            const double candidate = d + e.weight;
            if (candidate < dist[e.to]) {                     // strictly better only: also stops zero-weight cycles
                dist[e.to] = candidate;
                prev[e.to] = u;
                queue.push(Entry(candidate, e.to));
            }
        }
    }

    PathResult result;
    if (dist[t] == INF) return result;                        // reachable = false, empty path, cost = +infinity

    result.reachable = true;
    result.totalCost = dist[t];
    for (std::size_t v = t; v != NONE; v = prev[v]) result.path.push_back(ids_[v]);
    std::reverse(result.path.begin(), result.path.end());
    return result;
}

}  // namespace spl1
