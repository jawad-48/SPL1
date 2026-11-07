// graph.h -- a directed, weighted graph keyed by sensor ID strings, with Dijkstra's shortest path.
//
// Standard library only (C++17). Vertices are identified by their sensor ID (a string), never by an
// array index; indices exist only inside the class.
//
// Rules this class enforces (each one is covered by tests/test_graph.cpp):
//   * Edges are directed: addDirectedEdge("A", "B", w) does NOT create B -> A.
//   * Weights must be finite and >= 0. Negative, NaN and +/- infinity throw std::invalid_argument.
//     Nothing is silently converted to zero. (-0.0 is accepted and stored as +0.0.)
//   * Both endpoints of an edge must already exist (addVertex first); otherwise std::invalid_argument.
//     This catches a mistyped sensor ID instead of quietly creating a new, disconnected vertex.
//   * Adding the same directed edge (same from, same to) again KEEPS THE LOWER weight and does not
//     create a second edge. edgeCount() counts distinct directed edges.
//   * Self-loops (A -> A) are accepted; they can never shorten a path.
//   * An empty sensor ID is rejected (std::invalid_argument).
//   * shortestPath() with an unknown source or destination throws std::invalid_argument naming the ID.
//     "No route exists" is reported through PathResult::reachable, not through an exception.
#pragma once

#include <cstddef>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

namespace spl1 {

struct PathResult {
    bool reachable = false;                 // false: there is no directed route from source to destination
    std::vector<std::string> path;          // source ... destination, in order; empty when not reachable
    double totalCost = std::numeric_limits<double>::infinity();   // sum of edge weights; +infinity when not reachable
};

class Graph {
public:
    // Adds a vertex. Returns true if it was new, false if it already existed (nothing changes).
    bool addVertex(const std::string& sensorId);

    // Adds the directed edge from -> to. See the rules above for validation and duplicate handling.
    void addDirectedEdge(const std::string& from, const std::string& to, double weight);

    bool hasVertex(const std::string& sensorId) const;

    // Dijkstra's algorithm with an adjacency list and a min-priority queue: O((V + E) log V).
    // source == destination gives reachable, path = {source}, totalCost = 0.
    // When several routes have the same minimum cost, exactly one is returned. The choice is
    // deterministic for a given insertion order of vertices and edges (ties go to the vertex added first);
    // only the cost is guaranteed to be unique.
    // Limitation: if a path's cost would overflow double to infinity (weights near 1e308), it is reported unreachable.
    PathResult shortestPath(const std::string& source, const std::string& destination) const;

    std::size_t vertexCount() const;
    std::size_t edgeCount() const;          // distinct directed edges

private:
    struct Edge {
        std::size_t to;
        double weight;
    };

    std::size_t indexOf(const std::string& sensorId, const char* role) const;   // throws if the vertex is unknown

    std::unordered_map<std::string, std::size_t> index_;     // sensor ID -> internal index
    std::vector<std::string> ids_;                           // internal index -> sensor ID
    std::vector<std::vector<Edge>> adjacency_;               // adjacency list, one entry per vertex
    std::size_t edgeCount_ = 0;
};

}  // namespace spl1
