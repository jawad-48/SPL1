// test_graph.cpp -- tests for spl1::Graph and Dijkstra's shortest path (src/graph.h, src/graph.cpp).
//
// Build (from the repository root; write the program OUTSIDE the repo or into an ignored folder):
//   g++ -std=c++17 -O2 -Wall -Wextra -o build/test_graph tests/test_graph.cpp src/graph.cpp
// Run:
//   build/test_graph
//
// Self-contained: standard library only, no test framework. Exit code 0 only if every check passed.
#include <cmath>
#include <cstdio>
#include <initializer_list>
#include <map>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include "../src/graph.h"

using spl1::Graph;
using spl1::PathResult;

static const double INF = std::numeric_limits<double>::infinity();
static int passedCount = 0;
static int failedCount = 0;

static void expect(const char* name, bool ok) {
    std::printf("%s  %s\n", ok ? "PASS" : "FAIL", name);
    if (ok) ++passedCount; else ++failedCount;
}

// true if f() throws exactly an exception of type Ex (or derived from it)
template <class Ex, class F>
static bool throws(F f) {
    try { f(); } catch (const Ex&) { return true; } catch (...) { return false; }
    return false;
}

struct E { const char* from; const char* to; double weight; };

static Graph build(std::initializer_list<const char*> vertices, std::initializer_list<E> edges) {
    Graph g;
    for (const char* v : vertices) g.addVertex(v);
    for (const E& e : edges) g.addDirectedEdge(e.from, e.to, e.weight);
    return g;
}

static bool pathIs(const PathResult& r, std::initializer_list<const char*> expected) {
    if (r.path.size() != expected.size()) return false;
    size_t i = 0;
    for (const char* id : expected) if (r.path[i++] != id) return false;
    return true;
}

// ---------------------------------------------------------------------------------------------------------------------
static void testVertices() {
    Graph g;
    expect("vertices: an empty graph has no vertices and hasVertex is false", g.vertexCount() == 0 && !g.hasVertex("A"));
    expect("vertices: addVertex returns true for a new vertex", g.addVertex("A"));
    expect("vertices: hasVertex is true after addVertex, false for others", g.hasVertex("A") && !g.hasVertex("B"));
    expect("vertices: adding an existing vertex returns false and changes nothing", !g.addVertex("A") && g.vertexCount() == 1);
    g.addVertex("a");
    g.addVertex("007");
    g.addVertex("7");
    expect("vertices: IDs are case-sensitive strings and '007' differs from '7'", g.vertexCount() == 4 && g.hasVertex("a") && g.hasVertex("007") && g.hasVertex("7"));
    expect("vertices: an empty sensor ID is rejected", throws<std::invalid_argument>([&] { g.addVertex(""); }) && g.vertexCount() == 4);
    Graph metr;                                                   // real-looking sensor IDs, as used by the project data
    metr.addVertex("773869"); metr.addVertex("767541"); metr.addDirectedEdge("773869", "767541", 120.5);
    PathResult r = metr.shortestPath("773869", "767541");
    expect("vertices: numeric-looking sensor IDs work as strings", r.reachable && pathIs(r, {"773869", "767541"}) && r.totalCost == 120.5);
}

static void testEdgeValidation() {
    Graph g = build({"A", "B"}, {});
    expect("edges: a valid edge is accepted and counted", [&] { g.addDirectedEdge("A", "B", 2.5); return g.edgeCount() == 1; }());
    expect("edges: negative weight is rejected", throws<std::invalid_argument>([&] { g.addDirectedEdge("B", "A", -1.0); }));
    expect("edges: tiny negative weight is rejected", throws<std::invalid_argument>([&] { g.addDirectedEdge("B", "A", -1e-12); }));
    expect("edges: NaN weight is rejected", throws<std::invalid_argument>([&] { g.addDirectedEdge("B", "A", std::nan("")); }));
    expect("edges: +infinity weight is rejected", throws<std::invalid_argument>([&] { g.addDirectedEdge("B", "A", INF); }));
    expect("edges: -infinity weight is rejected", throws<std::invalid_argument>([&] { g.addDirectedEdge("B", "A", -INF); }));
    expect("edges: rejected weights add nothing (still 1 edge, B cannot reach A)", g.edgeCount() == 1 && !g.shortestPath("B", "A").reachable);
    expect("edges: an unknown source vertex is rejected", throws<std::invalid_argument>([&] { g.addDirectedEdge("X", "B", 1.0); }));
    expect("edges: an unknown target vertex is rejected", throws<std::invalid_argument>([&] { g.addDirectedEdge("A", "X", 1.0); }));
    expect("edges: a rejected edge does not create vertices", !g.hasVertex("X") && g.vertexCount() == 2 && g.edgeCount() == 1);
    bool message = false;
    try { g.addDirectedEdge("A", "ZZZ", 1.0); } catch (const std::invalid_argument& e) { message = std::string(e.what()).find("ZZZ") != std::string::npos; }
    expect("edges: the error message names the unknown vertex", message);
    Graph z = build({"A", "B"}, {{"A", "B", 0.0}});
    expect("edges: weight 0 is accepted", z.shortestPath("A", "B").reachable && z.shortestPath("A", "B").totalCost == 0.0);
    Graph nz = build({"A", "B"}, {{"A", "B", -0.0}});
    expect("edges: weight -0.0 is accepted and the cost is a plain +0", !std::signbit(nz.shortestPath("A", "B").totalCost));
    Graph big = build({"A", "B"}, {{"A", "B", 1e300}});
    expect("edges: a very large finite weight is accepted", big.shortestPath("A", "B").totalCost == 1e300);
}

static void testDirected() {
    Graph g = build({"A", "B"}, {{"A", "B", 3.0}});
    PathResult forward = g.shortestPath("A", "B"), backward = g.shortestPath("B", "A");
    expect("directed: A -> B exists", forward.reachable && pathIs(forward, {"A", "B"}) && forward.totalCost == 3.0);
    expect("directed: A -> B does not create B -> A", !backward.reachable && g.edgeCount() == 1);
    g.addDirectedEdge("B", "A", 7.0);
    expect("directed: the two directions keep their own weights", g.shortestPath("A", "B").totalCost == 3.0 && g.shortestPath("B", "A").totalCost == 7.0 && g.edgeCount() == 2);
    Graph oneWay = build({"A", "B", "C"}, {{"A", "B", 1.0}, {"C", "B", 1.0}});
    expect("directed: two edges pointing at B do not connect A and C", !oneWay.shortestPath("A", "C").reachable && !oneWay.shortestPath("C", "A").reachable);
}

static void testSourceEqualsDestination() {
    Graph g = build({"A", "B"}, {{"A", "B", 4.0}});
    PathResult r = g.shortestPath("A", "A");
    expect("same vertex: reachable, path = {A}, cost 0", r.reachable && pathIs(r, {"A"}) && r.totalCost == 0.0);
    PathResult lone = build({"Z"}, {}).shortestPath("Z", "Z");
    expect("same vertex: works for a vertex with no edges at all", lone.reachable && pathIs(lone, {"Z"}) && lone.totalCost == 0.0);
    Graph loop = build({"A", "B"}, {{"A", "A", 5.0}, {"A", "B", 1.0}});
    PathResult viaLoop = loop.shortestPath("A", "A");
    expect("same vertex: a self-loop never makes the answer more than 0", viaLoop.totalCost == 0.0 && pathIs(viaLoop, {"A"}) && loop.shortestPath("A", "B").totalCost == 1.0);
}

static void testUnreachableAndUnknown() {
    Graph g = build({"A", "B", "C", "D"}, {{"A", "B", 1.0}, {"C", "D", 1.0}});
    PathResult r = g.shortestPath("A", "D");
    expect("unreachable: disconnected components give reachable = false", !r.reachable);
    expect("unreachable: the path is empty and the cost is +infinity (not 0)", r.path.empty() && std::isinf(r.totalCost) && r.totalCost > 0);
    Graph isolated = build({"A", "B"}, {});
    expect("unreachable: an isolated vertex cannot be reached", !isolated.shortestPath("A", "B").reachable);
    expect("unknown: unknown source throws std::invalid_argument", throws<std::invalid_argument>([&] { g.shortestPath("X", "A"); }));
    expect("unknown: unknown destination throws std::invalid_argument", throws<std::invalid_argument>([&] { g.shortestPath("A", "X"); }));
    expect("unknown: both unknown throws", throws<std::invalid_argument>([&] { g.shortestPath("X", "Y"); }));
    expect("unknown: source == destination == unknown still throws (not 'reachable')", throws<std::invalid_argument>([&] { g.shortestPath("X", "X"); }));
    bool message = false;
    try { g.shortestPath("A", "NOPE"); } catch (const std::invalid_argument& e) { message = std::string(e.what()).find("NOPE") != std::string::npos; }
    expect("unknown: the error message names the unknown vertex", message);
    Graph empty;
    expect("unknown: an empty graph rejects any query", throws<std::invalid_argument>([&] { empty.shortestPath("A", "A"); }));
}

static void testRouteChoice() {
    // Classic: the cheapest first edge (A->B, 4) is not on the best route. Best: A -> C -> B -> D = 1 + 2 + 1.
    Graph g = build({"A", "B", "C", "D"}, {{"A", "B", 4.0}, {"A", "C", 1.0}, {"C", "B", 2.0}, {"B", "D", 1.0}, {"C", "D", 5.0}});
    PathResult r = g.shortestPath("A", "D");
    expect("routes: picks the cheapest of several routes (A,C,B,D cost 4, not A,B,D cost 5 or A,C,D cost 6)", pathIs(r, {"A", "C", "B", "D"}) && r.totalCost == 4.0);
    Graph hops = build({"A", "B", "C", "D", "E"}, {{"A", "E", 10.0}, {"A", "B", 1.0}, {"B", "C", 1.0}, {"C", "D", 1.0}, {"D", "E", 1.0}});
    PathResult many = hops.shortestPath("A", "E");
    expect("routes: four cheap hops beat one expensive direct edge", pathIs(many, {"A", "B", "C", "D", "E"}) && many.totalCost == 4.0);
    Graph direct = build({"A", "B", "C"}, {{"A", "C", 2.0}, {"A", "B", 1.5}, {"B", "C", 1.5}});
    expect("routes: the direct edge wins when it is cheaper than the detour", pathIs(direct.shortestPath("A", "C"), {"A", "C"}) && direct.shortestPath("A", "C").totalCost == 2.0);
    Graph fractional = build({"A", "B", "C"}, {{"A", "B", 0.25}, {"B", "C", 0.5}});
    expect("routes: fractional weights are summed exactly", fractional.shortestPath("A", "C").totalCost == 0.75);
    PathResult later = g.shortestPath("A", "D");
    g.addDirectedEdge("A", "D", 1.0);
    PathResult after = g.shortestPath("A", "D");
    expect("routes: a query does not modify the graph, and a later edge changes the answer", later.totalCost == 4.0 && after.totalCost == 1.0 && pathIs(after, {"A", "D"}));
    Graph improved = build({"A", "B", "C", "D"}, {{"A", "B", 10.0}, {"A", "C", 1.0}, {"C", "B", 1.0}, {"B", "D", 1.0}});
    PathResult viaImproved = improved.shortestPath("A", "D");
    expect("routes: a vertex first seen at cost 10 and later improved to 2 gives A,C,B,D = 3", pathIs(viaImproved, {"A", "C", "B", "D"}) && viaImproved.totalCost == 3.0);
    Graph stale = build({"A", "B", "C", "D", "E"}, {{"A", "B", 10.0}, {"A", "C", 1.0}, {"C", "B", 1.0}, {"B", "D", 1.0}, {"D", "E", 100.0}});
    expect("routes: a stale queue entry for B (cost 10) is skipped; far target E costs 103", stale.shortestPath("A", "E").totalCost == 103.0);
}

static void testCycles() {
    Graph g = build({"A", "B", "C", "D"}, {{"A", "B", 1.0}, {"B", "C", 1.0}, {"C", "A", 1.0}, {"C", "D", 1.0}});
    expect("cycles: a directed cycle does not loop forever; A to D is 3", pathIs(g.shortestPath("A", "D"), {"A", "B", "C", "D"}) && g.shortestPath("A", "D").totalCost == 3.0);
    PathResult back = g.shortestPath("B", "A");
    expect("cycles: going round the cycle is allowed when it is the only way (B,C,A = 2)", pathIs(back, {"B", "C", "A"}) && back.totalCost == 2.0);
    PathResult toD = g.shortestPath("D", "A");
    expect("cycles: A to C goes A,B,C, and D (no way out of it) cannot reach A", pathIs(g.shortestPath("A", "C"), {"A", "B", "C"}) && !toD.reachable);
    Graph twoCycles = build({"A", "B", "C", "D"}, {{"A", "B", 1.0}, {"B", "A", 1.0}, {"B", "C", 1.0}, {"C", "B", 1.0}, {"C", "D", 1.0}, {"D", "C", 1.0}});
    expect("cycles: two-way edges (a cycle of length 2) behave like an undirected road", twoCycles.shortestPath("A", "D").totalCost == 3.0 && twoCycles.shortestPath("D", "A").totalCost == 3.0);
}

static void testZeroWeights() {
    Graph g = build({"A", "B", "C"}, {{"A", "B", 0.0}, {"B", "C", 0.0}, {"A", "C", 1.0}});
    PathResult r = g.shortestPath("A", "C");
    expect("zero weights: a chain of zero-cost edges beats a cost-1 direct edge", pathIs(r, {"A", "B", "C"}) && r.totalCost == 0.0);
    Graph cycle = build({"A", "B", "C"}, {{"A", "B", 0.0}, {"B", "A", 0.0}, {"B", "C", 2.0}});
    PathResult r2 = cycle.shortestPath("A", "C");
    expect("zero weights: a zero-cost cycle terminates and does not corrupt the path", pathIs(r2, {"A", "B", "C"}) && r2.totalCost == 2.0);
    Graph allZero = build({"A", "B", "C", "D"}, {{"A", "B", 0.0}, {"B", "C", 0.0}, {"C", "A", 0.0}, {"C", "D", 0.0}});
    PathResult r3 = allZero.shortestPath("A", "D");
    expect("zero weights: a graph of only zero-cost edges (with a cycle) gives cost 0", r3.reachable && r3.totalCost == 0.0 && r3.path.front() == "A" && r3.path.back() == "D");
}

static void testEqualCostAlternatives() {
    Graph g = build({"A", "B", "C", "D"}, {{"A", "B", 1.0}, {"A", "C", 1.0}, {"B", "D", 1.0}, {"C", "D", 1.0}});
    PathResult r = g.shortestPath("A", "D");
    expect("ties: two equal-cost routes give cost 2 and a valid 3-vertex route", r.reachable && r.totalCost == 2.0 && r.path.size() == 3 && r.path.front() == "A" && r.path.back() == "D" && (r.path[1] == "B" || r.path[1] == "C"));
    expect("ties: the choice is deterministic (first-added vertex wins: A,B,D), run after run", pathIs(r, {"A", "B", "D"}) && pathIs(g.shortestPath("A", "D"), {"A", "B", "D"}));
    Graph swapped = build({"A", "C", "B", "D"}, {{"A", "C", 1.0}, {"A", "B", 1.0}, {"C", "D", 1.0}, {"B", "D", 1.0}});
    expect("ties: adding the vertices in another order changes which tie is chosen, never the cost", swapped.shortestPath("A", "D").totalCost == 2.0 && pathIs(swapped.shortestPath("A", "D"), {"A", "C", "D"}));
    Graph three = build({"S", "P", "Q", "R", "T"}, {{"S", "P", 2.0}, {"S", "Q", 2.0}, {"S", "R", 2.0}, {"P", "T", 3.0}, {"Q", "T", 3.0}, {"R", "T", 3.0}});
    expect("ties: three equal routes still give the right cost (5)", three.shortestPath("S", "T").totalCost == 5.0 && three.shortestPath("S", "T").path.size() == 3);
}

static void testDuplicateEdges() {
    Graph g = build({"A", "B"}, {});
    g.addDirectedEdge("A", "B", 5.0);
    g.addDirectedEdge("A", "B", 3.0);
    expect("duplicates: a second A->B with a lower weight replaces the first (cost 3), still one edge", g.shortestPath("A", "B").totalCost == 3.0 && g.edgeCount() == 1);
    g.addDirectedEdge("A", "B", 9.0);
    expect("duplicates: a later, higher weight is ignored (still 3)", g.shortestPath("A", "B").totalCost == 3.0 && g.edgeCount() == 1);
    Graph h = build({"A", "B"}, {{"A", "B", 3.0}, {"A", "B", 5.0}});
    expect("duplicates: order does not matter, the lower weight is kept (3)", h.shortestPath("A", "B").totalCost == 3.0 && h.edgeCount() == 1);
    Graph same = build({"A", "B"}, {{"A", "B", 4.0}, {"A", "B", 4.0}});
    expect("duplicates: an identical edge added twice is still one edge", same.edgeCount() == 1 && same.shortestPath("A", "B").totalCost == 4.0);
    Graph both = build({"A", "B"}, {{"A", "B", 1.0}, {"B", "A", 8.0}, {"B", "A", 2.0}});
    expect("duplicates: A->B and B->A are different edges (2 edges)", both.edgeCount() == 2 && both.shortestPath("B", "A").totalCost == 2.0 && both.shortestPath("A", "B").totalCost == 1.0);
    Graph zero = build({"A", "B"}, {{"A", "B", 6.0}, {"A", "B", 0.0}});
    expect("duplicates: a later zero weight wins", zero.shortestPath("A", "B").totalCost == 0.0 && zero.edgeCount() == 1);
}

// Compares the graph with an independent all-pairs Floyd-Warshall on many random graphs (integer weights 0..9, so every
// sum is exact): reachability, cost, and that the returned path is a real chain of edges whose weights add up to the cost.
static void testRandomAgainstFloydWarshall() {
    std::mt19937 rng(20261104u);
    auto roll = [&](unsigned n) { return static_cast<unsigned>(rng() % n); };
    bool reachabilityOk = true, costOk = true, pathOk = true, countOk = true;
    int graphs = 0, pairs = 0;
    for (int trial = 0; trial < 400; ++trial) {
        const unsigned n = 1 + roll(12);
        const unsigned m = roll(3 * n + 1);
        std::vector<std::string> names;
        std::map<std::string, unsigned> where;
        Graph g;
        for (unsigned i = 0; i < n; ++i) { names.push_back("S" + std::to_string(700000 + i)); where[names.back()] = i; g.addVertex(names.back()); }
        std::vector<std::vector<double>> best(n, std::vector<double>(n, INF));      // cheapest direct edge u -> v
        for (unsigned k = 0; k < m; ++k) {
            const unsigned u = roll(n), v = roll(n);
            const double w = static_cast<double>(roll(10));
            g.addDirectedEdge(names[u], names[v], w);
            if (w < best[u][v]) best[u][v] = w;
        }
        size_t distinct = 0;
        for (unsigned u = 0; u < n; ++u) for (unsigned v = 0; v < n; ++v) if (best[u][v] < INF) ++distinct;
        countOk = countOk && g.edgeCount() == distinct && g.vertexCount() == n;
        std::vector<std::vector<double>> d = best;
        for (unsigned i = 0; i < n; ++i) d[i][i] = 0.0;
        for (unsigned k = 0; k < n; ++k) for (unsigned i = 0; i < n; ++i) for (unsigned j = 0; j < n; ++j)
            if (d[i][k] + d[k][j] < d[i][j]) d[i][j] = d[i][k] + d[k][j];
        for (unsigned s = 0; s < n; ++s) {
            for (unsigned t = 0; t < n; ++t) {
                ++pairs;
                const PathResult r = g.shortestPath(names[s], names[t]);
                const bool should = d[s][t] < INF;
                reachabilityOk = reachabilityOk && r.reachable == should;
                if (!should) { pathOk = pathOk && r.path.empty() && std::isinf(r.totalCost); continue; }
                costOk = costOk && r.totalCost == d[s][t];
                bool chain = !r.path.empty() && r.path.front() == names[s] && r.path.back() == names[t];
                double sum = 0.0;
                for (size_t i = 0; chain && i + 1 < r.path.size(); ++i) {
                    const double w = best[where[r.path[i]]][where[r.path[i + 1]]];
                    if (!(w < INF)) chain = false; else sum += w;
                }
                pathOk = pathOk && chain && sum == r.totalCost;
            }
        }
        ++graphs;
    }
    std::printf("      (random check: %d graphs, %d source/destination pairs)\n", graphs, pairs);
    expect("random: reachable/unreachable agrees with Floyd-Warshall for every pair", reachabilityOk);
    expect("random: total cost equals the Floyd-Warshall distance for every reachable pair", costOk);
    expect("random: every returned path starts/ends correctly, uses real edges and sums to its cost", pathOk);
    expect("random: vertexCount and edgeCount (duplicates merged) are right", countOk);
}

static void testLargeGraphs() {
    {   // a 100,000-vertex chain with a costly shortcut: no recursion, long path rebuilt in order
        const int N = 100000;
        Graph g;
        std::vector<std::string> names;
        names.reserve(N);
        for (int i = 0; i < N; ++i) { names.push_back("N" + std::to_string(i)); g.addVertex(names.back()); }
        for (int i = 0; i + 1 < N; ++i) g.addDirectedEdge(names[i], names[i + 1], 1.0);
        g.addDirectedEdge(names[0], names[N - 1], 1.0e6);
        const PathResult r = g.shortestPath(names[0], names[N - 1]);
        bool ordered = r.path.size() == static_cast<size_t>(N);
        for (int i = 0; ordered && i < N; i += 997) ordered = r.path[i] == names[i];
        expect("large: a 100,000-vertex chain gives cost 99999 and the full ordered path", r.reachable && r.totalCost == 99999.0 && ordered && r.path.back() == names[N - 1]);
        expect("large: the same chain is unreachable backwards", !g.shortestPath(names[N - 1], names[0]).reachable);
    }
    {   // a 200 x 200 grid, roads both ways, unit weights: many equal-cost routes
        const int W = 200;
        Graph g;
        auto name = [&](int r, int c) { return std::to_string(r) + "_" + std::to_string(c); };
        for (int r = 0; r < W; ++r) for (int c = 0; c < W; ++c) g.addVertex(name(r, c));
        for (int r = 0; r < W; ++r) for (int c = 0; c < W; ++c) {
            if (r + 1 < W) { g.addDirectedEdge(name(r, c), name(r + 1, c), 1.0); g.addDirectedEdge(name(r + 1, c), name(r, c), 1.0); }
            if (c + 1 < W) { g.addDirectedEdge(name(r, c), name(r, c + 1), 1.0); g.addDirectedEdge(name(r, c + 1), name(r, c), 1.0); }
        }
        const PathResult r = g.shortestPath(name(0, 0), name(W - 1, W - 1));
        expect("large: a 200x200 grid corner to corner costs 398 over 399 vertices", r.reachable && r.totalCost == 398.0 && r.path.size() == 399);
    }
}

int main() {
    testVertices();
    testEdgeValidation();
    testDirected();
    testSourceEqualsDestination();
    testUnreachableAndUnknown();
    testRouteChoice();
    testCycles();
    testZeroWeights();
    testEqualCostAlternatives();
    testDuplicateEdges();
    testRandomAgainstFloydWarshall();
    testLargeGraphs();
    std::printf("\n%d passed, %d failed\n", passedCount, failedCount);
    return failedCount == 0 ? 0 : 1;
}
