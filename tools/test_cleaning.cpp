// test_cleaning.cpp -- checks the missing-value policy and the timestamp helpers on small hand-made examples.
//
// Build:  g++ -std=c++17 -O2 -Wall -Wextra -o build/test_cleaning tools/test_cleaning.cpp
// Run:    build/test_cleaning
//
// Plain checks, no test framework. The program exits with a non-zero code if any test fails.
#include "data_common.h"

#include <random>

using dp::NAN_VALUE;
typedef std::vector<double> Col;

static int failed = 0, passed = 0;

static void expect(const char* name, bool ok) {
    std::printf("%s  %s\n", ok ? "PASS" : "FAIL", name);
    ok ? ++passed : ++failed;
}

// n timestamps exactly 5 minutes apart
static std::vector<long long> grid(size_t n) {
    std::vector<long long> t;
    for (size_t i = 0; i < n; ++i) t.push_back(1330560000LL + 300 * static_cast<long long>(i));
    return t;
}

// Compare two columns where NaN means "empty".
static bool same(const Col& a, const Col& b) {
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); ++i) {
        bool an = std::isnan(a[i]), bn = std::isnan(b[i]);
        if (an != bn || (!an && a[i] != b[i])) return false;
    }
    return true;
}

static dp::CleanResult run(const Col& c) { return dp::cleanColumn(grid(c.size()), c); }

int main() {
    const double N = NAN_VALUE;
    const int L = dp::MAX_FILL_STEPS;

    {   // a short gap is filled with the previous reading
        dp::CleanResult r = run({50, 0, 0, 0, 60});
        expect("short gap is filled with the previous reading", same(r.values, {50, 50, 50, 50, 60}) && r.imputed == 3 && r.runs.at(3) == 1);
    }
    {   // a gap of exactly the limit is fully filled
        Col c = {40};
        for (int i = 0; i < L; ++i) c.push_back(0);
        c.push_back(55);
        Col want(L + 1, 40.0);
        want.push_back(55);
        dp::CleanResult r = run(c);
        expect("gap of exactly the limit is fully filled", same(r.values, want) && r.imputed == L);
    }
    {   // a longer gap is filled only up to the limit, then left empty
        Col c = {40};
        for (int i = 0; i < 9; ++i) c.push_back(0);
        c.push_back(55);
        dp::CleanResult r = run(c);
        bool ok = r.imputed == L && r.runs.at(9) == 1 && r.values.back() == 55;
        for (int i = 1; i <= 9; ++i) ok = ok && (i <= L ? r.values[i] == 40 : std::isnan(r.values[i]));
        expect("long gap is filled up to the limit, then left empty", ok);
    }
    {
        dp::CleanResult r = run({0, 0, 45, 46});
        expect("leading missing values stay empty", same(r.values, {N, N, 45, 46}) && r.imputed == 0);
    }
    {
        dp::CleanResult r = run({45, 0, 0});
        expect("trailing gap is filled up to the limit", same(r.values, {45, 45, 45}));
    }
    {
        dp::CleanResult r = run({30, N, -5, 0, 31});
        expect("NaN, negative and zero all count as missing", same(r.values, {30, 30, 30, 30, 31}) && r.imputed == 3);
    }
    {
        Col c = {61.5, 0, 62.25, 0, 0, 59.0};
        dp::CleanResult r = run(c);
        bool ok = true;
        for (size_t i = 0; i < c.size(); ++i) if (c[i] > 0 && r.values[i] != c[i]) ok = false;
        expect("real values are never changed", ok);
    }
    {   // one 5-minute slot is simply absent from the timestamps
        std::vector<long long> t = grid(5);
        t[3] += 300;
        t[4] += 300;
        dp::CleanResult r = dp::cleanColumn(t, {50, 0, 0, 0, 55});
        expect("nothing is carried across a hole in the time grid", r.values[1] == 50 && r.values[2] == 50 && std::isnan(r.values[3]));
    }
    {   // cleaning only the first k rows must give the same answer as cleaning everything
        std::mt19937 rng(7);
        bool ok = true;
        for (int round = 0; round < 200 && ok; ++round) {
            size_t n = 1 + rng() % 60;
            Col c;
            for (size_t i = 0; i < n; ++i) c.push_back((rng() % 100) < 35 ? 0.0 : static_cast<double>(5 + rng() % 66));
            std::vector<long long> t = grid(n);
            dp::CleanResult full = dp::cleanColumn(t, c);
            for (size_t k = 1; k <= n && ok; ++k) {
                dp::CleanResult part = dp::cleanColumn(std::vector<long long>(t.begin(), t.begin() + k), Col(c.begin(), c.begin() + k));
                ok = same(part.values, Col(full.values.begin(), full.values.begin() + k));
            }
        }
        expect("no look-ahead: cleaning a prefix equals the prefix of the full cleaning", ok);
    }
    {   // timestamp round trip, a leap day, and the weekday of the first METR-LA row
        long long t;
        bool ok = dp::parseTimestamp("2012-03-01 06:00:00", t) && dp::formatTimestamp(t) == "2012-03-01 06:00:00" &&
                  std::string(dp::weekdayName(t)) == "Thursday" && t == 1330581600LL;
        long long leap;
        ok = ok && dp::parseTimestamp("2012-02-29 23:55:00", leap) && dp::formatTimestamp(leap + 300) == "2012-03-01 00:00:00";
        ok = ok && !dp::parseTimestamp("2012-13-01 00:00:00", leap) && !dp::parseTimestamp("not a date", leap);
        expect("timestamps: round trip, leap day, weekday, invalid input rejected", ok);
    }
    {
        expect("formatSpeed: 4 decimals, empty for missing", dp::formatSpeed(62.5) == "62.5000" && dp::formatSpeed(62.666666666) == "62.6667" &&
                                                                dp::formatSpeed(NAN_VALUE) == "" && dp::formatSpeed(0.05) == "0.0500");
    }
    {
        Col c = {50, 50, 50, 0, 50, 51};
        expect("longest identical run ignores missing values", dp::longestIdenticalRun(c) == 3);
    }
    {
        std::vector<std::string> n = {"a", "b", "c"};
        std::vector<dp::Edge> ring = {{"a", "b"}, {"b", "c"}, {"c", "a"}}, chain = {{"a", "b"}, {"b", "c"}};
        expect("strongly connected: a ring is, a chain is not", dp::stronglyConnected(n, ring) && !dp::stronglyConnected(n, chain));
    }
    {
        std::vector<std::string> f = dp::splitLine("a,,c\r");
        expect("splitLine keeps empty fields and drops a trailing carriage return", f.size() == 3 && f[1].empty() && f[2] == "c");
    }

    std::printf("\n%d passed, %d failed\n", passed, failed);
    return failed == 0 ? 0 : 1;
}
