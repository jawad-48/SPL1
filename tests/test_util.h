// test_util.h -- a tiny test helper shared by the programs in tests/ (no test framework needed).
//
// Usage:   tu::expect("what is being checked", condition);   ...   return tu::summary();
#pragma once

#include <cmath>
#include <cstdio>

namespace tu {

inline int& passedCount() { static int n = 0; return n; }
inline int& failedCount() { static int n = 0; return n; }

inline void expect(const char* name, bool ok) {
    std::printf("%s  %s\n", ok ? "PASS" : "FAIL", name);
    if (ok) ++passedCount(); else ++failedCount();
}

inline bool near(double a, double b, double tolerance = 1e-9) { return std::fabs(a - b) <= tolerance; }

// Prints the totals and returns the process exit code (0 only if every check passed).
inline int summary() {
    std::printf("\n%d passed, %d failed\n", passedCount(), failedCount());
    return failedCount() == 0 ? 0 : 1;
}

}  // namespace tu
