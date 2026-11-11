#include "clock_origin.h"

#include <chrono>
#include <ctime>

#include "../tools/data_common.h"

namespace routing {
namespace {

long long naiveSeconds(const std::tm& t) {
    return dp::daysFromCivil(t.tm_year + 1900, t.tm_mon + 1, t.tm_mday) * 86400LL + t.tm_hour * 3600LL + t.tm_min * 60LL + t.tm_sec;
}

long long floorDiv(long long a, long long b) { return a >= 0 ? a / b : -((-a + b - 1) / b); }

}  // namespace

bool localClockSeconds(long long& clockSeconds, int& utcOffsetMinutes) {
    const std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    const std::tm* local = std::localtime(&now);
    if (local == nullptr) return false;
    const std::tm localCopy = *local;              // std::localtime and std::gmtime share one buffer: copy before the next call
    const std::tm* utc = std::gmtime(&now);
    if (utc == nullptr) return false;
    const long long l = naiveSeconds(localCopy);
    clockSeconds = l;
    utcOffsetMinutes = static_cast<int>((l - naiveSeconds(*utc)) / 60);
    return true;
}

int weekdayIndex(long long naiveSecondsValue) {
    return static_cast<int>(((floorDiv(naiveSecondsValue, 86400) % 7) + 7) % 7);
}

bool selectOriginForClock(const std::vector<long long>& times, size_t earliestRow, long long clockSeconds, ClockOrigin& result, size_t windowReadings) {
    if (times.empty() || earliestRow >= times.size() || windowReadings < 1) return false;
    const long long slot = clockSeconds - floorDiv(clockSeconds, 86400) * 86400;
    const long long wantedSlot = slot - slot % dp::STEP_SECONDS;
    const int wantedDay = weekdayIndex(clockSeconds);
    for (int pass = 0; pass < 2; ++pass) {                       // pass 0: same weekday; pass 1: any weekday
        for (size_t r = times.size(); r-- > earliestRow;) {      // newest first, so the first hit is the latest date
            if (r + 1 < windowReadings) break;                   // not enough earlier rows for the forecast window
            const long long t = times[r];
            if (t - floorDiv(t, 86400) * 86400 != wantedSlot) continue;
            if (pass == 0 && weekdayIndex(t) != wantedDay) continue;
            bool regular = true;                                 // every one of the steps inside the window must be exactly 5 minutes
            for (size_t i = r + 1 - windowReadings + 1; i <= r; ++i)
                if (times[i] - times[i - 1] != dp::STEP_SECONDS) { regular = false; break; }
            if (!regular) continue;                              // a hole, a duplicate or a reversed pair: skip this date
            result.row = r;
            result.sameWeekday = (weekdayIndex(t) == wantedDay);
            result.slotSeconds = wantedSlot;
            return true;
        }
    }
    return false;
}

}  // namespace routing
