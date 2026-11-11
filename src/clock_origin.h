// clock_origin.h -- choose the forecast origin from the computer's clock (a HISTORICAL REPLAY, not live traffic).
//
// The dataset is METR-LA from 2012. The program reads the computer's local date and time and maps the weekday and the
// time of day onto the dataset: the origin is the LATEST row that (1) is allowed as an origin (not inside the training
// period), (2) falls on the same weekday, (3) has the same time of day, rounded DOWN to the 5-minute grid, and (4) has
// five regular 5-minute steps before it. If no row has the same weekday, the latest row with the same time of day is used
// and the result says so. If nothing matches at all, nothing is substituted and the caller reports an error.
// Only rows up to the origin are ever used as inputs, so no future observation reaches the model.
#pragma once

#include <string>
#include <vector>

namespace routing {

struct ClockOrigin {
    size_t row = 0;              // the selected origin row
    bool sameWeekday = true;     // false: no row had the clock's weekday, so the weekday was ignored
    long long slotSeconds = 0;   // the clock's time of day rounded down to 5 minutes (seconds after midnight)
};

// The computer's local date and time as "naive" seconds (same scale as the dataset timestamps: seconds since 1970-01-01
// with no time-zone shift), and the local offset from UTC in minutes (Bangladesh: +360). Returns false if the operating
// system cannot convert the current time (nothing is guessed in that case).
bool localClockSeconds(long long& clockSeconds, int& utcOffsetMinutes);

// 0 = Thursday ... 6 = Wednesday, matching dp::weekdayName.
int weekdayIndex(long long naiveSeconds);

// Picks the origin. `times` = dataset timestamps (increasing), `earliestRow` = first row allowed as an origin,
// `clockSeconds` = the clock as naive seconds, `windowReadings` = how many readings the forecast needs (6). A candidate row r
// qualifies only if EACH of the windowReadings-1 steps before it is exactly 5 minutes (a missing timestamp, a duplicate or a
// reversed pair anywhere in the window disqualifies it, even if the first and last timestamps happen to be 25 minutes apart).
// Returns false (and leaves `result` untouched) if no row qualifies.
bool selectOriginForClock(const std::vector<long long>& times, size_t earliestRow, long long clockSeconds, ClockOrigin& result, size_t windowReadings = 6);

}  // namespace routing
