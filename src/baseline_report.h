// baseline_report.h -- turns a BaselineResult into a plain-text evaluation report.
//
// The report contains no dates, user names or machine details, so the same data and settings always
// produce the same text byte for byte. That is what lets a committed report be checked against a re-run.
#pragma once

#include <string>

#include "baseline_pipeline.h"

namespace baseline {

std::string formatReport(const BaselineResult& result);

}  // namespace baseline
