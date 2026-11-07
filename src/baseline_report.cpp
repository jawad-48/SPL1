#include "baseline_report.h"

namespace baseline {

static std::string joinSensors(const std::vector<std::string>& ids) {
    std::string s;
    for (size_t i = 0; i < ids.size(); ++i) s += (i ? ", " : "") + ids[i];
    return s;
}

static std::string num(double v) { return std::isnan(v) ? "     n/a" : dp::fmt("%8.4f", v); }

// "+12.3%" / "-4.0%" : how far the Linear Regression error is from the persistence error
static std::string change(double linear, double persistence) {
    if (std::isnan(linear) || std::isnan(persistence) || persistence == 0) return "     n/a";
    return dp::fmt("%+7.1f%%", 100.0 * (linear - persistence) / persistence);
}

static void resultRow(std::string& out, const std::string& label, const Metrics& lr, const Metrics& pe) {
    out += dp::fmt("  %-8s %5zu |%s %s |%s %s |%s\n", label.c_str(), lr.n, num(lr.mae).c_str(), num(lr.rmse).c_str(),
                   num(pe.mae).c_str(), num(pe.rmse).c_str(), change(lr.rmse, pe.rmse).c_str());
}

static std::string timeOf(const TrafficSeries& s, size_t row) { return dp::formatTimestamp(s.times[row]); }

std::string formatReport(const BaselineResult& r) {
    const TrafficSeries& s = r.series;
    const BaselineConfig& c = r.config;
    const int minutes = c.features.horizonSteps * 5;
    const double days = static_cast<double>(s.rows()) * 5.0 / 1440.0;
    std::string out;
    auto line = [&](const std::string& text = "") { out += text + "\n"; };

    line("LINEAR REGRESSION BASELINE REPORT");
    line("=================================");
    line("Data file   : " + c.dataPath);
    line(dp::fmt("Rows        : %zu (%s -> %s, %.2f days, 5-minute steps)", s.rows(), timeOf(s, 0).c_str(), timeOf(s, s.rows() - 1).c_str(), days));
    line(dp::fmt("Sensors     : %s (%zu sensors, %zu extra columns ignored)", joinSensors(s.sensors).c_str(), s.sensors.size(), s.ignoredColumns));
    line(dp::fmt("Target      : speed of the same sensor %d steps (%d min) after the forecast time", c.features.horizonSteps, minutes));
    line(dp::fmt("Features    : %d lags = speed at t, t-1, ..., t-%d (the last %d min); nothing later than t", c.features.lags, c.features.lags - 1, c.features.lags * 5));
    line(dp::fmt("Model       : one Linear Regression per sensor, batch gradient descent (at most %d epochs, tolerance %g);", c.gd.maxEpochs, c.gd.tolerance));
    line("              inputs and target standardised with TRAINING statistics only");
    line("Baseline    : persistence = the most recent reading carried forward");
    line("Randomness  : none (weights start at zero), so every run gives the same numbers");
    line("Unit        : mph (METR-LA convention); MAE and RMSE, lower is better");
    line();

    // ---- warnings that follow from the numbers themselves
    std::vector<std::string> warnings;
    if (days < 7.0)
        warnings.push_back(dp::fmt("The data spans only %.2f days (< 7) and the test part is a single stretch of %.2f hours, so these numbers\n"
                                   "    check that the pipeline works; they are NOT a reliable estimate of forecasting accuracy.",
                                   days, static_cast<double>(r.split.rows - r.split.validationEnd) * 5.0 / 60.0));
    for (const SensorResult& sr : r.sensors) {
        if (!sr.gd.converged) warnings.push_back("sensor " + sr.sensorId + ": gradient descent stopped at the epoch limit before converging.");
        if (sr.validationSamples == 0 || sr.testSamples == 0) warnings.push_back("sensor " + sr.sensorId + ": a held-out part has no usable samples.");
        if (sr.droppedMissing > 0 || sr.droppedIrregular > 0)
            warnings.push_back(dp::fmt("sensor %s: %zu candidate samples dropped (%zu missing values, %zu time-grid holes).", sr.sensorId.c_str(), sr.droppedMissing + sr.droppedIrregular, sr.droppedMissing, sr.droppedIrregular));
    }
    warnings.push_back("Values filled by the cleaning policy (last reading carried forward up to 30 min) look like real readings in the cleaned\n"
                       "    file, so they are used as inputs and targets. Carried-forward targets are easy for persistence to predict.");
    line("NOTES AND WARNINGS");
    for (const std::string& w : warnings) line("  * " + w);
    line();

    // ---- split
    line("SPLIT (chronological; a sample belongs to the part containing its TARGET row)");
    const SplitBoundaries& b = r.split;
    line(dp::fmt("  train       rows [%zu, %zu)   %s -> %s", size_t(0), b.trainEnd, timeOf(s, 0).c_str(), timeOf(s, b.trainEnd - 1).c_str()));
    line(dp::fmt("  validation  rows [%zu, %zu)   %s -> %s", b.trainEnd, b.validationEnd, timeOf(s, b.trainEnd).c_str(), timeOf(s, b.validationEnd - 1).c_str()));
    line(dp::fmt("  test        rows [%zu, %zu)   %s -> %s", b.validationEnd, b.rows, timeOf(s, b.validationEnd).c_str(), timeOf(s, b.rows - 1).c_str()));
    line(dp::fmt("  fractions   train %.2f / validation %.2f / test %.2f of the rows; validation is reported but nothing was tuned on it", c.trainFraction, c.validationFraction, 1.0 - c.trainFraction - c.validationFraction));
    line();

    line("SAMPLES PER SENSOR");
    line("  sensor   candidates  usable  dropped(missing)  dropped(grid)  train  validation  test");
    for (const SensorResult& sr : r.sensors)
        line(dp::fmt("  %-7s  %10zu  %6zu  %16zu  %13zu  %5zu  %10zu  %4zu", sr.sensorId.c_str(), sr.candidates, sr.usable, sr.droppedMissing, sr.droppedIrregular, sr.trainSamples, sr.validationSamples, sr.testSamples));
    line();

    line("TRAINING CHECK (training RMSE in mph: gradient descent vs the exact least-squares solution)");
    line("  sensor   epochs  converged  gradient-descent    exact");
    for (const SensorResult& sr : r.sensors)
        line(dp::fmt("  %-7s  %6d  %-9s  %16.4f  %s", sr.sensorId.c_str(), sr.gd.epochs, sr.gd.converged ? "yes" : "NO", sr.trainRmseGd,
                     sr.olsAvailable ? dp::fmt("%7.4f", sr.trainRmseOls).c_str() : "   n/a (singular)"));
    line();

    line("COEFFICIENTS (mph; prediction = intercept + sum of coefficient * speed(t - k))");
    std::string head = "  sensor   intercept";
    for (int k = 0; k < c.features.lags; ++k) head += dp::fmt("   lag_%d", k);
    line(head);
    for (const SensorResult& sr : r.sensors) {
        std::string row = dp::fmt("  %-7s  %9.4f", sr.sensorId.c_str(), sr.intercept);
        for (double v : sr.coefficients) row += dp::fmt("  %7.4f", v);
        line(row);
    }
    line("  (neighbouring lags are strongly correlated, so individual coefficients are not reliable on their own)");
    line();

    const char* parts[2] = {"VALIDATION", "TEST"};
    for (int p = 0; p < 2; ++p) {
        line(std::string("RESULTS ON ") + parts[p] + " DATA (held-out; mph)");
        line("                  Linear Regression   |    Persistence      | RMSE change");
        line("  sensor       n       MAE      RMSE   |     MAE      RMSE   | vs persist.");
        for (const SensorResult& sr : r.sensors)
            resultRow(out, sr.sensorId, p == 0 ? sr.linearValidation : sr.linearTest, p == 0 ? sr.persistenceValidation : sr.persistenceTest);
        resultRow(out, "ALL", p == 0 ? r.overallLinearValidation : r.overallLinearTest, p == 0 ? r.overallPersistenceValidation : r.overallPersistenceTest);
        line();
    }
    line("RMSE change = (Linear Regression RMSE - persistence RMSE) / persistence RMSE; negative means Linear Regression had the smaller error.");
    return out;
}

}  // namespace baseline
