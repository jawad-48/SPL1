#include "linear_regression.h"

#include <algorithm>
#include <cmath>

namespace baseline {

// ------------------------------------------------------------------ checks shared by fit()
static void validateTrainingData(const Matrix& X, const std::vector<double>& y) {
    if (X.empty() || X.size() != y.size()) throw std::invalid_argument("X and y must be non-empty and have one row per target");
    const size_t d = X[0].size();
    if (d == 0) throw std::invalid_argument("samples need at least one feature");
    for (const auto& row : X) if (row.size() != d) throw std::invalid_argument("all samples must have the same number of features");
    if (X.size() < d + 2) throw std::invalid_argument("too few training samples for this many features");
}

// ------------------------------------------------------------------ standardiser
void Standardizer::fit(const Matrix& X, const std::vector<double>& y) {
    const size_t n = X.size(), d = X[0].size();
    featureMean.assign(d, 0.0);
    featureStd.assign(d, 1.0);
    for (const auto& row : X) for (size_t k = 0; k < d; ++k) featureMean[k] += row[k] / static_cast<double>(n);
    for (size_t k = 0; k < d; ++k) {
        double ss = 0;
        for (const auto& row : X) ss += (row[k] - featureMean[k]) * (row[k] - featureMean[k]);
        const double s = std::sqrt(ss / static_cast<double>(n));
        featureStd[k] = s < 1e-12 ? 1.0 : s;
    }
    targetMean = 0;
    for (double v : y) targetMean += v / static_cast<double>(n);
    double ss = 0;
    for (double v : y) ss += (v - targetMean) * (v - targetMean);
    const double s = std::sqrt(ss / static_cast<double>(n));
    targetStd = s < 1e-12 ? 1.0 : s;
}

std::vector<double> Standardizer::scaleFeatures(const std::vector<double>& x) const {
    if (x.size() != featureMean.size()) throw std::invalid_argument("feature vector has the wrong length");
    std::vector<double> out(x.size());
    for (size_t k = 0; k < x.size(); ++k) out[k] = (x[k] - featureMean[k]) / featureStd[k];
    return out;
}

// ------------------------------------------------------------------ the exact solution (for checking)
bool solveNormalEquation(const Matrix& X, const std::vector<double>& y, std::vector<double>& weights, double& bias) {
    const size_t n = X.size(), d = X[0].size(), m = d + 1;          // the extra unknown is the bias
    std::vector<std::vector<double>> a(m, std::vector<double>(m + 1, 0.0));   // augmented matrix [A | b]
    for (size_t i = 0; i < n; ++i) {
        for (size_t p = 0; p < m; ++p) {
            const double xp = p < d ? X[i][p] : 1.0;
            for (size_t q = 0; q < m; ++q) a[p][q] += xp * (q < d ? X[i][q] : 1.0);
            a[p][m] += xp * y[i];
        }
    }
    for (size_t col = 0; col < m; ++col) {                          // Gaussian elimination with partial pivoting
        size_t pivot = col;
        for (size_t r = col + 1; r < m; ++r) if (std::fabs(a[r][col]) > std::fabs(a[pivot][col])) pivot = r;
        if (std::fabs(a[pivot][col]) < 1e-10) return false;
        std::swap(a[col], a[pivot]);
        for (size_t r = 0; r < m; ++r) {
            if (r == col) continue;
            const double factor = a[r][col] / a[col][col];
            for (size_t c = col; c <= m; ++c) a[r][c] -= factor * a[col][c];
        }
    }
    weights.assign(d, 0.0);
    for (size_t k = 0; k < d; ++k) weights[k] = a[k][m] / a[k][k];
    bias = a[d][m] / a[d][d];
    return true;
}

static double meanSquaredError(const Matrix& X, const std::vector<double>& y, const std::vector<double>& w, double b) {
    double sum = 0;
    for (size_t i = 0; i < X.size(); ++i) {
        double p = b;
        for (size_t k = 0; k < w.size(); ++k) p += w[k] * X[i][k];
        sum += (p - y[i]) * (p - y[i]);
    }
    return sum / static_cast<double>(X.size());
}

// ------------------------------------------------------------------ the model
GdResult LinearModel::fit(const Matrix& X, const std::vector<double>& y, const GdOptions& options) {
    validateTrainingData(X, y);
    const size_t n = X.size(), d = X[0].size();

    // 1. Learn the scaling from the training data only, then scale the training data.
    scaler_.fit(X, y);
    Matrix Xs;
    Xs.reserve(n);
    std::vector<double> ys(n);
    for (size_t i = 0; i < n; ++i) {
        Xs.push_back(scaler_.scaleFeatures(X[i]));
        ys[i] = (y[i] - scaler_.targetMean) / scaler_.targetStd;
    }

    // 2. Batch gradient descent on  loss = 0.5 * mean( (prediction - target)^2 ).
    //    With standardised inputs the largest curvature is at most d, so a step of at most 1/d never overshoots.
    const double rate = std::min(options.learningRate, 1.0 / static_cast<double>(d));
    weights_.assign(d, 0.0);
    bias_ = 0.0;
    GdResult result;
    double previousLoss = 0;
    std::vector<double> gradient(d);
    for (int epoch = 1; epoch <= options.maxEpochs; ++epoch) {
        std::fill(gradient.begin(), gradient.end(), 0.0);
        double gradientBias = 0, loss = 0;
        for (size_t i = 0; i < n; ++i) {
            double prediction = bias_;
            for (size_t k = 0; k < d; ++k) prediction += weights_[k] * Xs[i][k];
            const double residual = prediction - ys[i];
            loss += 0.5 * residual * residual / static_cast<double>(n);
            for (size_t k = 0; k < d; ++k) gradient[k] += residual * Xs[i][k] / static_cast<double>(n);
            gradientBias += residual / static_cast<double>(n);
        }
        // Stop when the fit is essentially perfect, or when one more epoch improves the loss by less than
        // `tolerance` of its own size (a RELATIVE test, so it also works when the loss itself is tiny).
        if (loss < 1e-14 || (epoch > 1 && std::fabs(previousLoss - loss) <= options.tolerance * previousLoss)) {
            result.converged = true;
            break;
        }
        previousLoss = loss;
        for (size_t k = 0; k < d; ++k) weights_[k] -= rate * gradient[k];
        bias_ -= rate * gradientBias;
        result.epochs = epoch;
    }
    const double mseStd = meanSquaredError(Xs, ys, weights_, bias_);
    result.finalLoss = 0.5 * mseStd;
    trainRmseGd_ = std::sqrt(mseStd) * scaler_.targetStd;            // back to mph

    // 3. Independent check: the exact least-squares fit on the same training data.
    std::vector<double> olsWeights;
    double olsBias = 0;
    olsAvailable_ = solveNormalEquation(Xs, ys, olsWeights, olsBias);
    trainRmseOls_ = olsAvailable_ ? std::sqrt(meanSquaredError(Xs, ys, olsWeights, olsBias)) * scaler_.targetStd : 0.0;

    fitted_ = true;
    return result;
}

double LinearModel::predict(const std::vector<double>& x) const {
    if (!fitted_) throw std::logic_error("predict() called before fit()");
    const std::vector<double> xs = scaler_.scaleFeatures(x);
    double p = bias_;
    for (size_t k = 0; k < xs.size(); ++k) p += weights_[k] * xs[k];
    return p * scaler_.targetStd + scaler_.targetMean;                // undo the target scaling
}

std::vector<double> LinearModel::coefficients() const {
    if (!fitted_) throw std::logic_error("coefficients() called before fit()");
    std::vector<double> c(weights_.size());
    for (size_t k = 0; k < c.size(); ++k) c[k] = weights_[k] * scaler_.targetStd / scaler_.featureStd[k];
    return c;
}

double LinearModel::intercept() const {
    if (!fitted_) throw std::logic_error("intercept() called before fit()");
    double b = scaler_.targetMean + bias_ * scaler_.targetStd;
    const std::vector<double> c = coefficients();
    for (size_t k = 0; k < c.size(); ++k) b -= c[k] * scaler_.featureMean[k];
    return b;
}

}  // namespace baseline
