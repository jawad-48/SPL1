// linear_regression.h -- Linear Regression trained by Batch Gradient Descent (standard library only).
//
// Idea:   prediction = w1*x1 + w2*x2 + ... + wd*xd + b
// Training repeatedly nudges every weight a little in the direction that lowers the mean squared error
// over ALL training samples (that is what "batch" means) until the error stops improving.
//
// Leakage rule: everything that is "learned from data" -- the mean and spread used to standardise the
// inputs and the target, and the weights -- is computed from the TRAINING samples handed to fit().
// Validation and test samples are only ever transformed with those fixed numbers.
#pragma once

#include <stdexcept>
#include <vector>

namespace baseline {

typedef std::vector<std::vector<double>> Matrix;   // one row per sample, one column per feature

// z-score scaling: (value - mean) / standard deviation. Gradient descent needs inputs on similar scales.
struct Standardizer {
    std::vector<double> featureMean, featureStd;
    double targetMean = 0, targetStd = 1;
    // Learns the numbers from X and y ONLY. A constant column gets std = 1 (so it becomes all zeros, not NaN).
    void fit(const Matrix& X, const std::vector<double>& y);
    std::vector<double> scaleFeatures(const std::vector<double>& x) const;   // uses the stored numbers, never refits
};

struct GdOptions {
    double learningRate = 0.1;     // step size; automatically capped at 1/(number of features), which is always stable
    int maxEpochs = 20000;         // one epoch = one pass over all training samples = one weight update
    double tolerance = 1e-12;      // stop when one epoch changes the loss by less than this fraction of the loss
};

struct GdResult {
    int epochs = 0;                // weight updates actually performed
    bool converged = false;        // true if it stopped because the loss stopped changing (not because of maxEpochs)
    double finalLoss = 0;          // 0.5 * mean squared error on the standardised training data
};

class LinearModel {
public:
    // Learns the scaling and the weights from (X, y). Throws std::invalid_argument for empty/ragged data or
    // fewer than d+2 samples.
    GdResult fit(const Matrix& X, const std::vector<double>& y, const GdOptions& options);

    // Prediction for one raw (unscaled) feature vector, returned in the original unit (mph).
    double predict(const std::vector<double>& x) const;

    bool isFitted() const { return fitted_; }
    // The same model written in raw units:  prediction = intercept + sum coefficient[k] * x[k]
    std::vector<double> coefficients() const;
    double intercept() const;

    // Diagnostics on the TRAINING data: gradient descent versus the exact least-squares solution.
    double trainRmseGradientDescent() const { return trainRmseGd_; }
    bool olsAvailable() const { return olsAvailable_; }
    double trainRmseOls() const { return trainRmseOls_; }

private:
    bool fitted_ = false;
    Standardizer scaler_;
    std::vector<double> weights_;      // in standardised space
    double bias_ = 0;
    double trainRmseGd_ = 0, trainRmseOls_ = 0;
    bool olsAvailable_ = false;
};

// Exact least squares by solving the normal equations (Gaussian elimination with partial pivoting).
// Returns false if the system is singular. Used only to CHECK gradient descent, never to train the baseline.
bool solveNormalEquation(const Matrix& X, const std::vector<double>& y, std::vector<double>& weights, double& bias);

}  // namespace baseline
