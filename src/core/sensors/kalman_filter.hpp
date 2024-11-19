#pragma once

#include <Eigen/Dense>
#include <stdexcept>
#include "sensor_data.hpp"

namespace autocore {
namespace sensors {

/**
 * @brief Kalman Filter implementation for state estimation
 * 
 * Implements a discrete-time Kalman filter for optimal state estimation
 * of linear systems with Gaussian noise.
 */
template<typename State, typename Measurement = State>
class KalmanFilter {
public:
    using Matrix = Eigen::Matrix<State, Eigen::Dynamic, Eigen::Dynamic>;
    using Vector = Eigen::Matrix<State, Eigen::Dynamic, 1>;

    KalmanFilter();
    ~KalmanFilter() = default;

    // State estimation
    void predict();
    void update(const SensorData& measurement);
    StateEstimate getStateEstimate() const;

    // Filter configuration
    void setProcessNoise(const Matrix& Q);
    void setMeasurementNoise(const Matrix& R);
    void setStateTransitionMatrix(const Matrix& F);
    void setMeasurementMatrix(const Matrix& H);
    void setState(const Vector& x0, const Matrix& P0);

    // Error handling
    class KalmanFilterException : public std::runtime_error {
        using std::runtime_error::runtime_error;
    };

private:
    // State variables
    Vector state_;              // Current state estimate
    Matrix covariance_;         // State covariance
    Matrix processNoise_;       // Process noise covariance (Q)
    Matrix measurementNoise_;   // Measurement noise covariance (R)
    Matrix stateTransition_;    // State transition matrix (F)
    Matrix measurementMatrix_;  // Measurement matrix (H)

    // Helper functions
    void initializeMatrices();
    void validateDimensions() const;
    bool isPositiveDefinite(const Matrix& matrix) const;
};

} // namespace sensors
} // namespace autocore
