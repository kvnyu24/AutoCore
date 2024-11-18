#pragma once

#include <vector>
#include "position.hpp"

namespace autocore {
namespace sensors {

struct Feature {
    double x;
    double y;
    double confidence;
    uint32_t id;
};

struct FusionParameters {
    double sensorWeight;
    double timeWindow;
    double confidenceThreshold;
};

struct TrackedObject {
    uint32_t id;
    Position position;
    double velocity;
    double heading;
    
    TrackedObject() = default;
    TrackedObject(const Feature& feature) {
        // Initialize from feature
    }
};

template<typename State, typename Measurement>
class KalmanFilter {
public:
    void predict();
    void update(const Measurement& measurement);
    State getState() const;
    
private:
    State state_;
    // Add Kalman filter implementation details
};

} // namespace sensors
} // namespace autocore