#pragma once

#include <vector>
#include "position.hpp"

namespace autocore {
namespace sensors {

struct Feature {
    Position position;
    float confidence;
    int id;
    std::vector<float> descriptor;
};

struct FusionParameters {
    float processNoise;
    float measurementNoise;
    float confidenceThreshold;
    int maxFeatures;
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