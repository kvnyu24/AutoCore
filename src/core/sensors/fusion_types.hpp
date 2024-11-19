#pragma once

#include <vector>
#include "../common/types.hpp"
#include "kalman_filter.hpp"

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

} // namespace sensors
} // namespace autocore