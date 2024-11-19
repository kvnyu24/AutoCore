#pragma once

#include "../sensors/sensor_types.hpp"
#include <chrono>

namespace autocore {
namespace autonomous {

struct SceneContext {
    std::chrono::system_clock::time_point timestamp;
    std::vector<sensors::Object> detectedObjects;
    sensors::LaneInfo laneInfo;
    sensors::Position currentPosition;
    sensors::Velocity currentVelocity;
    
    // Add any other context information needed
};

} // namespace autonomous
} // namespace autocore 