#pragma once

#include <chrono>
#include <vector>
#include "maneuver.hpp"

namespace autocore {
namespace autonomous {

struct SceneContext {
    std::chrono::system_clock::time_point timestamp;
    std::vector<TrackedObject> objects;
    VehicleState currentState;
    NavigationGoal goal;
};

struct BehaviorState {
    Maneuver maneuver;
    std::chrono::system_clock::time_point timestamp;
    float riskLevel;
    bool isValid{true};
};

struct PredictedBehavior {
    TrackedObject object;
    Trajectory trajectory;
    float confidence;
};

} // namespace autonomous
} // namespace autocore 