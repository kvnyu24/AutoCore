#pragma once

#include <chrono>
#include <vector>
#include "maneuver.hpp"
#include "../../sensors/fusion_types.hpp"
#include "../../vcs/vehicle_control_system.hpp"
#include "../../sensors/sensor_types.hpp"

namespace autocore {
namespace autonomous {

struct SceneContext {
    std::chrono::system_clock::time_point timestamp;
    std::vector<sensors::TrackedObject> objects;
    vcs::VehicleState currentState;
    NavigationGoal goal;
};

struct BehaviorState {
    Maneuver maneuver;
    std::chrono::system_clock::time_point timestamp;
    float riskLevel{0.0f};
    bool isValid{true};
};

struct PredictedBehavior {
    sensors::TrackedObject object;
    Trajectory trajectory;
    float confidence{0.0f};
    std::chrono::system_clock::time_point predictionTime;
};

} // namespace autonomous
} // namespace autocore 