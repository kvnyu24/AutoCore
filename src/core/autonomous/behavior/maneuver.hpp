#pragma once

#include <vector>
#include "../planning/trajectory_planner.hpp"
#include "../../sensors/sensor_types.hpp"
#include "scene_types.hpp"

namespace autocore {
namespace autonomous {

enum class ManeuverType {
    FOLLOW_LANE,
    CHANGE_LANE_LEFT,
    CHANGE_LANE_RIGHT,
    STOP,
    EMERGENCY_STOP
};

struct Maneuver {
    ManeuverType type;
    Trajectory trajectory;
    float cost;
    float safety_margin;
    bool is_feasible{true};
};

struct NavigationGoal {
    sensors::Position target;
    float desired_speed;
    float arrival_time;
};

class ManeuverPlanner {
public:
    ManeuverPlanner() = default;
    ~ManeuverPlanner() = default;

    // Core planning functions
    std::vector<Maneuver> generateManeuvers(const SceneContext& context);
    Maneuver optimizeManeuver(const Maneuver& maneuver);
    float evaluateManeuver(const Maneuver& maneuver);

private:
    // Helper methods for maneuver generation
    Maneuver generateLaneFollowManeuver(const SceneContext& context);
    Maneuver generateLaneChangeManeuver(const SceneContext& context, bool isLeft);
    Maneuver generateStopManeuver(const SceneContext& context);
    
    // Utility functions
    float calculateManeuverCost(const Maneuver& maneuver);
    float calculateSafetyMargin(const Maneuver& maneuver);
};

} // namespace autonomous
} // namespace autocore 