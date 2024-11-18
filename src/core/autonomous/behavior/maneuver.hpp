#pragma once

#include <vector>
#include "../planning/trajectory_planner.hpp"
#include "../../sensors/sensor_types.hpp"

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

} // namespace autonomous
} // namespace autocore 