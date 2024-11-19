#pragma once

#include "../common/types.hpp"
#include <vector>

namespace autocore {
namespace autonomous {

struct Waypoint {
    sensors::Position position;
    float speed;
    float heading;
    float curvature;
};

struct Obstacle {
    sensors::Position position;
    float radius;
    float velocity;
    bool isDynamic;
};

} // namespace autonomous
} // namespace autocore 