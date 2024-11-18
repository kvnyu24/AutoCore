#pragma once

#include <vector>

namespace autocore {
namespace sensors {

struct Position {
    float x;
    float y;
    float z;
};

struct Velocity {
    float vx;
    float vy;
    float vz;
};

enum class ObjectType {
    VEHICLE,
    PEDESTRIAN,
    CYCLIST,
    UNKNOWN
};

struct Path {
    std::vector<Position> waypoints;
    std::vector<float> timestamps;
};

} // namespace sensors
} // namespace autocore
