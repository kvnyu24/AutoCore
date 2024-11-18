
#pragma once

#include <vector>

namespace autocore {
namespace sensors {

struct Position {
    float x;
    float y;
    float z;

    Position() : x(0.0f), y(0.0f), z(0.0f) {}
    Position(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    // Basic vector operations
    Position operator+(const Position& other) const {
        return Position(x + other.x, y + other.y, z + other.z);
    }

    Position operator-(const Position& other) const {
        return Position(x - other.x, y - other.y, z - other.z);
    }

    Position operator*(float scalar) const {
        return Position(x * scalar, y * scalar, z * scalar);
    }

    bool operator==(const Position& other) const {
        return x == other.x && y == other.y && z == other.z;
    }

    // Utility functions
    float distanceTo(const Position& other) const {
        float dx = x - other.x;
        float dy = y - other.y;
        float dz = z - other.z;
        return sqrt(dx*dx + dy*dy + dz*dz);
    }
};

} // namespace sensors
} // namespace autocore
