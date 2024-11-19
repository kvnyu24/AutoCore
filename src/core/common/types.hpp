#pragma once

#include <vector>
#include <chrono>

namespace autocore {
namespace common {

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

enum class ObjectType {
    VEHICLE,
    PEDESTRIAN,
    CYCLIST,
    UNKNOWN
};

struct Velocity {
    float vx{0.0f};
    float vy{0.0f};
    float vz{0.0f};
};

struct Path {
    std::vector<Position> waypoints;
    std::vector<float> timestamps;
};

struct DiagnosticData {
    // Consolidated diagnostic data definition
};

struct SceneContext {
    std::chrono::system_clock::time_point timestamp;
    std::vector<sensors::TrackedObject> objects;
    vcs::VehicleState currentState;
    NavigationGoal goal;};
};

struct PredictedBehavior {
    // Consolidated behavior prediction definition
};

// Type aliases for backward compatibility
namespace sensors {
    using Position = common::Position;
    using Velocity = common::Velocity;
    using Path = common::Path;
}

namespace autonomous {
    using Position = common::Position;
    using ObjectType = common::ObjectType;
}

namespace adas {
    using Position = common::Position;
    using ObjectType = common::ObjectType;
}
} // namespace common
} // namespace autocore
