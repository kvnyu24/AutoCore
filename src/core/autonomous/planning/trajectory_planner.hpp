#pragma once

#include "../autonomous_types.hpp"
#include "../../sensors/sensor_types.hpp"
#include "a_star.hpp"
#include <vector>

namespace autocore {
namespace sensors {
    class FusionEngine;  // Forward declaration
}

namespace autonomous {

struct Trajectory {
    std::vector<Waypoint> waypoints;
    float totalDistance;
    float estimatedTime;

    bool intersectsWith(const Trajectory& other) const {
        for (const auto& waypoint : waypoints) {
            for (const auto& otherWaypoint : other.waypoints) {
                float distance = std::hypot(
                    waypoint.position.x - otherWaypoint.position.x,
                    waypoint.position.y - otherWaypoint.position.y
                );
                if (distance < 2.0f) { // Safety threshold in meters
                    return true;
                }
            }
        }
        return false;
    }
};

class TrajectoryPlanner {
public:
    TrajectoryPlanner(std::shared_ptr<sensors::FusionEngine> fusionEngine);
    ~TrajectoryPlanner() = default;

    // Core trajectory planning
    Trajectory planTrajectory(
        const Position& currentPos,
        const Position& destination,
        const std::vector<Obstacle>& obstacles);

    // Waypoint generation
    std::vector<Waypoint> generateWaypoints(
        const Position& start,
        const Position& end);

    // Trajectory validation and optimization
    bool validateTrajectory(
        const Trajectory& trajectory,
        const std::vector<Obstacle>& obstacles);
    
    // Configuration
    void setSpeedLimits(float minSpeed, float maxSpeed);
    void setAccelerationLimits(float maxAccel, float maxDecel);
    void setSafetyMargins(float lateralMargin, float longitudinalMargin);

private:
    std::shared_ptr<sensors::FusionEngine> fusionEngine_;
    AStar astar_;
    std::vector<Obstacle> obstacles_;
    
    // Planning parameters
    float minSpeed_{0.0f};
    float maxSpeed_{50.0f};  // m/s
    float maxAcceleration_{3.0f};  // m/s^2
    float maxDeceleration_{5.0f};  // m/s^2
    float lateralSafetyMargin_{1.0f};  // meters
    float longitudinalSafetyMargin_{2.0f};  // meters

    // Internal methods
    void initializePlanner();
    std::vector<Waypoint> smoothPath(const std::vector<Position>& path);
    float calculateOptimalSpeed(
        const std::vector<Waypoint>& waypoints,
        const std::vector<Obstacle>& obstacles);
    bool isWaypointSafe(
        const Waypoint& waypoint,
        const std::vector<Obstacle>& obstacles);
    Trajectory generateEmergencyTrajectory();
};

} // namespace autonomous
} // namespace autocore
