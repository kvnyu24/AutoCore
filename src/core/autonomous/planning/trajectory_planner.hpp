#pragma once

#include <memory>
#include <vector>
#include "../../sensors/fusion_engine.hpp"
#include "../../sensors/sensor_types.hpp"

namespace autocore {
namespace autonomous {

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
