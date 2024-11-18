#include "trajectory_planner.hpp"
#include <algorithm>

namespace autocore {
namespace autonomous {

TrajectoryPlanner::TrajectoryPlanner(
    std::shared_ptr<sensors::FusionEngine> fusionEngine)
    : fusionEngine_(std::move(fusionEngine)) {
    
    // Reference existing sensor fusion interface
    // See src/core/sensors/fusion_engine.hpp lines 14-16
    initializePlanner();
}

Trajectory TrajectoryPlanner::planTrajectory(
    const Position& currentPos,
    const Position& destination,
    const std::vector<Obstacle>& obstacles) {
    
    // Get current state estimate from fusion engine
    // See src/core/sensors/fusion_engine.hpp lines 19-20
    auto currentState = fusionEngine_->getStateEstimate();
    
    // Generate trajectory
    Trajectory trajectory;
    trajectory.waypoints = generateWaypoints(currentPos, destination);
    trajectory.speed = calculateOptimalSpeed(trajectory.waypoints, obstacles);
    
    // Validate trajectory safety
    if (!validateTrajectory(trajectory, obstacles)) {
        return generateEmergencyTrajectory();
    }
    
    return trajectory;
}

std::vector<Waypoint> TrajectoryPlanner::generateWaypoints(
    const Position& start,
    const Position& end) {
    
    std::vector<Waypoint> waypoints;
    
    // Use A* algorithm for path planning
    auto path = astar_.findPath(start, end);
    
    // Smooth path using cubic spline interpolation
    waypoints = smoothPath(path);
    
    return waypoints;
}

bool TrajectoryPlanner::validateTrajectory(
    const Trajectory& trajectory,
    const std::vector<Obstacle>& obstacles) {
    
    for (const auto& waypoint : trajectory.waypoints) {
        if (!isWaypointSafe(waypoint, obstacles)) {
            return false;
        }
    }
    
    return true;
}

} // namespace autonomous
} // namespace autocore