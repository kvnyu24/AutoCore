#include "behavior_planner.hpp"
#include <algorithm>

namespace autocore {
namespace autonomous {

BehaviorPlanner::BehaviorPlanner(
    std::shared_ptr<sensors::FusionEngine> fusionEngine,
    std::shared_ptr<sensors::SLAMEngine> slamEngine)
    : fusionEngine_(std::move(fusionEngine))
    , slamEngine_(std::move(slamEngine))
    , maneuverPlanner_(std::make_unique<ManeuverPlanner>()) {
}

BehaviorState BehaviorPlanner::planBehavior(const SceneContext& context) {
    // Get current state estimate from fusion engine
    auto stateEstimate = fusionEngine_->getStateEstimate();
    
    // Get current map and position from SLAM
    auto currentPosition = slamEngine_->getEstimatedPosition();
    auto currentMap = slamEngine_->getCurrentMap();
    
    // Update behavior predictions
    updatePredictions();
    
    // Generate possible maneuvers
    auto possibleManeuvers = maneuverPlanner_->generateManeuvers(context);
    
    // Select best maneuver based on safety and efficiency
    Maneuver bestManeuver;
    float bestScore = std::numeric_limits<float>::lowest();
    
    for (const auto& maneuver : possibleManeuvers) {
        if (validateManeuver(maneuver)) {
            // Use maneuverPlanner's evaluation method
            float score = maneuverPlanner_->evaluateManeuver(maneuver);
            if (score > bestScore) {
                bestScore = score;
                bestManeuver = maneuver;
            }
        }
    }
    
    // Update current state with selected maneuver
    currentState_.maneuver = bestManeuver;
    currentState_.timestamp = context.timestamp;
    
    return currentState_;
}

bool BehaviorPlanner::validateManeuver(const Maneuver& maneuver) {
    // Check physical feasibility
    if (!checkFeasibility(maneuver)) {
        return false;
    }
    
    // Evaluate risk level
    float risk = evaluateRisk(maneuver);
    if (risk > RISK_THRESHOLD) {
        return false;
    }
    
    // Check for collisions with predicted trajectories
    for (const auto& prediction : predictions_) {
        if (maneuver.trajectory.intersectsWith(prediction.trajectory)) {
            return false;
        }
    }
    
    return true;
}

void BehaviorPlanner::updatePredictions() {
    auto stateEstimate = fusionEngine_->getStateEstimate();
    
    predictions_.clear();
    for (const auto& object : stateEstimate.trackedObjects) {
        // Create TrackedObject properly
        sensors::TrackedObject trackedObj;
        trackedObj.id = object.id;
        trackedObj.position = object.position;
        trackedObj.velocity = std::abs(object.velocity);  // Convert scalar velocity to double
        trackedObj.heading = object.heading;

        PredictedBehavior prediction{
            trackedObj,                              // Properly constructed TrackedObject
            predictTrajectory(trackedObj),           // Trajectory
            0.8f,                                    // confidence
            std::chrono::system_clock::now()         // predictionTime
        };
        predictions_.push_back(prediction);
    }
}

Trajectory BehaviorPlanner::predictTrajectory(const sensors::TrackedObject& object) {
    Trajectory trajectory;
    const float predictionTime = 5.0f;  // 5 seconds prediction horizon
    const float timeStep = 0.1f;        // 100ms time step
    
    // Initialize trajectory metrics
    trajectory.totalDistance = 0.0f;
    trajectory.estimatedTime = predictionTime;
    
    // Previous waypoint for distance calculation
    Waypoint prevWaypoint;
    bool isFirst = true;

    for (float t = 0; t < predictionTime; t += timeStep) {
        Waypoint waypoint;
        
        // Calculate position based on current velocity and heading
        float distance = object.velocity * t;
        waypoint.position.x = object.position.x + distance * std::cos(object.heading);
        waypoint.position.y = object.position.y + distance * std::sin(object.heading);
        waypoint.position.z = object.position.z;  // Maintain same z-coordinate
        
        // Set waypoint properties
        waypoint.speed = object.velocity;  // Assume constant velocity
        waypoint.heading = object.heading;
        waypoint.curvature = 0.0f;  // Could be calculated based on path

        // Calculate cumulative distance
        if (!isFirst) {
            float segmentDistance = std::hypot(
                waypoint.position.x - prevWaypoint.position.x,
                waypoint.position.y - prevWaypoint.position.y
            );
            trajectory.totalDistance += segmentDistance;
        }

        trajectory.waypoints.push_back(waypoint);
        prevWaypoint = waypoint;
        isFirst = false;
    }
    
    return trajectory;
}

} // namespace autonomous
} // namespace autocore 