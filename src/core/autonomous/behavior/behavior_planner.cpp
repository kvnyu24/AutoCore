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
    // Reference to FusionEngine interface:
    // See src/core/sensors/fusion_engine.hpp lines 18-19
    auto stateEstimate = fusionEngine_->getStateEstimate();
    
    // Get current map and position from SLAM
    // Reference to SLAMEngine interface:
    // See src/core/sensors/slam_engine.hpp lines 17-18
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
            float score = evaluateManeuver(maneuver);
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
    // Get tracked objects from fusion engine
    auto stateEstimate = fusionEngine_->getStateEstimate();
    
    predictions_.clear();
    for (const auto& object : stateEstimate.trackedObjects) {
        PredictedBehavior prediction;
        prediction.object = object;
        prediction.trajectory = predictTrajectory(object);
        predictions_.push_back(prediction);
    }
}

} // namespace autonomous
} // namespace autocore 