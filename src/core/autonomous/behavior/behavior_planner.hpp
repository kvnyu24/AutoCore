#pragma once

#include <memory>
#include "../../sensors/fusion_engine.hpp"
#include "../../sensors/slam_engine.hpp"
#include "behavior_state.hpp"
#include "maneuver.hpp"

namespace autocore {
namespace autonomous {

struct PredictedBehavior {
    sensors::TrackedObject object;
    Trajectory trajectory;
    float confidence{0.0f};
    std::chrono::system_clock::time_point predictionTime;
};

class BehaviorPlanner {
public:
    BehaviorPlanner(
        std::shared_ptr<sensors::FusionEngine> fusionEngine,
        std::shared_ptr<sensors::SLAMEngine> slamEngine
    );

    // Behavior planning
    BehaviorState planBehavior(const SceneContext& context);
    Maneuver selectManeuver(const BehaviorState& state);
    bool validateManeuver(const Maneuver& maneuver);
    
    // State management
    void updateState();
    BehaviorState getCurrentState() const;
    std::vector<PredictedBehavior> getPredictedBehaviors() const;

private:
    std::shared_ptr<sensors::FusionEngine> fusionEngine_;
    std::shared_ptr<sensors::SLAMEngine> slamEngine_;
    std::unique_ptr<ManeuverPlanner> maneuverPlanner_;
    
    BehaviorState currentState_;
    std::vector<PredictedBehavior> predictions_;

    const float RISK_THRESHOLD{0.75f};
    
    void updatePredictions();
    float evaluateRisk(const Maneuver& maneuver);
    bool checkFeasibility(const Maneuver& maneuver);
    Trajectory predictTrajectory(const sensors::TrackedObject& object);
};

} // namespace autonomous
} // namespace autocore 