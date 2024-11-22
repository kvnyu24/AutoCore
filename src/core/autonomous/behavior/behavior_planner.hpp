#pragma once

#include <memory>
#include "../../sensors/fusion_engine.hpp"
#include "../../sensors/slam_engine.hpp"
#include "behavior_state.hpp"
#include "maneuver.hpp"
#include "scene_context.hpp"
#include "behavior_state.hpp"
#include "../common/types.hpp"
#include "../autonomous_types.hpp"
#include "../sensors/sensor_types.hpp"
#include "../planning/trajectory_planner.hpp"

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
    BehaviorState planBehavior(const autonomous::SceneContext& context);
    autonomous::Maneuver selectManeuver(const BehaviorState& state);
    bool validateManeuver(const autonomous::Maneuver& maneuver);
    
    // State management
    void updateState();
    BehaviorState getCurrentState() const;
    std::vector<PredictedBehavior> getPredictedBehaviors() const;

private:
    std::shared_ptr<sensors::FusionEngine> fusionEngine_;
    std::shared_ptr<sensors::SLAMEngine> slamEngine_;
    std::unique_ptr<autonomous::ManeuverPlanner> maneuverPlanner_;
    
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