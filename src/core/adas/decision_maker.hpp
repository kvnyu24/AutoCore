#pragma once

#include "perception_system.hpp"
#include "path_planner.hpp"
#include "../vcs/vehicle_control_system.hpp"

namespace autocore {
namespace adas {

class DecisionMaker {
public:
    DecisionMaker(std::shared_ptr<vcs::VehicleControlSystem> vcs);
    
    // Decision making
    void processScene(const SceneUnderstanding& scene);
    ControlCommand generateControlCommand();
    bool handleEmergency(const EmergencyEvent& event);
    
    // Safety checks
    bool validateAction(const ControlCommand& command);
    float calculateRiskLevel(const SceneUnderstanding& scene);

private:
    std::shared_ptr<vcs::VehicleControlSystem> vcs_;
    BehaviorTree behaviorTree_;
    RiskAssessor riskAssessor_;
    
    void updateBehaviorTree(const SceneUnderstanding& scene);
    ControlCommand optimizeCommand(const ControlCommand& command);
};

} // namespace adas
} // namespace autocore 