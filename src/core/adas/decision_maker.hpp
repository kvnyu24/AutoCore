#pragma once

#include <memory>
#include "../sensors/sensor_types.hpp"
#include "../vcs/vehicle_control_system.hpp"
#include "perception_system.hpp"
#include "path_planner.hpp"

namespace autocore {
namespace adas {

struct ControlCommand {
    float targetSpeed;
    float targetAcceleration;
    float steeringAngle;
    bool emergencyBrake{false};
};

struct EmergencyEvent {
    std::string type;
    float severity;
    std::chrono::system_clock::time_point timestamp;
    bool requiresImmediateAction{false};
};

struct SceneUnderstanding {
    std::vector<sensors::TrackedObject> objects;
    std::vector<PathSegment> availablePaths;
    float riskLevel;
    bool hasEmergency{false};
};

class DecisionMaker {
public:
    DecisionMaker(std::shared_ptr<vcs::VehicleControlSystem> vcs);
    ~DecisionMaker() = default;

    // Core decision making
    void processScene(const SceneUnderstanding& scene);
    ControlCommand generateControlCommand();
    bool handleEmergency(const EmergencyEvent& event);
    
    // Safety validation
    bool validateAction(const ControlCommand& command);
    float calculateRiskLevel(const SceneUnderstanding& scene);
    
    // State management
    void updateState();
    bool isActionSafe(const ControlCommand& command);
    std::vector<std::string> getActiveWarnings() const;

private:
    std::shared_ptr<vcs::VehicleControlSystem> vcs_;
    SceneUnderstanding currentScene_;
    ControlCommand lastCommand_;
    std::vector<std::string> activeWarnings_;
    
    // Decision making helpers
    ControlCommand optimizeCommand(const ControlCommand& command);
    bool checkSafetyConstraints(const ControlCommand& command);
    void updateRiskAssessment();
    void processWarnings();
    float evaluateActionCost(const ControlCommand& command);
};

} // namespace adas
} // namespace autocore 