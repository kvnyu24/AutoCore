#pragma once

#include <memory>
#include "../sensors/sensor_manager.hpp"
#include "../motor/motor_controller.hpp"
#include "../vcs/vehicle_control_system.hpp"
#include "../perception_system.hpp"
#include "path_planner.hpp"
#include "decision_maker.hpp"
#include "../common/types.hpp"
namespace autocore {
namespace adas {

enum class ADASFeatureState {
    DISABLED,
    ENABLED,
    ERROR,
    INITIALIZING
};

struct ADASStatus {
    bool adaptiveCruiseControlActive{false};
    bool laneKeepAssistActive{false};
    bool emergencyBrakingActive{false};
    float currentFollowingDistance{0.0f};
    float targetFollowingDistance{0.0f};
    std::vector<std::string> activeWarnings;
};

struct SafetyConfig {
    float minFollowingDistance{2.0f};  // seconds
    float maxFollowingDistance{4.0f};  // seconds
    float emergencyBrakeThreshold{5.0f};  // meters
    float laneDeviationThreshold{0.5f};  // meters
};

class ADASManager {
public:
    ADASManager(
        std::shared_ptr<sensors::SensorManager> sensorManager,
        std::shared_ptr<motor::MotorController> motorController,
        std::shared_ptr<vcs::VehicleControlSystem> vcs
    );
    
    // ADAS features control
    void enableAdaptiveCruiseControl(bool enable);
    void enableLaneKeepAssist(bool enable);
    void enableEmergencyBraking(bool enable);
    void setFollowingDistance(float distance);
    
    // System status
    ADASStatus getStatus() const;
    std::vector<diagnostics::SafetyAlert> getActiveWarnings() const;
    
    // Core functionality
    void update();
    void processEnvironment();
    bool emergencyStop();

private:
    std::shared_ptr<sensors::SensorManager> sensorManager_;
    std::shared_ptr<motor::MotorController> motorController_;
    std::shared_ptr<vcs::VehicleControlSystem> vcs_;
    
    std::unique_ptr<PerceptionSystem> perceptionSystem_;
    std::unique_ptr<PathPlanner> pathPlanner_;
    std::unique_ptr<DecisionMaker> decisionMaker_;
    
    std::unordered_map<std::string, ADASFeatureState> featureStates_;
    SafetyConfig safetyConfig_;
    ADASStatus currentStatus_;
    
    // Internal methods
    void updatePerception();
    void planPath();
    void executeDecisions();
    bool validateSafety();
    void updateFeatureStates();
    void processWarnings();
    bool checkSystemReadiness() const;
    void handleEmergencyEvents();
    float calculateSafeFollowingDistance() const;
    bool validateFeatureTransition(const std::string& feature, bool enable);
};

} // namespace adas
} // namespace autocore 