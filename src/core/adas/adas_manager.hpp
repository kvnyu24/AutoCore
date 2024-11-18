#pragma once

#include <memory>
#include "../sensors/sensor_manager.hpp"
#include "../motor/motor_controller.hpp"
#include "../vcs/vehicle_control_system.hpp"
#include "perception_system.hpp"
#include "path_planner.hpp"
#include "decision_maker.hpp"

namespace evlib {
namespace adas {

class ADASManager {
public:
    ADASManager(
        std::shared_ptr<sensors::SensorManager> sensorManager,
        std::shared_ptr<motor::MotorController> motorController,
        std::shared_ptr<vcs::VehicleControlSystem> vcs
    );

    // ADAS features
    void enableAdaptiveCruiseControl(bool enable);
    void enableLaneKeepAssist(bool enable);
    void enableEmergencyBraking(bool enable);
    void setFollowingDistance(float distance);
    
    // System status
    ADASStatus getStatus() const;
    std::vector<SafetyWarning> getActiveWarnings() const;
    
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
    
    ADASFeatures enabledFeatures_;
    SafetyConfig safetyConfig_;
    
    void updatePerception();
    void planPath();
    void executeDecisions();
    bool validateSafety();
};

} // namespace adas
} // namespace evlib 