#pragma once

#include <memory>
#include "../adas/adas_manager.hpp"
#include "../sensors/fusion_engine.hpp"
#include "../vcs/vehicle_control_system.hpp"
#include "planning/trajectory_planner.hpp"
#include "behavior/behavior_planner.hpp"
#include "localization/localization_system.hpp"

namespace evlib {
namespace autonomous {

class AutonomousManager {
public:
    AutonomousManager(
        std::shared_ptr<adas::ADASManager> adasManager,
        std::shared_ptr<sensors::FusionEngine> fusionEngine,
        std::shared_ptr<vcs::VehicleControlSystem> vcs
    );

    // Autonomous driving control
    void enableAutonomousMode(bool enable);
    void setDestination(const Position& destination);
    AutonomousState getState() const;
    
    // Planning and control
    void updatePerception();
    void planTrajectory();
    void executeTrajectory();
    
    // Safety and monitoring
    bool validateSafety() const;
    std::vector<SafetyEvent> getActiveSafetyEvents() const;
    void handleEmergency();

private:
    std::shared_ptr<adas::ADASManager> adasManager_;
    std::shared_ptr<sensors::FusionEngine> fusionEngine_;
    std::shared_ptr<vcs::VehicleControlSystem> vcs_;
    
    std::unique_ptr<TrajectoryPlanner> trajectoryPlanner_;
    std::unique_ptr<BehaviorPlanner> behaviorPlanner_;
    std::unique_ptr<LocalizationSystem> localizationSystem_;
    
    AutonomousState currentState_;
    SafetyMonitor safetyMonitor_;
    
    void updateLocalization();
    void updateBehaviorState();
    bool checkSystemReadiness();
};

} // namespace autonomous
} // namespace evlib