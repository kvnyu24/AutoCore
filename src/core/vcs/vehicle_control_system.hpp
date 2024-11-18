#pragma once

#include <memory>
#include <string>
#include <chrono>
#include <vector>
#include "../bms/battery_manager.hpp"
#include "../motor/motor_controller.hpp"
#include "../sensors/sensor_manager.hpp"
#include "../telematics/telematics_manager.hpp"
#include "../diagnostics/diagnostic_manager.hpp"

namespace autocore {
namespace vcs {

    enum class DrivingMode {
        NORMAL,
        SPORT,
        ECONOMY,
        EMERGENCY
    };

    enum class AlertSeverity {
        LOW,
        MEDIUM,
        HIGH,
        CRITICAL
    };

    struct VehicleState {
        float speed;
        float acceleration;
        float batteryLevel;
        float temperature;
    };

    struct SafetyAlert {
        std::string message;
        AlertSeverity severity;
        std::chrono::system_clock::time_point timestamp;
        bool isActive{true};
    };

    class VehicleControlSystem {
    public:
        VehicleControlSystem(
            std::shared_ptr<bms::BatteryManager> bms,
            std::shared_ptr<motor::MotorController> motorController,
            std::shared_ptr<sensors::SensorManager> sensorManager,
            std::shared_ptr<telematics::TelematicsManager> telematicsManager,
            std::shared_ptr<diagnostics::DiagnosticManager> diagnosticManager
        );

        // Vehicle control
        void setDrivingMode(DrivingMode mode);
        void updateVehicleState();
        void executeControlLoop();
        
        // Performance management
        void optimizePerformance();
        void setEconomyMode(bool enabled);
        float getEstimatedRange();
        
        // Safety systems
        bool performSystemCheck();
        void enableEmergencyMode();
        std::vector<SafetyAlert> getActiveAlerts();
        
        // Vehicle dynamics
        void setTargetSpeed(float speedKmh);
        void setTargetAcceleration(float accelerationMs2);
        VehicleState getCurrentState();

    private:
        std::shared_ptr<bms::BatteryManager> bms_;
        std::shared_ptr<motor::MotorController> motorController_;
        std::shared_ptr<sensors::SensorManager> sensorManager_;
        std::shared_ptr<telematics::TelematicsManager> telematicsManager_;
        std::shared_ptr<diagnostics::DiagnosticManager> diagnosticManager_;
        
        DrivingMode currentMode_;
        VehicleState currentState_;
        bool economyModeEnabled_;
        
        void updateSystems();
        void processSystemAlerts();
        float calculateOptimalPowerDistribution();
        void updateTelemetryData();
    };
    
} // namespace vcs
} // namespace autocore