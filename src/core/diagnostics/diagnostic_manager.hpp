#pragma once

#include <memory>
#include <vector>
#include "../bms/battery_manager.hpp"
#include "../motor/motor_controller.hpp"
#include "../sensors/sensor_manager.hpp"
#include "../telematics/telematics_manager.hpp"
#include "can_bus_monitor.hpp"
#include "fault_detector.hpp"
#include "health_monitor.hpp"

namespace autocore {
namespace diagnostics {

class DiagnosticManager {
public:
    DiagnosticManager(
        std::shared_ptr<bms::BatteryManager> bms,
        std::shared_ptr<motor::MotorController> motorController,
        std::shared_ptr<sensors::SensorManager> sensorManager
    );

    // System diagnostics
    SystemHealth getSystemHealth() const;
    std::vector<Fault> getActiveFaults() const;
    DiagnosticReport generateReport() const;
    
    // Real-time monitoring
    void updateDiagnostics();
    bool performSelfTest();
    
    // CAN bus monitoring
    std::vector<CANMessage> getCANMessages() const;
    void parseCANMessage(const CANMessage& message);
    
    // Predictive maintenance
    MaintenanceSchedule getPredictiveMaintenanceSchedule() const;
    float getComponentLifetimeEstimate(ComponentType type) const;
    
    // Error logging
    void logError(const Error& error);
    ErrorLog getErrorLog() const;

private:
    std::shared_ptr<bms::BatteryManager> bms_;
    std::shared_ptr<motor::MotorController> motorController_;
    std::shared_ptr<sensors::SensorManager> sensorManager_;
    
    std::unique_ptr<CANBusMonitor> canBusMonitor_;
    std::unique_ptr<FaultDetector> faultDetector_;
    std::unique_ptr<HealthMonitor> healthMonitor_;
    
    void analyzeDiagnosticData();
    void updatePredictiveModels();
    bool validateSystemState();
};

} // namespace diagnostics
} // namespace autocore 