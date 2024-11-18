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
#include "component_health.hpp"
#include "diagnostic_data.hpp"

namespace autocore {
namespace diagnostics {

struct Error {
    std::string message;
    std::chrono::system_clock::time_point timestamp;
    std::string component;
    int severity;
};

struct ErrorLog {
    std::vector<Error> errors;
    std::chrono::system_clock::time_point lastUpdate;
};

class DiagnosticManager {
public:
    DiagnosticManager(
        std::shared_ptr<bms::BatteryManager> bms,
        std::shared_ptr<motor::MotorController> motorController,
        std::shared_ptr<sensors::SensorManager> sensorManager,
        std::shared_ptr<telematics::TelematicsManager> telematicsManager
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

    // Advanced diagnostics
    bool validateSystemIntegrity() const;
    float getSystemReliabilityScore() const;
    std::vector<ComponentHealth> getComponentHealthStatus() const;
    
    // Thermal management
    void monitorThermalStatus();
    bool isThermalStateNormal() const;
    float getMaxTemperature() const;
    
    // Performance monitoring
    float getSystemEfficiency() const;
    PerformanceMetrics getPerformanceMetrics() const;
    
    // Safety checks
    bool performSafetyCheck();
    std::vector<SafetyAlert> getActiveSafetyAlerts() const;
    bool isEmergencyStateActive() const;

private:
    std::shared_ptr<bms::BatteryManager> bms_;
    std::shared_ptr<motor::MotorController> motorController_;
    std::shared_ptr<sensors::SensorManager> sensorManager_;
    std::shared_ptr<telematics::TelematicsManager> telematicsManager_;
    
    std::unique_ptr<CANBusMonitor> canBusMonitor_;
    std::unique_ptr<FaultDetector> faultDetector_;
    std::unique_ptr<HealthMonitor> healthMonitor_;
    
    // Internal diagnostic data
    DiagnosticData currentDiagnostics_;
    std::vector<SafetyAlert> activeAlerts_;
    PerformanceMetrics performanceMetrics_;
    bool emergencyState_{false};
    
    // Internal methods
    void analyzeDiagnosticData();
    void updatePredictiveModels();
    bool validateSystemState();
    void updateComponentHealth();
    void processHealthMetrics();
    void updateSafetyStatus();
    float calculateSystemReliability() const;
    void aggregatePerformanceMetrics();
    void handleEmergencyState();
    void updateThermalMetrics();
    bool validateSensorData() const;
    void processCANDiagnostics();
    void updateMaintenanceSchedule();
};

} // namespace diagnostics
} // namespace autocore 