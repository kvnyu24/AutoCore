#include "vehicle_control_system.hpp"
#include <algorithm>
#include <cmath>

namespace autocore {
namespace vcs {

VehicleControlSystem::VehicleControlSystem(
    std::shared_ptr<bms::BatteryManager> bms,
    std::shared_ptr<motor::MotorController> motorController,
    std::shared_ptr<sensors::SensorManager> sensorManager,
    std::shared_ptr<telematics::TelematicsManager> telematicsManager,
    std::shared_ptr<diagnostics::DiagnosticManager> diagnosticManager)
    : bms_(std::move(bms))
    , motorController_(std::move(motorController))
    , sensorManager_(std::move(sensorManager))
    , telematicsManager_(std::move(telematicsManager))
    , diagnosticManager_(std::move(diagnosticManager))
    , currentMode_(DrivingMode::NORMAL)
    , economyModeEnabled_(false) {
    
    // Initialize vehicle state
    currentState_ = VehicleState{
        .speed = 0.0f,
        .acceleration = 0.0f,
        .batteryLevel = bms_->getStateOfCharge(),
        .temperature = motorController_->getTemperature()
    };
    
    // Perform initial system check
    performSystemCheck();
}

void VehicleControlSystem::executeControlLoop() {
    // Update sensor data
    sensorManager_->updateSensorData();
    auto fusedData = sensorManager_->getFusedData();
    
    // Update vehicle state
    updateVehicleState();
    
    // Calculate optimal power distribution
    float optimalPower = calculateOptimalPowerDistribution();
    
    // Update motor control
    if (currentMode_ == DrivingMode::ECONOMY) {
        optimalPower *= 0.8f; // Reduce power for economy mode
    }
    
    motorController_->setTargetTorque(optimalPower / currentState_.speed);
    
    // Process system alerts
    processSystemAlerts();
    
    // Update telemetry
    updateTelemetryData();
}

void VehicleControlSystem::setTargetSpeed(float speedKmh) {
    float speedRPM = speedKmh * 9.5493f; // Convert km/h to RPM
    motorController_->setTargetSpeed(speedRPM);
    
    // Enable regenerative braking if slowing down
    if (speedKmh < currentState_.speed) {
        float brakingLevel = std::min(1.0f, 
            (currentState_.speed - speedKmh) / 50.0f);
        motorController_->setRegenerativeBrakingLevel(brakingLevel);
    }
}

float VehicleControlSystem::calculateOptimalPowerDistribution() {
    float batterySOC = bms_->getStateOfCharge();
    float motorTemp = motorController_->getTemperature();
    float vehicleSpeed = currentState_.speed;
    
    // Calculate base power requirement
    float basePower = vehicleSpeed * 0.015f; // Simple power model
    
    // Apply efficiency factors
    float efficiencyFactor = 1.0f;
    if (batterySOC < 0.2f) {
        efficiencyFactor *= 0.7f; // Reduce power when battery is low
    }
    if (motorTemp > 60.0f) {
        efficiencyFactor *= 0.8f; // Reduce power when motor is hot
    }
    
    return basePower * efficiencyFactor;
}

void VehicleControlSystem::processSystemAlerts() {
    auto diagnostics = diagnosticManager_->getSystemHealth();
    if (diagnostics.severity >= AlertSeverity::HIGH) {
        enableEmergencyMode();
    }
}

} // namespace vcs
} // namespace autocore