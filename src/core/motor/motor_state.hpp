#pragma once

#include <vector>
#include <string>

namespace autocore {
namespace motor {

class MotorState {
public:
    MotorState();
    ~MotorState() = default;

    // Core state getters
    float getCurrentSpeed() const;
    float getCurrentTorque() const;
    float getMaxSpeed() const;
    float getMaxTorque() const;
    
    // Temperature monitoring
    float getWindingTemperature() const;
    float getBearingTemperature() const;
    void updateTemperatures();

    // Sensor data
    bool checkSensors();
    std::vector<std::string> getSensorDiagnostics() const;
    void calibrateSensors();

    // State management
    void updateState(float speed, float torque);
    bool isWithinOperatingLimits() const;
    float getEfficiency() const;

private:
    float currentSpeed_;
    float currentTorque_;
    float maxSpeed_;
    float maxTorque_;
    float windingTemp_;
    float bearingTemp_;
    bool sensorsCalibrated_;

    void validateState();
    void updateEfficiencyMap();
    bool checkThermalLimits();
    void updateSensorReadings();
};

} // namespace motor
} // namespace autocore
