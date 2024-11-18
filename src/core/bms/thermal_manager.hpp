#pragma once

#include <vector>
#include "battery_state.hpp"

namespace autocore {
namespace bms {

class ThermalManager {
public:
    ThermalManager() = default;
    ~ThermalManager() = default;

    // Temperature management
    void setTemperatureLimits(float minTemp, float maxTemp) {
        minTemperature_ = minTemp;
        maxTemperature_ = maxTemp;
    }
    
    // Cooling control
    void enableCooling(bool enable) { coolingEnabled_ = enable; }
    void setFanSpeed(float speed) { fanSpeed_ = speed; }
    void setPumpSpeed(float speed) { pumpSpeed_ = speed; }
    
    // Heating control
    void enableHeating(bool enable) { heatingEnabled_ = enable; }
    void setHeatingPower(float power) { heatingPower_ = power; }
    
    // System status
    bool isCoolingActive() const { return coolingEnabled_; }
    bool isHeatingActive() const { return heatingEnabled_; }
    float getCurrentTemperature() const { return currentTemperature_; }
    
    // Thermal management
    void updateThermalState(const BatteryState& batteryState);
    void manageThermalSystem();
    bool isTemperatureInRange() const;

private:
    float minTemperature_{0.0f};
    float maxTemperature_{45.0f};
    float currentTemperature_{25.0f};
    
    bool coolingEnabled_{false};
    bool heatingEnabled_{false};
    float fanSpeed_{0.0f};      // 0.0 to 1.0
    float pumpSpeed_{0.0f};     // 0.0 to 1.0
    float heatingPower_{0.0f};  // 0.0 to 1.0
    
    // Internal methods
    void adjustCooling();
    void adjustHeating();
    void optimizeThermalControl();
};

} // namespace bms
} // namespace autocore
