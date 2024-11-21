#pragma once

#include <vector>
#include <deque>
#include <chrono>
#include "battery_state.hpp"

namespace autocore {
namespace bms {

struct ThermalZone {
    float temperature;
    float targetTemperature;
    float coolingPower;
    float heatingPower;
    bool active;
};

class ThermalManager {
public:
    ThermalManager();
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
    
    // New advanced cooling features
    void configureZones(const std::vector<ThermalZone>& zones);
    void setZoneTargetTemperature(size_t zoneIndex, float temperature);
    float getZoneTemperature(size_t zoneIndex) const;
    bool isZoneActive(size_t zoneIndex) const;
    
    // Predictive thermal management
    void updateThermalPrediction(float ambientTemp, float powerDraw);
    float getPredictedTemperature(std::chrono::seconds futureTime) const;
    bool willRequireCooling(std::chrono::seconds lookAhead) const;
    
    // Performance monitoring
    float getCoolingEfficiency() const;
    float getHeatingEfficiency() const;
    float getThermalSystemPower() const;

private:
    float currentTemperature_{0.0f};
    float targetTemperature_{25.0f};
    float minTemperature_{0.0f};
    float maxTemperature_{60.0f};
    bool coolingEnabled_{false};
    float fanSpeed_{0.0f};
    float pumpSpeed_{0.0f};
    float predictedTemperature_{0.0f};
    float ambientTemperature_{20.0f};
    std::deque<float> temperatureHistory_;
    std::vector<ThermalZone> thermalZones_;
    float heatingPower_{0.0f};
    
    // New members for enhanced functionality
    float coolingCapacity_{0.0f};
    float heatingCapacity_{0.0f};
    
    bool heatingEnabled_{false};
    
    // Enhanced internal methods
    void adjustCooling();
    void adjustHeating();
    void optimizeThermalControl();
    void updateThermalZones();
    float calculateRequiredCoolingPower();
    float predictTemperatureRise(float powerDraw) const;
    void balanceZonePower();
    bool validateThermalLimits() const;
};

} // namespace bms
} // namespace autocore
