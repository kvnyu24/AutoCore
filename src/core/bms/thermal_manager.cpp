#include "thermal_manager.hpp"
#include <algorithm>
#include <cmath>

namespace autocore {
namespace bms {

void ThermalManager::updateThermalState(const BatteryState& batteryState) {
    // Update current temperatures
    auto cellTemps = batteryState.getCellTemperatures();
    currentTemperature_ = *std::max_element(cellTemps.begin(), cellTemps.end());
    
    // Update thermal zones
    updateThermalZones();
    
    // Record temperature history
    temperatureHistory_.push_back(currentTemperature_);
    if (temperatureHistory_.size() > 3600) { // Keep 1 hour history
        temperatureHistory_.pop_front();
    }
}

void ThermalManager::manageThermalSystem() {
    // Calculate required cooling/heating power
    float requiredCooling = calculateRequiredCoolingPower();
    
    // Optimize thermal zones
    balanceZonePower();
    
    // Adjust cooling and heating systems
    if (requiredCooling > 0) {
        coolingEnabled_ = true;
        adjustCooling();
    } else {
        coolingEnabled_ = false;
        fanSpeed_ = 0.0f;
        pumpSpeed_ = 0.0f;
    }
    
    // Update thermal predictions
    predictedTemperature_ = predictTemperatureRise(getThermalSystemPower());
}

float ThermalManager::calculateRequiredCoolingPower() {
    float tempDelta = currentTemperature_ - targetTemperature_;
    float basePower = tempDelta * 100.0f; // Simple linear model
    
    // Apply efficiency factors
    float efficiencyFactor = 1.0f;
    if (ambientTemperature_ > 35.0f) {
        efficiencyFactor *= 0.8f; // Reduced efficiency in hot weather
    }
    
    return basePower * efficiencyFactor;
}

void ThermalManager::balanceZonePower() {
    if (thermalZones_.empty()) return;
    
    float totalPower = getThermalSystemPower();
    float powerPerZone = totalPower / thermalZones_.size();
    
    for (auto& zone : thermalZones_) {
        if (zone.active) {
            zone.coolingPower = powerPerZone;
        } else {
            zone.coolingPower = 0.0f;
        }
    }
}

} // namespace bms
} // namespace autocore 