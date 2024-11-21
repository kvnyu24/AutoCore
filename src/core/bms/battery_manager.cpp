#include "battery_manager.hpp"
#include <numeric>
#include <algorithm>

namespace autocore {
namespace bms {

float BatteryManager::getPredictedRemainingRange() const {
    float soc = getStateOfCharge();
    float avgPower = getAveragePowerConsumption();
    
    if (avgPower <= 0.0f) return 0.0f;
    
    float remainingEnergy = soc * batteryProfile_.nominalCapacity * 
                           batteryProfile_.nominalVoltage;
    return (remainingEnergy / avgPower) * 100.0f; // Assuming 100Wh/km efficiency
}

float BatteryManager::getInstantaneousPower() const {
    float voltage = batteryState_->getVoltage();
    float current = batteryState_->getCurrent();
    return voltage * current;
}

void BatteryManager::updatePowerHistory() {
    float power = getInstantaneousPower();
    powerHistory_.push_back(power);
    
    if (powerHistory_.size() > 3600) { // Keep 1 hour of history at 1Hz
        powerHistory_.pop_front();
    }
    
    // Update average power consumption
    averagePowerConsumption_ = std::accumulate(powerHistory_.begin(), 
                                             powerHistory_.end(), 0.0f) / 
                                             powerHistory_.size();
}

float BatteryManager::getOptimalChargingCurrent() const {
    float soc = getStateOfCharge();
    float temp = batteryState_->getMaxTemperature();
    
    // Implement CC-CV charging algorithm
    if (soc > 0.8f) {
        return batteryProfile_.maxChargeCurrent * (1.0f - (soc - 0.8f) / 0.2f);
    }
    
    // Temperature-based derating
    float tempFactor = 1.0f;
    if (temp > 35.0f) {
        tempFactor = std::max(0.0f, 1.0f - (temp - 35.0f) / 25.0f);
    }
    
    return batteryProfile_.maxChargeCurrent * tempFactor;
}

void BatteryManager::analyzeCellBalance() {
    auto voltages = batteryState_->getCellVoltages();
    float avgVoltage = std::accumulate(voltages.begin(), voltages.end(), 0.0f) / 
                      voltages.size();
    
    cellBalancingCurrents_.resize(voltages.size());
    for (size_t i = 0; i < voltages.size(); ++i) {
        if (voltages[i] > avgVoltage + 0.05f) { // 50mV threshold
            cellBalancingCurrents_[i] = 0.1f; // 100mA balancing current
        } else {
            cellBalancingCurrents_[i] = 0.0f;
        }
    }
}

} // namespace bms
} // namespace autocore 