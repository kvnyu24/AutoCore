#pragma once

#include <memory>
#include <vector>
#include <deque>
#include "battery_state.hpp"
#include "thermal_manager.hpp"

namespace autocore {
namespace bms {

struct BatteryProfile {
    float nominalVoltage;
    float nominalCapacity;
    float internalResistance;
    size_t numberOfCells;
    float maxChargeCurrent;
    float maxDischargeCurrent;
};

class BatteryManager {
public:
    BatteryManager();
    ~BatteryManager() = default;

    // Core BMS functions
    float getStateOfCharge() const;
    float getStateOfHealth() const;
    float getCellTemperature(size_t cellIndex) const;
    
    // Battery management
    void updateBatteryState();
    void balanceCells();
    bool detectFaults();

    // New advanced monitoring functions
    float getPredictedRemainingRange() const;
    float getInstantaneousPower() const;
    float getAveragePowerConsumption() const;
    std::vector<float> getCellBalancingStatus() const;
    
    // Enhanced thermal management
    void manageThermalSystem();
    void setThermalLimits(float minTemp, float maxTemp);
    bool isThermalManagementActive() const;
    
    // Predictive maintenance
    float getPredictedLifetime() const;
    std::vector<std::string> getHealthWarnings() const;
    bool needsMaintenance() const;
    
    // Power management
    void setChargingProfile(const BatteryProfile& profile);
    float getOptimalChargingCurrent() const;
    bool isChargingOptimal() const;

private:
    std::unique_ptr<BatteryState> batteryState_;
    std::unique_ptr<ThermalManager> thermalManager_;
    std::vector<float> cellVoltages_;
    std::vector<float> cellTemperatures_;
    
    // New members
    BatteryProfile batteryProfile_;
    std::deque<float> powerHistory_;
    std::vector<float> cellBalancingCurrents_;
    float averagePowerConsumption_;
    
    // Internal methods
    void calculateStateOfCharge();
    void monitorCellHealth();
    void updatePowerHistory();
    void analyzeCellBalance();
    float calculateOptimalChargingCurrent() const;
    void updateBatteryProfile();
    bool validateChargingParameters() const;
    void predictMaintenance();
};

} // namespace bms
} // namespace autocore 