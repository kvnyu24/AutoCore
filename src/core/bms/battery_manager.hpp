#pragma once

#include <memory>
#include <vector>
#include "battery_state.hpp"
#include "thermal_manager.hpp"

namespace evlib {
namespace bms {

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

    // Thermal management
    void manageThermalSystem();
    void setThermalLimits(float minTemp, float maxTemp);

private:
    std::unique_ptr<BatteryState> batteryState_;
    std::unique_ptr<ThermalManager> thermalManager_;
    std::vector<float> cellVoltages_;
    std::vector<float> cellTemperatures_;
    
    // Internal methods
    void calculateStateOfCharge();
    void monitorCellHealth();
};

} // namespace bms
} // namespace evlib 