#pragma once

#include <vector>
#include <chrono>
#include <deque>
#include <optional>

namespace autocore {
namespace bms {

struct CellHistory {
    std::deque<float> voltageHistory;
    std::deque<float> temperatureHistory;
    std::chrono::system_clock::time_point timestamp;
};

class BatteryState {
public:
    BatteryState() = default;
    ~BatteryState() = default;

    // Core battery state getters
    float getStateOfCharge() const { return stateOfCharge_; }
    float getStateOfHealth() const { return stateOfHealth_; }
    float getVoltage() const { return voltage_; }
    float getCurrent() const { return current_; }
    float getTemperature(size_t cellIndex) const;
    
    // Cell-specific information
    std::vector<float> getCellVoltages() const { return cellVoltages_; }
    std::vector<float> getCellTemperatures() const { return cellTemperatures_; }
    size_t getNumberOfCells() const { return cellVoltages_.size(); }
    
    // Enhanced cell monitoring
    float getCellImbalance() const;
    float getAverageCellVoltage() const;
    float getVoltageDelta() const;
    std::optional<size_t> getWeakestCell() const;
    
    // Historical data analysis
    void recordCellHistory();
    CellHistory getCellHistory(size_t cellIndex) const;
    float getCellDegradationRate(size_t cellIndex) const;
    
    // State updates
    void updateStateOfCharge(float soc) { 
        stateOfCharge_ = std::clamp(soc, 0.0f, 1.0f);
        recordCellHistory();
    }
    void updateStateOfHealth(float soh) { stateOfHealth_ = soh; }
    void updateVoltage(float v) { voltage_ = v; }
    void updateCurrent(float i) { current_ = i; }
    void updateCellVoltages(const std::vector<float>& voltages) { cellVoltages_ = voltages; }
    void updateCellTemperatures(const std::vector<float>& temperatures) { cellTemperatures_ = temperatures; }

    // Battery limits
    float getMaxVoltage() const { return maxVoltage_; }
    float getMinVoltage() const { return minVoltage_; }
    float getMaxCurrent() const { return maxCurrent_; }
    float getMaxTemperature() const { return maxTemperature_; }

    // New functionality
    bool isCellBalancingRequired() const;
    float getPredictedRemainingCapacity() const;
    std::vector<float> getCellBalancingPriorities() const;

private:
    float stateOfCharge_{1.0f};    // 0.0 to 1.0
    float stateOfHealth_{1.0f};    // 0.0 to 1.0
    float voltage_{0.0f};          // Current voltage
    float current_{0.0f};          // Current amperage
    
    // Cell-level information
    std::vector<float> cellVoltages_;
    std::vector<float> cellTemperatures_;
    
    // New members
    static constexpr size_t HISTORY_SIZE = 1000;
    static constexpr float CELL_IMBALANCE_THRESHOLD = 0.1f;
    std::vector<CellHistory> cellHistories_;
    float internalResistance_{0.0f};
    float capacityDegradation_{0.0f};
    
    // Battery limits
    const float maxVoltage_{400.0f};      // Maximum safe voltage
    const float minVoltage_{280.0f};      // Minimum safe voltage
    const float maxCurrent_{300.0f};      // Maximum safe current
    const float maxTemperature_{60.0f};   // Maximum safe temperature
    
    std::chrono::system_clock::time_point lastUpdate_;
    
    // Helper methods
    void updateCellHistory(size_t cellIndex);
    float calculateCellDegradation(const CellHistory& history) const;
};

} // namespace bms
} // namespace autocore 
