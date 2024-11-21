#include "battery_state.hpp"
#include <numeric>
#include <algorithm>

namespace autocore {
namespace bms {

float BatteryState::getCellImbalance() const {
    if (cellVoltages_.empty()) return 0.0f;
    
    float minVoltage = *std::min_element(cellVoltages_.begin(), cellVoltages_.end());
    float maxVoltage = *std::max_element(cellVoltages_.begin(), cellVoltages_.end());
    
    return maxVoltage - minVoltage;
}

float BatteryState::getAverageCellVoltage() const {
    if (cellVoltages_.empty()) return 0.0f;
    
    return std::accumulate(cellVoltages_.begin(), cellVoltages_.end(), 0.0f) / 
           cellVoltages_.size();
}

std::optional<size_t> BatteryState::getWeakestCell() const {
    if (cellVoltages_.empty()) return std::nullopt;
    
    return std::distance(cellVoltages_.begin(),
        std::min_element(cellVoltages_.begin(), cellVoltages_.end()));
}

void BatteryState::recordCellHistory() {
    auto now = std::chrono::system_clock::now();
    
    // Initialize histories if needed
    if (cellHistories_.empty()) {
        cellHistories_.resize(cellVoltages_.size());
    }
    
    for (size_t i = 0; i < cellVoltages_.size(); ++i) {
        updateCellHistory(i);
    }
    
    lastUpdate_ = now;
}

float BatteryState::getCellDegradationRate(size_t cellIndex) const {
    if (cellIndex >= cellHistories_.size()) return 0.0f;
    
    const auto& history = cellHistories_[cellIndex];
    return calculateCellDegradation(history);
}

} // namespace bms
} // namespace autocore 