#pragma once

#include <vector>
#include <chrono>
#include <string>

namespace autocore {
namespace sensors {

enum class SensorHealthStatus {
    HEALTHY,
    DEGRADED,
    FAULTY,
    OFFLINE
};

struct SensorHealth {
    SensorHealthStatus status{SensorHealthStatus::HEALTHY};
    float healthScore{1.0f};  // 0.0 to 1.0
    std::chrono::system_clock::time_point lastUpdate;
    std::vector<std::string> diagnosticMessages;
};

} // namespace sensors
} // namespace autocore
