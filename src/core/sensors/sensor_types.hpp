#pragma once

#include <vector>
#include <chrono>
#include <string>
#include "../common/types.hpp"

namespace autocore {
namespace sensors {

using common::Position;
using common::Velocity;

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

struct SensorData {
    std::vector<float> values;
    std::chrono::system_clock::time_point timestamp;
    bool isValid{true};
};

struct StateEstimate {
    Position position;
    Velocity velocity;
    float uncertainty;
};

} // namespace sensors
} // namespace autocore
