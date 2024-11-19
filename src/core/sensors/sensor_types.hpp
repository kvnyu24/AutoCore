#pragma once

#include <vector>
#include <chrono>
#include <string>
#include "../common/types.hpp"

namespace autocore {
namespace sensors {

using common::Position;
using common::Velocity;

enum class SensorType {
    LIDAR,
    CAMERA,
    RADAR,
    IMU,
    GPS,
    ULTRASONIC
};

enum class SensorHealthStatus {
    HEALTHY,
    DEGRADED,
    FAULTY,
    OFFLINE
};


struct SensorHealth {
    SensorHealthStatus status{SensorHealthStatus::HEALTHY};
    float healthScore{1.0f};
    std::chrono::system_clock::time_point lastUpdate;
    std::vector<std::string> diagnosticMessages;
};

struct SensorData {
    SensorType type;
    std::chrono::system_clock::time_point timestamp;
    std::vector<float> values;
    bool isValid{true};
};

struct StateEstimate {
    Position position;
    Velocity velocity;
    std::vector<float> acceleration;
    std::vector<float> orientation;
    std::vector<float> angularVelocity;
    float uncertainty;
};

} // namespace sensors
} // namespace autocore
