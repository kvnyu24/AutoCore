#pragma once

#include <vector>
#include <chrono>
#include <string>
#include "../common/types.hpp"

namespace autocore {
    namespace sensors {

    using common::Position;
    using common::Velocity;
    using common::Orientation;

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

    struct RawSensorData {
        SensorType type;
        std::chrono::system_clock::time_point timestamp;
        std::vector<float> rawValues;
        bool isValid{true};
    };

    struct FusedData {
        std::vector<SensorData> sensorInputs;
        std::chrono::system_clock::time_point timestamp;
        bool isValid{true};
    };

    struct Feature {
        double x;
        double y;
        double confidence;
        uint32_t id;
    };

    struct TrackedObject {
        uint32_t id;
        Position position;
        double velocity;
        double heading;
    };

    struct StateEstimate {
        Position position;
        Velocity velocity;
        std::vector<float> acceleration;
        std::vector<float> orientation;
        std::vector<float> angularVelocity;
        float uncertainty;
        std::chrono::system_clock::time_point timestamp;
    };

    } // namespace sensors
} // namespace autocore
