#pragma once

#include <vector>
#include <chrono>
#include "fusion_types.hpp"
#include "../common/types.hpp"
namespace autocore {
namespace sensors {

// Forward declaration
struct TrackedObject;

enum class SensorType {
    LIDAR,
    CAMERA,
    RADAR,
    IMU,
    GPS
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

struct StateEstimate {
    std::vector<float> position;
    std::vector<float> velocity;
    std::vector<float> acceleration;
    std::vector<float> orientation;
    std::vector<float> angularVelocity;
    std::vector<TrackedObject*> trackedObjects;
    std::chrono::system_clock::time_point timestamp;
};

} // namespace sensors
} // namespace autocore 