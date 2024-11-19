#pragma once

#include <vector>
#include <chrono>
#include "fusion_types.hpp"
#include "../common/types.hpp"
#include "sensor_types.hpp"

namespace autocore {
namespace sensors {


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

} // namespace sensors
} // namespace autocore 