#pragma once

#include <vector>
#include <chrono>

namespace autocore {
namespace diagnostics {

struct DiagnosticData {
    float temperature;
    float voltage;
    float current;
    std::chrono::system_clock::time_point timestamp;
    std::vector<float> sensorReadings;
    bool isValid{true};
};

} // namespace diagnostics
} // namespace autocore 