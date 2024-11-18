#pragma once

#include <string>
#include <vector>
#include <chrono>

namespace autocore {
namespace telematics {

struct Alert {
    std::string message;
    std::chrono::system_clock::time_point timestamp;
    bool isActive{true};
    int severity{0};  // 0-3: LOW to CRITICAL
};

struct VehicleStatus {
    float speed;                  // Current speed in km/h
    float batteryLevel;          // Battery level (0-1)
    float powerConsumption;      // Current power consumption in kW
    bool isCharging{false};
    std::vector<Alert> alerts;
    std::chrono::system_clock::time_point timestamp;
};

} // namespace telematics
} // namespace autocore