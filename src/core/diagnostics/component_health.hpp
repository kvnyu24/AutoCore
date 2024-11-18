#pragma once

#include <string>
#include <chrono>
#include <vector>

namespace autocore {
namespace diagnostics {

enum class ComponentType {
    MOTOR,
    BATTERY,
    INVERTER,
    SENSOR_SYSTEM,
    COOLING_SYSTEM,
    BRAKING_SYSTEM
};

enum class HealthStatus {
    HEALTHY,
    WARNING,
    CRITICAL,
    FAILED
};

struct ComponentHealth {
    ComponentType type;
    HealthStatus status;
    float healthScore;           // 0.0 to 1.0
    float degradationRate;       // Rate of wear/degradation
    std::chrono::system_clock::time_point lastUpdate;
    std::vector<std::string> activeWarnings;
    
    // Helper methods
    bool isHealthy() const { return status == HealthStatus::HEALTHY; }
    bool needsMaintenance() const { return healthScore < 0.7f; }
    bool requiresImmediate() const { return status == HealthStatus::CRITICAL; }
};

struct SystemHealth {
    float overallScore;          // 0.0 to 1.0
    HealthStatus systemStatus;
    std::chrono::system_clock::time_point lastUpdate;
    std::vector<ComponentHealth> componentStates;
};

} // namespace diagnostics
} // namespace autocore
