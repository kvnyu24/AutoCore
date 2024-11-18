#pragma once

#include <chrono>
#include <vector>
#include <string>
#include "component_health.hpp"

namespace autocore {
namespace diagnostics {

struct DiagnosticData {
    std::chrono::system_clock::time_point timestamp;
    
    // Core measurements
    float temperature;
    float voltage;
    float current;
    float power;
    
    // Component-specific data
    struct ComponentData {
        ComponentType type;
        std::vector<float> measurements;
        std::vector<std::string> sensorReadings;
        bool isValid;
    };
    std::vector<ComponentData> componentData;
    
    // Performance metrics
    float efficiency;
    float degradation;
    std::vector<float> performanceMetrics;
    
    // System status
    bool systemOperational;
    std::vector<std::string> activeAlerts;
    
    // Helper methods
    bool isValid() const { return systemOperational && !componentData.empty(); }
    bool hasWarnings() const { return !activeAlerts.empty(); }
    ComponentData getComponentData(ComponentType type) const;
};

struct DiagnosticReport {
    SystemHealth systemHealth;
    std::vector<ComponentHealth> componentHealthStates;
    std::vector<std::string> recommendations;
    std::chrono::system_clock::time_point generationTime;
    
    bool hasIssues() const {
        return st
}
