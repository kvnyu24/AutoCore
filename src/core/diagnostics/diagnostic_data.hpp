#pragma once

#include <chrono>
#include <vector>
#include <string>
#include "component_health.hpp"
#include "fault_detector.hpp"

namespace autocore {
namespace diagnostics {

struct PerformanceMetrics {
    float cpuUsage;
    float memoryUsage;
    float systemLatency;
    float processingTime;
    float powerEfficiency;
    std::chrono::system_clock::time_point timestamp;
};

struct DiagnosticData {
    std::chrono::system_clock::time_point timestamp;
    
    // Core measurements
    float temperature;
    float voltage;
    float current;
    float power;
    
    // Component-specific data
    struct ComponentData {
        ComponentType componentType;
        std::vector<float> measurements;
        std::vector<std::string> sensorReadings;
        float healthScore;           // 0.0 to 1.0
        float degradationRate;       // Rate of wear over time
        std::chrono::system_clock::time_point lastUpdate;
        bool isValid;
        
        // Helper methods
        bool needsMaintenance() const { return healthScore < 0.7f; }
        bool requiresImmediate() const { return healthScore < 0.3f; }
    };
    std::vector<ComponentData> componentData;
    
    // Performance metrics
    float efficiency;
    float degradation;
    std::vector<float> performanceMetrics;
    struct PerformanceData {
        float cpuUsage;
        float memoryUsage;
        float networkLatency;
        float processingTime;
    } performance;
    
    // System status
    bool systemOperational;
    std::vector<std::string> activeAlerts;
    std::vector<Fault> activeFaults;
    
    // Helper methods
    bool isValid() const { 
        return systemOperational && !componentData.empty(); 
    }
    
    bool hasWarnings() const { 
        return !activeAlerts.empty() || !activeFaults.empty(); 
    }
    
    bool hasCriticalFaults() const {
        return std::any_of(activeFaults.begin(), activeFaults.end(),
            [](const Fault& fault) { 
                return fault.severity == FaultSeverity::CRITICAL; 
            });
    }
    
    ComponentData getComponentData(ComponentType type) const;
    
    float getOverallHealth() const {
        if (componentData.empty()) return 0.0f;
        float sum = 0.0f;
        for (const auto& comp : componentData) {
            sum += comp.healthScore;
        }
        return sum / componentData.size();
    }
};

struct SafetyAlert {
    std::string message;
    int severity;
    std::chrono::system_clock::time_point timestamp;
    bool isActive{true};
    std::string componentId;
};

struct DiagnosticReport {
    SystemHealth systemHealth;
    std::vector<ComponentHealth> componentHealthStates;
    std::vector<std::string> recommendations;
    std::vector<Fault> detectedFaults;
    std::chrono::system_clock::time_point generationTime;
    
    bool hasIssues() const {
        return systemHealth.systemStatus != HealthStatus::HEALTHY || 
               !componentHealthStates.empty() || 
               !recommendations.empty() ||
               !detectedFaults.empty();
    }
    
    bool hasCriticalIssues() const {
        return systemHealth.systemStatus == HealthStatus::CRITICAL ||
               std::any_of(detectedFaults.begin(), detectedFaults.end(),
                   [](const Fault& fault) { 
                       return fault.severity == FaultSeverity::CRITICAL; 
                   });
    }
};

struct MaintenanceSchedule {
    std::vector<ComponentType> componentsNeedingService;
    std::vector<float> estimatedLifetimes;
    std::vector<std::string> maintenanceRecommendations;
    std::chrono::system_clock::time_point nextServiceDate;
    bool hasMaintenanceItems() const { return !componentsNeedingService.empty(); }
};

} // namespace diagnostics
} // namespace autocore
