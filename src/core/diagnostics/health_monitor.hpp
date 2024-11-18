#pragma once

#include <map>
#include <string>
#include "component_health.hpp"
#include "predictive_model.hpp"

namespace evlib {
namespace diagnostics {

class HealthMonitor {
public:
    HealthMonitor();
    
    // Health monitoring
    ComponentHealth getComponentHealth(ComponentType type) const;
    SystemHealth getOverallHealth() const;
    
    // Predictive maintenance
    float predictRemainingLifetime(ComponentType type) const;
    MaintenanceSchedule generateMaintenanceSchedule() const;
    
    // Health updates
    void updateHealthMetrics(const DiagnosticData& data);
    void logComponentDegradation(ComponentType type, float degradation);

private:
    std::map<ComponentType, ComponentHealth> componentHealth_;
    std::unique_ptr<PredictiveModel> predictiveModel_;
    
    void updateDegradationModels();
    void calculateHealthScores();
    bool detectAnomalousWear();
};

} // namespace diagnostics
} // namespace evlib 