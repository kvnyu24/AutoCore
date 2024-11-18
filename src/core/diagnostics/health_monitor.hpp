#pragma once

#include <map>
#include <string>
#include "component_health.hpp"
#include "../ml/anomaly_detector.hpp"

namespace autocore {
namespace diagnostics {

class HealthMonitor {
public:
    HealthMonitor() {
        // Initialize anomaly detector with appropriate config
        ml::ModelConfig config;
        config.type = ml::ModelType::ISOLATION_FOREST;
        config.numFeatures = 5;  // Adjust based on your diagnostic features
        config.anomalyThreshold = 0.95f;
        config.useTemporalFeatures = true;
        
        anomalyDetector_ = std::make_unique<ml::AnomalyDetector>();
        anomalyDetector_->setModelParameters(config);
    }
    
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
    std::unique_ptr<ml::AnomalyDetector> anomalyDetector_;
    
    void updateDegradationModels();
    void calculateHealthScores();
    bool detectAnomalousWear();
};

} // namespace diagnostics
} // namespace autocore 