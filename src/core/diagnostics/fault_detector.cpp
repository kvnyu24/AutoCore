#include "fault_detector.hpp"
#include <algorithm>
#include <cmath>

namespace autocore {
namespace diagnostics {

FaultDetector::FaultDetector() {
    initializeFaultPatterns();
    loadMachineLearningModel();
}

std::vector<Fault> FaultDetector::detectFaults(const DiagnosticData& data) {
    std::vector<Fault> detectedFaults;
    
    // Rule-based fault detection
    detectRuleBasedFaults(data, detectedFaults);
    
    // ML-based anomaly detection
    detectAnomalies(data, detectedFaults);
    
    // Pattern matching
    detectPatternBasedFaults(data, detectedFaults);
    
    // Prioritize and deduplicate faults
    processFaults(detectedFaults);
    
    return detectedFaults;
}

void FaultDetector::detectRuleBasedFaults(
    const DiagnosticData& data,
    std::vector<Fault>& faults) {
    
    // Check temperature thresholds
    if (data.temperature > TEMP_THRESHOLD) {
        faults.push_back(Fault{
            FaultType::TEMPERATURE_WARNING,
            "Temperature exceeds threshold",
            data.timestamp,
            FaultSeverity::HIGH
        });
    }
    
    // Check voltage levels
    if (data.voltage < VOLTAGE_MIN_THRESHOLD) {
        faults.push_back(Fault{
            FaultType::VOLTAGE_WARNING,
            "Voltage below minimum threshold",
            data.timestamp,
            FaultSeverity::CRITICAL
        });
    }
}

void FaultDetector::detectAnomalies(
    const DiagnosticData& data,
    std::vector<Fault>& faults) {
    
    // Use ML model for anomaly detection
    if (mlModel_->predictAnomaly(data)) {
        faults.push_back(Fault{
            FaultType::ANOMALY_DETECTED,
            "ML model detected anomaly",
            data.timestamp,
            FaultSeverity::MEDIUM
        });
    }
}

private:
    void processFaults(std::vector<Fault>& faults) {
        // Remove duplicates
        std::sort(faults.begin(), faults.end());
        faults.erase(std::unique(faults.begin(), faults.end()), faults.end());
        
        // Prioritize by severity
        std::sort(faults.begin(), faults.end(), 
            [](const Fault& a, const Fault& b) {
                return a.severity > b.severity;
            });
    }
};

} // namespace diagnostics
} // namespace autocore 