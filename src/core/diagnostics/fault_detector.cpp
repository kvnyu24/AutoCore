#include "fault_detector.hpp"
#include <algorithm>
#include <cmath>
#include <deque>
#include <chrono>
#include <map>
#include "common/types.hpp"

namespace autocore {
namespace diagnostics {
namespace ml {
// ML namespace content
}

FaultDetector::FaultDetector() : mlModel_(std::make_unique<ml::AnomalyDetector>()) {
    initializeFaultPatterns();
    loadMachineLearningModel();
    
    // Initialize temporal analysis
    temporalAnalysis_.analysisWindow = std::chrono::seconds(3600);
    temporalAnalysis_.maxFaultHistory = 100;
}

std::vector<Fault> FaultDetector::detectFaults(const DiagnosticData& data) {
    std::vector<Fault> detectedFaults;
    
    // Rule-based fault detection
    detectRuleBasedFaults(data, detectedFaults);
    
    // ML-based anomaly detection
    detectAnomalies(data, detectedFaults);
    
    // Pattern matching
    detectPatternBasedFaults(data, detectedFaults);
    
    // New temporal analysis
    analyzeTemporalPatterns(data);
    
    // Escalate severity based on patterns
    escalateSeverity(detectedFaults);
    
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

void FaultDetector::processFaults(std::vector<Fault>& faults) {
    // Remove duplicates
    std::sort(faults.begin(), faults.end());
    faults.erase(std::unique(faults.begin(), faults.end()), faults.end());
    
    // Prioritize by severity
    std::sort(faults.begin(), faults.end(), 
        [](const Fault& a, const Fault& b) {
            return a.severity > b.severity;
        });
}

void FaultDetector::detectPatternBasedFaults(
    const DiagnosticData& data,
    std::vector<Fault>& faults) {
    
    // Check each fault pattern
    for (const auto& pattern : faultPatterns_) {
        // Check if current faults match pattern sequence
        bool patternMatch = true;
        auto currentTime = data.timestamp;
        
        for (const auto& faultType : pattern.sequence) {
            // Check if this fault type exists within time window
            bool foundFault = std::any_of(faults.begin(), faults.end(),
                [&](const Fault& fault) {
                    return fault.type == faultType &&
                           (currentTime - fault.timestamp) <= pattern.timeWindow;
                });
                
            if (!foundFault) {
                patternMatch = false;
                break;
            }
        }
        
        // If pattern matches, add composite fault
        if (patternMatch) {
            faults.push_back(Fault{
                FaultType::ANOMALY_DETECTED,
                pattern.description,
                data.timestamp,
                pattern.resultingSeverity
            });
        }
    }
}

void FaultDetector::initializeFaultPatterns() {
    // Initialize default fault patterns
    faultPatterns_.clear();
    // Add patterns as needed
}

void FaultDetector::loadMachineLearningModel() {
    // Load and initialize ML model
    mlModel_->loadModel("path/to/model");
    // Configure model parameters as needed
}

void FaultDetector::analyzeTemporalPatterns(const DiagnosticData& data) {
    // Add current faults to history
    for (const auto& fault : temporalAnalysis_.recentFaults) {
        if (temporalAnalysis_.recentFaults.size() >= temporalAnalysis_.maxFaultHistory) {
            temporalAnalysis_.recentFaults.pop_front();
        }
        temporalAnalysis_.recentFaults.push_back(fault);
    }
    
    // Analyze patterns
    for (const auto& pattern : faultPatterns_) {
        bool patternFound = true;
        auto currentTime = data.timestamp;
        
        for (const auto& requiredFault : pattern.sequence) {
            if (!matchesTemporalPattern(requiredFault, pattern)) {
                patternFound = false;
                break;
            }
        }
        
        if (patternFound) {
            auto& match = temporalAnalysis_.activePatterns[pattern.description];
            match.pattern = pattern;
            match.occurrences++;
            
            if (match.occurrences == 1) {
                match.firstOccurrence = currentTime;
            }
        }
    }
}

void FaultDetector::escalateSeverity(std::vector<Fault>& faults) {
    for (auto& fault : faults) {
        // Check for repeated occurrences
        int occurrences = std::count_if(
            temporalAnalysis_.recentFaults.begin(),
            temporalAnalysis_.recentFaults.end(),
            [&](const Fault& f) { return f.type == fault.type; }
        );
        
        // Escalate severity based on frequency
        if (occurrences > 5 && fault.severity != FaultSeverity::CRITICAL) {
            fault.severity = static_cast<FaultSeverity>(
                std::min(static_cast<int>(FaultSeverity::CRITICAL),
                        static_cast<int>(fault.severity) + 1)
            );
        }
    }
}

bool FaultDetector::matchesTemporalPattern(const FaultType& faultType, const FaultPattern& pattern) {
    return std::any_of(temporalAnalysis_.recentFaults.begin(),
                      temporalAnalysis_.recentFaults.end(),
                      [&](const Fault& fault) {
                          return fault.type == faultType;
                      });
}

} // namespace diagnostics
} // namespace autocore 